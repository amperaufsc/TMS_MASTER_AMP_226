/*
 * can.c — Módulo de comunicação CAN da Master
 *
 *  Created on: Sep 7, 2025
 *      Author: Guilherme Lettmann
 *
 *  Responsabilidades:
 *    - Receber e processar mensagens das 4 Slaves (temperaturas e erros) via FDCAN1
 *    - Enviar resumo do TMS para o barramento geral do carro via FDCAN2
 *    - [testLoopback] Enviar dados de teste pelo CAN1 e drenar recepção do CAN2
 *
 *  Arquitetura FreeRTOS:
 *    ISR (HAL_FDCAN_RxFifo0Callback) → Message Queue → processRxQueue() na Thread
 *
 *  Padrão de envio (espelhado da Slave):
 *    sendSingleFrame() centraliza a lógica de retry e detecção de falha fatal.
 *    Cada quadro tem até CAN_TX_RETRY_MAX tentativas. Se CAN_TX_FAULT_THRESHOLD
 *    falhas consecutivas ocorrerem sem sucesso intermediário, o sistema entra em
 *    Error_Handler → Shutdown (SDC).
 */

#include "can.h"
#include "errors.h"
#include "string.h"
#include "cmsis_os.h"

/* ==================== Handles dos periféricos FDCAN =================== */
extern FDCAN_HandleTypeDef hfdcan2;   // FDCAN2: barramento geral do carro (TX)

#ifdef testLoopback
extern FDCAN_HandleTypeDef hfdcan1;   // FDCAN1: barramento das Slaves (loopback TX)
#endif

/* ==================== Variáveis de TX (FDCAN2) ======================== */
uint8_t FDCAN2TxData[8];              // Buffer de payload para transmissão CAN2
FDCAN_TxHeaderTypeDef FDCAN2TxHeader; // Header de transmissão CAN2 (Extended ID)

#ifdef testLoopback
uint8_t FDCAN1TxData[8];              // Buffer de payload para transmissão CAN1
FDCAN_TxHeaderTypeDef FDCAN1TxHeader; // Header de transmissão CAN1 (Standard ID)
#endif

/* ==================== Código de erro do TMS =========================== */
int tmsErrorCode = 0;

/* ==================== Buffers de temperatura das Slaves =============== */
float slave1TempBuffer[thermistorsRecieved] = {0},
      slave2TempBuffer[thermistorsRecieved] = {0},
      slave3TempBuffer[thermistorsRecieved] = {0},
      slave4TempBuffer[thermistorsRecieved] = {0};

/* Timestamp da última mensagem recebida de cada Slave (para timeout) */
uint32_t slave1LastMessageTick = 0, slave2LastMessageTick = 0,
         slave3LastMessageTick = 0, slave4LastMessageTick = 0;

/* Arrays de ponteiros para acesso indexado por número do Slave (0–3) */
float* slaveTempBuffers[4] = {
		slave1TempBuffer,
		slave2TempBuffer,
		slave3TempBuffer,
		slave4TempBuffer
};

uint32_t* slaveLastMessageTicks[4] = {
		&slave1LastMessageTick,
		&slave2LastMessageTick,
		&slave3LastMessageTick,
		&slave4LastMessageTick
};

uint32_t slaveBurstBaseId[4] = {
		idSlave1Burst0,
		idSlave2Burst0,
		idSlave3Burst0,
		idSlave4Burst0
};

/* ==================== Contador de falhas TX ============================ */
static uint8_t canConsecutiveFailures = 0;

/* ==================== Funções de processamento RX ===================== */

/**
 * @brief  Processa um burst de temperatura recebido de uma Slave.
 */
static void processSlaveBurst(uint8_t *data, uint8_t slave, uint8_t burst)
{
	float temp1, temp2;
	memcpy(&temp1, &data[0], sizeof(float));
	memcpy(&temp2, &data[4], sizeof(float));

	uint8_t index = burst * 2;

	slaveTempBuffers[slave][index]     = temp1;
	slaveTempBuffers[slave][index + 1] = temp2;

	*slaveLastMessageTicks[slave] = HAL_GetTick();
}

/**
 * @brief  Processa uma mensagem CAN1 recebida (desempacotada da Queue).
 */
static void dispatchRxMessage(CAN_RxMsg_t *msg)
{
	uint32_t id = msg->header.Identifier;

	/* Verifica se é um quadro de erro de termistor */
	if (id == idSlave1ThermistorError ||
		id == idSlave2ThermistorError ||
		id == idSlave3ThermistorError ||
		id == idSlave4ThermistorError)
	{
		tmsErrorCode |= thermistorConnectionFault;
		osDelay(5);
//		Error_Handler();
		return;
	}

	/* Identifica qual Slave e qual burst baseado no ID */
	for (uint8_t slave = 0; slave < numberOfSlaves; slave++)
	{
		uint32_t base = slaveBurstBaseId[slave];

		if (id >= base && id < base + 8)
		{
			uint8_t burst = id - base;
			processSlaveBurst(msg->data, slave, burst);
			return;
		}
	}

	/* ID desconhecido (ex: loopback 0x00A): ignorar silenciosamente */
}

/**
 * @brief  Drena a fila de recepção CAN1 e processa cada mensagem.
 */
void processRxQueue(void)
{
	CAN_RxMsg_t rxMsg;
	extern CAN_RxMsg_t lastRxMsg;

	while (osMessageQueueGet(canRxQueueHandle, &rxMsg, NULL, 0) == osOK)
	{
		lastRxMsg = rxMsg;
		dispatchRxMessage(&rxMsg);
	}
}

#ifdef testLoopback
/**
 * @brief  Drena a fila de recepção CAN2 (apenas para debug em loopback).
 *
 * Não processa a mensagem — apenas atualiza lastRx2Msg para
 * visualização no Live Expressions.
 */
void processRx2Queue(void)
{
	CAN_RxMsg_t rxMsg;
	extern CAN_RxMsg_t lastRx2Msg;

	while (osMessageQueueGet(canRx2QueueHandle, &rxMsg, NULL, 0) == osOK)
	{
		lastRx2Msg = rxMsg;
	}
}
#endif

/* ==================== Funções de transmissão TX ======================= */

/**
 * @brief  Envia um único quadro CAN pelo FDCAN2 com retry e detecção de falha.
 *
 * @param  identifier: ID CAN do quadro (29 bits, Extended ID)
 * @param  data:       Ponteiro para os 8 bytes de payload a enviar
 * @retval CAN_TX_OK / CAN_TX_FAIL / CAN_TX_FATAL
 */
static CAN_TxStatus_t sendSingleFrame(uint32_t identifier, uint8_t *data)
{
	if (hfdcan2.State != HAL_FDCAN_STATE_BUSY)
	{
		canConsecutiveFailures++;
		tmsErrorCode |= masterCANFault;
		if (canConsecutiveFailures >= CAN_TX_FAULT_THRESHOLD)
			return CAN_TX_FATAL;
		return CAN_TX_FAIL;
	}

	FDCAN2TxHeader.Identifier = identifier;
	FDCAN2TxHeader.DataLength = FDCAN_DLC_BYTES_8;

	uint8_t retry = 0;
	while (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &FDCAN2TxHeader, data) != HAL_OK)
	{
		if (osKernelGetState() == osKernelRunning)
			osDelay(1);
		else
			HAL_Delay(1);

		if (++retry >= CAN_TX_RETRY_MAX)
		{
			canConsecutiveFailures++;
			tmsErrorCode |= masterCANFault;
			if (canConsecutiveFailures >= CAN_TX_FAULT_THRESHOLD)
				return CAN_TX_FATAL;
			return CAN_TX_FAIL;
		}
	}

	canConsecutiveFailures = 0;
	return CAN_TX_OK;
}

#ifdef testLoopback
/**
 * @brief  Envia um único quadro CAN pelo FDCAN1 (loopback de teste).
 *
 * Mesma lógica de retry da sendSingleFrame mas usando FDCAN1 e Standard ID.
 */
static CAN_TxStatus_t sendSingleFrameCAN1(uint32_t identifier, uint8_t *data)
{
	if (hfdcan1.State != HAL_FDCAN_STATE_BUSY)
	{
		canConsecutiveFailures++;
		tmsErrorCode |= masterCANFault;
		if (canConsecutiveFailures >= CAN_TX_FAULT_THRESHOLD)
			return CAN_TX_FATAL;
		return CAN_TX_FAIL;
	}

	FDCAN1TxHeader.Identifier = identifier;
	FDCAN1TxHeader.DataLength = FDCAN_DLC_BYTES_8;

	uint8_t retry = 0;
	while (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN1TxHeader, data) != HAL_OK)
	{
		if (osKernelGetState() == osKernelRunning)
			osDelay(1);
		else
			HAL_Delay(1);

		if (++retry >= CAN_TX_RETRY_MAX)
		{
			canConsecutiveFailures++;
			tmsErrorCode |= masterCANFault;
			if (canConsecutiveFailures >= CAN_TX_FAULT_THRESHOLD)
				return CAN_TX_FATAL;
			return CAN_TX_FAIL;
		}
	}

	canConsecutiveFailures = 0;
	return CAN_TX_OK;
}
#endif

/**
 * @brief  Envia o resumo do TMS para o barramento geral do carro via FDCAN2.
 *
 * Payload (8 bytes):
 *   Byte 0: Código de erro do TMS (bitmask)
 *   Byte 1–4: Temperatura máxima de cada Slave
 *   Bytes 5–7: Reservados (0x00)
 */
CAN_TxStatus_t sendMasterInfoToCAN(int temp1, int temp2, int temp3, int temp4, int error)
{
	memset(FDCAN2TxData, 0, sizeof(FDCAN2TxData));
	FDCAN2TxData[0] = error;
	FDCAN2TxData[1] = temp1;
	FDCAN2TxData[2] = temp2;
	FDCAN2TxData[3] = temp3;
	FDCAN2TxData[4] = temp4;

	CAN_TxStatus_t result = sendSingleFrame(idMasterCAN2, FDCAN2TxData);
	/* Comentado durante o teste de loopback para evitar hangs */
	/* if (result == CAN_TX_FATAL) Error_Handler(); */
	return result;
}

#ifdef testLoopback
/**
 * @brief  Envia um quadro de teste pelo CAN1 para validar loopback.
 *
 * Preenche o payload com o valor 67 (0x43) e envia usando o ID de teste.
 * No Live Expressions, se o loopback estiver funcionando, lastRxMsg deve
 * mostrar ID = 0x00A e data = {67, 67, 67, 67, 67, 67, 67, 67}.
 */
CAN_TxStatus_t sendCAN1Loopback(void)
{
	memset(FDCAN1TxData, 67, sizeof(FDCAN1TxData));

	CAN_TxStatus_t result = sendSingleFrameCAN1(idTestCAN1, FDCAN1TxData);
	/* Comentado durante o teste de loopback para evitar hangs */
	/* if (result == CAN_TX_FATAL) Error_Handler(); */
	return result;
}
#endif
