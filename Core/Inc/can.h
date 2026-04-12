/*
 * can.h — Interface do módulo de comunicação CAN da Master
 *
 *  Created on: Sep 7, 2025
 *      Author: Guilherme Lettmann
 *
 *  A Master possui dois barramentos CAN:
 *    FDCAN1 (Standard ID) → Recebe temperaturas das 4 Slaves
 *    FDCAN2 (Extended ID) → Envia resumo do TMS para o barramento geral do carro
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "main.h"
#include "cmsis_os.h"

/* =================== Flag de modo de teste =========================== */
/* Quando definido, habilita o envio e recepção em ambos os CANs para
 * validação em modo Internal Loopback. Remover para operação no carro. */
#define testLoopback

/* ======================== Constantes gerais =========================== */
#define numberOfSlaves 4

/* ================ Capacidade da fila de recepção CAN ================= */
#define CAN_RX_QUEUE_SIZE 32   // 4 Slaves × 8 quadros = 32 quadros por ciclo (pior caso)

/* ================ Parâmetros de robustez do envio CAN ================ */
#define CAN_TX_RETRY_MAX       20   // Tentativas por quadro antes de desistir (≈20ms)
#define CAN_TX_FAULT_THRESHOLD 10   // Falhas consecutivas antes de acionar Shutdown

/* ============= Status de retorno das funções de transmissão ========== */
typedef enum {
	CAN_TX_OK = 0,       // Quadro enviado com sucesso
	CAN_TX_FAIL,         // Falha pontual (quadro descartado, sistema segue)
	CAN_TX_FATAL         // Falha crítica: rede CAN inoperante
} CAN_TxStatus_t;

/* ==================== Mapa de IDs CAN ================================ */

/* ID da Master no barramento geral (CAN2 — Extended ID, 29 bits) */
#define idMasterCAN2 0x19308082

/* ID de teste para loopback no CAN1 (Standard ID, 11 bits) */
#define idTestCAN1   0x00A

/* IDs base dos bursts de temperatura de cada Slave (CAN1 — Standard ID).
 * Cada Slave envia 8 quadros sequenciais (2 floats por quadro = 16 termistores).
 * Ex: Slave 1 usa IDs 0x010, 0x011, 0x012, ..., 0x017 */
#define idSlave1Burst0 0x010
#define idSlave2Burst0 0x020
#define idSlave3Burst0 0x030
#define idSlave4Burst0 0x040

/* IDs de erro de leitura de termistor (curto-circuito ou circuito aberto) */
#define idSlave1ThermistorError 0x050
#define idSlave2ThermistorError 0x051
#define idSlave3ThermistorError 0x052
#define idSlave4ThermistorError 0x053

/* ============= Structs para debug e enfileiramento =================== */

/* Struct RX: encapsula um quadro CAN recebido (header + dados) */
typedef struct {
	FDCAN_RxHeaderTypeDef header;
	uint8_t data[8];
} CAN_RxMsg_t;

/* ====================== Protótipos de funções ======================== */
void processRxQueue(void);
CAN_TxStatus_t sendMasterInfoToCAN(int temp1, int temp2, int temp3, int temp4, int error);

#ifdef testLoopback
void processRx2Queue(void);
CAN_TxStatus_t sendCAN1Loopback(void);
extern osMessageQueueId_t canRx2QueueHandle;   // Fila RX do CAN2 (loopback)
#endif

/* ====================== Handle da fila RX ============================ */
extern osMessageQueueId_t canRxQueueHandle;   // Fila RX do CAN1 (Slaves → Master)

#endif /* INC_CAN_H_ */
