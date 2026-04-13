/*
 * can.h
 *
 *  Created on: Sep 7, 2025
 *      Author: Guilherme Lettmann
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "errors.h"
#include "main.h"
#include "adc.h"

//#define testLoopbackCAN1
//#define testLoopbackCAN2


#define numberOfSlaves 1

#define idMasterCAN1 0x00A
#define idMasterCAN2 0x19308082

#define idSlave1Burst0 0x010

#define idSlave2Burst0 0x020

#define idSlave3Burst0 0x030

#define idSlave4Burst0 0x040

#define idSlave1ThermistorError 0x050
#define idSlave2ThermistorError 0x051
#define idSlave3ThermistorError 0x052
#define idSlave4ThermistorError 0x053

/* ================ Parâmetros de robustez do envio CAN ================ */
#define CAN_TX_RETRY_MAX      20   // Tentativas por quadro antes de desistir (≈20ms)
#define CAN_TX_FAULT_THRESHOLD 10  // Falhas consecutivas antes de acionar Shutdown (SDC)

/* ============= Status de retorno das funções de transmissão ========== */
typedef enum {
	CAN_TX_OK = 0,       // Todos os quadros enviados com sucesso
	CAN_TX_FAIL,         // Falha pontual (quadro descartado, sistema continua operando)
	CAN_TX_FATAL         // Falha crítica: rede CAN inoperante, Shutdown será acionado
} CAN_TxStatus_t;

void processSlaveBurst(uint8_t slave, uint8_t burst);
void receiveCANFromSlaves();
void receiveCANFromGeral();
void sendMasterInfoToCAN(float *slaveMaxTemps, int error);
void sendToSlavesCAN(uint32_t id, uint8_t *data, uint32_t len);
void simulateSlaveBurst(uint8_t slaveIdx, uint8_t burstIdx);

/* ============= Struct para fila de recepção CAN (Queue) ============== */
/* Encapsula um quadro CAN completo (header + 8 bytes de dados).
 * Usada pela ISR (HAL_FDCAN_RxFifo0Callback) para empurrar mensagens
 * recebidas para a fila do FreeRTOS sem usar variáveis globais. */
typedef struct {
	FDCAN_RxHeaderTypeDef header;   // Header com ID, DLC, timestamp, etc.
	uint8_t data[8];                // Payload de dados (até 8 bytes no CAN clássico)
} CAN_RxMsg_t;

#endif /* INC_CAN_H_ */
