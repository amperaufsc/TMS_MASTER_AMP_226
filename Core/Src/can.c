/*
 * can.c
 *
 *  Created on: Sep 7, 2025
 *      Author: Guilherme Lettmann
 */

#include "can.h"
#include "string.h"

extern FDCAN_HandleTypeDef hfdcan1;
uint8_t FDCAN1RxData[8];
FDCAN_RxHeaderTypeDef FDCAN1RxHeader;
uint8_t FDCAN1TxData[8];
FDCAN_RxHeaderTypeDef FDCAN1TxHeader;

extern FDCAN_HandleTypeDef hfdcan2;
uint8_t FDCAN2TxData[8];
FDCAN_TxHeaderTypeDef FDCAN2TxHeader;
uint8_t FDCAN2RxData[8];
FDCAN_RxHeaderTypeDef FDCAN2RxHeader;

volatile int tmsErrorCode = 0;

float slaveTempBuffers[numberOfSlaves][thermistorsRecieved] = {0};
uint32_t slaveLastMessageTicks[numberOfSlaves] = {0};

extern CAN_RxMsg_t lastRx1Msg;
extern CAN_RxMsg_t lastRx2Msg;

static uint8_t canConsecutiveFailures = 0;

uint32_t slaveBurstBaseId[4] = {
		idSlave1Burst0,
		idSlave2Burst0,
		idSlave3Burst0,
		idSlave4Burst0
};

uint32_t slaveErrorIds[4] = {
		idSlave1ThermistorError,
		idSlave2ThermistorError,
		idSlave3ThermistorError,
		idSlave4ThermistorError
};

static CAN_TxStatus_t sendSingleFrame(FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *pHeader, uint8_t *pData){
	if (hfdcan->State != HAL_FDCAN_STATE_BUSY)
	{
		canConsecutiveFailures++;
		if (canConsecutiveFailures >= CAN_TX_FAULT_THRESHOLD) {
			return CAN_TX_FATAL;
		}
		return CAN_TX_FAIL;
	}

	uint8_t retry = 0;
	while (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, pHeader, pData) != HAL_OK)
	{
		osDelay(1);
		if (++retry >= CAN_TX_RETRY_MAX)
		{
			canConsecutiveFailures++;
			if (canConsecutiveFailures >= CAN_TX_FAULT_THRESHOLD) {
				return CAN_TX_FATAL;
			}
			return CAN_TX_FAIL;
		}
	}

	canConsecutiveFailures = 0;
	return CAN_TX_OK;
}

void processSlaveBurst(uint8_t slave, uint8_t burst){
	float *payload = (float*)FDCAN1RxData;
	float temp1 = payload[0];
	float temp2 = payload[1];

	uint8_t index = burst * 2;

	slaveTempBuffers[slave][index]     = temp1;
	slaveTempBuffers[slave][index + 1] = temp2;

	slaveLastMessageTicks[slave] = HAL_GetTick();
}

void receiveCANFromSlaves(){
	lastRx1Msg.header = FDCAN1RxHeader;
	memcpy(lastRx1Msg.data, FDCAN1RxData, 8);

	uint32_t id = FDCAN1RxHeader.Identifier;

	for (uint8_t i = 0; i < numberOfSlaves; i++) {
		if (id == slaveErrorIds[i]) {
			tmsErrorCode |= thermistorConnectionFault;
			return;
		}
	}

	for (uint8_t slave = 0; slave < numberOfSlaves; slave++) {
		uint32_t base = slaveBurstBaseId[slave];
		if (id >= base && id < base + 8) {
			uint8_t burst = id - base;
			processSlaveBurst(slave, burst);
		}
	}
}

void receiveCANFromGeral(){
	lastRx2Msg.header = FDCAN2RxHeader;
	memcpy(lastRx2Msg.data, FDCAN2RxData, 8);
}

void sendMasterInfoToCAN(float *slaveMaxTemps, int error){
	FDCAN2TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	FDCAN2TxHeader.Identifier = idMasterCAN2;
	FDCAN2TxHeader.IdType = FDCAN_EXTENDED_ID;

	FDCAN2TxData[0] = error;

	for(int i = 0; i < 4 && i < numberOfSlaves; i++){
		FDCAN2TxData[i+1] = (uint8_t)slaveMaxTemps[i];
	}

	CAN_TxStatus_t result = sendSingleFrame(&hfdcan2, &FDCAN2TxHeader, FDCAN2TxData);
	if(result == CAN_TX_FATAL){
		tmsErrorCode |= masterCANFault;
		Error_Handler();
	}
}

void sendToSlavesCAN(uint32_t id, uint8_t *data, uint32_t len){
	FDCAN1TxHeader.Identifier = id;
	FDCAN1TxHeader.IdType = FDCAN_STANDARD_ID;
	
	// Mapeamento correto de DLC para FDCAN
	if (len == 0) FDCAN1TxHeader.DataLength = FDCAN_DLC_BYTES_0;
	else if (len <= 8) FDCAN1TxHeader.DataLength = len; // Para 1-8, os valores 1-8 coincidem com as macros
	else FDCAN1TxHeader.DataLength = FDCAN_DLC_BYTES_8;

	memcpy(FDCAN1TxData, data, (len > 8) ? 8 : len);

	CAN_TxStatus_t result = sendSingleFrame(&hfdcan1, &FDCAN1TxHeader, FDCAN1TxData);
	if(result == CAN_TX_FATAL){
		tmsErrorCode |= masterCANFault;
		Error_Handler();
	}
}

void simulateSlaveBurst(uint8_t slaveIdx, uint8_t burstIdx){
	if (slaveIdx >= numberOfSlaves) return;

	uint32_t simId = slaveBurstBaseId[slaveIdx] + burstIdx;
	float simData[2] = {50.0f, 50.0f};

	sendToSlavesCAN(simId, (uint8_t*)simData, 8);
}

