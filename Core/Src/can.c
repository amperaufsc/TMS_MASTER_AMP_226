/**
 * @file can.c
 * @brief Logic for CAN transmission, reception, and safety monitoring.
 *
 * This module manages:
 * - Direct processing of CAN messages within ISR callbacks (low latency).
 * - Robust transmission mechanism with retry and safety-threshold tracking.
 * - Buffer management for slave temperature data.
 */

#include "can.h"
#include <string.h>
#include <stdbool.h>

/* ==================== Extern Globals from main.c ===================== */
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

extern uint8_t FDCAN1RxData[8];
extern FDCAN_RxHeaderTypeDef FDCAN1RxHeader;
extern uint8_t FDCAN1TxData[8];
extern FDCAN_TxHeaderTypeDef FDCAN1TxHeader;

extern uint8_t FDCAN2TxData[8];
extern FDCAN_TxHeaderTypeDef FDCAN2TxHeader;
extern uint8_t FDCAN2RxData[8];
extern FDCAN_RxHeaderTypeDef FDCAN2RxHeader;

extern float bmsVoltage, bmsCurrent;
extern float cellMaxVoltage, cellMinVoltage, cellAvgVoltage;
extern uint32_t bmsProtectionFlags;
extern bool bmsSafetyFlag;
extern int bmsCANError, chargerSOC;

/** @brief Composite error bitfield reflecting system state */
volatile int tmsErrorCode = 0;

/** @brief Global buffer storing the latest temperatures from all slaves */
float slaveTempBuffers[numberOfSlaves][thermistorsRecieved] = {0};

/** @brief Timestamps of the last received message from each slave (for timeout detection) */
uint32_t slaveLastMessageTicks[numberOfSlaves] = {0};

/** @brief Latest raw messages for debugging and telemetry monitoring */
extern CAN_RxMsg_t lastRx1Msg;
extern CAN_RxMsg_t lastRx2Msg;

/** @brief Persistent counter for consecutive TX failures across any CAN channel */
static uint8_t canConsecutiveFailures = 0;

/* ==================== Lookup Tables ================================= */
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

/* ==================== Static Internal Functions ====================== */

/**
 * @brief Centralized function for single frame transmission with retry and safety monitoring.
 *
 * Flow:
 * 1. Checks if the peripheral state is operational.
 * 2. Attempts to add the message to the TX FIFO.
 * 3. On failure (FIFO full), waits 1ms and retries up to CAN_TX_RETRY_MAX times.
 * 4. Tracks 'canConsecutiveFailures'. If the limit is reached, it returns FATAL.
 * 5. On any success, the failure counter is reset.
 *
 * @param hfdcan Peripheral handle (FDCAN1 or FDCAN2).
 * @param pHeader CAN header (Standard or Extended).
 * @param pData Data payload.
 * @return CAN_TxStatus_t OK, FAIL (one frame dropped), or FATAL (Safety Shutdown).
 */
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

/* ==================== Public Functions =============================== */

void processSlaveBurst(uint8_t slave, uint8_t burst){
	/* Cast RX buffer to float array (Slave sends 2 floats per 8-byte frame) */
	float *payload = (float*)FDCAN1RxData;
	float temp1 = payload[0];
	float temp2 = payload[1];

	uint8_t index = burst * 2;

	/* Update persistent temp buffer */
	slaveTempBuffers[slave][index]     = temp1;
	slaveTempBuffers[slave][index + 1] = temp2;

	/* Record timestamp for heartbeat/communication loss monitoring */
	slaveLastMessageTicks[slave] = HAL_GetTick();
}

void receiveCANFromSlaves(){
	/* Store latest raw message for monitoring */
	lastRx1Msg.header = FDCAN1RxHeader;
	memcpy(lastRx1Msg.data, FDCAN1RxData, 8);

	uint32_t id = FDCAN1RxHeader.Identifier;

	/* Check for Thermistor Disconnection faults sent by slaves */
	for (uint8_t i = 0; i < numberOfSlaves; i++) {
		if (id == slaveErrorIds[i]) {
			tmsErrorCode |= thermistorConnectionFault;
			return;
		}
	}

	/* Map sequential IDs to slave burst indices */
	for (uint8_t slave = 0; slave < numberOfSlaves; slave++) {
		uint32_t base = slaveBurstBaseId[slave];
		if (id >= base && id < base + 8) {
			uint8_t burst = id - base;
			processSlaveBurst(slave, burst);
		}
	}
}

void receiveCANFromGeral() {

	lastRx2Msg.header = FDCAN2RxHeader;
	memcpy(lastRx2Msg.data, FDCAN2RxData, 8);

	uint32_t id = FDCAN2RxHeader.Identifier;

	if (id == CANSplitterID3) {
		bmsCANError = FDCAN2RxData[0];

		if (bmsCANError != 0) {
			tmsErrorCode |= CANSplitterCANFault;
			Error_Handler();
		}
		return;
	}

	if (id == CANSplitterID1) {

		memcpy(&bmsVoltage, &FDCAN2RxData[0], 4);
		memcpy(&bmsCurrent, &FDCAN2RxData[4], 4);

	}
	else if (id == CANSplitterID2) {

		memcpy(&bmsProtectionFlags, &FDCAN2RxData[0], 4);

		cellMinVoltage = (float)FDCAN2RxData[4] / 50.0f;
		cellMaxVoltage = (float)FDCAN2RxData[5] / 50.0f;
		cellAvgVoltage = (float)FDCAN2RxData[6] / 50.0f;
		chargerSOC = FDCAN2RxData[7];

		bmsSafetyFlag = (bmsProtectionFlags != 0) ? true : false;
	}
}

void sendMasterInfoToCAN(float *slaveMaxTemps, int error){
	FDCAN2TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	FDCAN2TxHeader.Identifier = idMasterCAN2;
	FDCAN2TxHeader.IdType = FDCAN_EXTENDED_ID;

	FDCAN2TxData[0] = error;

	/* Pack max temperatures from the first 4 slaves */
	for(int i = 0; i < 4 && i < numberOfSlaves; i++){
		FDCAN2TxData[i+1] = (uint8_t)slaveMaxTemps[i];
	}

	/* Use robust TX mechanism; trigger shutdown on fatal network failure */
	CAN_TxStatus_t result = sendSingleFrame(&hfdcan2, &FDCAN2TxHeader, FDCAN2TxData);
	if(result == CAN_TX_FATAL){
		tmsErrorCode |= masterCANFault;
		Error_Handler(); 
	}
}

void sendToSlavesCAN(uint32_t id, uint8_t *data, uint32_t len){
	FDCAN1TxHeader.Identifier = id;
	FDCAN1TxHeader.IdType = FDCAN_STANDARD_ID;
	
	/* Correct mapping of byte length to FDCAN DLC codes */
	if (len == 0) FDCAN1TxHeader.DataLength = FDCAN_DLC_BYTES_0;
	else if (len <= 8) FDCAN1TxHeader.DataLength = len;
	else FDCAN1TxHeader.DataLength = FDCAN_DLC_BYTES_8;

	memcpy(FDCAN1TxData, data, (len > 8) ? 8 : len);

	/* Use robust TX mechanism; trigger shutdown on fatal network failure */
	CAN_TxStatus_t result = sendSingleFrame(&hfdcan1, &FDCAN1TxHeader, FDCAN1TxData);
	if(result == CAN_TX_FATAL){
		tmsErrorCode |= masterCANFault;
		Error_Handler();
	}
}

void simulateSlaveBurst(uint8_t slaveIdx, uint8_t burstIdx){
	/* Helper function to facilitate loopback testing by generating artificial packets */
	uint8_t fakeData[8] = {0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48}; // Example floats
	uint32_t simulatedId = slaveBurstBaseId[slaveIdx] + burstIdx;
	sendToSlavesCAN(simulatedId, fakeData, 8);
}
