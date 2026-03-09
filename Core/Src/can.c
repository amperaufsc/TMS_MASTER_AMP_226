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

extern FDCAN_HandleTypeDef hfdcan2;
uint8_t FDCAN2TxData[8];
FDCAN_TxHeaderTypeDef FDCAN2TxHeader;

int tmsErrorCode = 0;

float slave1TempBuffer[thermistorsRecieved] = {0}, slave2TempBuffer[thermistorsRecieved] = {0},
	  slave3TempBuffer[thermistorsRecieved] = {0}, slave4TempBuffer[thermistorsRecieved] = {0};

uint32_t slave1LastMessageTick = 0, slave2LastMessageTick = 0, slave3LastMessageTick = 0, slave4LastMessageTick = 0;

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

void processSlaveBurst(uint8_t slave, uint8_t burst){
	float *payload = (float*)FDCAN1RxData;
	float temp1 = payload[0];
	float temp2 = payload[1];

	uint8_t index = burst * 2;

	slaveTempBuffers[slave][index]     = temp1;
	slaveTempBuffers[slave][index + 1] = temp2;

	*slaveLastMessageTicks[slave] = HAL_GetTick();
}

void receiveCANFromSlaves(){
	uint32_t id = FDCAN1RxHeader.Identifier;

	if(id == idSlave1ThermistorError ||
			id == idSlave2ThermistorError ||
			id == idSlave3ThermistorError ||
			id == idSlave4ThermistorError)
	{
		tmsErrorCode = thermistorConnectionFault;
		Error_Handler();
		return;
	}

	for(uint8_t slave = 0; slave < numberOfSlaves; slave++){
		uint32_t base = slaveBurstBaseId[slave];

		if(id >= base && id < base + 8){
			uint8_t burst = id - base;
			processSlaveBurst(slave, burst);
			return;
		}
	}
}

void sendMasterInfoToCAN(int temp1, int temp2, int temp3, int temp4, int error){
	FDCAN2TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	FDCAN2TxHeader.Identifier = idMaster;

	FDCAN2TxData[0] = error;
	FDCAN2TxData[1] = temp1;
	FDCAN2TxData[2] = temp2;
	FDCAN2TxData[3] = temp3;
	FDCAN2TxData[4] = temp4;

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &FDCAN2TxHeader, FDCAN2TxData);

}

