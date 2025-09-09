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

extern double slave1LastMessageTick, slave2LastMessageTick, slave3LastMessageTick, slave4LastMessageTick;


float slave1TempBuffer[16] = {0}, slave2TempBuffer[16] = {0}, slave3TempBuffer[16] = {0}, slave4TempBuffer[16] = {0};

void receiveCANFromSlaves(){
	switch(FDCAN1RxHeader.Identifier)
	{
	case idSlave1Burst0:
		memcpy(&slave1TempBuffer[0], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave1TempBuffer[1], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave1LastMessageTick = HAL_GetTick();
		break;

	case idSlave1Burst1:
		memcpy(&slave1TempBuffer[2], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave1TempBuffer[3], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave1LastMessageTick = HAL_GetTick();
		break;

	case idSlave1Burst2:
		memcpy(&slave1TempBuffer[4], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave1TempBuffer[5], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave1LastMessageTick = HAL_GetTick();
		break;

	case idSlave1Burst3:
		memcpy(&slave1TempBuffer[6], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave1TempBuffer[7], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave1LastMessageTick = HAL_GetTick();
		break;

	case idSlave1Burst4:
		memcpy(&slave1TempBuffer[8], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave1TempBuffer[9], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave1LastMessageTick = HAL_GetTick();
		break;

	case idSlave1Burst5:
		memcpy(&slave1TempBuffer[10], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave1TempBuffer[11], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave1LastMessageTick = HAL_GetTick();
		break;

	case idSlave1Burst6:
		memcpy(&slave1TempBuffer[12], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave1TempBuffer[13], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave1LastMessageTick = HAL_GetTick();
		break;

	case idSlave1Burst7:
		memcpy(&slave1TempBuffer[14], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave1TempBuffer[15], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave1LastMessageTick = HAL_GetTick();
		break;

	case idSlave2Burst0:
		memcpy(&slave2TempBuffer[0], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave2TempBuffer[1], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave2LastMessageTick = HAL_GetTick();
		break;

	case idSlave2Burst1:
		memcpy(&slave2TempBuffer[2], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave2TempBuffer[3], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave2LastMessageTick = HAL_GetTick();
		break;

	case idSlave2Burst2:
		memcpy(&slave2TempBuffer[4], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave2TempBuffer[5], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave2LastMessageTick = HAL_GetTick();
		break;

	case idSlave2Burst3:
		memcpy(&slave2TempBuffer[6], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave2TempBuffer[7], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave2LastMessageTick = HAL_GetTick();
		break;

	case idSlave2Burst4:
		memcpy(&slave2TempBuffer[8], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave2TempBuffer[9], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave2LastMessageTick = HAL_GetTick();
		break;

	case idSlave2Burst5:
		memcpy(&slave2TempBuffer[10], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave2TempBuffer[11], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave2LastMessageTick = HAL_GetTick();
		break;

	case idSlave2Burst6:
		memcpy(&slave2TempBuffer[12], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave2TempBuffer[13], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave2LastMessageTick = HAL_GetTick();
		break;

	case idSlave2Burst7:
		memcpy(&slave2TempBuffer[14], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave2TempBuffer[15], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave2LastMessageTick = HAL_GetTick();
		break;

	case idSlave3Burst0:
		memcpy(&slave3TempBuffer[0], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave3TempBuffer[1], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave3LastMessageTick = HAL_GetTick();
		break;

	case idSlave3Burst1:
		memcpy(&slave3TempBuffer[2], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave3TempBuffer[3], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave3LastMessageTick = HAL_GetTick();
		break;

	case idSlave3Burst2:
		memcpy(&slave3TempBuffer[4], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave3TempBuffer[5], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave3LastMessageTick = HAL_GetTick();
		break;

	case idSlave3Burst3:
		memcpy(&slave3TempBuffer[6], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave3TempBuffer[7], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave3LastMessageTick = HAL_GetTick();
		break;

	case idSlave3Burst4:
		memcpy(&slave3TempBuffer[8], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave3TempBuffer[9], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave3LastMessageTick = HAL_GetTick();
		break;

	case idSlave3Burst5:
		memcpy(&slave3TempBuffer[10], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave3TempBuffer[11], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave3LastMessageTick = HAL_GetTick();
		break;

	case idSlave3Burst6:
		memcpy(&slave3TempBuffer[12], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave3TempBuffer[13], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave3LastMessageTick = HAL_GetTick();
		break;

	case idSlave3Burst7:
		memcpy(&slave3TempBuffer[14], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave3TempBuffer[15], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave3LastMessageTick = HAL_GetTick();
		break;

	case idSlave4Burst0:
		memcpy(&slave4TempBuffer[0], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave4TempBuffer[1], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave4LastMessageTick = HAL_GetTick();
		break;

	case idSlave4Burst1:
		memcpy(&slave4TempBuffer[2], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave4TempBuffer[3], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave4LastMessageTick = HAL_GetTick();
		break;

	case idSlave4Burst2:
		memcpy(&slave4TempBuffer[4], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave4TempBuffer[5], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave4LastMessageTick = HAL_GetTick();
		break;

	case idSlave4Burst3:
		memcpy(&slave4TempBuffer[6], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave4TempBuffer[7], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave4LastMessageTick = HAL_GetTick();
		break;

	case idSlave4Burst4:
		memcpy(&slave4TempBuffer[8], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave4TempBuffer[9], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave4LastMessageTick = HAL_GetTick();
		break;

	case idSlave4Burst5:
		memcpy(&slave4TempBuffer[10], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave4TempBuffer[11], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave4LastMessageTick = HAL_GetTick();
		break;

	case idSlave4Burst6:
		memcpy(&slave4TempBuffer[12], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave4TempBuffer[13], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave4LastMessageTick = HAL_GetTick();
		break;

	case idSlave4Burst7:
		memcpy(&slave4TempBuffer[14], &FDCAN1RxData[0], sizeof(float));
		memcpy(&slave4TempBuffer[15], &FDCAN1RxData[4], sizeof(float));

		if(FDCAN1RxData[0] >= maxTemperatureThreshold || FDCAN1RxData[4] >= maxTemperatureThreshold)
		{
			tmsErrorCode = overTemperatureFault;
			Error_Handler();
		}
		slave4LastMessageTick = HAL_GetTick();
		break;

	}
}

void sendMasterInfoToCAN(int temp1, int temp2, int temp3, int temp4, int error){
	FDCAN2TxHeader.DataLength = 8;

	FDCAN2TxHeader.Identifier = idMaster;
	FDCAN2TxData[0] = error;
	FDCAN2TxData[1] = temp1;
	FDCAN2TxData[2] = temp2;
	FDCAN2TxData[3] = temp3;
	FDCAN2TxData[4] = temp4;

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &FDCAN2TxHeader, FDCAN2TxData);

}

