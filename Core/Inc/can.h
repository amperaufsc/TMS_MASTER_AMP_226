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

#define idMaster 0x00A

#define idSlave1Burst0 0x010
#define idSlave1Burst1 0x011
#define idSlave1Burst2 0x012
#define idSlave1Burst3 0x013
#define idSlave1Burst4 0x014
#define idSlave1Burst5 0x015
#define idSlave1Burst6 0x016
#define idSlave1Burst7 0x017

#define idSlave2Burst0 0x020
#define idSlave2Burst1 0x021
#define idSlave2Burst2 0x022
#define idSlave2Burst3 0x023
#define idSlave2Burst4 0x024
#define idSlave2Burst5 0x025
#define idSlave2Burst6 0x026
#define idSlave2Burst7 0x027

#define idSlave3Burst0 0x030
#define idSlave3Burst1 0x031
#define idSlave3Burst2 0x032
#define idSlave3Burst3 0x033
#define idSlave3Burst4 0x034
#define idSlave3Burst5 0x035
#define idSlave3Burst6 0x036
#define idSlave3Burst7 0x037

#define idSlave4Burst0 0x040
#define idSlave4Burst1 0x041
#define idSlave4Burst2 0x042
#define idSlave4Burst3 0x043
#define idSlave4Burst4 0x044
#define idSlave4Burst5 0x045
#define idSlave4Burst6 0x046
#define idSlave4Burst7 0x047

void receiveCANFromSlaves();
void sendMasterInfoToCAN(int temp1, int temp2, int temp3, int temp4, int error);

#endif /* INC_CAN_H_ */
