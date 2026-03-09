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

#define numberOfSlaves 4
#define idMaster 0x00A

#define idSlave1Burst0 0x010

#define idSlave2Burst0 0x020

#define idSlave3Burst0 0x030

#define idSlave4Burst0 0x040

#define idSlave1ThermistorError 0x050
#define idSlave2ThermistorError 0x051
#define idSlave3ThermistorError 0x052
#define idSlave4ThermistorError 0x053

void processSlaveBurst(uint8_t slave, uint8_t burst);
void receiveCANFromSlaves();
void sendMasterInfoToCAN(int temp1, int temp2, int temp3, int temp4, int error);

#endif /* INC_CAN_H_ */
