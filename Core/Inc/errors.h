/*
 * errors.h
 *
 *  Created on: Sep 7, 2025
 *      Author: Guilherme Lettmann
 */

#ifndef INC_ERRORS_H_
#define INC_ERRORS_H_

#include "can.h"
#include "main.h"

#define commFault 1<<0
#define overTemperatureFault 1<<1
#define thermistorConnectionFault 1<<2
#define masterCANFault 1<<3
#define CANSplitterCANFault 1<<4
#define overTemperatureFaultEst 1<<5

#define maxTemperatureThresholdNTC 55
#define maxTemperatureThresholdEst 60

#define thermistorsPerSlave 0
#define thermistorsRecieved thermistorsPerSlave

int findMaxVal(const float *buffer);
void injectFault(float *temp);

#endif /* INC_ERRORS_H_ */
