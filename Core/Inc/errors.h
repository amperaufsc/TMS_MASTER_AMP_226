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

#define maxTemperatureThreshold 60

#define thermistorsRecieved 16

int findMaxVal(const float *buffer);
void injectFault(int temp);

#endif /* INC_ERRORS_H_ */
