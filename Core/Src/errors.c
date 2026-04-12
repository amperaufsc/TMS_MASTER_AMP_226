/*
 * errors.c
 *
 *  Created on: Sep 7, 2025
 *      Author: Guilherme Lettmann
 */

#include "errors.h"
#include "stdbool.h"
#include "stdint.h"

bool simulateHighTemp = false;
bool simulateCommLoss = false;

extern uint32_t slave1LastMessageTick;

float findMaxVal(const float *buffer) {
	float maxVal = buffer[0];
	for (size_t i = 1; i < thermistorsRecieved; ++i) {
		if (buffer[i] > maxVal) {
			maxVal = buffer[i];
		}
	}

	return maxVal;
}

void injectFault(int temp){
	if(simulateHighTemp){
		temp = 100;
	}
	else if(simulateCommLoss){
		slave1LastMessageTick = 0;
	}
}

