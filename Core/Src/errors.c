/*
 * errors.c
 *
 *  Created on: Sep 7, 2025
 *      Author: Guilherme Lettmann
 */

#include "errors.h"
#include "stdbool.h"
#include "stdint.h"

/* Global flags for fault injection testing */
bool simulateHighTemp = false;
bool simulateCommLoss = false;

extern uint32_t slaveLastMessageTicks[numberOfSlaves];

/**
 * @brief Iterates through the temperature buffer to identify the maximum value.
 * @param buffer Pointer to the array of temperature readings.
 * @return The highest temperature found, cast to an integer.
 */
int findMaxVal(const float *buffer) {
	float maxVal = buffer[0];
	for (size_t i = 1; i < thermistorsRecieved; ++i) {
		if (buffer[i] > maxVal) {
			maxVal = buffer[i];
		}
	}

	return (int)maxVal;
}

void injectFault(float *temp){
	if(simulateHighTemp){
		*temp = 100.0f;
	}
	else if(simulateCommLoss){
		slaveLastMessageTicks[0] = 0;
	}
}

