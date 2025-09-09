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

extern double slave1LastMessageTick;

int findMaxVal(float buffer[]){
	int len = sizeof(buffer)/sizeof(buffer[0]);
	int maxVal = buffer[0];
	for(int i = 0; i < len; i++){
		if(buffer[i] > buffer[0]){
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
		slave1LastMessageTick = 10000000;
	}
}

