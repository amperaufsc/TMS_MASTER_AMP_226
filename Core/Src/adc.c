/*
 * adc.c
 *
 *  Created on: Sep 3, 2025
 *      Author: Guilherme Lettmann
 */
#include "math.h"
#include "stdlib.h"
#include "adc.h"
#include "stdio.h"

float readingsHistory [numberOfThermistors][windowSize], filteredReadings[numberOfThermistors];
int circularIndex [numberOfThermistors] = {0}, validReadingsCount[numberOfThermistors] = {0};

float convertBitsToVoltage(uint16_t rawAdcVal){
	return (rawAdcVal*VREF)/adcResolution;
}

float convertVoltageToTemperature(float voltage){
	return C0 + C1 * voltage + C2 * pow(voltage, 2) + C3 * pow(voltage, 3) + C4 *pow(voltage, 4);
}

void initializeHistory(){
	for (int i = 0; i < numberOfThermistors; i++){
		for(int j = 0; j < windowSize; j++){
			readingsHistory[i][j] = 0;
		}
		filteredReadings[i] = 0;
		validReadingsCount[i] = 0;
	}
}

void applyMovingAverageFilter(float rawReadings[]){
	for (int i = 0; i < numberOfThermistors; i++) {

		runningSum[i] -= readingsHistory[i][circularIndex[i]];

		readingsHistory[i][circularIndex[i]] = rawReadings[i];
		runningSum[i] += rawReadings[i];


		circularIndex[i] = (circularIndex[i] + 1) % windowSize;

		if (validReadingsCount[i] < windowSize) {
			validReadingsCount[i]++;
		}

		filteredReadings[i] = runningSum[i] / validReadingsCount[i];
	}

}
