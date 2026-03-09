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

uint16_t filteredAdcBuffer[numberOfThermistors] = {0}, medianBuffer[numberOfThermistors][3] = {0};
uint8_t medianIndex[numberOfThermistors];

uint16_t median3(uint16_t a, uint16_t b, uint16_t c)
{
	if(a > b){
		uint16_t t = a;
		a = b;
		b = t;
	}

	if(b > c){
		uint16_t t = b;
		b = c;
		c = t;
	}

	if(a > b){
		uint16_t t = a;
		a = b;
		b = t;
	}

	return b;
}

uint16_t applyMedianFilter(uint16_t newReading, uint8_t index)
{
	medianBuffer[index][medianIndex[index]] = newReading;

	uint16_t a = medianBuffer[index][0];
	uint16_t b = medianBuffer[index][1];
	uint16_t c = medianBuffer[index][2];

	medianIndex[index]++;
	if(medianIndex[index] >= 3)
		medianIndex[index] = 0;

	return median3(a,b,c);
}

uint16_t applyIIRFilter(uint16_t newReading, uint8_t index){
	filteredAdcBuffer[index] += (newReading - filteredAdcBuffer[index]) >> 3; // alpha = 1/8 log(2)8 = 3
	return filteredAdcBuffer[index];
}

float convertBitsToVoltage(uint16_t rawAdcVal){
	return (rawAdcVal*vcc)/adcResolution;
}

float convertVoltageToTemperature(float voltage){
	return C0 + C1 * voltage + C2 * pow(voltage, 2) + C3 * pow(voltage, 3) + C4 *pow(voltage, 4);
}

thermStatus checkThermistorConnection(uint16_t rawAdcVal){
	if (rawAdcVal <= shortCircuitThreshold){
		return THERM_SHORT;
	}
	if (rawAdcVal >= openCircuitThreshhold){
		return THERM_OPEN;
	}
	return OK;
}

void initTemperatureFilters(uint16_t rawAdcBuffer[])
{
    for(int i = 0; i < numberOfThermistors; i++)
    {
        filteredAdcBuffer[i] = rawAdcBuffer[i];

        for(int j = 0; j < 3; j++)
        {
            medianBuffer[i][j] = rawAdcBuffer[i];
        }

        medianIndex[i] = 0;
    }
}
