/*
 * adc.h
 *
 *  Created on: Sep 3, 2025
 *      Author: Guilherme Lettmann
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "main.h"
#include "cmsis_os.h"

#define numberOfThermistors 1
#define windowSize 10
#define adcResolution 4095
#define VREF 3.3
/*T(V) - coefficients*/
#define C4 (6.03)
#define C3 (-40.82)
#define C2 (105.64)
#define C1 (-156.45)
#define C0 (134.25)

float convertBitsToVoltage(uint16_t rawAdcVal);
float convertVoltageToTemperature(float voltage);
void initializeHistory();
void applyMovingAverageFilter(float rawReadings[]);

#endif /* INC_ADC_H_ */
