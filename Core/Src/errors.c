/*
 * errors.c
 *
 *  Created on: Sep 7, 2025
 *      Author: Guilherme Lettmann
 */

#include "errors.h"
#include "stdbool.h"
#include "stdint.h"

/* Global flags for fault injection testing.
 * volatile: garante que a escrita feita pelo Live Expressions/debugger
 * seja sempre relida pelo firmware (senão o compilador pode cachear). */
volatile bool simulateHighTemp = false;

/* Máscara de simulação de perda de comunicação (bitmask, 1 bit por slave):
 *   bit0 (=1) -> Slave 1     bit2 (=4) -> Slave 3
 *   bit1 (=2) -> Slave 2     bit3 (=8) -> Slave 4
 * Ex.: simulateCommLoss = 1 mata o Slave 1; = 5 mata Slave 1 e 3;
 *      = 15 mata todos; = 0 volta ao normal.
 * Os quadros do slave marcado sao DESCARTADOS no RX (can.c), como se o cabo
 * tivesse caido -> o timeout de 2s levanta commFault sozinho. */
volatile uint8_t simulateCommLoss = 0;

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
	/* simulateCommLoss NAO e' tratado aqui: zerar o timestamp era inutil (a ISR
	 * de RX reescrevia no quadro seguinte). A simulacao agora descarta os
	 * quadros direto no RX -> ver receiveCANFromSlaves() em can.c. */
}

