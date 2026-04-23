/**
 * @file bms_temp_est.h
 * @brief Estimador de temperatura interna da bateria via pulso de resistência (R_DC).
 */

#ifndef BMS_TEMP_EST_H
#define BMS_TEMP_EST_H

#include <stdint.h>
#include <stdbool.h>

#define BMS_TEMP_EST_SOC_POINTS 11
#define FILTER_WINDOW_SIZE      8

// --- Constantes Físicas e Thresholds da Máquina de Estados ---
// Constante de Boltzmann em eV/K
#define K_B 8.617e-5f 

// Limiar mínimo de variação de corrente [A] para acionar detecção de pulso
#define DELTA_I_THRESHOLD 2.0f

// Duração de espera (delta T) da resposta da célula [ms]
#define PULSE_WAIT_MS 150

/**
 * @brief Estrutura de estado do estimador de temperatura
 */
typedef struct {
    float currentSoc;
    
    /* Variáveis para detecção de pulso */
    float lastCurrentA;
    float lastVoltageV;
    bool  isPulseActive;
    uint32_t pulseStartTimeMs;
    
    float estimatedTempC;
    bool  isTempValid;

    /* Filtro de média móvel */
    float tempBuffer[FILTER_WINDOW_SIZE];
    uint8_t tempBufferIdx;
    uint8_t tempBufferCount;
} bmsTempEstState;

/**
 * @brief Inicializa o estimador de temperatura
 * @param state Ponteiro para a estrutura de estado
 */
void bmsTempEstInit(bmsTempEstState *state);

/**
 * @brief Atualiza o SOC atual para cálculo dos parâmetros de Arrhenius
 * @param state Ponteiro para a estrutura de estado
 * @param soc Valor de SOC (0.0 a 1.0)
 */
void bmsTempEstSetSoc(bmsTempEstState *state, float soc);

/**
 * @brief Alimenta o estimador com novas amostras de corrente e tensão
 * @param state Ponteiro para a estrutura de estado
 * @param currentA Corrente instantânea do pack [A]
 * @param voltageV Tensão instantânea da célula [V]
 */
void bmsTempEstFeedSample(bmsTempEstState *state, float currentA, float voltageV);

/**
 * @brief Verifica se a temperatura estimada atual é válida
 * @param state Ponteiro para a estrutura de estado
 * @return true se o valor puder ser usado, false caso contrário
 */
bool bmsTempEstIsValid(bmsTempEstState *state);

/**
 * @brief Retorna a última temperatura estimada calculada
 * @param state Ponteiro para a estrutura de estado
 * @return Temperatura em graus Celsius [°C]
 */
float bmsTempEstGetTemp(bmsTempEstState *state);

#endif /* BMS_TEMP_EST_H */
