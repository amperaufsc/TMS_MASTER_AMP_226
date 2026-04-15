/**
 * @file bms_temp_est.c
 * @brief Implementação do estimador de temperatura interna da bateria via pulso de resistência (R_DC).
 * 
 * Este arquivo implementa a máquina de estados que isola a Resistência Ohmmica (Rs)
 * durante saltos de corrente repentinos, usando a equação empírica de Arrhenius 
 * para revelar a temperatura real do jelly roll da bateria.
 */

#include "bms_temp_est.h"
#include <math.h>
#include <stddef.h>

/* Parâmetros de Arrhenius por SOC (calibrados para a Molicel P28A baseados em dados GITT 3-Temp) */

/** 
 * \brief Tabela de compensação ôhmica por SOC (R0).
 * Constante associada aos contatos fixos das células e abas dos coletores.
 */
static const float r0Lut[BMS_TEMP_EST_SOC_POINTS] = {
-0.081636f, -0.081636f, 0.005101f, 0.009964f, 0.009108f, 0.004168f, 0.001307f, 0.004737f, 0.010057f, 0.006521f, 0.006521f
};

/**
 * \brief Tabela de fator pré-exponencial por SOC (R1).
 * Fator de escala matemática pura do decaimento da mobilidade iônica.
 */
static const float r1Lut[BMS_TEMP_EST_SOC_POINTS] = {
3.480254e-02f, 3.480254e-02f, 3.867334e-06f, 1.760914e-08f, 9.712452e-08f, 5.307442e-06f, 2.045779e-05f, 1.997041e-06f, 1.282443e-08f, 4.304753e-07f, 4.304753e-07f
};

/**
 * \brief Tabela de Energia de Ativação por SOC (Ea).
 * Barreiras de ativação eletroquímica derivadas da calibração Molicel.
 */
static const float eaLut[BMS_TEMP_EST_SOC_POINTS] = {
0.027305f, 0.027305f, 0.206345f, 0.329481f, 0.290453f, 0.200746f, 0.171306f, 0.224720f, 0.342309f, 0.262706f, 0.262706f
};


/**
 * @brief Inicializa o estimador de temperatura
 * Reseta a máquina de estados, destrava o bloqueio de pulso preenchendo as variáveies,
 * e satura o filtro da média móvel com 25 graus para não alarmar na largada.
 */
void bmsTempEstInit(bmsTempEstState *state)
{
    if (state == NULL) return;
    
    state->currentSoc = 0.5f;
    state->lastCurrentA = 0.0f;
    state->lastVoltageV = 0.0f;
    state->isPulseActive = false;
    state->pulseStartTimeMs = 0;
    
    state->estimatedTempC = 25.0f;
    state->isTempValid = false;
    
    state->tempBufferIdx = 0;
    state->tempBufferCount = 0;
    
    // Inicialização morna (25°C) para o filtro MAC evitar predições drásticas inicialmente
    for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
        state->tempBuffer[i] = 25.0f;
    }
}

/**
 * @brief Modifica a variável global SOC para balizar o acesso das matrizes de Arrhenius.
 */
void bmsTempEstSetSoc(bmsTempEstState *state, float soc)
{
    if (state == NULL) return;
    state->currentSoc = soc;
}

/**
 * @brief Interpola as constantes eletroquimicas mapeando a curva baseado no nível de carga
 * Utiliza o degrau fixo da aproximação indexada arredondada (*10.0) ou extrapolando no zero/limit.
 */
static void getArrheniusParams(float soc, float *r0, float *r1, float *ea) {
    if (soc <= 0.0f) {
        *r0 = r0Lut[0]; *r1 = r1Lut[0]; *ea = eaLut[0];
        return;
    }
    if (soc >= 1.0f) {
        *r0 = r0Lut[BMS_TEMP_EST_SOC_POINTS-1]; *r1 = r1Lut[BMS_TEMP_EST_SOC_POINTS-1]; *ea = eaLut[BMS_TEMP_EST_SOC_POINTS-1];
        return;
    }
    // Simplificação de interpolação: busca pelo índice em fator x10 mais próximo 
    int idx = (int)(soc * 10.0f);
    if(idx >= BMS_TEMP_EST_SOC_POINTS) idx = BMS_TEMP_EST_SOC_POINTS - 1;
    *r0 = r0Lut[idx];
    *r1 = r1Lut[idx];
    *ea = eaLut[idx];
}

extern uint32_t HAL_GetTick(void);

/**
 * @brief Task Machine Loop do Estimador Térmico (Alimenta as variaveis)
 * Roda contínuamente sondando agressões na corrente e registrando uma fotografia 
 * do antes e depois da aceleração para isolar o Rs real do circuito de thevenin.
 */
void bmsTempEstFeedSample(bmsTempEstState *state, float currentA, float voltageV)
{
    if (state == NULL) return;
    
    uint32_t nowMs = HAL_GetTick();

    if (!state->isPulseActive) {
        // [Fase 1]: Vigia por degraus violentos que ultrapassem o Delta Threshold
        if (fabsf(currentA - state->lastCurrentA) > DELTA_I_THRESHOLD) {
            state->isPulseActive = true;
            state->pulseStartTimeMs = nowMs;
            // IMPORTANTE: "lastCurrentA/lastVoltageV" NÃO VÃO ser atualizados aqui
            // para funcionarem como estacas âncoras da tensão anterior (o repouso logo préximo ao pulso)
        } else {
            // Fica copiando os sinais até achar a aceleração
            state->lastCurrentA = currentA;
            state->lastVoltageV = voltageV;
        }
    } else {
        // [Fase 2]: O pulso aconteceu, estamos no meio do transiente.
        // O algoritmo vai congelar a leitura de "antes", e aguardará 150ms 
        // para isolar R_dc da constante do condensador polarizador do ECM.
        if ((nowMs - state->pulseStartTimeMs) >= PULSE_WAIT_MS) {
            float deltaI = currentA - state->lastCurrentA;
            float deltaV = voltageV - state->lastVoltageV;
            
            // Revalida: A corrente do motor ficou segurando acima de 50% ou vacilou ao 0?
            if (fabsf(deltaI) > (DELTA_I_THRESHOLD * 0.5f)) {
                
                // Extraindo a resistência pura na lei de ohm basica R = V / I
                float r_dc = fabsf(deltaV / deltaI);
                
                // Trava limite fisiológico humano dos NTC (De 5mOhm pra 100mOhm seria loucura)
                if (r_dc >= 0.005f && r_dc <= 0.100f) {
                    float r0, r1, ea;
                    getArrheniusParams(state->currentSoc, &r0, &r1, &ea);
                    
                    // Condição de segurança matemática: Rdc deve ser maior que o r0 
                    // para evitar assintotas e logs com número negativo.
                    if (r_dc > r0) {
                        float ln_val = logf((r_dc - r0) / r1);
                        if (ln_val != 0.0f) {
                            // Aplicando a função invessa de Arrhenius extraída. 
                            float temp_kelvin = ea / (K_B * ln_val);
                            float temp_celsius = temp_kelvin - 273.15f;
                            
                            // Barreira final: despresamos estimativas absurdas/erradas acima de 100oC
                            // Ou frias de mais (-20oC) para que não travem a máquina em safe fault fakes 
                            if (temp_celsius >= -20.0f && temp_celsius <= 100.0f) {
                                
                                // Buffer de Média Móvel Limitada em 8 blocos (Moving Average Filter)
                                state->tempBuffer[state->tempBufferIdx] = temp_celsius;
                                state->tempBufferIdx = (state->tempBufferIdx + 1) % FILTER_WINDOW_SIZE;
                                
                                if (state->tempBufferCount < FILTER_WINDOW_SIZE) {
                                    state->tempBufferCount++;
                                }
                                
                                // Calcula e publica o novo status da temperatura térmica média filtrada!
                                float sum = 0.0f;
                                for (int i = 0; i < state->tempBufferCount; i++) {
                                    sum += state->tempBuffer[i];
                                }
                                state->estimatedTempC = sum / state->tempBufferCount;
                                state->isTempValid = true;
                            }
                        }
                    }
                }
            }
            
            // Relaxamento: Reseta a trigger para buscar as próximas pisadas de acelerador 
            state->isPulseActive = false;
            state->lastCurrentA = currentA;
            state->lastVoltageV = voltageV;
        }
    }
}

/**
 * @brief Checa se o filtro já formou alguma média ou deu NaN nas aquisições.
 */
bool bmsTempEstIsValid(bmsTempEstState *state)
{
    if (state == NULL) return false;
    return state->isTempValid;
}

/**
 * @brief Libera a leitura estagnada limpa finalizada (A temperatura média T_R que alimenta o derating).
 */
float bmsTempEstGetTemp(bmsTempEstState *state)
{
    if (state == NULL) return 0.0f;
    return state->estimatedTempC;
}
