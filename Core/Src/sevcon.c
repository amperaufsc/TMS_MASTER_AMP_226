/**
 * @file sevcon.c
 * @brief Driver base para manipular H-Protocol J1939 
 */

#include "sevcon.h"
#include <string.h>

// Sequence Counters Independentes
static uint8_t seq_hc1 = 0;
static uint8_t seq_hc2 = 0;
static uint8_t seq_hc3 = 0;

/* ================== FUNÇÕES PRIVADAS DE PROTOCOLO ================== */

/**
 * @brief Monta a cabeça do J1939 Extend 29-bit (Dica da Borgwarner)
 */
static uint32_t build_j1939_id(uint8_t priority, uint8_t data_page, uint8_t pf, uint8_t dest_addr, uint8_t src_addr) {
    uint32_t id = (priority & 0x07) << 26;
    id |= (data_page & 0x01) << 24;
    id |= (pf & 0xFF) << 16;
    id |= (dest_addr & 0xFF) << 8;
    id |= (src_addr & 0xFF);
    return id;
}

/**
 * @brief Soma e travas os 6 data-bytes junto o número de sequência preenchendo as travas p/ evitar rejeição fantasma 
 */
static uint8_t calc_checksum(uint8_t *data, uint8_t seq) {
    uint32_t sum = 0;
    for (int i = 0; i < 6; i++) {
        sum += data[i];
    }
    sum += seq;
    return (uint8_t)(sum & 0xFF);
}

/* ================== INICIALIZAÇÃO DE LIMITES UNUSED ================== */

void Sevcon_InitDemands(Sevcon_TxDemands_t *demands) {
    if(!demands) return;
    
    // Torque nulo
    demands->torqueDemandNm = 0.0f;
    demands->controlWord = SEVCON_CTRL_SHUTDOWN;
    
    // *Regras Unused da Borgwarner*: Envie limites muito abertos ou absurdos 
    // Em vez de Zero. Se mandar Zero ele não move ou causa faults imediatos no Derating!
    
    demands->driveTorqueLimitNm = 500.0f;        // Torque de Drive livre máximo
    demands->regenTorqueLimitNm = -500.0f;       // Cuidado, esse é negativo, Regen massivo de sobra
    demands->fwdSpeedLimitRPM = 12000;           // MAX RPM Forward livre
    demands->revSpeedLimitRPM = 2000;            // Rev Limit alto
    
    demands->batDischargeLimitA = 400;           // 400A Corrente Máx. Download 
    demands->batChargeLimitA = -100;             // Corrente máx recarga (-100A regen)
    demands->targetCapVoltage = 300.0f;          // Target base
}


/* ================== FILA DE ENVIO ================== */

void Sevcon_Send_HC1(FDCAN_HandleTypeDef *hfdcan, Sevcon_TxDemands_t *demands) {
    FDCAN_TxHeaderTypeDef txHeader;
    uint8_t data[8];

    // Montar Header e J1939 PGN=0x11000 - PF=0x10, DA=0x01, SA=0x00
    txHeader.Identifier = build_j1939_id(6, 1, 0x10, SEVCON_SA_DEFAULT, VCU_SA_DEFAULT);
    txHeader.IdType = FDCAN_EXTENDED_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    // Converte floats (Resolução de 0.0625 Nm/b = multplica por 16)
    int16_t trq = (int16_t)(demands->torqueDemandNm * 16.0f);
    int16_t lim = (int16_t)(demands->driveTorqueLimitNm * 16.0f);
    uint16_t cmd = (uint16_t)demands->controlWord;

    data[0] = (uint8_t)(trq & 0xFF);
    data[1] = (uint8_t)((trq >> 8) & 0xFF);
    
    data[2] = (uint8_t)(cmd & 0xFF);
    data[3] = (uint8_t)((cmd >> 8) & 0xFF);
    
    data[4] = (uint8_t)(lim & 0xFF);
    data[5] = (uint8_t)((lim >> 8) & 0xFF);
    
    data[6] = seq_hc1;
    data[7] = calc_checksum(data, seq_hc1);
    
    // Atualiza SEQ c/ clamping pular sem perdas
    seq_hc1 = (seq_hc1 + 1) & 0xFF;

    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, data);
}

void Sevcon_Send_HC2(FDCAN_HandleTypeDef *hfdcan, Sevcon_TxDemands_t *demands) {
    FDCAN_TxHeaderTypeDef txHeader;
    uint8_t data[8];

    // Montar Header e J1939 PGN=0x11100 - PF=0x11
    txHeader.Identifier = build_j1939_id(6, 1, 0x11, SEVCON_SA_DEFAULT, VCU_SA_DEFAULT);
    txHeader.IdType = FDCAN_EXTENDED_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    int16_t regenLim = (int16_t)(demands->regenTorqueLimitNm * 16.0f);

    data[0] = (uint8_t)(regenLim & 0xFF);
    data[1] = (uint8_t)((regenLim >> 8) & 0xFF);
    
    data[2] = (uint8_t)(demands->fwdSpeedLimitRPM & 0xFF);
    data[3] = (uint8_t)((demands->fwdSpeedLimitRPM >> 8) & 0xFF);
    
    data[4] = (uint8_t)(demands->revSpeedLimitRPM & 0xFF);
    data[5] = (uint8_t)((demands->revSpeedLimitRPM >> 8) & 0xFF);
    
    data[6] = seq_hc2;
    data[7] = calc_checksum(data, seq_hc2);
    
    seq_hc2 = (seq_hc2 + 1) & 0xFF;

    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, data);
}

void Sevcon_Send_HC3(FDCAN_HandleTypeDef *hfdcan, Sevcon_TxDemands_t *demands) {
    FDCAN_TxHeaderTypeDef txHeader;
    uint8_t data[8];

    // Montar Header e J1939 PGN=0x11200 - PF=0x12
    txHeader.Identifier = build_j1939_id(6, 1, 0x12, SEVCON_SA_DEFAULT, VCU_SA_DEFAULT);
    txHeader.IdType = FDCAN_EXTENDED_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    int16_t vcap = (int16_t)(demands->targetCapVoltage * 16.0f);

    data[0] = (uint8_t)(demands->batDischargeLimitA & 0xFF);
    data[1] = (uint8_t)((demands->batDischargeLimitA >> 8) & 0xFF);
    
    data[2] = (uint8_t)(demands->batChargeLimitA & 0xFF);
    data[3] = (uint8_t)((demands->batChargeLimitA >> 8) & 0xFF);
    
    data[4] = (uint8_t)(vcap & 0xFF);
    data[5] = (uint8_t)((vcap >> 8) & 0xFF);
    
    data[6] = seq_hc3;
    data[7] = calc_checksum(data, seq_hc3);
    
    seq_hc3 = (seq_hc3 + 1) & 0xFF;

    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, data);
}

/* ================== LEITURA DAS MENSAGENS ================== */

bool Sevcon_ParseStatus(uint32_t canId, uint8_t *data, uint8_t dlc, Sevcon_RxStatus_t *status) {
    if (!status || dlc < 8) return false;

    // Isola o PGN do CABEÇALHO Extendido ignorando o Source Address 
    // J1939 Extrai os bits PGN (18 bits shiftados por 8 de CA/SA)
    uint32_t pgn = (canId >> 8) & 0x3FFFF;

    switch (pgn) {
        case PGN_HS1:
            // Torque em 0.0625 Nm/b (divide por 16)
            status->actualTorqueNm = (float)((int16_t)((data[1] << 8) | data[0])) / 16.0f;
            status->motorSpeedRPM = (int16_t)((data[3] << 8) | data[2]);
            status->batteryCurrentA = (int16_t)((data[5] << 8) | data[4]);
            return true;

        case PGN_HS2:
            status->availFwdTorqueNm = (float)((int16_t)((data[1] << 8) | data[0])) / 16.0f;
            status->availRevTorqueNm = (float)((int16_t)((data[3] << 8) | data[2])) / 16.0f;
            status->statusWord = (SevconState_t)(data[4] & 0x0F);
            status->torqueLimitCode = data[5];
            return true;

        case PGN_HS3:
            status->invTempMarginC = (int16_t)((data[1] << 8) | data[0]);
            status->motorTempC = (int16_t)((data[3] << 8) | data[2]);
            status->capacitorVoltage = (float)((int16_t)((data[5] << 8) | data[4])) / 16.0f;
            return true;

        case PGN_HS4:
            status->faultCode = (uint16_t)((data[1] << 8) | data[0]);
            status->faultDebug = ((uint32_t)data[5] << 24) | ((uint32_t)data[4] << 16) | ((uint32_t)data[3] << 8) | data[2];
            return true;

        case PGN_HS5:
            status->encoderAngleRaw = ((uint32_t)data[3] << 24) | ((uint32_t)data[2] << 16) | ((uint32_t)data[1] << 8) | data[0];
            // 32-bit * 1.525×10⁻⁵ se converte em rad ou graus se quiser. Aqui guardamos o RAW
            return true;
            
        default:
            return false;
    }
}

/* ================== ESTADO DE MÁQUINA (STARTUP SEQUENCE) ================== */

static VcuSevconState_t internalState = VCU_SEVCON_INIT;

void Sevcon_RunStateMachine(Sevcon_RxStatus_t *rx_status, Sevcon_TxDemands_t *demands) {
    if (!rx_status || !demands) return;

    // Trava de segurança suprema: Se o inversor entrar em FAULT, derruba a VCU também para o estado de erro
    if (rx_status->statusWord == SEVCON_STATE_FAULT_REA || rx_status->statusWord == SEVCON_STATE_FAULT_OFF) {
        internalState = VCU_SEVCON_FAULT;
        demands->controlWord = SEVCON_CTRL_SHUTDOWN;
        return;
    }

    switch (internalState) {
        case VCU_SEVCON_INIT:
            // Aguarda o Inversor terminar o autoteste pular pro mode Shutdown (Pronto de fábrica)
            demands->controlWord = SEVCON_CTRL_NO_COMMAND;
            if (rx_status->statusWord == SEVCON_STATE_SHUTDOWN) {
                internalState = VCU_SEVCON_WAIT_SHUTDOWN;
            }
            break;

        case VCU_SEVCON_WAIT_SHUTDOWN:
            /* ========================================================
             * PLACEHOLDER PARA FSAE E SAFETY RULES REAL:
             * Aqui é onde o carro bloqueia o motor antes do READY TO DRIVE.
             * if (Botao_Start_Pressionado && Pedal_Freio > Threshold) {
             *     demands->controlWord = SEVCON_CTRL_ENERGISE;
             *     internalState = VCU_SEVCON_ENERGIZING;
             * } else {
             *     demands->controlWord = SEVCON_CTRL_NO_COMMAND; // Segura em repouso
             * }
             * ========================================================
             */
            
            // ABAIXO: Pulo automático solto apenas para testes em bancada!
            demands->controlWord = SEVCON_CTRL_ENERGISE;
            internalState = VCU_SEVCON_ENERGIZING;
            break;

        case VCU_SEVCON_ENERGIZING:
            // Após comandar Energise, monitora na malha pelo retorno HS2 do Sevcon
            if (rx_status->statusWord == SEVCON_STATE_ENERGISED) {
                // Ao receber o sinal positivo, emite a Enable Word para fechar as pontes IGBT
                demands->controlWord = SEVCON_CTRL_ENABLE;
                internalState = VCU_SEVCON_ENABLING;
            } else {
            	// Force retention
            	demands->controlWord = SEVCON_CTRL_ENERGISE;
            }
            break;

        case VCU_SEVCON_ENABLING:
            // Aguarda o status de potência Enable
            if (rx_status->statusWord == SEVCON_STATE_ENABLED) {
                // Carro operacional ("RTD") e motor no seu comando! Liberar acelerador!
                internalState = VCU_SEVCON_RUNNING;
            } else {
            	demands->controlWord = SEVCON_CTRL_ENABLE;
            }
            break;

        case VCU_SEVCON_RUNNING:
            // Garante que o Bit de Control Word vai continuar sustentando ENABLE a cada pulso de 5ms
            demands->controlWord = SEVCON_CTRL_ENABLE;
            // Nesse estado, o código da sua Main() lidará puramente com o demands->torqueDemandNm!
            break;

        case VCU_SEVCON_FAULT:
            // O Gen 5 / HVLP demanda de Power Cycle de baixa-tensão caso crashe e solte faísca mental (Erros tipo Overtemp etc).
            // CAN FAULT_RESET (0x09) NÃO FUNCIONA no Gen5. Fica retido em shutoff até desligar o botão do carro inteiro!
            demands->controlWord = SEVCON_CTRL_SHUTDOWN;
            break;
    }
}
