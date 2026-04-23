/**
 * @file sevcon.h
 * @brief Driver para protocolo Borgwarner H-Protocol (Gen5 Size 9).
 */

#ifndef SEVCON_H
#define SEVCON_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h" // Importa as HALs

// Constantes PGN e Endereços
#define SEVCON_SA_DEFAULT 0x01
#define VCU_SA_DEFAULT    0x00

#define PGN_HC1 0x11000
#define PGN_HC2 0x11100
#define PGN_HC3 0x11200
#define PGN_HS1 0x11800
#define PGN_HS2 0x11900
#define PGN_HS3 0x11A00
#define PGN_HS4 0x11B00
#define PGN_HS5 0x11C00

/**
 * @brief Palavras de Controle do H-Protocol
 */
typedef enum {
    SEVCON_CTRL_NO_COMMAND = 0x0000,
    SEVCON_CTRL_ENERGISE   = 0x0003,
    SEVCON_CTRL_ENABLE     = 0x0005,
    SEVCON_CTRL_SHUTDOWN   = 0x0006
} SevconControlWord_t;

/**
 * @brief Máquina de Estados da Sevcon Gen5
 */
typedef enum {
    SEVCON_STATE_NOT_READY = 0x01,
    SEVCON_STATE_SHUTDOWN  = 0x02,
    SEVCON_STATE_PRECHARGE = 0x04,
    SEVCON_STATE_ENERGISED = 0x07,
    SEVCON_STATE_ENABLED   = 0x08,
    SEVCON_STATE_FAULT_REA = 0x0B,
    SEVCON_STATE_FAULT_OFF = 0x0D,
    SEVCON_STATE_ACTIVE_SH = 0x0E
} SevconState_t;

/**
 * @brief Maquina de Estados interna da VCU (Para sequenciamento de Boot Seguro)
 */
typedef enum {
    VCU_SEVCON_INIT = 0,
    VCU_SEVCON_WAIT_SHUTDOWN,
    VCU_SEVCON_ENERGIZING,
    VCU_SEVCON_ENABLING,
    VCU_SEVCON_RUNNING,
    VCU_SEVCON_FAULT
} VcuSevconState_t;

/**
 * @brief Estrutura que guarda todas as demandas da VCU (Limites e Torques)
 */
typedef struct {
    float torqueDemandNm;
    SevconControlWord_t controlWord;
    float driveTorqueLimitNm;
    
    float regenTorqueLimitNm;
    int16_t fwdSpeedLimitRPM;
    int16_t revSpeedLimitRPM;

    int16_t batDischargeLimitA;
    int16_t batChargeLimitA;
    float targetCapVoltage;
} Sevcon_TxDemands_t;

/**
 * @brief Estado retornado pele Inversor via CAN (Feedback da telemetria)
 */
typedef struct {
    // HS1
    float actualTorqueNm;
    int16_t motorSpeedRPM;
    int16_t batteryCurrentA;
    
    // HS2
    float availFwdTorqueNm;
    float availRevTorqueNm;
    SevconState_t statusWord;
    uint8_t torqueLimitCode;
    
    // HS3
    int16_t invTempMarginC;
    int16_t motorTempC;
    float capacitorVoltage;
    
    // HS4
    uint16_t faultCode;
    uint32_t faultDebug;
    
    // HS5
    uint32_t encoderAngleRaw;
    float encoderAngleRad;
} Sevcon_RxStatus_t;

/**
 * @brief Inicializa a estrutura de TX setando os campos "unused" para números extremos e permitindo o funcionamento.
 * @param demands Estrutura de dados p/ carregar pro inversor
 */
void Sevcon_InitDemands(Sevcon_TxDemands_t *demands);

/**
 * @brief Envia o Comando de Torque e Drive States (Envio: a cada 5~10ms)
 */
void Sevcon_Send_HC1(FDCAN_HandleTypeDef *hfdcan, Sevcon_TxDemands_t *demands);

/**
 * @brief Envia os Limites Dinâmicos Mecânicos (Envio: a cada 5~10ms)
 */
void Sevcon_Send_HC2(FDCAN_HandleTypeDef *hfdcan, Sevcon_TxDemands_t *demands);

/**
 * @brief Envia as Restrições Elétricas da Bateria (Envio: a cada 50~100ms)
 */
void Sevcon_Send_HC3(FDCAN_HandleTypeDef *hfdcan, Sevcon_TxDemands_t *demands);

/**
 * @brief Mastiga os quadros da Sevcon em variáveis limpas. Colocar no RX Fifo 0 Callback.
 * @param canId ID Extendido recebido (29-bit)
 * @param data Array Data [8 bytes]
 * @param dlc Tamanho (usualmente 8)
 * @param status Ponteiro da variavel global p/ ser populada
 * @return true se era mensagem conhecida da sevcon, false caso não fosse.
 */
bool Sevcon_ParseStatus(uint32_t canId, uint8_t *data, uint8_t dlc, Sevcon_RxStatus_t *status);

/**
 * @brief Máquina de Estados: Engata marchas do protocolo com base no RX lido.
 * @param rx_status Variavel carregada pelo Parse do CAN2
 * @param demands Variavel para Setar a control Word dinamicamente
 */
void Sevcon_RunStateMachine(Sevcon_RxStatus_t *rx_status, Sevcon_TxDemands_t *demands);

#endif /* SEVCON_H */
