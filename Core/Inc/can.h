/**
 * @file can.h
 * @brief Configuration and constants for CAN communication between Master and Slaves.
 *
 * This module defines the communication protocol for the TMS system:
 * - CAN1: Communication with Slave boards (Standard IDs).
 * - CAN2: General system communication and Master state transmission (Extended IDs).
 *
 * Created on: Sep 7, 2025
 * Author: Guilherme Lettmann
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "errors.h"
#include "main.h"
#include "adc.h"

/* ================== Test Modes & Debug Macros ================== */
/** @brief Enable to simulate CAN1 Slave traffic via internal loopback */
//#define testLoopbackCAN1
/** @brief Enable to simulate CAN2 Master traffic via internal loopback */
//#define testLoopbackCAN2

/* ================== System Configuration ======================= */
/** @brief Number of Slave modules connected to the CAN1 network */
#define numberOfSlaves 1

/* ================== CAN Identifiers (CAN1 - Slaves) ============= */
/** @brief Identifier for messages sent by the Master on CAN1 */
#define idMasterCAN1 0x00A

/** @brief Base Identifier for Slave 1 temperature burst (Standard ID) */
#define idSlave1Burst0 0x010
/** @brief Base Identifier for Slave 2 temperature burst (Standard ID) */
#define idSlave2Burst0 0x020
/** @brief Base Identifier for Slave 3 temperature burst (Standard ID) */
#define idSlave3Burst0 0x030
/** @brief Base Identifier for Slave 4 temperature burst (Standard ID) */
#define idSlave4Burst0 0x040

/** @brief Error Identifier for Slave 1 thermistor disconnection */
#define idSlave1ThermistorError 0x050
/** @brief Error Identifier for Slave 2 thermistor disconnection */
#define idSlave2ThermistorError 0x051
/** @brief Error Identifier for Slave 3 thermistor disconnection */
#define idSlave3ThermistorError 0x052
/** @brief Error Identifier for Slave 4 thermistor disconnection */
#define idSlave4ThermistorError 0x053

/* ================== CAN Identifiers (CAN2 - General) ============ */
/** @brief Extended Identifier for Master status and telemetry (CAN2) */
#define idMasterCAN2 0x19308082

/* ================== Transmission Robustness Parameters ========= */
/** @brief Maximum retries per CAN frame before dropping it (~20ms wait) */
#define CAN_TX_RETRY_MAX      20
/** @brief Threshold for consecutive failed frames before triggering SDC Shutdown */
#define CAN_TX_FAULT_THRESHOLD 10

/**
 * @brief CAN Transmission Status Enumeration.
 */
typedef enum {
	CAN_TX_OK = 0,       /**< All frames sent and accepted by hardware queue */
	CAN_TX_FAIL,         /**< Frame dropped after retries (system remains active) */
	CAN_TX_FATAL         /**< Safety threshold reached: SDC Shutdown will be triggered */
} CAN_TxStatus_t;

/* ================== Function Prototypes ======================== */

/**
 * @brief Processes a temperature burst packet received from a slave.
 * @param slave Index of the slave (0-3).
 * @param burst Index of the 8-byte burst (0-7).
 */
void processSlaveBurst(uint8_t slave, uint8_t burst);

/**
 * @brief Handle reception on CAN1 (Slave communication).
 * Called from ISR context to process headers and update buffers.
 */
void receiveCANFromSlaves();

/**
 * @brief Handle reception on CAN2 (General bus communication).
 * Saves the latest message for telemetry and debugging.
 */
void receiveCANFromGeral();

/**
 * @brief Broadcasts Master status and slave maximum temperatures to CAN2.
 * @param slaveMaxTemps Array of maximum temperatures from each slave.
 * @param error Current composite system error code.
 */
void sendMasterInfoToCAN(float *slaveMaxTemps, int error);

/**
 * @brief Sends a generic message to the Slave network (CAN1).
 * @param id CAN Identifier (Standard).
 * @param data Message payload (up to 8 bytes).
 * @param len Actual data length.
 */
void sendToSlavesCAN(uint32_t id, uint8_t *data, uint32_t len);

/**
 * @brief Simulates slave temperature transmission for loopback testing.
 * @param slaveIdx Index of the slave to simulate.
 * @param burstIdx Index of the burst to simulate.
 */
void simulateSlaveBurst(uint8_t slaveIdx, uint8_t burstIdx);

/* ============= CAN RX Message structure for Queuing/Monitoring ============= */
typedef struct {
	FDCAN_RxHeaderTypeDef header;   /**< HAL Header containing ID, DLC, and state */
	uint8_t data[8];                /**< 8-byte data payload */
} CAN_RxMsg_t;

#endif /* INC_CAN_H_ */
