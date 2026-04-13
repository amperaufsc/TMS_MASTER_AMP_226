# TMS Master Unit - Firmware Documentation

This repository contains the firmware for the **TMS Master Unit** (based on STM32G4). The system is designed to monitor battery pack temperatures by coordinating multiple Slave modules over a distributed CAN bus network.

## 1. System Architecture

The firmware is built on **FreeRTOS** and utilizes a task-based approach to ensure real-time responsiveness and system safety.

### Core Tasks
| Task Name | Priority | Frequency | Purpose |
| :--- | :--- | :--- | :--- |
| `xSendCAN` | Normal | 10 Hz | Broadcasts system status and max temperatures to the vehicle bus (CAN2). Checks for thermal limits. |
| `xCheckComms` | Low | 100 Hz | Monitors heartbeats from all slaves. Triggers SDC shutdown if communication is lost for >2 seconds. |
| `xReadTemp` | Low | 10 Hz | (Simulation/Debug) Reads local ADCs through DMA, applies DSP filters, and simulates thermistor data. |

## 2. Communication Protocol

The Master unit manages two independent CAN buses:

### CAN1: Slave Network (Standard ID)
- **Architecture**: Master-Slave (Passive Slaves).
- **Format**: Classic CAN, 500kbps (configurable).
- **Reception**: Processes 8-frame bursts from each slave. Each frame contains 2 float values (4 bytes each).
- **IDs**: 
  - Slave 1: `0x010` - `0x017`
  - Slave 2: `0x020` - `0x027`
  - ... and so on.

### CAN2: General/Telemetry Bus (Extended ID)
- **Format**: Classic CAN, Extended ID support.
- **Master Broadcast (`0x19308082`)**:
  - `Byte 0`: System Error Code (Bitmask).
  - `Bytes 1-4`: Maximum temperature detected by Slave 1, 2, 3, and 4 respectively.

## 3. Safety and Robustness Features

### SDC Shutdown (Safety Disconnect Circuit)
The firmware is designed to automatically trip the SDC (via `GPIOC Pin 7`) under the following conditions:
1. **Critical Over-Temperature**: Any thermistor reading exceeds `60°C`.
2. **Communication Loss**: No data received from a slave for more than `2000ms`.
3. **Hardware Runtime Failures**: Any internal HAL error or peripheral failure.
4. **Network Failure**: More than 10 consecutive CAN transmission failures (Network Fault).

### Robust Transmission Mechanism
All CAN transmissions utilize a retry logic:
- If the hardware TX FIFO is full, the system waits and retries up to **20 times**.
- A global counter tracks **consecutive failures**. If the network remains blocked, the system enters a safe state (Shutdown).

## 4. Debug and Simulation Modes

The firmware includes built-in simulation tools for development without physical hardware:
- **`#define simulateSlave`**: Enables local ADC sampling and DSP filtering to simulate slave data.
- **`#ifdef testLoopbackCAN1/2`**: Configures the hardware for Internal/External Loopback, allowing the Master to receive its own transmissions for verification.
- **Simulation Controls**: (Accessible via Live Expressions)
  - `simulateHighTemp`: Triggers an artificial 100°C fault.
  - `simulateCommLoss`: Triggers a communication timeout fault.

---
**Author**: Guilherme Lettmann  
**Version**: 1.0 (Callback-based FDCAN Architecture)
