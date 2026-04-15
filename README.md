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

## 5. Internal Temperature Estimation (Pulse Resistance)

The Master Unit now features a real-time Internal Temperature Estimator based on the Ohmic Resistance ($R_{DC}$) of the cells. Unlike standard NTC thermistors which measure the surface of the cell, this algorithm accurately predicts the core (jelly roll) temperature by measuring the instantaneous voltage drop during current transients (e.g., rapid acceleration or regeneration).

### Algorithm Flow (`bms_temp_est.c`)
1. **Edge Detection**: Monitors `bmsCurrent`. If the transient exceeds `DELTA_I_THRESHOLD` (5.0A), a timer is started.
2. **Pulse Evaluation**: After a delay of `PULSE_WAIT_MS` (150ms), the instantaneous voltage drop ($\Delta U$) across the pack is recorded.
3. **Arrhenius Mapping**: The algorithm calculates $R_{DC} = \Delta U / \Delta I$ and mathematically inverts the Arrhenius equation to extract temperature using parameters calibrated specifically for the targeted SOC.
4. **Moving Average**: A rolling window filter stabilizes the final output (`estimatedTempC`).

### Python Calibration (`calibrate.py`)
To map raw lab resistances into the micro-controller logic, a standalone Python script evaluates points from the **Molicel P28A** cell at 5°C, 25°C, and 40°C. Using the Bisection Method, the script reverse-engineers the following variables for all 9 State of Charge (SOC) breakpoints:
- **$R_0$**: Fixed metallic contact offsets.
- **$R_1$**: Pre-exponential factor scaling.
- **$E_a$**: Activation energy mapping the lithium ion sluggishness curve.
*Note: The generated C-arrays are already hardcoded, but the script remains in the repository for future battery models.*

### References 📚
The theoretical framework and physical cell parameters implemented in this module were derived from the following academic research:
1. **Pulse Resistance Method**: Ludwig, S.; Steinhardt, M.; Jossen, A. *"Determination of Internal Temperature Differences for Various Cylindrical Lithium-Ion Batteries Using a Pulse Resistance Approach"* — MDPI Batteries, 2022, 8(7), 60.
2. **Molicel P28A Characterization**: Dalla Palma, F. et al. *"Holistic Testing and Characterization of Commercial 18650 Lithium-Ion Cells"* — MDPI Batteries 2024, 10(7), 248.

---
**Author**: Guilherme Lettmann  
**Version**: 1.1 (Includes Active Internal Temp Estimator & Loopback Test Mocks)
