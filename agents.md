# Repository Agents & Guidelines

## Project Overview
This repository contains the embedded firmware and host-side telemetry scripts for an **STM32F407G-DISC1** based mobile robotics platform. The system coordinates a **50Hz IMU data acquisition pipeline** and a **4-channel motor encoder driver** via dual I2C buses, streaming real-time status packets to a **Raspberry Pi 4B host** via UART DMA.

## Architecture & Component Map
- **Main Controller**: STM32F407VGT6 (ARM Cortex-M4) running at 168MHz.
- **Motion Subsystem**: Hiwonder 4-Channel Encoder Motor Driver connected via **I2C2** (`PB10`/`PB11`), running internal velocity PID loops.
- **Inertial Sensor**: MPU6050 6-axis IMU connected via **I2C1** (`PB6`/`PB7`), with data ready interrupts managed via external interrupt lines (`PC4`).
- **Telemetry & Command Bridge**: UART2 configured with **DMA** streaming 50Hz combined binary telemetry packets (IMU + wheel encoders) to the host and receiving velocity commands.
- **Host System**: Raspberry Pi 4B running ROS 2 nodes for state estimation, odometry calculation, and motion planning.

## File Structure Guidelines
- `Core/Src/`, `Core/Inc/`: STM32CubeMX generated drivers and modular user application code.
- Drivers must be strictly isolated into dedicated header/source modules (`mpu6050.h`/`.c`, `motor_driver.h`/`.c`).
- Keep `main.c` lean by offloading hardware initialization, kinematic calculations, and I2C transaction handling to respective module callbacks.

## Coding Standards & System Robustness
- **Non-blocking I2C Execution**: Never invoke blocking I2C calls directly inside Interrupt Service Routines (ISRs). Use interrupt flags to schedule I2C transactions in the main loop.
- **Fault Recovery**: All I2C and UART communications must implement peripheral reset logic (`HAL_I2C_DeInit` / `MX_I2C_Init`) upon detecting `HAL_BUSY` or `HAL_ERROR` status.
- **Data Integrity**: Enforce XOR checksum verification across all custom binary serial packet formats.
- **Timing Synchronization**: Maintain precise 50Hz execution using TIM6 hardware timer flags for deterministic wheel speed and IMU telemetry sampling.