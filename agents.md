# Repository Agents & Guidelines

## Project Overview
This repository contains the embedded firmware and host-side telemetry scripts for an **STM32F407G-DISC1** based mobile robotics platform. It manages a **50Hz IMU data acquisition pipeline** (**50Hz IMU data acquisition pipeline**) using **DMA transmission** (**DMA transmission**) and a **4-channel motor driver board** (**4-channel motor driver board**).

## Architecture & Component Map
- **Microcontroller**: STM32F407VGT6 (ARM Cortex-M4)
- **Sensor**: MPU6050 6-axis IMU connected via **I2C1** (**I2C1**), triggered by an external interrupt line (`PC4`).
- **Telemetry**: UART2 configured with **DMA** (**Direct Memory Access**) streaming 22-byte custom binary packets.
- **Host Interface**: Raspberry Pi 4B running Python scripts for real-time parsing.

## File Structure Guidelines
- `Core/Src/`, `Core/Inc/`: STM32CubeMX generated files and user application code.
- Drivers must be modularized into dedicated header/source pairs (e.g., `mpu6050.h`, `mpu6050.c`).
- Avoid cluttering `main.c` with hardware initialization logic or sensor-specific implementations.

## Coding Standards
- Follow strict **HAL driver patterns** (**HAL driver patterns**) provided by STMicroelectronics.
- Ensure all communication error states include robust recovery mechanisms (e.g., peripheral de-init/re-init).
- Use clear, descriptive naming conventions for variables, registers, and functions.