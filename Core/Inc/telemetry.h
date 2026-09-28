/*
 * telemetry.h
 *
 *  Created on: 21 Sept 2026
 *      Author: tim
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include "main.h"

#pragma pack(push, 1)

// Upstream: STM32 -> Raspberry Pi 4B chassis telemetry packet
typedef struct {
    uint8_t header1;        // 0xAA
    uint8_t header2;        // 0x55
    uint8_t robot_id;       // 0x01
    uint32_t timestamp;     // Timestamp in ms

    // MPU6050 6-Axis Data
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    // Wheel Encoder and Speed Data
    int32_t encoder[4];     // Cumulative pulse counts
    float speed_rpm[4];     // Speed in RPM

    uint8_t checksum;       // XOR Checksum
} Chassis_Telemetry_Packet_t;

// Downstream: Raspberry Pi 4B -> STM32 control command packet
typedef struct {
    uint8_t header1;          // 0xAA
    uint8_t header2;          // 0x55
    uint8_t robot_id;         // 0x01
    uint16_t steering_target; // Steering position target (0 - 1000)
    int8_t motor_speeds[4];   // Target motor speeds (-100 to 100)
    uint8_t checksum;         // XOR Checksum
} Control_Command_Packet_t;

#pragma pack(pop)

#endif /* INC_TELEMETRY_H_ */
