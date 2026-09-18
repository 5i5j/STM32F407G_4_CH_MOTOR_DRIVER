#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(push, 1)
typedef struct {
    uint8_t header1;
    uint8_t header2;
    uint8_t robot_id;
    uint32_t timestamp;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temperature;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    uint8_t checksum;
} IMU_Packet_t;
#pragma pack(pop)

extern volatile uint8_t mpu_data_ready;

uint8_t MPU6050_Init_Sequence(void);
HAL_StatusTypeDef MPU6050_Read_Data(IMU_Packet_t *packet);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */