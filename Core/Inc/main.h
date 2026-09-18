/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#pragma pack(1) // 确保结构体按 1 字节紧凑对齐 (Ensure byte alignment)
typedef struct {
    uint8_t header1;       // 帧头 1 (Frame Header 1)
    uint8_t header2;       // 帧头 2 (Frame Header 2)
    uint8_t robot_id;      // 机器人编号 (Robot ID)
    uint32_t timestamp;    // 时间戳 (Timestamp)
    int16_t accel_x;       // X轴加速度 (X-axis acceleration)
    int16_t accel_y;       // Y轴加速度 (Y-axis acceleration)
    int16_t accel_z;       // Z轴加速度 (Z-axis acceleration)
    int16_t temperature;   // 温度 (Temperature)
    int16_t gyro_x;        // X轴陀螺仪 (X-axis gyroscope)
    int16_t gyro_y;        // Y轴陀螺仪 (Y-axis gyroscope)
    int16_t gyro_z;        // Z轴陀螺仪 (Z-axis gyroscope)
    uint8_t checksum;      // 校验和 (Checksum)
} __attribute__((packed)) IMU_Packet_t;
#pragma pack()
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IMU_ITR_Pin GPIO_PIN_4
#define IMU_ITR_GPIO_Port GPIOC
#define IMU_ITR_EXTI_IRQn EXTI4_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
