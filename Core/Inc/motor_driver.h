/*
 * motor_driver.h
 *
 *  Created on: 19 Sept 2026
 *      Author: tim
 */


#ifndef INC_MOTOR_DRIVER_H_
#define INC_MOTOR_DRIVER_H_

#include "main.h"

// 对应驱动板 4 通道的结构体
typedef struct {
	uint32_t timestamp;
    int32_t encoder[4];          // 4 路电机当前的累积脉冲总数
    int32_t delta_encoder[4];    // 两次采样之间的增量脉冲
    int32_t last_encoder[4];     // 上一次采样时的脉冲总数
    float   speed_rpm[4];        // 计算得到的各轮转速 (RPM)
} Motor_Status_t;

extern Motor_Status_t g_motor_status;

// 函数声明
HAL_StatusTypeDef Motor_Init(void);
HAL_StatusTypeDef Motor_SetSpeeds(int8_t speeds[4]);
HAL_StatusTypeDef Motor_ReadEncoders(int32_t total_pulses[4]);
void Motor_Update_Callback(void); // 供定时器调用的周期性更新函数

#endif /* INC_MOTOR_DRIVER_H_ */
