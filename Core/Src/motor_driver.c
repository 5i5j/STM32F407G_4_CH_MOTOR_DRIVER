/*
 * motor_driver.c
 *
 *  Created on: 19 Sept 2026
 *      Author: tim
 */



#include "motor_driver.h"
#include "i2c.h"

#define MOTOR_DRIVER_I2C_ADDR    (0x34 << 1) // 驱动板 8-bit I2C 地址 (0x68)

Motor_Status_t g_motor_status = {0};

// Encoder counts reported by the motor controller, per motor-shaft revolution.
#define MOTOR_ENCODER_COUNTS_PER_REV  44.0f
#define GEAR_RATIO                    90.0f
#define PULSES_PER_REV \
    (MOTOR_ENCODER_COUNTS_PER_REV * GEAR_RATIO) // 3960 counts per wheel revolution

/**
  * @brief  初始化电机驱动板（配置电机类型与编码器极性）
  */
HAL_StatusTypeDef Motor_Init(void)
{
    uint8_t motor_type = 3;        // MOTOR_TYPE_JGB
    uint8_t encoder_polarity = 0;
    HAL_StatusTypeDef status = HAL_OK;

    // 1. 配置电机类型 (寄存器 20)
    uint8_t type_buf[4] = {motor_type, 0, 0, 0};
    status = HAL_I2C_Mem_Write(&hi2c2, MOTOR_DRIVER_I2C_ADDR, 20,
                              I2C_MEMADD_SIZE_8BIT, type_buf, 4, 100);
    if (status != HAL_OK) {
        return status;
    }
    HAL_Delay(5);

    // 2. 配置编码器极性 (寄存器 21)
    status = HAL_I2C_Mem_Write(&hi2c2, MOTOR_DRIVER_I2C_ADDR, 21,
                              I2C_MEMADD_SIZE_8BIT, &encoder_polarity, 1, 100);
    if (status != HAL_OK) {
        return status;
    }
    HAL_Delay(5);

    return HAL_OK;
}

/**
  * @brief  设置 4 通道电机目标速度
  * @param  speeds 包含 4 个通道速度值的数组 (-100 到 100)
  */
HAL_StatusTypeDef Motor_SetSpeeds(int8_t speeds[4])
{
    // 向寄存器 51 写入 4 个 int8_t 速度值
    return HAL_I2C_Mem_Write(&hi2c2, MOTOR_DRIVER_I2C_ADDR, 51,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t *)speeds, 4, 50);
}

/**
  * @brief  读取 4 路编码器累积脉冲计数
  * @param  total_pulses 接收 4 个电机脉冲数据的 int32_t 数组
  */
HAL_StatusTypeDef Motor_ReadEncoders(int32_t total_pulses[4])
{
    // 从寄存器 60 读取 16 字节 (4 通道 * 4 字节 int32)
    return HAL_I2C_Mem_Read(&hi2c2, MOTOR_DRIVER_I2C_ADDR, 60, I2C_MEMADD_SIZE_8BIT, (uint8_t *)total_pulses, 16, 50);
}

/**
  * @brief  周期性采样与状态更新函数
  * @note   放入 TIM6 中断回调函数中执行
  */
void Motor_Update_Callback(void)
{
    int32_t current_pulses[4] = {0};
    static uint32_t previous_sample_tick;
    static uint8_t sample_initialized;

    // 1. 记录当前采样的时间戳 (Record current sample timestamp)
    // 2. 通过 I2C 读取当前总脉冲数 (Read total pulses via I2C)
    if (Motor_ReadEncoders(current_pulses) == HAL_OK)
    {
        const uint32_t sample_tick = HAL_GetTick();
        const uint32_t elapsed_ms = sample_tick - previous_sample_tick;
        const float elapsed_sec = (float)elapsed_ms / 1000.0f;

        g_motor_status.timestamp = sample_tick;
        for (int i = 0; i < 4; i++)
        {
            // 3. 更新累积脉冲并计算增量 (Update cumulative pulses and calculate delta)
            g_motor_status.encoder[i] = current_pulses[i];
            g_motor_status.delta_encoder[i] = current_pulses[i] - g_motor_status.last_encoder[i];
            g_motor_status.last_encoder[i] = current_pulses[i];

            // The first reading has no preceding sample, so it has no speed.
            if (!sample_initialized || elapsed_ms == 0U) {
                g_motor_status.speed_rpm[i] = 0.0f;
            } else {
                g_motor_status.speed_rpm[i] =
                    ((float)g_motor_status.delta_encoder[i] / PULSES_PER_REV) *
                    (60.0f / elapsed_sec);
            }
        }

        previous_sample_tick = sample_tick;
        sample_initialized = 1U;
    }
}
