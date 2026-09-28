/*
 * motor_driver.c
 *
 *  Created on: 19 Sept 2026
 *      Author: tim
 */



#include "motor_driver.h"
#include "i2c.h"

#define MOTOR_DRIVER_I2C_ADDR    (0x34 << 1)
#define MOTOR_DRIVER_I2C_TIMEOUT_MS 5U

Motor_Status_t g_motor_status = {0};

// Encoder counts reported by the motor controller, per motor-shaft revolution.
#define MOTOR_ENCODER_COUNTS_PER_REV  44.0f
#define GEAR_RATIO                    90.0f
#define PULSES_PER_REV \
    (MOTOR_ENCODER_COUNTS_PER_REV * GEAR_RATIO) // 3960 counts per wheel revolution

/**
    * @brief  Initialize the motor controller and configure encoder polarity.
  */
HAL_StatusTypeDef Motor_Init(void)
{
    uint8_t motor_type = 3;        // MOTOR_TYPE_JGB
    uint8_t encoder_polarity = 0;
    HAL_StatusTypeDef status = HAL_OK;

    // Configure motor type in register 20.
    uint8_t type_buf[4] = {motor_type, 0, 0, 0};
    status = HAL_I2C_Mem_Write(&hi2c2, MOTOR_DRIVER_I2C_ADDR, 20,
                              I2C_MEMADD_SIZE_8BIT, type_buf, 4, 100);
    if (status != HAL_OK) {
        return status;
    }
    HAL_Delay(5);

    // Configure encoder polarity in register 21.
    status = HAL_I2C_Mem_Write(&hi2c2, MOTOR_DRIVER_I2C_ADDR, 21,
                              I2C_MEMADD_SIZE_8BIT, &encoder_polarity, 1, 100);
    if (status != HAL_OK) {
        return status;
    }
    HAL_Delay(5);

    return HAL_OK;
}

/**
    * @brief  Set target speeds for all four motor channels.
    * @param  speeds Array of four channel speeds from -100 to 100.
  */
HAL_StatusTypeDef Motor_SetSpeeds(int8_t speeds[4])
{
    // Write four signed speed values to register 51.
    return HAL_I2C_Mem_Write(&hi2c2, MOTOR_DRIVER_I2C_ADDR, 51,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t *)speeds, 4,
                             MOTOR_DRIVER_I2C_TIMEOUT_MS);
}

/**
    * @brief  Read the accumulated pulse counts for all four encoders.
    * @param  total_pulses Output array for the four motor pulse counts.
  */
HAL_StatusTypeDef Motor_ReadEncoders(int32_t total_pulses[4])
{
    // Read 16 bytes from register 60: four 32-bit channel counters.
    return HAL_I2C_Mem_Read(&hi2c2, MOTOR_DRIVER_I2C_ADDR, 60,
                            I2C_MEMADD_SIZE_8BIT, (uint8_t *)total_pulses, 16,
                            MOTOR_DRIVER_I2C_TIMEOUT_MS);
}

/**
    * @brief  Periodically sample encoder data and update motor status.
  */
void Motor_Update_Callback(void)
{
    int32_t current_pulses[4] = {0};
    static uint32_t previous_sample_tick;
    static uint8_t sample_initialized;

    // Read the current accumulated pulse counts over I2C.
    if (Motor_ReadEncoders(current_pulses) == HAL_OK)
    {
        const uint32_t sample_tick = HAL_GetTick();
        const uint32_t elapsed_ms = sample_tick - previous_sample_tick;
        const float elapsed_sec = (float)elapsed_ms / 1000.0f;

        g_motor_status.timestamp = sample_tick;
        for (int i = 0; i < 4; i++)
        {
            // Update cumulative pulses and calculate the change since the last sample.
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
