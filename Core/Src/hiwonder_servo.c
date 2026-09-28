/*
 * hiwonder_servo.c
 *
 *  Created on: 23 Sept 2026
 *      Author: tim
 */

#include "hiwonder_servo.h"
#include "usart.h"
#include <string.h>

Servo_Feedback_t g_servo_feedback = {0};
uint8_t g_servo_rx_buf[SERVO_RX_BUF_SIZE];

// Calculate checksum for Hiwonder bus servo protocol safely
static uint8_t Servo_Checksum(const uint8_t *buf) {
    uint16_t sum = 0;
    uint16_t checksum_index = (uint16_t)buf[3] + 2U;

    for (uint16_t i = 2U; i < checksum_index; ++i) {
        sum += buf[i];
    }
    return (uint8_t)(~sum);
}

// Initialize servo driver buffers and feedback structures
void Servo_Init(void) {
    memset(g_servo_rx_buf, 0, sizeof(g_servo_rx_buf));
    g_servo_feedback.id = 0;
    g_servo_feedback.current_position = 0;
    g_servo_feedback.rx_complete_flag = 0;
}

// Transmit target position and move duration to servo with status check
HAL_StatusTypeDef Servo_SetPosition(uint8_t id, uint16_t position, uint16_t time_ms) {
    static uint8_t tx_buf[10];

    if (huart3.gState == HAL_UART_STATE_BUSY_TX) {
        return HAL_BUSY;
    }

    if (position > 1000) {
        position = 1000;
    }

    tx_buf[0] = SERVO_FRAME_HEADER;
    tx_buf[1] = SERVO_FRAME_HEADER;
    tx_buf[2] = id;
    tx_buf[3] = 7;
    tx_buf[4] = CMD_SERVO_MOVE_TIME_WRITE;
    tx_buf[5] = (uint8_t)(position & 0xFF);
    tx_buf[6] = (uint8_t)((position >> 8) & 0xFF);
    tx_buf[7] = (uint8_t)(time_ms & 0xFF);
    tx_buf[8] = (uint8_t)((time_ms >> 8) & 0xFF);
    tx_buf[9] = Servo_Checksum(tx_buf);

    return HAL_UART_Transmit_DMA(&huart3, tx_buf, 10);
}

// Request current position feedback from servo
HAL_StatusTypeDef Servo_RequestPosition(uint8_t id) {
    static uint8_t tx_buf[6];
    HAL_StatusTypeDef rx_status;

    if (huart3.gState == HAL_UART_STATE_BUSY_TX) {
        return HAL_BUSY;
    }

    // Enable DMA receive-to-idle before transmitting read command
    rx_status = HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_servo_rx_buf, SERVO_RX_BUF_SIZE);
    if (rx_status != HAL_OK && rx_status != HAL_BUSY) {
        return rx_status;
    }

    tx_buf[0] = SERVO_FRAME_HEADER;
    tx_buf[1] = SERVO_FRAME_HEADER;
    tx_buf[2] = id;
    tx_buf[3] = 3;
    tx_buf[4] = CMD_SERVO_POS_READ;
    tx_buf[5] = Servo_Checksum(tx_buf);

    return HAL_UART_Transmit_DMA(&huart3, tx_buf, 6);
}

// Parse incoming telemetry packet with strict bounds and checksum checking
void Servo_ParseRxData(const uint8_t *buf, uint16_t len) {
    if (buf == NULL || len < 8U) return;
    if (buf[0] != SERVO_FRAME_HEADER || buf[1] != SERVO_FRAME_HEADER) return;
    if (buf[3] != 5U) return;
    if (buf[7] != Servo_Checksum(buf)) return;
    if (buf[4] != CMD_SERVO_POS_READ) return;

    g_servo_feedback.id = buf[2];
    g_servo_feedback.current_position = (int16_t)((uint16_t)buf[5] | ((uint16_t)buf[6] << 8));
    g_servo_feedback.rx_complete_flag = 1U;
}

void Servo_RxEventCallback(uint16_t size) {
    Servo_ParseRxData(g_servo_rx_buf, size);

    // Re-arm DMA idle receiver for next packet
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_servo_rx_buf, SERVO_RX_BUF_SIZE);
}
