/*
 * hiwonder_servo.h
 *
 *  Created on: 23 Sept 2026
 *      Author: tim
 */

#ifndef HIWONDER_SERVO_H_
#define HIWONDER_SERVO_H_

#include "main.h"

#define SERVO_FRAME_HEADER        0x55
#define CMD_SERVO_MOVE_TIME_WRITE 1
#define CMD_SERVO_POS_READ        28
#define SERVO_RX_BUF_SIZE         32

typedef struct {
    volatile uint8_t id;
    volatile int16_t current_position;
    volatile uint8_t rx_complete_flag;
} Servo_Feedback_t;

extern Servo_Feedback_t g_servo_feedback;
extern uint8_t g_servo_rx_buf[SERVO_RX_BUF_SIZE];

void Servo_Init(void);
HAL_StatusTypeDef Servo_SetPosition(uint8_t id, uint16_t position, uint16_t time_ms);
HAL_StatusTypeDef Servo_RequestPosition(uint8_t id);
void Servo_ParseRxData(const uint8_t *buf, uint16_t len);
void Servo_RxEventCallback(uint16_t size);

#endif /* HIWONDER_SERVO_H_ */
