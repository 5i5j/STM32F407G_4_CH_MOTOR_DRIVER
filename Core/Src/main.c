/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for production deployment
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "mpu6050.h"
#include "motor_driver.h"
#include "telemetry.h"
#include "hiwonder_servo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CMD_RX_BUF_SIZE 64   // Expanded RX buffer for DMA ReceiveToIdle stability
#define COMM_TIMEOUT_MS 500  // Safety timeout: stop motors if no RPi cmd for 500ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
IMU_Packet_t imu_packet;
Chassis_Telemetry_Packet_t chassis_packet;

volatile uint8_t motor_timer_flag = 0;

// Dynamic control targets updated by RPi4B commands
volatile uint16_t g_steering_target = 500;
volatile int8_t g_target_motor_speeds[4] = {0, 0, 0, 0};

// UART Command RX buffer & timestamp tracking
uint8_t g_cmd_rx_buf[CMD_RX_BUF_SIZE];
volatile uint32_t g_last_cmd_timestamp = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void I2C1_Force_Reset(void);
void Process_RPi_Command(const uint8_t *buf, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Parse incoming UART command packet from RPi4B with sliding window header search
void Process_RPi_Command(const uint8_t *buf, uint16_t len) {
    if (buf == NULL || len < 10) return;

    // Slide window across buffer to find valid header 0xAA 0x55
    for (uint16_t i = 0; i <= len - 10; i++) {
        if (buf[i] == 0xAA && buf[i + 1] == 0x55) {
            // Calculate XOR checksum for 9-byte payload (header1 to motor_speeds[3])
            uint8_t checksum = 0;
            for (uint16_t j = 0; j < 9; j++) {
                checksum ^= buf[i + j];
            }

            // Verify checksum against the 10th byte
            if (checksum == buf[i + 9]) {
                // Parse steering target (Little-Endian uint16_t)
                uint16_t steering = (uint16_t)buf[i + 3] | ((uint16_t)buf[i + 4] << 8);

                g_steering_target = steering;
                g_target_motor_speeds[0] = (int8_t)buf[i + 5];
                g_target_motor_speeds[1] = (int8_t)buf[i + 6];
                g_target_motor_speeds[2] = (int8_t)buf[i + 7];
                g_target_motor_speeds[3] = (int8_t)buf[i + 8];

                // Refresh communication watchdog timestamp
                g_last_cmd_timestamp = HAL_GetTick();
                break; // Successfully processed a valid frame
            }
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration */
  HAL_Init();
  SystemClock_Config();

  /* Initialize peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  // 1. Initialize bus servo interface
  Servo_Init();
  Servo_SetPosition(1, 500, 500); // Set steering to center position initially
  HAL_Delay(500);

  // 2. Initialize MPU6050
  uint8_t imu_init_result = MPU6050_Init_Sequence();
  (void)imu_init_result;

  imu_packet.header1 = 0xAA;
  imu_packet.header2 = 0x55;
  imu_packet.robot_id = 0x01;

  chassis_packet.header1 = 0xAA;
  chassis_packet.header2 = 0x55;
  chassis_packet.robot_id = 0x01;

  // 3. Initialize motor driver
  HAL_Delay(200);
  if (Motor_Init() != HAL_OK) {
      Error_Handler();
  }
  Motor_SetSpeeds((int8_t[4]){0, 0, 0, 0}); // Keep motors idle on boot

  // Refresh command timestamp to prevent immediate watchdog trigger
  g_last_cmd_timestamp = HAL_GetTick();

  // 4. Start USART2 DMA receive to idle for incoming commands from RPi4B
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_cmd_rx_buf, CMD_RX_BUF_SIZE);

  // 5. Start TIM6 timer for 50Hz control loop
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    g_steering_target = 600;
    g_target_motor_speeds[0] = 50;
    g_target_motor_speeds[2] = 50;
    g_last_cmd_timestamp = HAL_GetTick();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (motor_timer_flag) {
      motor_timer_flag = 0;

      // --- Safety Check: Watchdog timeout for RPi4B communication ---
      if (HAL_GetTick() - g_last_cmd_timestamp > COMM_TIMEOUT_MS) {
          // Stop all motors if RPi connection drops or freezes
          int8_t zero_speeds[4] = {0, 0, 0, 0};
          Motor_SetSpeeds(zero_speeds);
      } else {
          // Apply target motor speeds received from RPi4B
          Motor_SetSpeeds((int8_t *)g_target_motor_speeds);
      }

      // 1. Update encoder readings and speed calculations
      Motor_Update_Callback();
      chassis_packet.timestamp = HAL_GetTick();

      for (int i = 0; i < 4; i++) {
        chassis_packet.encoder[i] = g_motor_status.encoder[i];
        chassis_packet.speed_rpm[i] = g_motor_status.speed_rpm[i];
      }

      // 2. Read IMU data (I2C1)
      if (MPU6050_Read_Data(&imu_packet) == HAL_OK) {
        chassis_packet.accel_x = imu_packet.accel_x;
        chassis_packet.accel_y = imu_packet.accel_y;
        chassis_packet.accel_z = imu_packet.accel_z;
        chassis_packet.gyro_x = imu_packet.gyro_x;
        chassis_packet.gyro_y = imu_packet.gyro_y;
        chassis_packet.gyro_z = imu_packet.gyro_z;
      } else {
        chassis_packet.accel_x = 0;
        chassis_packet.accel_y = 0;
        chassis_packet.accel_z = 0;
        chassis_packet.gyro_x = 0;
        chassis_packet.gyro_y = 0;
        chassis_packet.gyro_z = 0;

        static uint8_t reset_attempts = 0;
        if (++reset_attempts >= 50) {
          reset_attempts = 0;
          I2C1_Force_Reset();
        }
      }

      // 3. Update steering servo angle via USART3
      Servo_SetPosition(1, g_steering_target, 20);

      // 4. Calculate checksum for telemetry frame
      uint8_t *packet_bytes = (uint8_t *)&chassis_packet;
      uint8_t checksum = 0;
      for (size_t i = 0; i < sizeof(Chassis_Telemetry_Packet_t) - 1; i++) {
        checksum ^= packet_bytes[i];
      }
      chassis_packet.checksum = checksum;

      // 5. Send telemetry data to RPi4B via USART2 DMA
      if (huart2.gState == HAL_UART_STATE_READY) {
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&chassis_packet, sizeof(Chassis_Telemetry_Packet_t));
      }
    }
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
    motor_timer_flag = 1;
  }
}

// Global UART Receive Event Callback
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART2) {
        // Parse incoming command from RPi4B
        Process_RPi_Command(g_cmd_rx_buf, Size);

        // Re-arm DMA receive-to-idle for USART2
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_cmd_rx_buf, CMD_RX_BUF_SIZE);
    } else if (huart->Instance == USART3) {
        Servo_RxEventCallback(Size);
    }
}

void I2C1_Force_Reset(void)
{
  GPIO_InitTypeDef gpio_init = {0};

  HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

  gpio_init.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio_init);

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
  uint32_t start_cycles = DWT->CYCCNT;
  while ((DWT->CYCCNT - start_cycles) < (SystemCoreClock / 200000U)) {
  }

  for (uint8_t pulse = 0; pulse < 9U &&
     HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET; ++pulse) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    start_cycles = DWT->CYCCNT;
    while ((DWT->CYCCNT - start_cycles) < (SystemCoreClock / 200000U)) {
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    start_cycles = DWT->CYCCNT;
    while ((DWT->CYCCNT - start_cycles) < (SystemCoreClock / 200000U)) {
    }
  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  start_cycles = DWT->CYCCNT;
  while ((DWT->CYCCNT - start_cycles) < (SystemCoreClock / 200000U)) {
  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  start_cycles = DWT->CYCCNT;
  while ((DWT->CYCCNT - start_cycles) < (SystemCoreClock / 200000U)) {
  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

    MX_I2C1_Init();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // 清除溢出错误和帧错误标志
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);

        // 重新激活 USART2 DMA 接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_cmd_rx_buf, CMD_RX_BUF_SIZE);
    }
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
