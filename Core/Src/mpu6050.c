#include "mpu6050.h"
#include "main.h"

#define MPU6050_ADDRESS_0 0xD0U
#define MPU6050_ADDRESS_1 0xD2U
#define MPU6050_DATA_REGISTER 0x3B
#define MPU6050_DATA_LENGTH 14U
#define MPU6050_I2C_TIMEOUT_MS 5U

extern I2C_HandleTypeDef hi2c1;

volatile uint8_t mpu_data_ready = 0;

static uint8_t mpu6050_rx_data[MPU6050_DATA_LENGTH];
static uint16_t mpu6050_address = MPU6050_ADDRESS_0;

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//    if (GPIO_Pin == IMU_ITR_Pin) {
//        mpu_data_ready = 1;
//    }
//}

HAL_StatusTypeDef MPU6050_Read_Data(IMU_Packet_t *packet)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &hi2c1,
        mpu6050_address,
        MPU6050_DATA_REGISTER,
        I2C_MEMADD_SIZE_8BIT,
        mpu6050_rx_data,
        MPU6050_DATA_LENGTH,
        MPU6050_I2C_TIMEOUT_MS);

    if (status != HAL_OK) {
        return status;
    }

    packet->accel_x = (int16_t)((mpu6050_rx_data[0] << 8) | mpu6050_rx_data[1]);
    packet->accel_y = (int16_t)((mpu6050_rx_data[2] << 8) | mpu6050_rx_data[3]);
    packet->accel_z = (int16_t)((mpu6050_rx_data[4] << 8) | mpu6050_rx_data[5]);
    packet->temperature = (int16_t)((mpu6050_rx_data[6] << 8) | mpu6050_rx_data[7]);
    packet->gyro_x = (int16_t)((mpu6050_rx_data[8] << 8) | mpu6050_rx_data[9]);
    packet->gyro_y = (int16_t)((mpu6050_rx_data[10] << 8) | mpu6050_rx_data[11]);
    packet->gyro_z = (int16_t)((mpu6050_rx_data[12] << 8) | mpu6050_rx_data[13]);

    return HAL_OK;
}

uint8_t MPU6050_Init_Sequence(void)
{
    uint8_t check_val = 0;
    uint8_t data;
    HAL_StatusTypeDef status;

    HAL_StatusTypeDef address_0_status = HAL_I2C_IsDeviceReady(
        &hi2c1, MPU6050_ADDRESS_0, 1, MPU6050_I2C_TIMEOUT_MS);
    HAL_StatusTypeDef address_1_status = HAL_I2C_IsDeviceReady(
        &hi2c1, MPU6050_ADDRESS_1, 1, MPU6050_I2C_TIMEOUT_MS);

    if (address_0_status == HAL_OK) {
        mpu6050_address = MPU6050_ADDRESS_0;
    } else if (address_1_status == HAL_OK) {
        mpu6050_address = MPU6050_ADDRESS_1;
    } else {
        return 1;
    }

    data = 0x00;
    status = HAL_I2C_Mem_Write(&hi2c1, mpu6050_address, 0x6B,
                               I2C_MEMADD_SIZE_8BIT, &data, 1,
                               MPU6050_I2C_TIMEOUT_MS);
    if (status != HAL_OK) return 1;

    status = HAL_I2C_Mem_Read(&hi2c1, mpu6050_address, 0x75,
                              I2C_MEMADD_SIZE_8BIT, &check_val, 1,
                              MPU6050_I2C_TIMEOUT_MS);
    if (status != HAL_OK || check_val != 0x68) return 2;

    data = 0x03;
    status = HAL_I2C_Mem_Write(&hi2c1, mpu6050_address, 0x1A,
                               I2C_MEMADD_SIZE_8BIT, &data, 1,
                               MPU6050_I2C_TIMEOUT_MS);
    if (status != HAL_OK) return 3;

    data = 0x13;
    status = HAL_I2C_Mem_Write(&hi2c1, mpu6050_address, 0x19,
                               I2C_MEMADD_SIZE_8BIT, &data, 1,
                               MPU6050_I2C_TIMEOUT_MS);
    if (status != HAL_OK) return 4;

    data = 0x00;
    status = HAL_I2C_Mem_Write(&hi2c1, mpu6050_address, 0x1C,
                               I2C_MEMADD_SIZE_8BIT, &data, 1,
                               MPU6050_I2C_TIMEOUT_MS);
    if (status != HAL_OK) return 5;

    data = 0x01;
    status = HAL_I2C_Mem_Write(&hi2c1, mpu6050_address, 0x38,
                               I2C_MEMADD_SIZE_8BIT, &data, 1,
                               MPU6050_I2C_TIMEOUT_MS);
    if (status != HAL_OK) return 6;

    return 0;
}
