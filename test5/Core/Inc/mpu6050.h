#ifndef MPU6050_H_
#define MPU6050_H_

#include "stm32f4xx_hal.h"

// MPU6050 I2C Address
#define MPU6050_ADDR 0xD0 // 0x68 << 1

// MPU6050 Registers
#define MPU6050_SMPLRT_DIV_REG 0x19
#define MPU6050_GYRO_CONFIG_REG 0x1B
#define MPU6050_ACCEL_CONFIG_REG 0x1C
#define MPU6050_ACCEL_XOUT_H_REG 0x3B
#define MPU6050_GYRO_XOUT_H_REG 0x43
#define MPU6050_PWR_MGMT_1_REG 0x6B
#define MPU6050_WHO_AM_I_REG 0x75

// Structure to hold sensor data
typedef struct {
    // Raw sensor values
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;

    // Gyroscope offset values for calibration
    float Gx_offset;
    float Gy_offset;
    float Gz_offset;

    // Processed data in physical units
    float Ax; // Acceleration in g's
    float Ay;
    float Az;
    float Gx; // Angular velocity in rad/s
    float Gy;
    float Gz;

} MPU6050_t;


uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);

void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *data);

/**
 * @brief Calibrates the gyroscope by reading a number of samples and averaging them.
 * @note The sensor MUST be stationary during calibration.
 * @param hi2c Pointer to I2C handle.
 * @param data Pointer to MPU6050 data structure to store offsets.
 * @param num_samples Number of samples to average (e.g., 2000).
 */
void MPU6050_Calibrate_Gyro(I2C_HandleTypeDef *hi2c, MPU6050_t* data, uint16_t num_samples);

#endif /* MPU6050_H_ */

