#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"

// MPU6050 Register Addresses
#define MPU6050_ADDR            0xD0 // (0x68 << 1)
#define MPU6050_WHO_AM_I        0x75
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_GYRO_XOUT_H     0x43

// Structure to hold raw sensor data
typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    float Ax, Ay, Az; // In g's

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    float Gx, Gy, Gz; // In degrees/sec

    // ADD THESE LINES for calibration offsets
    float Gyro_X_Bias;
    float Gyro_Y_Bias;
    float Gyro_Z_Bias;

    float Temperature;
} MPU6050_Data_t;


/**
 * @brief Initializes the MPU6050 sensor.
 * @param I2C_Handle: Pointer to the I2C handle.
 * @return HAL_StatusTypeDef: Status of the initialization.
 */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *I2C_Handle);

/**
 * @brief Reads all sensor data (accelerometer, gyroscope).
 * @param I2C_Handle: Pointer to the I2C handle.
 * @param data: Pointer to the MPU6050_Data_t structure to store data.
 */
void MPU6050_Read_All(I2C_HandleTypeDef *I2C_Handle, MPU6050_Data_t *data);


#endif /* MPU6050_H */


