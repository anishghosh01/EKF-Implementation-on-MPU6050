#include "mpu6050.h"
#include <math.h>

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *I2C_Handle) {
    uint8_t check;
    uint8_t data;
    HAL_StatusTypeDef status;

    // Check device ID
    status = HAL_I2C_Mem_Read(I2C_Handle, MPU6050_ADDR, MPU6050_WHO_AM_I, 1, &check, 1, HAL_MAX_DELAY);
    if (status != HAL_OK || check != 0x68) {
        return HAL_ERROR;
    }

    // Wake up sensor
    data = 0;
    status = HAL_I2C_Mem_Write(I2C_Handle, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Set sample rate to 1KHz
    data = 0x07;
    status = HAL_I2C_Mem_Write(I2C_Handle, MPU6050_ADDR, MPU6050_SMPLRT_DIV, 1, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Set accelerometer configuration to +/- 2g
    data = 0x00;
    status = HAL_I2C_Mem_Write(I2C_Handle, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Set gyroscope configuration to +/- 250 degrees/s
    data = 0x00;
    status = HAL_I2C_Mem_Write(I2C_Handle, MPU6050_ADDR, MPU6050_GYRO_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

void MPU6050_Calibrate(I2C_HandleTypeDef *I2C_Handle, MPU6050_Data_t *data) {
    const int num_samples = 2000;
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;

    printf("Calibrating Gyro... Keep the sensor still.\r\n");

    for (int i = 0; i < num_samples; i++) {
        uint8_t buffer[6];
        HAL_I2C_Mem_Read(I2C_Handle, MPU6050_ADDR, MPU6050_GYRO_XOUT_H, 1, buffer, 6, HAL_MAX_DELAY);

        int16_t gyro_x_raw = (int16_t)(buffer[0] << 8 | buffer[1]);
        int16_t gyro_y_raw = (int16_t)(buffer[2] << 8 | buffer[3]);
        int16_t gyro_z_raw = (int16_t)(buffer[4] << 8 | buffer[5]);

        gx_sum += gyro_x_raw / 131.0f;
        gy_sum += gyro_y_raw / 131.0f;
        gz_sum += gyro_z_raw / 131.0f;
        HAL_Delay(3); // Small delay between readings
    }

    data->Gyro_X_Bias = gx_sum / num_samples;
    data->Gyro_Y_Bias = gy_sum / num_samples;
    data->Gyro_Z_Bias = gz_sum / num_samples;

    printf("Calibration complete.\r\n");
    printf("Bias Gx: %.2f, Gy: %.2f, Gz: %.2f\r\n", data->Gyro_X_Bias, data->Gyro_Y_Bias, data->Gyro_Z_Bias);
}


void MPU6050_Read_All(I2C_HandleTypeDef *I2C_Handle, MPU6050_Data_t *data) {
    uint8_t buffer[14];

    HAL_I2C_Mem_Read(I2C_Handle, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1, buffer, 14, HAL_MAX_DELAY);

    // Accelerometer data
    data->Accel_X_RAW = (int16_t)(buffer[0] << 8 | buffer[1]);
    data->Accel_Y_RAW = (int16_t)(buffer[2] << 8 | buffer[3]);
    data->Accel_Z_RAW = (int16_t)(buffer[4] << 8 | buffer[5]);

    // Gyroscope data
    data->Gyro_X_RAW = (int16_t)(buffer[8] << 8 | buffer[9]);
    data->Gyro_Y_RAW = (int16_t)(buffer[10] << 8 | buffer[11]);
    data->Gyro_Z_RAW = (int16_t)(buffer[12] << 8 | buffer[13]);

    // Convert raw values to physical units
    data->Ax = data->Accel_X_RAW / 16384.0f;
    data->Ay = data->Accel_Y_RAW / 16384.0f;
    data->Az = data->Accel_Z_RAW / 16384.0f;

    // Convert and APPLY bias correction
    data->Gx = (data->Gyro_X_RAW / 131.0f) - data->Gyro_X_Bias;
    data->Gy = (data->Gyro_Y_RAW / 131.0f) - data->Gyro_Y_Bias;
    data->Gz = (data->Gyro_Z_RAW / 131.0f) - data->Gyro_Z_Bias;
}


