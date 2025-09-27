#include "mpu6050.h"
#include <math.h>

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (3.1415926535f / 180.0f)
#endif

uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t check;
    uint8_t data;

    // Check device ID
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_WHO_AM_I_REG, 1, &check, 1, HAL_MAX_DELAY);

    if (check == 0x68)  // 0x68 is the default WHO_AM_I value
    {
        // Power management register 0x6B - reset the device
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1_REG, 1, &data, 1, HAL_MAX_DELAY);

        // Set sample rate to 1KHz
        data = 0x07;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_SMPLRT_DIV_REG, 1, &data, 1, HAL_MAX_DELAY);

        // Set accelerometer configuration to ±2g
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_ACCEL_CONFIG_REG, 1, &data, 1, HAL_MAX_DELAY);

        // Set gyroscope configuration to ±250 dps
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_GYRO_CONFIG_REG, 1, &data, 1, HAL_MAX_DELAY);
        return 0;
    }
    return 1;
}

void MPU6050_Calibrate_Gyro(I2C_HandleTypeDef *hi2c, MPU6050_t* data, uint16_t num_samples) {
    int16_t gyro_raw[3];
    long gyro_sum[3] = {0, 0, 0};
    uint8_t raw_data[6];

    for (uint16_t i = 0; i < num_samples; i++) {
        HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_GYRO_XOUT_H_REG, 1, raw_data, 6, HAL_MAX_DELAY);
        gyro_raw[0] = (int16_t)(raw_data[0] << 8 | raw_data[1]);
        gyro_raw[1] = (int16_t)(raw_data[2] << 8 | raw_data[3]);
        gyro_raw[2] = (int16_t)(raw_data[4] << 8 | raw_data[5]);

        gyro_sum[0] += gyro_raw[0];
        gyro_sum[1] += gyro_raw[1];
        gyro_sum[2] += gyro_raw[2];
        HAL_Delay(2); // Small delay between reads
    }

    data->Gx_offset = (float)gyro_sum[0] / num_samples;
    data->Gy_offset = (float)gyro_sum[1] / num_samples;
    data->Gz_offset = (float)gyro_sum[2] / num_samples;
}


void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *data)
{
    uint8_t Rec_Data[14];

    // Read 14 bytes of data starting from ACCEL_XOUT_H register
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H_REG, 1, Rec_Data, 14, HAL_MAX_DELAY);

    // Accelerometer raw values
    data->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    data->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    data->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // Gyroscope raw values
    data->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    data->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    data->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    // --- Apply Calibration Offsets ---
    float gx_calibrated = data->Gyro_X_RAW - data->Gx_offset;
    float gy_calibrated = data->Gyro_Y_RAW - data->Gy_offset;
    float gz_calibrated = data->Gyro_Z_RAW - data->Gz_offset;

    // Convert raw values to physical units
    data->Ax = data->Accel_X_RAW / 16384.0f;
    data->Ay = data->Accel_Y_RAW / 16384.0f;
    data->Az = data->Accel_Z_RAW / 16384.0f;

    // Sensitivity for ±250 dps is 131.0 LSB/dps
    data->Gx = gx_calibrated / 131.0f * DEG_TO_RAD;
    data->Gy = gy_calibrated / 131.0f * DEG_TO_RAD;
    data->Gz = gz_calibrated / 131.0f * DEG_TO_RAD;
}

