#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"

/* I2C 地址 (AD0 接 GND → 0x68, 左移一位 = 0xD0) */
#define MPU_ADDR        0xD0

/* 暫存器 */
#define MPU_PWR_MGMT_1  0x6B
#define MPU_SMPLRT_DIV  0x19
#define MPU_CONFIG      0x1A
#define MPU_GYRO_CFG    0x1B
#define MPU_ACCEL_CFG   0x1C
#define MPU_DATA_START  0x3B   /* ACCEL_XOUT_H，連續讀 14 bytes */
#define MPU_WHO_AM_I    0x75

/* 換算係數 (±2g, ±250 deg/s) */
#define ACCEL_SENS      16384.0f
#define GYRO_SENS       131.0f

typedef struct {
    /* raw int16 */
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    /* 換算後 (g, deg/s)，已扣除 offset */
    float ax, ay, az;
    float gx, gy, gz;

    /* 校正 offset */
    float ax_off, ay_off, az_off;
    float gx_off, gy_off, gz_off;
} MPU6050_t;

HAL_StatusTypeDef MPU6050_Init      (MPU6050_t *imu, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_Read      (MPU6050_t *imu, I2C_HandleTypeDef *hi2c);
void              MPU6050_Calibrate (MPU6050_t *imu, I2C_HandleTypeDef *hi2c,
                                     uint16_t samples);

#endif
