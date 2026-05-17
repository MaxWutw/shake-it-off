#include "mpu6050.h"
#include <string.h>

/* ---------- 私有 helper ---------- */
static HAL_StatusTypeDef reg_write(I2C_HandleTypeDef *hi2c,
                                   uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(hi2c, MPU_ADDR, reg, 1, &val, 1, 100);
}

static HAL_StatusTypeDef reg_read(I2C_HandleTypeDef *hi2c,
                                  uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(hi2c, MPU_ADDR, reg, 1, buf, len, 100);
}

/* ---------- 初始化 ---------- */
HAL_StatusTypeDef MPU6050_Init(MPU6050_t *imu, I2C_HandleTypeDef *hi2c)
{
    uint8_t who = 0;
    /* 確認裝置有回應 */
    if (reg_read(hi2c, MPU_WHO_AM_I, &who, 1) != HAL_OK) return HAL_ERROR;
    if (who == 0xFF) return HAL_ERROR;   /* 完全沒回應 */

    /* 喚醒，選 X gyro 為 clock (比內部 RC 穩定) */
    if (reg_write(hi2c, MPU_PWR_MGMT_1, 0x01) != HAL_OK) return HAL_ERROR;
    HAL_Delay(100);

    /* 取樣率 = 1kHz / (1+7) = 125 Hz */
    reg_write(hi2c, MPU_SMPLRT_DIV, 0x07);

    /* DLPF = 44 Hz  (Week 6 低通濾波，把振動雜訊砍掉) */
    reg_write(hi2c, MPU_CONFIG,    0x03);

    /* Gyro  ±250 deg/s  → sensitivity 131 LSB/(deg/s) */
    reg_write(hi2c, MPU_GYRO_CFG,  0x00);

    /* Accel ±2 g        → sensitivity 16384 LSB/g */
    reg_write(hi2c, MPU_ACCEL_CFG, 0x00);

    /* 清零 offset */
    imu->ax_off = imu->ay_off = imu->az_off = 0.0f;
    imu->gx_off = imu->gy_off = imu->gz_off = 0.0f;

    return HAL_OK;
}

/* ---------- 讀一筆資料 ---------- */
HAL_StatusTypeDef MPU6050_Read(MPU6050_t *imu, I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[14];
    if (reg_read(hi2c, MPU_DATA_START, buf, 14) != HAL_OK) return HAL_ERROR;

    imu->ax_raw = (int16_t)((buf[0]  << 8) | buf[1]);
    imu->ay_raw = (int16_t)((buf[2]  << 8) | buf[3]);
    imu->az_raw = (int16_t)((buf[4]  << 8) | buf[5]);
    /* buf[6~7] = 溫度，略過 */
    imu->gx_raw = (int16_t)((buf[8]  << 8) | buf[9]);
    imu->gy_raw = (int16_t)((buf[10] << 8) | buf[11]);
    imu->gz_raw = (int16_t)((buf[12] << 8) | buf[13]);

    /* 換算成物理單位，扣除校正 offset */
    imu->ax = imu->ax_raw / ACCEL_SENS - imu->ax_off;
    imu->ay = imu->ay_raw / ACCEL_SENS - imu->ay_off;
    imu->az = imu->az_raw / ACCEL_SENS - imu->az_off;
    imu->gx = imu->gx_raw / GYRO_SENS  - imu->gx_off;
    imu->gy = imu->gy_raw / GYRO_SENS  - imu->gy_off;
    imu->gz = imu->gz_raw / GYRO_SENS  - imu->gz_off;

    return HAL_OK;
}

/* ---------- 靜態校正 ---------- */
/* 把底座放平，呼叫這個函式，取 samples 筆平均當 offset  */
void MPU6050_Calibrate(MPU6050_t *imu, I2C_HandleTypeDef *hi2c,
                        uint16_t samples)
{
    /* 先清零既有 offset，才能讓 Read() 拿到真實值 */
    imu->ax_off = imu->ay_off = imu->az_off = 0.0f;
    imu->gx_off = imu->gy_off = imu->gz_off = 0.0f;

    double ax_s=0, ay_s=0, az_s=0;
    double gx_s=0, gy_s=0, gz_s=0;

    for (uint16_t i = 0; i < samples; i++) {
        MPU6050_Read(imu, hi2c);
        ax_s += imu->ax;  ay_s += imu->ay;  az_s += imu->az;
        gx_s += imu->gx;  gy_s += imu->gy;  gz_s += imu->gz;
        HAL_Delay(3);
    }

    imu->ax_off = (float)(ax_s / samples);
    imu->ay_off = (float)(ay_s / samples);
    imu->az_off = (float)(az_s / samples) - 1.0f; /* z 靜止應 = 1g */
    imu->gx_off = (float)(gx_s / samples);
    imu->gy_off = (float)(gy_s / samples);
    imu->gz_off = (float)(gz_s / samples);
}
