#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "mpu6050.h"

typedef struct {
    float pitch;   /* 前後傾 (deg)，前緣朝上為正 */
    float roll;    /* 左右傾 (deg)，右緣朝上為正 */
    float raw_pitch;
    float raw_roll;
    float alpha;   /* 互補濾波係數 (建議 0.98) */
    float dt;      /* 控制週期 (秒) */
    float pitch_zero;
    float roll_zero;
    float pitch_scale;
    float roll_scale;
} Attitude_t;

void Attitude_Init   (Attitude_t *att, float alpha, float dt);
void Attitude_Update (Attitude_t *att, MPU6050_t *imu);
void Attitude_SetCalibration(Attitude_t *att,
                             float pitch_zero, float pitch_scale,
                             float roll_zero, float roll_scale);
void Attitude_GetInstantAngles(const MPU6050_t *imu,
                               float *pitch_deg, float *roll_deg);

#endif
