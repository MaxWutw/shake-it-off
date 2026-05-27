#include "attitude.h"
#include <math.h>

#define R2D  (180.0f / 3.14159265f)

static void calc_instant_angles(const MPU6050_t *imu,
                                float *pitch_acc,
                                float *roll_acc)
{
    *pitch_acc = atan2f(imu->ay, imu->az) * R2D;
    *roll_acc  = -atan2f(-imu->ax,
                         sqrtf(imu->ay * imu->ay + imu->az * imu->az)) * R2D;
}

void Attitude_Init(Attitude_t *att, float alpha, float dt)
{
    att->pitch = 0.0f;
    att->roll  = 0.0f;
    att->raw_pitch = 0.0f;
    att->raw_roll  = 0.0f;
    att->alpha = alpha;
    att->dt    = dt;
    att->pitch_zero  = 0.0f;
    att->roll_zero   = 0.0f;
    att->pitch_scale = 1.0f;
    att->roll_scale  = 1.0f;
}

void Attitude_SetCalibration(Attitude_t *att,
                             float pitch_zero, float pitch_scale,
                             float roll_zero, float roll_scale)
{
    att->pitch_zero  = pitch_zero;
    att->roll_zero   = roll_zero;
    att->pitch_scale = pitch_scale;
    att->roll_scale  = roll_scale;

    att->raw_pitch = pitch_zero;
    att->raw_roll  = roll_zero;
    att->pitch = 0.0f;
    att->roll  = 0.0f;
}

void Attitude_GetInstantAngles(const MPU6050_t *imu,
                               float *pitch_deg, float *roll_deg)
{
    calc_instant_angles(imu, pitch_deg, roll_deg);
}

/*
 * 互補濾波公式 (對應 Week 6-7 Fourier)：
 *
 *   θ_k = α·(θ_{k-1} + ω·Δt) + (1-α)·θ_acc
 *
 * α = 0.98 → cutoff ≈ 0.65 Hz
 *   低於 0.65 Hz：信任加速度（補漂移）
 *   高於 0.65 Hz：信任陀螺儀（抗震動）
 *
 * 加速度計靜態角 (晶片朝上，Z+ 對重力)：
 *   pitch_acc = atan2(-ax, sqrt(ay²+az²))
 *   roll_acc  = atan2( ay, az)
 *
 * ⚠ gyro 軸對應：gy → pitch 積分, gx → roll 積分
 *   若傾斜方向與角度符號相反，把對應項加負號
 */
void Attitude_Update(Attitude_t *att, MPU6050_t *imu)
{
    /* --- 加速度計算靜態角 --- */
	float pitch_acc = 0.0f;
	float roll_acc  = 0.0f;
	calc_instant_angles(imu, &pitch_acc, &roll_acc);

    /* --- 互補濾波 --- */
    att->raw_pitch = att->alpha * (att->raw_pitch + imu->gy * att->dt)
                   + (1.0f - att->alpha) * pitch_acc;

    att->raw_roll  = att->alpha * (att->raw_roll + imu->gx * att->dt)
                   + (1.0f - att->alpha) * roll_acc;

    att->pitch = (att->raw_pitch - att->pitch_zero) * att->pitch_scale;
    att->roll  = (att->raw_roll  - att->roll_zero)  * att->roll_scale;
}
