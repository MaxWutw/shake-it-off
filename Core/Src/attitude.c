#include "attitude.h"
#include <math.h>

#define R2D  (180.0f / 3.14159265f)

void Attitude_Init(Attitude_t *att, float alpha, float dt)
{
    att->pitch = 0.0f;
    att->roll  = 0.0f;
    att->alpha = alpha;
    att->dt    = dt;
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
	/*
    float pitch_acc = atan2f(-imu->ax,
                              sqrtf(imu->ay * imu->ay + imu->az * imu->az))
                      * R2D;
    float roll_acc  = atan2f(imu->ay, imu->az) * R2D;
	*/
	/* 軸向對調：原本的 roll 公式才是 pitch */
    float pitch_acc = atan2f(imu->ay, imu->az) * R2D;
    
    /* 軸向對調 + 反號：原本的 pitch 公式給 roll，加負號 */
    float roll_acc  = -atan2f(-imu->ax,
                              sqrtf(imu->ay * imu->ay + imu->az * imu->az))
                      * R2D;

    /* --- 互補濾波 --- */
    att->pitch = att->alpha * (att->pitch + imu->gy * att->dt)
               + (1.0f - att->alpha) * pitch_acc;

    att->roll  = att->alpha * (att->roll  + imu->gx * att->dt)
               + (1.0f - att->alpha) * roll_acc;
}
