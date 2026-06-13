/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : INSTRUMENTED VERSION — 系統分析用
  ******************************************************************************
  * 在 improved 版本基礎上加入：
  *   A. DWT Cycle Counter 精確計時 (微秒級)
  *   B. 各子任務 (I2C讀取, 濾波, PID, 幾何, Servo) 分段計時
  *   C. Step Response 偵測 + settling time 自動量測
  *   D. 高速資料記錄模式 (每個控制週期都輸出)
  *   E. Deadline miss 計數
  *
  * USB CDC 輸出格式 (高速模式):
  *   DATA,<tick_ms>,<pitch>,<roll>,<target_pitch>,<target_roll>,
  *        <gx>,<gy>,<dt_us>,<t_imu>,<t_filt>,<t_pid>,<t_geom>,<t_servo>,
  *        <deadline_miss>,<settling_state>
  *
  * settling_state: 0=idle, 1=disturbed(偵測到擾動), 2=settling, 3=settled
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "mpu6050.h"
#include "attitude.h"
#include "servo.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
MPU6050_t  imu;
Attitude_t att;
Servo_t    s1, s2, s3, s4;

char       log_buf[512];
uint32_t   t_ctrl = 0;
uint32_t   t_log  = 0;

#define CTRL_MS   2       // 500Hz 控制週期
#define LOG_MS  100       // 普通 log 模式: 10Hz

// ═══════════════════════════════════════════════
//  分析模式控制
// ═══════════════════════════════════════════════
#define ANALYSIS_MODE       1    // 1=開啟高速記錄, 0=普通模式
#define HIGHSPEED_LOG_MS    2    // 高速模式: 每個控制週期都記錄 (= CTRL_MS)
#define TIMING_PROFILE      1    // 1=開啟 DWT 計時分析

// Step response 偵測參數
#define DISTURBANCE_THRESHOLD_DEG  3.0f   // 超過此角度視為「被擾動」
#define SETTLED_THRESHOLD_DEG      0.5f   // 低於此角度視為「回穩」
#define SETTLED_HOLD_LOOPS         50     // 連續 N 個 loop 都低於門檻才算「settled」

// ═══════════════════════════════════════════════
//  PID 增益 (同 improved 版)
// ═══════════════════════════════════════════════
float target_mech_pitch = 0.0f;
float target_mech_roll  = 0.0f;
float pitch_prev_err  = 0.0f;
float roll_prev_err   = 0.0f;
float pitch_d_err_lpf = 0.0f;
float roll_d_err_lpf  = 0.0f;

float last_s1_angle = 90.0f;
float last_s2_angle = 90.0f;
float last_s3_angle = 90.0f;
float last_s4_angle = 90.0f;

#define KI  0.018f
#define KP  0.10f
#define KP_DIRECT  0.08f
#define KD_GYRO    0.003f

#define MAX_MECH_ANGLE  30.0f
#define MIN_MECH_ANGLE -30.0f

#define ERR_SOFT_DEADBAND_DEG  0.15f
#define DERR_LPF_ALPHA   0.80f
#define TARGET_LEAK_GAIN 0.003f
#define MAX_STEP_PER_LOOP  1.0f

float base_alpha_R = 0.0f;
float base_alpha_L = 0.0f;
uint32_t last_ctrl_tick = 0;

// ═══════════════════════════════════════════════
//  分析用計時與統計變數
// ═══════════════════════════════════════════════
#if TIMING_PROFILE
volatile uint32_t t_imu_us  = 0;   // I2C 讀取時間
volatile uint32_t t_filt_us = 0;   // 互補濾波時間
volatile uint32_t t_pid_us  = 0;   // PID 計算時間
volatile uint32_t t_geom_us = 0;   // 幾何計算時間
volatile uint32_t t_servo_us = 0;  // Servo PWM 更新時間
volatile uint32_t t_total_us = 0;  // 整個 control loop 時間
#endif

// Deadline 分析
volatile uint32_t deadline_miss_count = 0;
volatile uint32_t total_loop_count    = 0;
volatile uint32_t worst_case_us       = 0;

// Step response 分析
typedef enum {
    SR_IDLE       = 0,   // 平穩狀態
    SR_DISTURBED  = 1,   // 剛偵測到擾動
    SR_SETTLING   = 2,   // 正在回穩中
    SR_SETTLED    = 3    // 已回穩
} SettlingState;

SettlingState settling_state = SR_IDLE;
uint32_t disturbance_tick   = 0;    // 擾動開始時間
uint32_t settled_tick       = 0;    // 回穩時間
float    peak_disturbance   = 0.0f; // 最大偏移角度
uint32_t settle_hold_counter = 0;   // 穩定計數器
float    settling_time_ms   = 0.0f; // 結算的 settling time
uint32_t response_count     = 0;    // 第幾次 step response

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN 0 */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ═══════════════════════════════════════════════
//  DWT Cycle Counter (Cortex-M4 精確計時)
// ═══════════════════════════════════════════════
//  DWT->CYCCNT 是 32-bit 計數器, 每個 CPU clock 加 1
//  SYSCLK = 84MHz → 1 tick = ~11.9ns, 換算 μs 除以 84

static inline void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // 啟用 trace
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;              // 啟用 cycle counter
}

static inline uint32_t DWT_GetCycles(void)
{
    return DWT->CYCCNT;
}

// 將 cycle count 差值轉換為微秒 (SYSCLK = 84MHz)
static inline uint32_t Cycles_to_us(uint32_t cycles)
{
    return cycles / 84;
}

// ═══════════════════════════════════════════════
//  Step Response 狀態機
// ═══════════════════════════════════════════════
static void StepResponse_Update(float pitch, float roll, uint32_t now_ms)
{
    float magnitude = sqrtf(pitch * pitch + roll * roll);

    switch (settling_state)
    {
    case SR_IDLE:
        if (magnitude > DISTURBANCE_THRESHOLD_DEG) {
            settling_state = SR_DISTURBED;
            disturbance_tick = now_ms;
            peak_disturbance = magnitude;
            settle_hold_counter = 0;
            response_count++;
        }
        break;

    case SR_DISTURBED:
        // 等待角度開始下降 → 進入 settling
        if (magnitude > peak_disturbance) {
            peak_disturbance = magnitude;
        }
        settling_state = SR_SETTLING;
        break;

    case SR_SETTLING:
        if (magnitude > peak_disturbance) {
            peak_disturbance = magnitude;
        }
        if (magnitude < SETTLED_THRESHOLD_DEG) {
            settle_hold_counter++;
            if (settle_hold_counter >= SETTLED_HOLD_LOOPS) {
                settling_state = SR_SETTLED;
                settled_tick = now_ms;
                settling_time_ms = (float)(settled_tick - disturbance_tick);
            }
        } else {
            settle_hold_counter = 0;
        }
        break;

    case SR_SETTLED:
        // 印出結果後回到 IDLE 等待下一次擾動
        {
            int n = sprintf(log_buf,
                "STEP_RESP,%lu,peak=%.2f,settle=%.1fms,resp#%lu\r\n",
                (unsigned long)disturbance_tick,
                peak_disturbance,
                settling_time_ms,
                (unsigned long)response_count);
            CDC_Transmit_FS((uint8_t*)log_buf, n);
            HAL_Delay(2);  // 確保 USB 送出
        }
        settling_state = SR_IDLE;
        break;
    }
}

// ═══════════════════════════════════════════════
//  既有工具函式 (同 improved 版)
// ═══════════════════════════════════════════════

typedef struct {
    float length_L;
    float length_R;
    float angle_L;
    float angle_R;
} ActuatorTarget;

#define CALIB_SETTLE_MS   1200U
#define CALIB_SAMPLES     120U
#define SERVO_UPDATE_DEADBAND 0.3f
#define CALIB_MIN_SPAN_DEG 5.0f

static void SetServoPairAngle(Servo_t *first, Servo_t *second, float angle_deg)
{
  Servo_SetAngle(first, angle_deg);
  Servo_SetAngle(second, angle_deg);
}

static float SampleAxisAngle(uint8_t use_pitch_axis, uint16_t samples)
{
  float sum = 0.0f;
  uint16_t count = 0;
  for (uint16_t i = 0; i < samples; i++) {
    if (MPU6050_Read(&imu, &hi2c1) != HAL_OK) { HAL_Delay(5); continue; }
    float pitch_deg = 0.0f, roll_deg = 0.0f;
    Attitude_GetInstantAngles(&imu, &pitch_deg, &roll_deg);
    sum += use_pitch_axis ? pitch_deg : roll_deg;
    count++;
    HAL_Delay(5);
  }
  return (count == 0) ? 0.0f : sum / (float)count;
}

static void CalibrateAxisFromServos(Servo_t *servo_a, Servo_t *servo_b,
                  uint8_t use_pitch_axis,
                  float *axis_zero, float *axis_scale)
{
  float measured_180 = 0.0f, measured_0 = 0.0f;
  SetServoPairAngle(servo_a, servo_b, 90.0f);
  HAL_Delay(CALIB_SETTLE_MS);
  SetServoPairAngle(servo_a, servo_b, 180.0f);
  HAL_Delay(CALIB_SETTLE_MS);
  measured_180 = SampleAxisAngle(use_pitch_axis, CALIB_SAMPLES);
  SetServoPairAngle(servo_a, servo_b, 0.0f);
  HAL_Delay(CALIB_SETTLE_MS);
  measured_0 = SampleAxisAngle(use_pitch_axis, CALIB_SAMPLES);

  float span = measured_180 - measured_0;
  int n = sprintf(log_buf,
                  use_pitch_axis
                  ? "[CALIB] pitch sweep 180=%.3f 0=%.3f span=%.3f\r\n"
                  : "[CALIB] roll sweep 180=%.3f 0=%.3f span=%.3f\r\n",
                  measured_180, measured_0, span);
  CDC_Transmit_FS((uint8_t*)log_buf, n);

  if (fabsf(span) < CALIB_MIN_SPAN_DEG) {
    *axis_zero = 0.5f * (measured_180 + measured_0);
    *axis_scale = 1.0f;
    return;
  }
  *axis_zero = 0.5f * (measured_180 + measured_0);
  *axis_scale = (span < 0.0f) ? -1.0f : 1.0f;
}

static float adaptive_step_limit(float error_deg)
{
  float abs_error = fabsf(error_deg);
  if (abs_error < 1.5f)  return 0.15f;
  if (abs_error < 5.0f)  return 0.35f;
  if (abs_error < 12.0f) return 0.60f;
  return MAX_STEP_PER_LOOP;
}

static float apply_soft_deadband(float err_deg, float deadband_deg)
{
  float abs_err = fabsf(err_deg);
  if (abs_err <= deadband_deg) return 0.0f;
  return (err_deg > 0.0f) ? (err_deg - deadband_deg) : (err_deg + deadband_deg);
}

ActuatorTarget calculateTargets(float theta_deg, float a, float b, float k, float C) {
    ActuatorTarget target;
    float theta_rad = theta_deg * M_PI / 180.0f;
    float sin_t = sinf(theta_rad);
    float cos_t = cosf(theta_rad);

    float dx_R = -a * sin_t + k * cos_t - k;
    float dy_R = b + a * cos_t + k * sin_t;
    target.length_R = sqrtf(dx_R * dx_R + dy_R * dy_R);

    float dx_L = -a * sin_t - k * cos_t + k;
    float dy_L = b + a * cos_t - k * sin_t;
    target.length_L = sqrtf(dx_L * dx_L + dy_L * dy_L);

    target.angle_R = (360.0f * target.length_R) / C;
    target.angle_L = (360.0f * target.length_L) / C;
    return target;
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  HAL_Delay(3000);
  int n;

  // ★ 初始化 DWT cycle counter
  DWT_Init();

  Servo_Init(&s1, &htim2, TIM_CHANNEL_1);
  Servo_Init(&s2, &htim2, TIM_CHANNEL_2);
  Servo_Init(&s3, &htim2, TIM_CHANNEL_3);
  Servo_Init(&s4, &htim2, TIM_CHANNEL_4);
  s1.reversed = 0;
  s2.reversed = 0;
  s3.reversed = 1;
  s4.reversed = 1;

  ActuatorTarget init_cmd = calculateTargets(0.0f, 2.2f, 4.5f, 11.4f, 5.1f);
  base_alpha_R = init_cmd.angle_R;
  base_alpha_L = init_cmd.angle_L;

  Servo_SetAngle(&s1, 90.0f);
  Servo_SetAngle(&s2, 90.0f);
  Servo_SetAngle(&s3, 90.0f);
  Servo_SetAngle(&s4, 90.0f);
  HAL_Delay(1000);

  Attitude_Init(&att, 0.96f, CTRL_MS / 1000.0f);
  MPU6050_Init(&imu, &hi2c1);
  HAL_Delay(500);

  n = sprintf(log_buf, "[CALIB] Sweeping pitch axis...\r\n");
  CDC_Transmit_FS((uint8_t*)log_buf, n);
  float pitch_zero = 0.0f, pitch_scale = 1.0f;
  CalibrateAxisFromServos(&s1, &s4, 1U, &pitch_zero, &pitch_scale);
  n = sprintf(log_buf, "[CALIB] Pitch zero=%.3f scale=%.3f\r\n", pitch_zero, pitch_scale);
  CDC_Transmit_FS((uint8_t*)log_buf, n);

  n = sprintf(log_buf, "[CALIB] Sweeping roll axis...\r\n");
  CDC_Transmit_FS((uint8_t*)log_buf, n);
  float roll_zero = 0.0f, roll_scale = 1.0f;
  CalibrateAxisFromServos(&s2, &s3, 0U, &roll_zero, &roll_scale);
  n = sprintf(log_buf, "[CALIB] Roll zero=%.3f scale=%.3f\r\n", roll_zero, roll_scale);
  CDC_Transmit_FS((uint8_t*)log_buf, n);

  Attitude_SetCalibration(&att, pitch_zero, pitch_scale, roll_zero, roll_scale);
  SetServoPairAngle(&s1, &s4, 90.0f);
  SetServoPairAngle(&s2, &s3, 90.0f);
  HAL_Delay(500);

  // ★ 印出 CSV header
#if ANALYSIS_MODE
  n = sprintf(log_buf,
    "DATA_HEADER,tick_ms,pitch,roll,tgt_pitch,tgt_roll,"
    "gx,gy,dt_us,t_imu,t_filt,t_pid,t_geom,t_servo,"
    "deadline_miss,settling_state\r\n");
  CDC_Transmit_FS((uint8_t*)log_buf, n);
  HAL_Delay(5);
#endif

  n = sprintf(log_buf, "[READY] INSTRUMENTED mode, CTRL=%dms\r\n", CTRL_MS);
  CDC_Transmit_FS((uint8_t*)log_buf, n);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  t_ctrl = HAL_GetTick();
  t_log  = HAL_GetTick();
  last_ctrl_tick = HAL_GetTick();
  /* USER CODE END 2 */

  while (1)
  {
    uint32_t now = HAL_GetTick();

    if (now - t_ctrl >= CTRL_MS) {
        t_ctrl = now;
        total_loop_count++;

        // ═══ 實際 dt 測量 ═══
        float actual_dt = (now - last_ctrl_tick) / 1000.0f;
        last_ctrl_tick = now;
        if (actual_dt < 0.0005f) actual_dt = 0.002f;
        if (actual_dt > 0.02f)   actual_dt = 0.005f;
        att.dt = actual_dt;
        uint32_t dt_us = (uint32_t)(actual_dt * 1000000.0f);

        uint32_t cyc_total_start = DWT_GetCycles();

        // ═══════════════════════════
        //  Task 1: IMU 讀取 (I2C)
        // ═══════════════════════════
        uint32_t cyc0 = DWT_GetCycles();
        MPU6050_Read(&imu, &hi2c1);
        uint32_t cyc1 = DWT_GetCycles();
        t_imu_us = Cycles_to_us(cyc1 - cyc0);

        // ═══════════════════════════
        //  Task 2: 互補濾波
        // ═══════════════════════════
        Attitude_Update(&att, &imu);
        uint32_t cyc2 = DWT_GetCycles();
        t_filt_us = Cycles_to_us(cyc2 - cyc1);

        // ═══════════════════════════
        //  Task 3: PID 控制計算
        // ═══════════════════════════
        float err_pitch_raw = 0.0f - att.pitch;
        float err_roll_raw  = 0.0f - att.roll;

        float err_pitch = apply_soft_deadband(err_pitch_raw, ERR_SOFT_DEADBAND_DEG);
        float err_roll  = apply_soft_deadband(err_roll_raw, ERR_SOFT_DEADBAND_DEG);

        float d_pitch = err_pitch - pitch_prev_err;
        float d_roll  = err_roll  - roll_prev_err;

        pitch_d_err_lpf = (DERR_LPF_ALPHA * pitch_d_err_lpf)
                        + ((1.0f - DERR_LPF_ALPHA) * d_pitch);
        roll_d_err_lpf  = (DERR_LPF_ALPHA * roll_d_err_lpf)
                        + ((1.0f - DERR_LPF_ALPHA) * d_roll);

        float gyro_d_pitch = -KD_GYRO * imu.gy;
        float gyro_d_roll  = -KD_GYRO * imu.gx;

        float delta_pitch = (KP_DIRECT * err_pitch)
                          + (KI * err_pitch)
                          + (KP * pitch_d_err_lpf)
                          + gyro_d_pitch
                          - (TARGET_LEAK_GAIN * target_mech_pitch);

        float delta_roll  = (KP_DIRECT * err_roll)
                          + (KI * err_roll)
                          + (KP * roll_d_err_lpf)
                          + gyro_d_roll
                          - (TARGET_LEAK_GAIN * target_mech_roll);

        float pitch_step_limit = adaptive_step_limit(err_pitch);
        float roll_step_limit  = adaptive_step_limit(err_roll);

        if (delta_pitch > pitch_step_limit) delta_pitch = pitch_step_limit;
        if (delta_pitch < -pitch_step_limit) delta_pitch = -pitch_step_limit;
        if (delta_roll > roll_step_limit) delta_roll = roll_step_limit;
        if (delta_roll < -roll_step_limit) delta_roll = -roll_step_limit;

        pitch_prev_err = err_pitch;
        roll_prev_err  = err_roll;

        float next_mech_pitch = target_mech_pitch + delta_pitch;
        float next_mech_roll  = target_mech_roll  + delta_roll;

        if(next_mech_pitch > MAX_MECH_ANGLE) next_mech_pitch = MAX_MECH_ANGLE;
        if(next_mech_pitch < MIN_MECH_ANGLE) next_mech_pitch = MIN_MECH_ANGLE;
        if(next_mech_roll > MAX_MECH_ANGLE) next_mech_roll = MAX_MECH_ANGLE;
        if(next_mech_roll < MIN_MECH_ANGLE) next_mech_roll = MIN_MECH_ANGLE;

        uint32_t cyc3 = DWT_GetCycles();
        t_pid_us = Cycles_to_us(cyc3 - cyc2);

        // ═══════════════════════════
        //  Task 4: 幾何計算 (cable length)
        // ═══════════════════════════
        float a = 2.2f, b = 4.5f, k = 11.4f, C_val = 5.1f;

        ActuatorTarget pitch_cmd = calculateTargets(next_mech_pitch, a, b, k, C_val);
        float s1_angle = 90.0f + (pitch_cmd.angle_R - base_alpha_R);
        float s4_angle = 90.0f - (pitch_cmd.angle_L - base_alpha_L);

        if (s1_angle < 0.0f || s1_angle > 180.0f || s4_angle < 0.0f || s4_angle > 180.0f) {
            pitch_cmd = calculateTargets(target_mech_pitch, a, b, k, C_val);
            s1_angle = 90.0f + (pitch_cmd.angle_R - base_alpha_R);
            s4_angle = 90.0f - (pitch_cmd.angle_L - base_alpha_L);
        } else {
            target_mech_pitch = next_mech_pitch;
        }

        ActuatorTarget roll_cmd = calculateTargets(next_mech_roll, a, b, k, C_val);
        float s2_angle = 90.0f + (roll_cmd.angle_R - base_alpha_R);
        float s3_angle = 90.0f - (roll_cmd.angle_L - base_alpha_L);

        if (s2_angle < 0.0f || s2_angle > 180.0f || s3_angle < 0.0f || s3_angle > 180.0f) {
            roll_cmd = calculateTargets(target_mech_roll, a, b, k, C_val);
            s2_angle = 90.0f + (roll_cmd.angle_R - base_alpha_R);
            s3_angle = 90.0f - (roll_cmd.angle_L - base_alpha_L);
        } else {
            target_mech_roll = next_mech_roll;
        }

        uint32_t cyc4 = DWT_GetCycles();
        t_geom_us = Cycles_to_us(cyc4 - cyc3);

        // ═══════════════════════════
        //  Task 5: Servo PWM 輸出
        // ═══════════════════════════
        if (fabsf(s1_angle - last_s1_angle) > SERVO_UPDATE_DEADBAND) {
          Servo_SetAngle(&s1, s1_angle); last_s1_angle = s1_angle;
        }
        if (fabsf(s4_angle - last_s4_angle) > SERVO_UPDATE_DEADBAND) {
          Servo_SetAngle(&s4, s4_angle); last_s4_angle = s4_angle;
        }
        if (fabsf(s2_angle - last_s2_angle) > SERVO_UPDATE_DEADBAND) {
          Servo_SetAngle(&s2, s2_angle); last_s2_angle = s2_angle;
        }
        if (fabsf(s3_angle - last_s3_angle) > SERVO_UPDATE_DEADBAND) {
          Servo_SetAngle(&s3, s3_angle); last_s3_angle = s3_angle;
        }

        uint32_t cyc5 = DWT_GetCycles();
        t_servo_us = Cycles_to_us(cyc5 - cyc4);
        t_total_us = Cycles_to_us(cyc5 - cyc_total_start);

        // ═══════════════════════════
        //  Deadline 檢查
        // ═══════════════════════════
        uint32_t deadline_us = CTRL_MS * 1000;
        if (t_total_us > deadline_us) {
            deadline_miss_count++;
        }
        if (t_total_us > worst_case_us) {
            worst_case_us = t_total_us;
        }

        // ═══════════════════════════
        //  Step Response 狀態機更新
        // ═══════════════════════════
        StepResponse_Update(att.pitch, att.roll, now);

        // ═══════════════════════════
        //  高速資料輸出
        // ═══════════════════════════
#if ANALYSIS_MODE
        {
          int n = sprintf(log_buf,
            "DATA,%lu,%.3f,%.3f,%.3f,%.3f,"
            "%.2f,%.2f,%lu,%lu,%lu,%lu,%lu,%lu,"
            "%lu,%d\r\n",
            (unsigned long)now,
            att.pitch, att.roll,
            target_mech_pitch, target_mech_roll,
            imu.gx, imu.gy,
            (unsigned long)dt_us,
            (unsigned long)t_imu_us,
            (unsigned long)t_filt_us,
            (unsigned long)t_pid_us,
            (unsigned long)t_geom_us,
            (unsigned long)t_servo_us,
            (unsigned long)deadline_miss_count,
            (int)settling_state);
          CDC_Transmit_FS((uint8_t*)log_buf, n);
        }
#endif
    }

    // ═══════════════════════════
    //  普通模式 log (低速)
    // ═══════════════════════════
#if !ANALYSIS_MODE
    if (now - t_log >= LOG_MS) {
        t_log = now;
        int n = sprintf(log_buf,
          "P:%6.2f R:%6.2f tgtP:%6.2f tgtR:%6.2f | "
          "t_imu=%lu t_tot=%lu miss=%lu/%lu wcet=%lu\r\n",
          att.pitch, att.roll, target_mech_pitch, target_mech_roll,
          (unsigned long)t_imu_us,
          (unsigned long)t_total_us,
          (unsigned long)deadline_miss_count,
          (unsigned long)total_loop_count,
          (unsigned long)worst_case_us);
        CDC_Transmit_FS((uint8_t*)log_buf, n);
    }
#endif

    // ═══════════════════════════
    //  每 5 秒印一次統計摘要
    // ═══════════════════════════
    {
      static uint32_t t_summary = 0;
      if (now - t_summary >= 5000) {
        t_summary = now;
        int n = sprintf(log_buf,
          "SUMMARY,loops=%lu,miss=%lu,wcet=%luus,util=%.1f%%\r\n",
          (unsigned long)total_loop_count,
          (unsigned long)deadline_miss_count,
          (unsigned long)worst_case_us,
          (total_loop_count > 0)
            ? (100.0f * (float)worst_case_us / (float)(CTRL_MS * 1000))
            : 0.0f);
        CDC_Transmit_FS((uint8_t*)log_buf, n);
      }
    }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim2);
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif
