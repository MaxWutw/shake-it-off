/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "mpu6050.h"
#include "attitude.h"
#include "servo.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

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

#define CTRL_MS   2     // 1000 Hz 控制週期
#define LOG_MS  100     // 10 Hz log

// 速度型 PID (Velocity Form) 狀態變數
float target_mech_pitch = 0.0f;
float target_mech_roll  = 0.0f;

float pitch_prev_err  = 0.0f;
float roll_prev_err   = 0.0f;
float pitch_d_err_lpf = 0.0f;
float roll_d_err_lpf  = 0.0f;

/* Keep last commanded servo angles to avoid tiny jitter updates */
float last_s1_angle = 90.0f;
float last_s2_angle = 90.0f;
float last_s3_angle = 90.0f;
float last_s4_angle = 90.0f;

// 速度型控制增益（對應實際 PID：I 與 P）
#define KI  0.01f   // I 步進增益(累積回正力): 調高可更快消除中小誤差但較易過衝/低頻擺動; 調低更穩但殘差較大、回正較慢
#define KP  0.1f    // P 差分增益(阻尼/煞車): 調高可更抑制過衝與快速晃動但可能變鈍、放大噪聲; 調低反應較靈敏但較容易震盪
#define MAX_MECH_ANGLE  50.0f   // 機構安全極限角度
#define MIN_MECH_ANGLE -50.0f
#define ERR_SOFT_DEADBAND_DEG  0.3f  // 軟死區(度): 調高可減少抖動但殘差變大; 調低可改善小誤差回正但可能更容易微抖
#define DERR_LPF_ALPHA   0.85f    // 差分低通係數(0~1): 調高更平滑、噪聲更少但反應較慢; 調低反應更快但更容易受噪聲影響
#define TARGET_LEAK_GAIN 0.004f   // 目標角泄放係數(每迴圈往0拉回): 調高可抑制累積與自激但殘差可能變大; 調低回正力可維持較久但可能累積過頭
#define MAX_STEP_PER_LOOP  0.6f   // 每迴圈最大步進(度/loop): 調高回正更快但過衝風險增加; 調低更穩定但回正較慢

// 記錄平台水平 (0度) 時，公式算出來的初始機構角度基準，用來對應伺服馬達的 90 度，用來扣除的
// cmd.angle - base_alpha = delta_alpha -> motor angle = 90 + delta_alpha
float base_alpha_R = 0.0f;
float base_alpha_L = 0.0f;
uint32_t last_ctrl_tick = 0;

// ─── 分析記錄模式 ──────────────────────────────────────────
#define ANALYSIS_MODE              1       // 1=高速DATA輸出, 0=普通可讀log
#define DISTURBANCE_THRESHOLD_DEG  3.0f
#define SETTLED_THRESHOLD_DEG      0.5f
#define SETTLED_HOLD_LOOPS         50

// DWT 計時變數 (μs)
volatile uint32_t t_imu_us   = 0;
volatile uint32_t t_filt_us  = 0;
volatile uint32_t t_pid_us   = 0;
volatile uint32_t t_geom_us  = 0;
volatile uint32_t t_servo_us = 0;
volatile uint32_t t_total_us = 0;

// Deadline 統計
volatile uint32_t deadline_miss_count = 0;
volatile uint32_t total_loop_count    = 0;
volatile uint32_t worst_case_us       = 0;

// Step response 偵測狀態
typedef enum { SR_IDLE=0, SR_DISTURBED=1, SR_SETTLING=2, SR_SETTLED=3 } SettlingState;
SettlingState settling_state     = SR_IDLE;
uint32_t      disturbance_tick   = 0;
uint32_t      settled_tick       = 0;
float         peak_disturbance   = 0.0f;
uint32_t      settle_hold_counter = 0;
float         settling_time_ms   = 0.0f;
uint32_t      response_count     = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

typedef struct {
    float length_L;
    float length_R;
    float angle_L;
    float angle_R;
} ActuatorTarget;

#define CALIB_SETTLE_MS   1200U
#define CALIB_SAMPLES     120U
#define SERVO_UPDATE_DEADBAND 0.5f

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
    if (MPU6050_Read(&imu, &hi2c1) != HAL_OK) {
      HAL_Delay(5);
      continue;
    }

    float pitch_deg = 0.0f;
    float roll_deg  = 0.0f;
    Attitude_GetInstantAngles(&imu, &pitch_deg, &roll_deg);
    sum += use_pitch_axis ? pitch_deg : roll_deg;
    count++;
    HAL_Delay(5);
  }

  if (count == 0) {
    return 0.0f;
  }

  return sum / (float)count;
}

static void CalibrateAxisFromServos(Servo_t *servo_a, Servo_t *servo_b,
                  uint8_t use_pitch_axis,
                  float *axis_zero, float *axis_scale)
{
  float measured_180 = 0.0f;
  float measured_0 = 0.0f;

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
    /* Sweep didn't move the platform enough. Keep only the zero offset. */
    *axis_zero = 0.5f * (measured_180 + measured_0);
    *axis_scale = 1.0f;
    return;
  }

  *axis_zero = 0.5f * (measured_180 + measured_0);
  /* Use the sweep only to determine direction, not a fake degree scaling. */
  *axis_scale = (span < 0.0f) ? -1.0f : 1.0f;
}

static float adaptive_step_limit(float error_deg)
{
  float abs_error = fabsf(error_deg);

  if (abs_error < 4.0f) {
    return 0.12f;
  }
  if (abs_error < 10.0f) {
    return 0.25f;
  }
  if (abs_error < 20.0f) {
    return 0.40f;
  }
  return MAX_STEP_PER_LOOP;
}

static float apply_soft_deadband(float err_deg, float deadband_deg)
{
  float abs_err = fabsf(err_deg);

  if (abs_err <= deadband_deg) {
    return 0.0f;
  }

  return (err_deg > 0.0f) ? (err_deg - deadband_deg) : (err_deg + deadband_deg);
}

ActuatorTarget calculateTargets(float theta_deg, float a, float b, float k, float C) {
    ActuatorTarget target;
    
    // 1. 將角度轉換為弧度
    float theta_rad = theta_deg * M_PI / 180.0f;
    
    // 2. 預先計算 sin 和 cos
    float sin_t = sinf(theta_rad);
    float cos_t = cosf(theta_rad);
    
    // 3. 代入公式計算 l_R 與 l_L (注意這裡拆解成 X與Y 的平方相加)
    float dx_R = -a * sin_t + k * cos_t - k;
    float dy_R = b + a * cos_t + k * sin_t;
    target.length_R = sqrtf(dx_R * dx_R + dy_R * dy_R);
    
    float dx_L = -a * sin_t - k * cos_t + k;
    float dy_L = b + a * cos_t - k * sin_t;
    target.length_L = sqrtf(dx_L * dx_L + dy_L * dy_L);
    
    // 4. 計算馬達旋轉角度 (alpha)
    target.angle_R = (360.0f * target.length_R) / C;
    target.angle_L = (360.0f * target.length_L) / C;
    
    return target;
}


// ─── DWT Cycle Counter (Cortex-M4, SYSCLK = 84MHz) ───────────────────────────
static inline void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}
static inline uint32_t DWT_GetCycles(void)  { return DWT->CYCCNT; }
static inline uint32_t Cycles_to_us(uint32_t c) { return c / 84; }

// ─── Step Response 狀態機 ──────────────────────────────────────────────────
static void StepResponse_Update(float pitch, float roll, uint32_t now_ms)
{
    float mag = sqrtf(pitch * pitch + roll * roll);
    switch (settling_state) {
    case SR_IDLE:
        if (mag > DISTURBANCE_THRESHOLD_DEG) {
            settling_state = SR_DISTURBED;
            disturbance_tick = now_ms;
            peak_disturbance = mag;
            settle_hold_counter = 0;
            response_count++;
        }
        break;
    case SR_DISTURBED:
        if (mag > peak_disturbance) peak_disturbance = mag;
        settling_state = SR_SETTLING;
        break;
    case SR_SETTLING:
        if (mag > peak_disturbance) peak_disturbance = mag;
        if (mag < SETTLED_THRESHOLD_DEG) {
            if (++settle_hold_counter >= SETTLED_HOLD_LOOPS) {
                settling_state   = SR_SETTLED;
                settled_tick     = now_ms;
                settling_time_ms = (float)(settled_tick - disturbance_tick);
            }
        } else {
            settle_hold_counter = 0;
        }
        break;
    case SR_SETTLED: {
        int sn = sprintf(log_buf,
            "STEP_RESP,%lu,peak=%.2f,settle=%.1fms,resp#%lu\r\n",
            (unsigned long)disturbance_tick, peak_disturbance,
            settling_time_ms, (unsigned long)response_count);
        CDC_Transmit_FS((uint8_t*)log_buf, sn);
        HAL_Delay(2);
        settling_state = SR_IDLE;
        break;
		}
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  // HAL_Delay(3000);  // 等 USB CDC 連上

  int n;
  
  /* ── Servo 初始化 ── */
  Servo_Init(&s1, &htim2, TIM_CHANNEL_1);  // P0, pitch
  Servo_Init(&s2, &htim2, TIM_CHANNEL_2);  // P1, roll
  Servo_Init(&s3, &htim2, TIM_CHANNEL_3);  // P2, roll
  Servo_Init(&s4, &htim2, TIM_CHANNEL_4);  // P3, pitch
  
  s1.reversed = 0;
  s2.reversed = 0;
  s3.reversed = 1;  // 實測確認需要反向
  s4.reversed = 1;  // 實測確認需要反向
  
  ActuatorTarget init_cmd = calculateTargets(0.0f, 2.2f, 4.5f, 11.4f, 5.1f);
  base_alpha_R = init_cmd.angle_R;
  base_alpha_L = init_cmd.angle_L;

  Servo_SetAngle(&s1, 90.0f);
  Servo_SetAngle(&s2, 90.0f);
  Servo_SetAngle(&s3, 90.0f);
  Servo_SetAngle(&s4, 90.0f);
  HAL_Delay(1000);
  
  /* ── IMU 初始化 ── */
  if (MPU6050_Init(&imu, &hi2c1) != HAL_OK) {
      n = sprintf(log_buf, "[ERROR] IMU init failed\r\n");
      CDC_Transmit_FS((uint8_t*)log_buf, n);
      while (1);
  }
  
  /* ── 校正：把平台放水平，靜止 3 秒 ── */
  n = sprintf(log_buf, "[CALIB] Keep platform FLAT and STILL...\r\n");
  CDC_Transmit_FS((uint8_t*)log_buf, n);
  HAL_Delay(3000);
  MPU6050_Calibrate(&imu, &hi2c1, 500);
  
  n = sprintf(log_buf, "[CALIB] Done! ax_off=%.3f ay_off=%.3f az_off=%.3f\r\n",
              imu.ax_off, imu.ay_off, imu.az_off);
  CDC_Transmit_FS((uint8_t*)log_buf, n);
  HAL_Delay(200);

  /* ── 姿態估計初始化 ── */
  Attitude_Init(&att, 0.96f, CTRL_MS / 1000.0f);

  n = sprintf(log_buf, "[CALIB] Sweeping pitch axis...\r\n");
  CDC_Transmit_FS((uint8_t*)log_buf, n);
  float pitch_zero = 0.0f;
  float pitch_scale = 1.0f;
  CalibrateAxisFromServos(&s1, &s4, 1U, &pitch_zero, &pitch_scale);

  n = sprintf(log_buf, "[CALIB] Pitch zero=%.3f scale=%.3f\r\n",
              pitch_zero, pitch_scale);
  CDC_Transmit_FS((uint8_t*)log_buf, n);

  /* 回中 pitch 伺服，讓平台回到水平再做 roll 掃描 */
  SetServoPairAngle(&s1, &s4, 90.0f);
  HAL_Delay(CALIB_SETTLE_MS);

  n = sprintf(log_buf, "[CALIB] Sweeping roll axis...\r\n");
  CDC_Transmit_FS((uint8_t*)log_buf, n);
  float roll_zero = 0.0f;
  float roll_scale = 1.0f;
  CalibrateAxisFromServos(&s2, &s3, 0U, &roll_zero, &roll_scale);

  n = sprintf(log_buf, "[CALIB] Roll zero=%.3f scale=%.3f\r\n",
              roll_zero, roll_scale);
  CDC_Transmit_FS((uint8_t*)log_buf, n);

  Attitude_SetCalibration(&att, pitch_zero, pitch_scale, roll_zero, roll_scale);

  SetServoPairAngle(&s1, &s4, 90.0f);
  SetServoPairAngle(&s2, &s3, 90.0f);
  HAL_Delay(500);
  
  n = sprintf(log_buf, "[READY] Closed-loop starting...\r\n");
  CDC_Transmit_FS((uint8_t*)log_buf, n);

  DWT_Init();

#if ANALYSIS_MODE
  n = sprintf(log_buf,
    "DATA_HEADER,tick_ms,pitch,roll,tgt_pitch,tgt_roll,"
    "gx,gy,dt_us,t_imu,t_filt,t_pid,t_geom,t_servo,"
    "deadline_miss,settling_state\r\n");
  CDC_Transmit_FS((uint8_t*)log_buf, n);
  HAL_Delay(5);
#endif

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  t_ctrl = HAL_GetTick();
  t_log  = HAL_GetTick();
  last_ctrl_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    if (now - t_ctrl >= CTRL_MS) {
        t_ctrl = now;
        total_loop_count++;

        float actual_dt = (now - last_ctrl_tick) / 1000.0f;
        last_ctrl_tick = now;
        if (actual_dt < 0.0005f) actual_dt = CTRL_MS / 1000.0f;
        if (actual_dt > 0.020f)  actual_dt = CTRL_MS / 1000.0f;
        att.dt = actual_dt;
        uint32_t dt_us = (uint32_t)(actual_dt * 1000000.0f);

        uint32_t cyc_start = DWT_GetCycles();

        /* 1. IMU 讀取 */
        uint32_t cyc0 = DWT_GetCycles();
        MPU6050_Read(&imu, &hi2c1);
        uint32_t cyc1 = DWT_GetCycles();
        t_imu_us = Cycles_to_us(cyc1 - cyc0);

        /* 2. 互補濾波 */
        Attitude_Update(&att, &imu);
        uint32_t cyc2 = DWT_GetCycles();
        t_filt_us = Cycles_to_us(cyc2 - cyc1);

        /* 3. PID 計算 */
        float err_pitch_raw = 0.0f - att.pitch;
        float err_roll_raw  = 0.0f - att.roll;

        float err_pitch = apply_soft_deadband(err_pitch_raw, ERR_SOFT_DEADBAND_DEG);
        float err_roll  = apply_soft_deadband(err_roll_raw,  ERR_SOFT_DEADBAND_DEG);

        float d_pitch = err_pitch - pitch_prev_err;
        float d_roll  = err_roll  - roll_prev_err;

        pitch_d_err_lpf = (DERR_LPF_ALPHA * pitch_d_err_lpf)
                        + ((1.0f - DERR_LPF_ALPHA) * d_pitch);
        roll_d_err_lpf  = (DERR_LPF_ALPHA * roll_d_err_lpf)
                        + ((1.0f - DERR_LPF_ALPHA) * d_roll);

        float delta_pitch = (KI * err_pitch) + (KP * pitch_d_err_lpf)
                          - (TARGET_LEAK_GAIN * target_mech_pitch);
        float delta_roll  = (KI * err_roll)  + (KP * roll_d_err_lpf)
                          - (TARGET_LEAK_GAIN * target_mech_roll);

        float pitch_step_limit = adaptive_step_limit(err_pitch);
        float roll_step_limit  = adaptive_step_limit(err_roll);

        if (delta_pitch >  pitch_step_limit) delta_pitch =  pitch_step_limit;
        if (delta_pitch < -pitch_step_limit) delta_pitch = -pitch_step_limit;
        if (delta_roll  >  roll_step_limit)  delta_roll  =  roll_step_limit;
        if (delta_roll  < -roll_step_limit)  delta_roll  = -roll_step_limit;

        pitch_prev_err = err_pitch;
        roll_prev_err  = err_roll;

        float next_mech_pitch = target_mech_pitch + delta_pitch;
        float next_mech_roll  = target_mech_roll  + delta_roll;

        if (next_mech_pitch >  MAX_MECH_ANGLE) next_mech_pitch =  MAX_MECH_ANGLE;
        if (next_mech_pitch < MIN_MECH_ANGLE)  next_mech_pitch = MIN_MECH_ANGLE;
        if (next_mech_roll  >  MAX_MECH_ANGLE) next_mech_roll  =  MAX_MECH_ANGLE;
        if (next_mech_roll  < MIN_MECH_ANGLE)  next_mech_roll  = MIN_MECH_ANGLE;

        uint32_t cyc3 = DWT_GetCycles();
        t_pid_us = Cycles_to_us(cyc3 - cyc2);

        /* 4. 幾何計算 */
        float a = 2.2f, b = 4.5f, k = 11.4f, C = 5.1f;

        ActuatorTarget pitch_cmd = calculateTargets(next_mech_pitch, a, b, k, C);
        float s1_angle = 90.0f + (pitch_cmd.angle_R - base_alpha_R);
        float s4_angle = 90.0f - (pitch_cmd.angle_L - base_alpha_L);

        if (s1_angle < 0.0f || s1_angle > 180.0f || s4_angle < 0.0f || s4_angle > 180.0f) {
            pitch_cmd = calculateTargets(target_mech_pitch, a, b, k, C);
            s1_angle = 90.0f + (pitch_cmd.angle_R - base_alpha_R);
            s4_angle = 90.0f - (pitch_cmd.angle_L - base_alpha_L);
        } else {
            target_mech_pitch = next_mech_pitch;
        }

        ActuatorTarget roll_cmd = calculateTargets(next_mech_roll, a, b, k, C);
        float s2_angle = 90.0f + (roll_cmd.angle_R - base_alpha_R);
        float s3_angle = 90.0f - (roll_cmd.angle_L - base_alpha_L);

        if (s2_angle < 0.0f || s2_angle > 180.0f || s3_angle < 0.0f || s3_angle > 180.0f) {
            roll_cmd = calculateTargets(target_mech_roll, a, b, k, C);
            s2_angle = 90.0f + (roll_cmd.angle_R - base_alpha_R);
            s3_angle = 90.0f - (roll_cmd.angle_L - base_alpha_L);
        } else {
            target_mech_roll = next_mech_roll;
        }

        uint32_t cyc4 = DWT_GetCycles();
        t_geom_us = Cycles_to_us(cyc4 - cyc3);

        /* 5. Servo 輸出 */
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
        t_total_us = Cycles_to_us(cyc5 - cyc_start);

        if (t_total_us > (uint32_t)(CTRL_MS * 1000U)) deadline_miss_count++;
        if (t_total_us > worst_case_us) worst_case_us = t_total_us;

        StepResponse_Update(att.pitch, att.roll, now);

#if ANALYSIS_MODE
        {
            /* Log physical angles in pin order A0–A3.
               s3/s4 are reversed=1: Servo_SetAngle flips them internally,
               so physical position = 180 - commanded_angle. */
            float phys_s1 = s1_angle;               /* A0, reversed=0 */
            float phys_s2 = s2_angle;               /* A1, reversed=0 */
            float phys_s3 = 180.0f - s3_angle;      /* A2, reversed=1 */
            float phys_s4 = 180.0f - s4_angle;      /* A3, reversed=1 */
            int n = sprintf(log_buf,
                "DATA,%lu,%.3f,%.3f,%.3f,%.3f,"
                "%.2f,%.2f,%lu,%lu,%lu,%lu,%lu,%lu,"
                "%lu,%d,%.2f,%.2f,%.2f,%.2f\r\n",
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
                (int)settling_state,
                phys_s1, phys_s2, phys_s3, phys_s4);
            CDC_Transmit_FS((uint8_t*)log_buf, n);
        }
#endif
    }

#if !ANALYSIS_MODE
    if (now - t_log >= LOG_MS) {
        t_log = now;
        float inst_pitch = 0.0f, inst_roll = 0.0f;
        Attitude_GetInstantAngles(&imu, &inst_pitch, &inst_roll);
        int n = sprintf(log_buf,
            "P:%6.2f R:%6.2f uP:%6.2f uR:%6.2f | "
            "rawP:%6.2f rawR:%6.2f instP:%6.2f instR:%6.2f "
            "gx:%5.2f gy:%5.2f ax:%5.2f ay:%5.2f\r\n",
            att.pitch, att.roll, pitch_prev_err, roll_prev_err,
            att.raw_pitch, att.raw_roll, inst_pitch, inst_roll,
            imu.gx, imu.gy, imu.ax, imu.ay);
        CDC_Transmit_FS((uint8_t*)log_buf, n);
    }
#endif

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
                    ? (100.0f * (float)worst_case_us / (float)(CTRL_MS * 1000U))
                    : 0.0f);
            CDC_Transmit_FS((uint8_t*)log_buf, n);
        }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
