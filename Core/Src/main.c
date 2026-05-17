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

char       log_buf[200];
uint32_t   t_ctrl = 0;
uint32_t   t_log  = 0;

#define CTRL_MS   5     // 200 Hz 控制週期
#define LOG_MS  100     // 10 Hz log

// PID 狀態
float pitch_integral  = 0.0f;
float pitch_prev_err  = 0.0f;
float roll_integral   = 0.0f;
float roll_prev_err   = 0.0f;

// PID 增益（先用這組初值，後面再調）
#define KP  4.0f
#define KI  0.0f   // 先不開積分，等 Kp 調好再加
#define KD  0.0f   // 先不開微分
#define I_LIMIT   20.0f
#define OUT_LIMIT 90.0f   // servo 最多從中位偏移 ±25°
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
float PID_compute(float setpoint, float measured,
                  float *integral, float *prev_err,
                  float kp, float ki, float kd,
                  float dt)
{
    float err = setpoint - measured;

    // 積分（含 anti-windup）
    *integral += err * dt;
    if (*integral >  I_LIMIT) *integral =  I_LIMIT;
    if (*integral < -I_LIMIT) *integral = -I_LIMIT;

    // 微分
    float deriv = (err - *prev_err) / dt;
    *prev_err = err;

    // 三項合成
    float u = kp * err + ki * (*integral) + kd * deriv;

    // 輸出飽和
    if (u >  OUT_LIMIT) u =  OUT_LIMIT;
    if (u < -OUT_LIMIT) u = -OUT_LIMIT;

    return u;
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
  HAL_Delay(3000);  // 等 USB CDC 連上

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
  
  Servo_SetAngle(&s1, 90); Servo_SetAngle(&s2, 90);
  Servo_SetAngle(&s3, 90); Servo_SetAngle(&s4, 90);
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
  Attitude_Init(&att, 0.98f, CTRL_MS / 1000.0f);
  
  n = sprintf(log_buf, "[READY] Closed-loop starting...\r\n");
  CDC_Transmit_FS((uint8_t*)log_buf, n);
  
  t_ctrl = HAL_GetTick();
  t_log  = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	uint32_t now = HAL_GetTick();
    float dt = CTRL_MS / 1000.0f;

    /* ── 控制週期 5ms / 200Hz ── */
    if (now - t_ctrl >= CTRL_MS) {
        t_ctrl = now;

        /* 1. 讀 IMU */
        MPU6050_Read(&imu, &hi2c1);

        /* 2. 更新姿態（互補濾波）*/
        Attitude_Update(&att, &imu);

        /* 3. PID 計算
         *    目標 pitch = 0°, 目標 roll = 0°（水平）
         *    ⚠️ 這裡的 gyro 軸對應要實測確認，
         *       若平台傾斜方向和角度符號相反，
         *       在 attitude.c 裡把對應軸加負號   */
        float u_pitch = PID_compute(0.0f, att.pitch,
                                    &pitch_integral, &pitch_prev_err,
                                    KP, KI, KD, dt);
        float u_roll  = PID_compute(0.0f, att.roll,
                                    &roll_integral,  &roll_prev_err,
                                    KP, KI, KD, dt);

        /* 4. 分配給 4 顆 servo
         *
         *    pitch+（前緣高）→ P0 收線(+)、P3 放線(reversed 自動)
         *    roll+ （右緣高）→ P1 收線(+)、P2 放線(reversed 自動)
         *
         *    ⚠️ 若平台反應方向錯（越控越歪），
         *       把 u_pitch 或 u_roll 前面加負號   */
        Servo_SetAngle(&s1, 90.0f + u_pitch);  // P0
        Servo_SetAngle(&s4, 90.0f + u_pitch);  // P3（reversed 自動反向）
        Servo_SetAngle(&s2, 90.0f + u_roll);   // P1
        Servo_SetAngle(&s3, 90.0f + u_roll);   // P2（reversed 自動反向）
    }

    /* ── Log 週期 100ms / 10Hz ── */
    if (now - t_log >= LOG_MS) {
        t_log = now;
        int n = sprintf(log_buf,
            "P:%6.2f R:%6.2f uP:%6.2f uR:%6.2f | ax:%5.2f ay:%5.2f\r\n",
            att.pitch, att.roll, pitch_prev_err, roll_prev_err,
            imu.ax, imu.ay);
        CDC_Transmit_FS((uint8_t*)log_buf, n);
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
