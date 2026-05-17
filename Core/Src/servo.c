/* servo.c */
#include "servo.h"

void Servo_Init(Servo_t *s, TIM_HandleTypeDef *htim, uint32_t ch)
{
    s->htim      = htim;
    s->ch        = ch;
    s->pulse_min = 500;    /* 調整到你的 MG90S 實際最小脈寬 */
    s->pulse_max = 2500;   /* 調整到你的 MG90S 實際最大脈寬 */
    HAL_TIM_PWM_Start(htim, ch);
    Servo_SetAngle(s, 90.0f);   /* 上電先回中位 */
}

void Servo_SetAngle(Servo_t *s, float angle_deg)
{
	if (s->reversed) angle_deg = 180.0f - angle_deg;
    if (angle_deg < 0.0f)   angle_deg = 0.0f;
    if (angle_deg > 180.0f) angle_deg = 180.0f;

    uint32_t pulse = s->pulse_min +
        (uint32_t)((angle_deg / 180.0f) * (float)(s->pulse_max - s->pulse_min));

    __HAL_TIM_SET_COMPARE(s->htim, s->ch, pulse);
}
