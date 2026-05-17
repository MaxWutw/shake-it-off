/* servo.h */
#ifndef SERVO_H
#define SERVO_H

#include "stm32f4xx_hal.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           ch;
    uint32_t           pulse_min;   /* us，對應   0° */
    uint32_t           pulse_max;   /* us，對應 180° */
	uint8_t            reversed;
} Servo_t;

void Servo_Init     (Servo_t *s, TIM_HandleTypeDef *htim, uint32_t ch);
void Servo_SetAngle (Servo_t *s, float angle_deg);

#endif
