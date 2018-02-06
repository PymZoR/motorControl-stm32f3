#ifndef PWM_H
#define PWM_H

#include "stm32f3xx_hal.h"

void pwm_init(TIM_HandleTypeDef* htim16_, TIM_HandleTypeDef* htim17_);
void pwm_left_set(uint16_t value);
void pwm_right_set(uint16_t value);

TIM_HandleTypeDef* tim16;
TIM_HandleTypeDef* tim17;

#endif
