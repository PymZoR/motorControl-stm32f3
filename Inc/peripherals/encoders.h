#ifndef ENCODERS_H
#define ENCODERS_H

#include "stm32f3xx_hal.h"

void encoder_init(TIM_HandleTypeDef* htim2_, TIM_HandleTypeDef* htim3_);
void encoder_update();

int16_t encoder_left_getVal();
int16_t encoder_right_getVal();
int16_t encoder_left_getPos();
int16_t encoder_right_getPos();

TIM_HandleTypeDef* tim2;
TIM_HandleTypeDef* tim3;

uint16_t previousLeftTicks;
uint16_t previousRightTicks;
int16_t leftVel;
int16_t rightVel;
int16_t leftPos;
int16_t rightPos;

#endif
