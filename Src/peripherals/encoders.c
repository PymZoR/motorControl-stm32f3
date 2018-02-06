#include "peripherals/encoders.h"

void encoder_init(TIM_HandleTypeDef* htim2_, TIM_HandleTypeDef* htim3_) {
    tim2               = htim2_;
    tim3               = htim3_;
    previousLeftTicks  = 0;
    previousRightTicks = 0;
    leftVel            = 0;
    rightVel           = 0;
    leftPos            = 0;
    rightPos           = 0;
}

void encoder_update() {
    uint16_t leftTicks  = __HAL_TIM_GetCounter(tim2);
    uint16_t rightTicks = __HAL_TIM_GetCounter(tim3);
    
    leftVel             = leftTicks - previousLeftTicks;
    rightVel            = rightTicks - previousRightTicks;
    
    leftPos             += leftVel;
    rightPos            += rightVel;
}

int16_t encoder_left_getVal() {
    return leftVel;
}

int16_t encoder_right_getVal() {
    return rightVel;
}

int16_t encoder_left_getPos() {
    return leftPos;
}

int16_t encoder_right_getPos() {
    return rightPos;
}
