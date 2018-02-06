#include "peripherals/pwm.h"

void pwm_init(TIM_HandleTypeDef* htim16_, TIM_HandleTypeDef* htim17_) {
    tim16 = htim16_;
    tim17 = htim17_;

    pwm_left_set(0);
    pwm_right_set(0);

    HAL_TIM_PWM_Start(tim16,TIM_CHANNEL_1); 
    HAL_TIM_PWM_Start(tim17,TIM_CHANNEL_1); 
}

void pwm_left_set(uint16_t value) {
    __HAL_TIM_SET_COMPARE(tim17, TIM_CHANNEL_1, value);
}

void pwm_right_set(uint16_t value) {
    __HAL_TIM_SET_COMPARE(tim16, TIM_CHANNEL_1, value);
}
