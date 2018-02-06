#include "motionController.h"

void motionController_init(UART_HandleTypeDef* huart_, CAN_HandleTypeDef* hcan_, TIM_HandleTypeDef* htim2_,
    TIM_HandleTypeDef* htim3_, TIM_HandleTypeDef* htim16_, TIM_HandleTypeDef* htim17_) {

    can_init(hcan_);
    uart_init(huart_);
    encoder_init(htim2_, htim3_);
    pwm_init(htim16_, htim17_);

    for (;;) {
        motionController_loop();
    }
}

void motionController_loop() {
    control_iter();
    uartProtocol_handleMessage();
}
