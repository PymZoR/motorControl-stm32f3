#ifndef MOTIONCONTROLLER_H
#define MOTIONCONTROLLER_H

#include "stm32f3xx_hal.h"
#include "peripherals/can.h"
#include "peripherals/uart.h"
#include "peripherals/encoders.h"
#include "peripherals/pwm.h"
#include "motionControl/control.h"
#include "communication/uartProtocol.h"

void motionController_init(UART_HandleTypeDef* huart_, CAN_HandleTypeDef* hcan_, TIM_HandleTypeDef* htim2_,
    TIM_HandleTypeDef* htim3_, TIM_HandleTypeDef* htim16_, TIM_HandleTypeDef* htim17_);

void motionController_loop();

#endif
