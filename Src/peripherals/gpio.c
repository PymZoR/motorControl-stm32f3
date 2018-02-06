#include "peripherals/gpio.h"

void gpio_init() {
    leftMotDir.port       = MOT_L_DIR_GPIO_Port;
    leftMotDir.pinNumber  = MOT_L_DIR_Pin;
    
    rightMotDir.port      = MOT_R_DIR_GPIO_Port;
    rightMotDir.pinNumber = MOT_R_DIR_Pin;
}

void gpio_digitalWrite(Pin* pin, GPIO_PinState pinState) {
    HAL_GPIO_WritePin(pin->port, pin->pinNumber, pinState);
}

Pin leftMotDir;
Pin rightMotDir;
