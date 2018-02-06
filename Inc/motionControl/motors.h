#ifndef MOTORS_H
#define MOTORS_H

#include "stm32f3xx_hal.h"
#include <math.h>
#include "peripherals/gpio.h"
#include "peripherals/pwm.h"
#include "motionControl/control.h"

void motor_left_drive(int16_t value);
void motor_left_stop();
void motor_right_drive(int16_t value);
void motor_right_stop();
#endif
