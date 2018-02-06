#ifndef CONTROL_H
#define CONTROL_H

#include "stm32f3xx_hal.h"
#include <stdbool.h>
#include <stdlib.h>
#include "constants.h"
#include "peripherals/encoders.h"
#include "motionControl/motors.h"

typedef enum {
    LEFT_VEL,
    LEFT_POS,
    RIGHT_VEL,
    RIGHT_POS,
    BOTH_VEL,
    BOTH_POS,
    DISTANCE,
    ANGLE,
    POLAR
} ControlType;

void control_iter();

unsigned long lastIter;
extern bool controlRunning;
extern ControlType currentControl;
#endif
