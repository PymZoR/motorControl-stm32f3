#ifndef PERPIPH_CAN_H
#define PERIPH_CAN_H

#include "stm32f3xx_hal.h"

void can_init(CAN_HandleTypeDef* hcan_);

CAN_HandleTypeDef* can;

#endif
