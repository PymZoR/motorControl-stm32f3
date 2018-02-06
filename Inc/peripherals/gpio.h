#ifndef GPIO_H
#define GPIO_H

#include "stm32f3xx_hal.h"

#define HIGH GPIO_PIN_SET
#define LOW  GPIO_PIN_RESET

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pinNumber;
} Pin;

void gpio_init();
void gpio_digitalWrite(Pin* pin, GPIO_PinState pinState);

extern Pin leftMotDir;
extern Pin rightMotDir;

#endif
