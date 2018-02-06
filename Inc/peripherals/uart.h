#ifndef PERIPH_UART_H
#define PERIPH_UART_H

#include "stm32f3xx_hal.h"
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include "constants.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *_huart);
void uart_init(UART_HandleTypeDef* huart_);
void uart_reset();

UART_HandleTypeDef* uart;
uint8_t uart_rx_index;
uint8_t uart_rx_buf[1];
extern uint8_t uart_rx_data[UART_MAX_LEN];
extern bool uart_rx_cplt;

#endif
