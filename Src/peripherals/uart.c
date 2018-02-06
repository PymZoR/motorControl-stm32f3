#include "peripherals/uart.h"

int _write(int file, char *ptr, int len) {
    switch (file) {
        case STDOUT_FILENO:
            HAL_UART_Transmit_IT(uart, (uint8_t*)ptr, len);
            break;
        
        case STDERR_FILENO: 
            HAL_UART_Transmit_IT(uart, (uint8_t*)ptr, len);
            break;

        default:
            return -1;
    }

    while (HAL_UART_GetState(uart) == HAL_UART_STATE_BUSY_TX || HAL_UART_GetState(uart) == HAL_UART_STATE_BUSY_TX_RX);
    return len;
}

void uart_init(UART_HandleTypeDef* huart_) {
    uart = huart_;
    uart_reset();
    HAL_UART_Receive_IT(uart, uart_rx_buf, 1);
}

void uart_reset() {
    uart_rx_index   = 0;
    memset(uart_rx_data, 0, UART_MAX_LEN);
    uart_rx_buf[0] = 1;
    uart_rx_cplt   = false;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart_) {
    if (uart_rx_index == 0) {
        for (uint8_t i = 0; i < UART_MAX_LEN; i++) {
            uart_rx_data[i] = 0; 
        } 
    }

    uart_rx_data[uart_rx_index++] = uart_rx_buf[0];
    if (uart_rx_buf[0] == '\n') {  
        uart_rx_index = 0;
        uart_rx_cplt = 1;
    } 

    HAL_UART_Receive_IT(uart, uart_rx_buf, 1);
}

uint8_t uart_rx_data[UART_MAX_LEN];
bool uart_rx_cplt;
