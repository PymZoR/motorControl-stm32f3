#include "communication/uartProtocol.h"

void uartProtocol_handleMessage() {
    if (uart_rx_cplt) {
        uart_rx_cplt = 0;

        char buffer[UART_MAX_LEN];
        memcpy(buffer, uart_rx_data, UART_MAX_LEN);

        char* args = strtok(buffer, ":");
        int commandId = atoi(args);

        switch (commandId) {
            default:
                printf("el moumouxe %i\n", commandId);
        }
    }
}
