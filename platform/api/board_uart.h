#ifndef BOARD_UART_H
#define BOARD_UART_H

#include <stdbool.h>
#include <stdint.h>

void board_uart_init(uint32_t baudrate);
void board_uart_write(const char *data, uint32_t length);
uint32_t board_uart_tx_dropped_bytes(void);
bool board_uart_try_read_char(char *character);

#endif
