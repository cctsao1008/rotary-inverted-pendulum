#ifndef BOARD_OLED_H
#define BOARD_OLED_H

#include <stddef.h>
#include <stdint.h>

void board_oled_init(void);
void board_oled_reset(void *context);
void board_oled_delay_ms(
    uint32_t milliseconds,
    void *context);
void board_oled_write_command(
    uint8_t command,
    void *context);
void board_oled_write_data(
    const uint8_t *data,
    size_t length,
    void *context);

#endif
