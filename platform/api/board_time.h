#ifndef BOARD_TIME_H
#define BOARD_TIME_H

#include <stdint.h>

void board_time_init(void);
uint32_t board_time_micros(void);
uint32_t board_time_ticks(void);

#endif
