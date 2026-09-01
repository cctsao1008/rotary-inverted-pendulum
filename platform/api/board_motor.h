#ifndef BOARD_MOTOR_H
#define BOARD_MOTOR_H

#include <stdint.h>

typedef enum {
    BOARD_MOTOR_CHANNEL_D1 = 0,
    BOARD_MOTOR_CHANNEL_D2
} board_motor_channel_t;

void board_motor_init(void);
void board_motor_select_channel(board_motor_channel_t channel);
board_motor_channel_t board_motor_get_channel(void);
void board_motor_set_percent(int8_t signed_percent);
void board_motor_stop(void);

#endif
