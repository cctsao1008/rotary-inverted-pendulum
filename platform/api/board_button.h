#ifndef BOARD_BUTTON_H
#define BOARD_BUTTON_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BOARD_KEY_M = 0,
    BOARD_KEY_X,
    BOARD_KEY_PLUS,
    BOARD_KEY_MINUS,
    BOARD_KEY_USER,
    BOARD_KEY_COUNT
} board_key_t;

#define BOARD_KEY_MASK(key) (1UL << (uint32_t)(key))

void board_button_init(void);
bool board_key_is_pressed(board_key_t key);
uint32_t board_keys_pressed_mask(void);

/* Compatibility helper retained while legacy call sites migrate. */
bool board_button_is_m_pressed(void);

#endif
