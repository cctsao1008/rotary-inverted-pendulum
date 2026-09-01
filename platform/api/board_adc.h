#ifndef BOARD_ADC_H
#define BOARD_ADC_H

#include <stdint.h>

void board_adc_init(void);
uint16_t board_adc_read_pendulum_raw(void);
uint16_t board_adc_read_vbus_raw(void);
uint32_t board_adc_vbus_raw_to_millivolts(uint16_t raw);
uint32_t board_adc_read_vbus_millivolts(void);

#endif
