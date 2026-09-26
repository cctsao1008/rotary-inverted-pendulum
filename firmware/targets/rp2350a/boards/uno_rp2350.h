/*
 * Uno RP2350 board definition for Raspberry Pi Pico SDK.
 *
 * Hardware profile derived from the board schematic:
 * - RP2350A package
 * - 12 MHz crystal
 * - Winbond W25Q128JVSIQ external QSPI flash (16 MiB)
 * - blue user LED on D13 / GPIO13
 * - onboard WS2812 data on GPIO14
 */

#ifndef _BOARDS_UNO_RP2350_H
#define _BOARDS_UNO_RP2350_H

pico_board_cmake_set(PICO_PLATFORM, rp2350)

#define RIP_UNO_RP2350 1
#define PICO_RP2350A 1

#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 13
#endif

#define RIP_UNO_NEOPIXEL_PIN 14

// Winbond W25Q128 family, 16 MiB.
#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif

pico_board_cmake_set_default(PICO_FLASH_SIZE_BYTES, (16 * 1024 * 1024))
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)
#endif

#endif
