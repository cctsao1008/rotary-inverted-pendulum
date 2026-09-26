#include "rip/board.hpp"

#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "ws2812.pio.h"

namespace {

void configure_output_low(std::uint32_t gpio) {
    gpio_init(gpio);
    gpio_put(gpio, 0);
    gpio_set_dir(gpio, GPIO_OUT);
}

PIO g_neopixel_pio = pio0;
unsigned int g_neopixel_sm = 0;
bool g_neopixel_ready = false;

constexpr std::uint8_t kRgbLevel = 32;
constexpr std::uint8_t kWhiteLevel = 16;

void init_neopixel() {
    if (g_neopixel_ready) return;
    g_neopixel_sm = pio_claim_unused_sm(g_neopixel_pio, true);
    const unsigned int offset = pio_add_program(g_neopixel_pio, &ws2812_program);
    ws2812_program_init(g_neopixel_pio, g_neopixel_sm, offset, rip::board::kNeopixelGpio,
                        800000.0f);
    g_neopixel_ready = true;
}

void put_neopixel_rgb(std::uint8_t red, std::uint8_t green, std::uint8_t blue) {
    if (!g_neopixel_ready) init_neopixel();
    const std::uint32_t grb =
        (static_cast<std::uint32_t>(green) << 16) |
        (static_cast<std::uint32_t>(red) << 8) |
        static_cast<std::uint32_t>(blue);
    pio_sm_put_blocking(g_neopixel_pio, g_neopixel_sm, grb << 8);
    sleep_us(80);
}

}  // namespace

namespace rip::board {

void init_safe_idle() {
    // Active motor channel B.
    configure_output_low(kArmMotorPwmGpio);
    configure_output_low(kArmMotorIn1Gpio);
    configure_output_low(kArmMotorIn2Gpio);

    // Unused motor channel A. D13 is also the onboard blue user LED; PWMA is
    // held low so later LED activity cannot create channel-A motor output.
    configure_output_low(kUnusedMotorAPwmGpio);
    configure_output_low(kUnusedMotorAIn2Gpio);
    configure_output_low(kUserLedGpio);

    // The onboard WS2812 is a separate GPIO14 diagnostic resource.
    configure_output_low(kNeopixelGpio);
    init_neopixel();
    set_neopixel(NeopixelColor::Off);
}

void set_user_led(bool on) { gpio_put(kUserLedGpio, on ? 1 : 0); }

void set_neopixel(NeopixelColor color) {
    switch (color) {
        case NeopixelColor::Off:
            put_neopixel_rgb(0, 0, 0);
            break;
        case NeopixelColor::Red:
            put_neopixel_rgb(kRgbLevel, 0, 0);
            break;
        case NeopixelColor::Green:
            put_neopixel_rgb(0, kRgbLevel, 0);
            break;
        case NeopixelColor::Blue:
            put_neopixel_rgb(0, 0, kRgbLevel);
            break;
        case NeopixelColor::White:
            put_neopixel_rgb(kWhiteLevel, kWhiteLevel, kWhiteLevel);
            break;
    }
}

}  // namespace rip::board
