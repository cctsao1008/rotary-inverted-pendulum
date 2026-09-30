#include "rip/board.hpp"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

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
constexpr float kWs2812BitRateHz = 800000.0f;
constexpr int kWs2812CyclesPerBit = 10;

void init_neopixel() {
    if (g_neopixel_ready) return;

    // Assemble the four-instruction WS2812 transmitter directly with the Pico
    // SDK instruction helpers. This avoids a host-side pioasm build dependency
    // while keeping the exact PIO timing used by the Pico SDK WS2812 example.
    std::uint16_t instructions[4] = {
        static_cast<std::uint16_t>(pio_encode_out(pio_x, 1) |
                                   pio_encode_sideset(1, 0) |
                                   pio_encode_delay(3)),
        static_cast<std::uint16_t>(pio_encode_jmp_not_x(3) |
                                   pio_encode_sideset(1, 1) |
                                   pio_encode_delay(2)),
        static_cast<std::uint16_t>(pio_encode_jmp(0) |
                                   pio_encode_sideset(1, 1) |
                                   pio_encode_delay(2)),
        static_cast<std::uint16_t>(pio_encode_nop() |
                                   pio_encode_sideset(1, 0) |
                                   pio_encode_delay(2)),
    };

    pio_program_t program{};
    program.instructions = instructions;
    program.length = 4;
    program.origin = -1;
    program.pio_version = 0;
#if PICO_PIO_VERSION > 0
    program.used_gpio_ranges =
        static_cast<std::uint8_t>(1u << (rip::board::kNeopixelGpio / 16u));
#endif

    g_neopixel_sm = pio_claim_unused_sm(g_neopixel_pio, true);
    const int offset = pio_add_program(g_neopixel_pio, &program);
    if (offset < 0) {
        panic("WS2812 PIO program load failed");
    }

    pio_gpio_init(g_neopixel_pio, rip::board::kNeopixelGpio);
    pio_sm_set_consecutive_pindirs(g_neopixel_pio, g_neopixel_sm,
                                   rip::board::kNeopixelGpio, 1, true);

    pio_sm_config config = pio_get_default_sm_config();
    sm_config_set_wrap(&config, static_cast<unsigned int>(offset),
                       static_cast<unsigned int>(offset + 3));
    sm_config_set_sideset(&config, 1, false, false);
    sm_config_set_sideset_pins(&config, rip::board::kNeopixelGpio);
    sm_config_set_out_shift(&config, false, true, 24);
    sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_TX);

    const float clock_div =
        static_cast<float>(clock_get_hz(clk_sys)) /
        (kWs2812BitRateHz * static_cast<float>(kWs2812CyclesPerBit));
    sm_config_set_clkdiv(&config, clock_div);

    pio_sm_init(g_neopixel_pio, g_neopixel_sm, static_cast<unsigned int>(offset), &config);
    pio_sm_set_enabled(g_neopixel_pio, g_neopixel_sm, true);
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
