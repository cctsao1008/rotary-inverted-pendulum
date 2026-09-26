#include "rip/board.hpp"

#include "hardware/gpio.h"

namespace {

void configure_output_low(std::uint32_t gpio) {
    gpio_init(gpio);
    gpio_put(gpio, 0);
    gpio_set_dir(gpio, GPIO_OUT);
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

    // Keep the WS2812 data line quiet until a dedicated driver uses it.
    configure_output_low(kNeopixelGpio);
}

}  // namespace rip::board
