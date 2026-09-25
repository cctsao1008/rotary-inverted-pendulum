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
    configure_output_low(kArmMotorPwmGpio);
    configure_output_low(kArmMotorIn1Gpio);
    configure_output_low(kArmMotorIn2Gpio);
}

}  // namespace rip::board
