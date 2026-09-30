#include "rip/platform.hpp"

#include "hardware/gpio.h"
#include "rip/board.hpp"

namespace rip::platform {

bool read_arm_encoder_a() { return gpio_get(board::kArmEncoderAGpio); }
bool read_arm_encoder_b() { return gpio_get(board::kArmEncoderBGpio); }

}  // namespace rip::platform
