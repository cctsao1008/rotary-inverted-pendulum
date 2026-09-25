#pragma once

#include <cstdint>

namespace rip::board {

// Canonical Rotary Inverted Pendulum mapping for UNO Balance channel A.
inline constexpr std::uint32_t kArmMotorPwmGpio = 10;      // D10 / PWMA
inline constexpr std::uint32_t kArmMotorIn1Gpio = 13;      // D13 / AIN1
inline constexpr std::uint32_t kArmMotorIn2Gpio = 12;      // D12 / AIN2
inline constexpr std::uint32_t kArmEncoderAGpio = 9;       // D9 / ENCODER1_A
inline constexpr std::uint32_t kArmEncoderBGpio = 2;       // D2 / ENCODER1_B
inline constexpr std::uint32_t kPendulumAdcGpio = 26;      // A0 / ADC0
inline constexpr std::uint32_t kPendulumAdcChannel = 0;

static_assert(kArmMotorPwmGpio != kArmMotorIn1Gpio);
static_assert(kArmMotorPwmGpio != kArmMotorIn2Gpio);
static_assert(kArmEncoderAGpio != kArmEncoderBGpio);

// Configure every motor command line to a non-actuating state before any USB,
// telemetry, sensing, or control initialization is allowed to proceed.
void init_safe_idle();

}  // namespace rip::board
