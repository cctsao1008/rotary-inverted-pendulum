#pragma once

#include <cstdint>

namespace rip::board {

// Canonical Rotary Inverted Pendulum mapping for the UNO Balance J7/J8
// interface. Use motor channel B + Encoder2 so the UNO RP2350 D13 user LED
// remains available to firmware.
inline constexpr std::uint32_t kArmMotorPwmGpio = 3;       // D3  / PWMB
inline constexpr std::uint32_t kArmMotorIn1Gpio = 7;       // D7  / BIN1
inline constexpr std::uint32_t kArmMotorIn2Gpio = 8;       // D8  / BIN2
inline constexpr std::uint32_t kArmEncoderAGpio = 10;      // D10 / ENCODER2_A
inline constexpr std::uint32_t kArmEncoderBGpio = 4;       // D4  / ENCODER2_B
inline constexpr std::uint32_t kPendulumAdcGpio = 26;      // A0 / ADC0
inline constexpr std::uint32_t kPendulumAdcChannel = 0;

// UNO RP2350 onboard indicators.
inline constexpr std::uint32_t kUserLedGpio = 13;          // D13 / blue user LED
inline constexpr std::uint32_t kNeopixelGpio = 14;         // onboard WS2812 data

// Motor channel A is deliberately unused. D13/AIN1 is shared with the user
// LED, so keeping PWMA low guarantees that LED activity cannot drive channel A.
inline constexpr std::uint32_t kUnusedMotorAPwmGpio = 6;   // D6  / PWMA
inline constexpr std::uint32_t kUnusedMotorAIn2Gpio = 12;  // D12 / AIN2

static_assert(kArmMotorPwmGpio != kArmMotorIn1Gpio);
static_assert(kArmMotorPwmGpio != kArmMotorIn2Gpio);
static_assert(kArmEncoderAGpio != kArmEncoderBGpio);
static_assert(kArmMotorPwmGpio != kArmEncoderAGpio);
static_assert(kArmMotorPwmGpio != kArmEncoderBGpio);
static_assert(kUserLedGpio != kArmMotorPwmGpio);
static_assert(kUserLedGpio != kArmMotorIn1Gpio);
static_assert(kUserLedGpio != kArmMotorIn2Gpio);
static_assert(kNeopixelGpio != kArmMotorPwmGpio);

// Establish a deterministic non-actuating board state before USB, telemetry,
// sensing, or control initialization proceeds.
void init_safe_idle();

}  // namespace rip::board
