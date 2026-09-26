#pragma once

#include <cstdint>

namespace rip::board {

// Canonical Rotary Inverted Pendulum mapping for the UNO Balance shield.
// The TB6612 channel-B control pins are taken from the U3 net labels; J8 pins
// D6/D3 are only breakout PWM-capable Arduino pins and are not connected to
// PWMA/PWMB. Use motor channel B + Encoder2 so the UNO RP2350 D13 user LED
// remains available to firmware.
inline constexpr std::uint32_t kArmMotorPwmGpio = 9;       // D9  / PWMB
inline constexpr std::uint32_t kArmMotorIn1Gpio = 7;       // D7  / BIN1
inline constexpr std::uint32_t kArmMotorIn2Gpio = 8;       // D8  / BIN2
inline constexpr std::uint32_t kArmEncoderAGpio = 10;      // D10 / ENCODER2_A
inline constexpr std::uint32_t kArmEncoderBGpio = 4;       // D4  / ENCODER2_B
inline constexpr std::uint32_t kPendulumAdcGpio = 26;      // A0 / ADC0
inline constexpr std::uint32_t kPendulumAdcChannel = 0;

// UNO RP2350 onboard indicators.
inline constexpr std::uint32_t kUserLedGpio = 13;          // D13 / blue user LED
inline constexpr std::uint32_t kNeopixelGpio = 14;         // onboard WS2812 data

// Motor channel A is deliberately unused. On this shield D10 is shared by
// PWMA and ENCODER2_A, while D13/D12 are AIN1/AIN2. Keeping AIN1 and AIN2
// equal guarantees that channel A never commands a direction even though its
// PWM input follows Encoder2_A.
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

enum class NeopixelColor : std::uint8_t {
    Off = 0,
    Red = 1,
    Green = 2,
    Blue = 3,
    White = 4,
};

// Establish a deterministic non-actuating board state before USB, telemetry,
// sensing, or control initialization proceeds.
void init_safe_idle();

// Bare-board diagnostic output. D13 is also channel-A AIN1, so D12/AIN2 is
// mirrored with it to keep the unused bridge in a non-driving equal-input
// state while the user LED changes.
void set_user_led(bool on);

// Bare-board WS2812 diagnostic output on GPIO14. Colors are deliberately kept
// dim because this is only intended to validate the onboard RGB signal path.
void set_neopixel(NeopixelColor color);

}  // namespace rip::board
