#include "rip/platform.hpp"

#include <algorithm>

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "rip/board.hpp"
#include "rip/config.hpp"

namespace rip::platform {
namespace {

volatile std::int32_t g_encoder_count = 0;
volatile std::uint8_t g_encoder_state = 0;
uint g_pwm_slice = 0;
uint g_pwm_channel = 0;
std::uint16_t g_pwm_wrap = 0;
std::uint64_t g_next_opportunity_us = 0;
SchedulerEvidence g_scheduler_evidence{};

constexpr std::int8_t kQuadratureDelta[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0,
};

std::uint8_t read_encoder_state() {
    return static_cast<std::uint8_t>((gpio_get(board::kArmEncoderAGpio) ? 2u : 0u) |
                                     (gpio_get(board::kArmEncoderBGpio) ? 1u : 0u));
}

void encoder_irq(uint gpio, std::uint32_t events) {
    (void)gpio;
    (void)events;
    const std::uint8_t next = read_encoder_state();
    const std::uint8_t transition = static_cast<std::uint8_t>((g_encoder_state << 2) | next);
    g_encoder_count += kQuadratureDelta[transition & 0x0fu];
    g_encoder_state = next;
}

void set_pwm_fraction(float duty) {
    duty = std::clamp(duty, 0.0f, 1.0f);
    const auto level = static_cast<std::uint16_t>(duty * static_cast<float>(g_pwm_wrap + 1u));
    pwm_set_chan_level(g_pwm_slice, g_pwm_channel, level);
}

}  // namespace

void init() {
    // Safety boundary first: no higher-level initialization precedes safe-off.
    board::init_safe_idle();

    adc_init();
    adc_gpio_init(board::kPendulumAdcGpio);
    adc_select_input(board::kPendulumAdcChannel);

    gpio_init(board::kArmEncoderAGpio);
    gpio_set_dir(board::kArmEncoderAGpio, GPIO_IN);
    gpio_disable_pulls(board::kArmEncoderAGpio);
    gpio_init(board::kArmEncoderBGpio);
    gpio_set_dir(board::kArmEncoderBGpio, GPIO_IN);
    gpio_disable_pulls(board::kArmEncoderBGpio);
    g_encoder_state = read_encoder_state();
    gpio_set_irq_enabled_with_callback(board::kArmEncoderAGpio,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true, &encoder_irq);
    gpio_set_irq_enabled(board::kArmEncoderBGpio,
                         GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    gpio_set_function(board::kArmMotorPwmGpio, GPIO_FUNC_PWM);
    g_pwm_slice = pwm_gpio_to_slice_num(board::kArmMotorPwmGpio);
    g_pwm_channel = pwm_gpio_to_channel(board::kArmMotorPwmGpio);
    const std::uint32_t sys_hz = clock_get_hz(clk_sys);
    const std::uint32_t wrap = std::max<std::uint32_t>(1u, sys_hz / config::kMotorPwmHz) - 1u;
    g_pwm_wrap = static_cast<std::uint16_t>(std::min<std::uint32_t>(wrap, 65535u));
    pwm_config pwm_cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&pwm_cfg, 1.0f);
    pwm_config_set_wrap(&pwm_cfg, g_pwm_wrap);
    pwm_init(g_pwm_slice, &pwm_cfg, false);
    set_pwm_fraction(0.0f);
    pwm_set_enabled(g_pwm_slice, true);

    // Direction pins were already driven low by board::init_safe_idle().
    scheduler_init();
}

std::uint64_t now_us() { return time_us_64(); }

std::uint16_t read_pendulum_adc() {
    adc_select_input(board::kPendulumAdcChannel);
    return adc_read();
}

std::int32_t read_arm_encoder_count() { return g_encoder_count; }

void safe_off() {
    set_pwm_fraction(0.0f);
    gpio_put(board::kArmMotorIn1Gpio, 0);
    gpio_put(board::kArmMotorIn2Gpio, 0);
}

void apply_tb6612(const Tb6612ElectricalActuation& frame) {
    // Break-before-make: direction never changes with non-zero PWM authority.
    set_pwm_fraction(0.0f);

    switch (frame.mode) {
        case Tb6612BridgeMode::Coast:
            gpio_put(board::kArmMotorIn1Gpio, 0);
            gpio_put(board::kArmMotorIn2Gpio, 0);
            break;
        case Tb6612BridgeMode::DrivePositive:
            gpio_put(board::kArmMotorIn1Gpio, 1);
            gpio_put(board::kArmMotorIn2Gpio, 0);
            break;
        case Tb6612BridgeMode::DriveNegative:
            gpio_put(board::kArmMotorIn1Gpio, 0);
            gpio_put(board::kArmMotorIn2Gpio, 1);
            break;
        case Tb6612BridgeMode::Brake:
            gpio_put(board::kArmMotorIn1Gpio, 1);
            gpio_put(board::kArmMotorIn2Gpio, 1);
            break;
    }

    const float duty = (frame.mode == Tb6612BridgeMode::DrivePositive ||
                        frame.mode == Tb6612BridgeMode::DriveNegative)
                           ? frame.duty_fraction
                           : 0.0f;
    set_pwm_fraction(duty);
}

void watchdog_enable() { ::watchdog_enable(config::kHardwareWatchdogMs, true); }
void watchdog_feed() { watchdog_update(); }

void scheduler_init() {
    g_scheduler_evidence = {};
    g_next_opportunity_us = now_us() + config::kControlPeriodUs;
}

SchedulerEvidence wait_next_opportunity() {
    const std::uint64_t before = now_us();
    if (before < g_next_opportunity_us) {
        sleep_until(from_us_since_boot(g_next_opportunity_us));
    }

    const std::uint64_t actual = now_us();
    if (actual > g_next_opportunity_us + config::kControlPeriodUs) {
        const std::uint64_t late = actual - g_next_opportunity_us;
        const std::uint32_t missed = static_cast<std::uint32_t>(late / config::kControlPeriodUs);
        g_scheduler_evidence.missed_opportunities += missed;
        ++g_scheduler_evidence.deadline_overruns;
        // Do not replay missed control opportunities as a burst.
        g_next_opportunity_us = actual + config::kControlPeriodUs;
    } else {
        g_next_opportunity_us += config::kControlPeriodUs;
    }
    return g_scheduler_evidence;
}

}  // namespace rip::platform
