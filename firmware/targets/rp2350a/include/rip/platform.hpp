#pragma once

#include <cstdint>

#include "rip/runtime.hpp"

namespace rip::platform {

void init();
std::uint64_t now_us();

// Acquisition backends. Encoder count is maintained continuously from both
// non-consecutive quadrature inputs (D9/GPIO9 and D2/GPIO2).
std::uint16_t read_pendulum_adc();
std::int32_t read_arm_encoder_count();
bool read_arm_encoder_a();
bool read_arm_encoder_b();

// Physical-output backend. `apply_tb6612()` always removes PWM before changing
// bridge direction (break-before-make). `safe_off()` is unqualified.
void safe_off();
void apply_tb6612(const Tb6612ElectricalActuation& frame);

void watchdog_enable();
void watchdog_feed();

struct SchedulerEvidence {
    std::uint32_t missed_opportunities{0};
    std::uint32_t deadline_overruns{0};
};

// Wait for one 1 kHz opportunity. If execution falls behind, elapsed
// opportunities are coalesced rather than replayed as a burst.
void scheduler_init();
SchedulerEvidence wait_next_opportunity();

}  // namespace rip::platform
