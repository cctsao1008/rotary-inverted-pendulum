#pragma once

#include <cmath>

#include "rip/runtime.hpp"

namespace rip {

// Target-side empirical guard for the start-vs-running hysteresis observed
// during motor commissioning. The actuator inverse model owns the kinetic
// command deadzone; this gate only removes authority, never adds it, so output
// safety limits and slew bounds established upstream cannot be exceeded.
inline BoundedActuatorCommand apply_stationary_stiction_gate(
    BoundedActuatorCommand requested,
    float arm_rate_rad_s,
    float static_start_command,
    float moving_rate_threshold_rad_s) {
    BoundedActuatorCommand out = requested;

    const bool valid = std::isfinite(requested.command) &&
                       std::isfinite(requested.predicted_arm_torque_nm) &&
                       std::isfinite(arm_rate_rad_s) &&
                       std::isfinite(static_start_command) &&
                       std::isfinite(moving_rate_threshold_rad_s) &&
                       static_start_command >= 0.0f && static_start_command <= 1.0f &&
                       moving_rate_threshold_rad_s >= 0.0f;
    if (!valid) {
        out.command = 0.0f;
        out.predicted_arm_torque_nm = 0.0f;
        return out;
    }

    const float magnitude = std::fabs(requested.command);
    const bool effectively_stationary =
        std::fabs(arm_rate_rad_s) <= moving_rate_threshold_rad_s;
    const bool below_observed_start_authority =
        magnitude > 0.0f && magnitude < static_start_command;

    if (effectively_stationary && below_observed_start_authority) {
        out.command = 0.0f;
        out.predicted_arm_torque_nm = 0.0f;
    }
    return out;
}

}  // namespace rip
