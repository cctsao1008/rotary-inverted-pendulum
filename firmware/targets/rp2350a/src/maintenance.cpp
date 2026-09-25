#include "rip/maintenance.hpp"

#include <algorithm>
#include <cmath>

#include "rip/config.hpp"

namespace rip {

bool MaintenanceAuthority::enter(std::uint64_t now_us) {
    active_ = true;
    last_update_us_ = now_us;
    lease_deadline_us_ = now_us;
    command_ = {};
    return true;
}

void MaintenanceAuthority::exit() {
    active_ = false;
    last_update_us_ = 0;
    lease_deadline_us_ = 0;
    command_ = {};
}

bool MaintenanceAuthority::set_command(float requested, std::uint64_t now_us,
                                       std::uint32_t lease_ms) {
    if (!active_ || !std::isfinite(requested)) return false;
    if (std::fabs(requested) > config::kMaintenanceMaxAbsCommand) return false;
    if (lease_ms == 0) lease_ms = config::kMaintenanceDefaultLeaseMs;
    if (lease_ms > config::kMaintenanceMaxLeaseMs || now_us < last_update_us_) return false;

    const float dt_s = static_cast<float>(now_us - last_update_us_) * 1.0e-6f;
    const float max_delta = config::kMaintenanceMaxSlewPerSec * dt_s;
    const float bounded = std::clamp(requested, command_.command - max_delta,
                                     command_.command + max_delta);

    command_.command = bounded;
    command_.saturated = bounded != requested;
    command_.predicted_arm_torque_nm = bounded * config::kActuatorTorquePerEffectiveCommandNm;
    last_update_us_ = now_us;
    lease_deadline_us_ = now_us + static_cast<std::uint64_t>(lease_ms) * 1000u;
    return true;
}

void MaintenanceAuthority::tick(std::uint64_t now_us) {
    if (active_ && lease_deadline_us_ != 0 && now_us > lease_deadline_us_) exit();
}

}  // namespace rip
