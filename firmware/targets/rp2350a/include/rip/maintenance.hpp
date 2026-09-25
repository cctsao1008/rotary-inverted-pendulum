#pragma once

#include <cstdint>

#include "rip/runtime.hpp"

namespace rip {

// Firmware-owned maintenance authority for host-driven commissioning.
// It is deliberately separate from automatic closed-loop authority.
class MaintenanceAuthority {
  public:
    bool enter(std::uint64_t now_us);
    void exit();

    bool active() const { return active_; }
    bool set_command(float requested, std::uint64_t now_us, std::uint32_t lease_ms);
    void tick(std::uint64_t now_us);

    BoundedActuatorCommand command() const { return command_; }
    std::uint64_t lease_deadline_us() const { return lease_deadline_us_; }

  private:
    bool active_{false};
    std::uint64_t last_update_us_{0};
    std::uint64_t lease_deadline_us_{0};
    BoundedActuatorCommand command_{};
};

}  // namespace rip
