#pragma once

#include <cstddef>
#include <cstdint>

#include "rip/runtime.hpp"

namespace rip::usb {

void init();
void task();
void log(const char* text);
void log_status(const RuntimeSnapshot& snapshot);
bool send_hid_snapshot(const RuntimeSnapshot& snapshot);

bool telemetry_enabled();
void set_telemetry_enabled(bool enabled);

// CDC remains diagnostic-only. Machine-facing commissioning commands are owned
// by rip::commissioning and arrive through the same composite USB device.
void set_snapshot_source(const RuntimeSnapshot* snapshot);

}  // namespace rip::usb
