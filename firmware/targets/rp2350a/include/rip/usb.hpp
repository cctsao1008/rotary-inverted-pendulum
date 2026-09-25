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

// Match the existing assembly policy: telemetry is off at boot and can be
// explicitly enabled by the host through the CDC console.
bool telemetry_enabled();
void set_telemetry_enabled(bool enabled);

// Console commands are intentionally non-authoritative. They expose status and
// diagnostics only; they do not grant physical motor authority.
void set_snapshot_source(const RuntimeSnapshot* snapshot);

}  // namespace rip::usb
