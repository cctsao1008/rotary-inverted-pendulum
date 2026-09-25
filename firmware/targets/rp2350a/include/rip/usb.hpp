#pragma once

#include <cstddef>
#include <cstdint>

#include "rip/runtime.hpp"

namespace rip::usb {

inline constexpr std::uint8_t kHidProtocolVersion = 1;

enum class HidMessageType : std::uint8_t {
    Command = 0x01,
    Telemetry = 0x80,
    Ack = 0x81,
};

enum class HidCommand : std::uint8_t {
    GetStatus = 0x01,
    TelemetryOn = 0x02,
    TelemetryOff = 0x03,
    MaintenanceEnter = 0x10,
    MaintenanceExit = 0x11,
    SetMotorCommand = 0x12,
    SafeOff = 0x13,
};

enum class HidCommandStatus : std::uint8_t {
    Ok = 0,
    Invalid = 1,
    Denied = 2,
    Range = 3,
    Busy = 4,
};

struct HidCommandMessage {
    HidCommand command{HidCommand::GetStatus};
    std::uint8_t flags{0};
    std::uint32_t sequence{0};
    float value0{0.0f};
    float value1{0.0f};
    std::uint32_t duration_ms{0};
};

void init();
void task();
void log(const char* text);
void log_status(const RuntimeSnapshot& snapshot);
bool send_hid_snapshot(const RuntimeSnapshot& snapshot);

bool telemetry_enabled();
void set_telemetry_enabled(bool enabled);

// HID commands are consumed one-at-a-time by the main firmware loop. CDC
// remains diagnostic-only and never grants physical output authority.
bool take_hid_command(HidCommandMessage& command);
void queue_hid_ack(const HidCommandMessage& command, HidCommandStatus status,
                   std::uint32_t detail = 0, float value0 = 0.0f, float value1 = 0.0f);

// Commissioning state is reflected in machine-readable telemetry without
// changing the automatic closed-loop runtime contract.
void set_commissioning_state(bool maintenance_active, Tb6612BridgeMode motor_mode,
                             float applied_command);

void set_snapshot_source(const RuntimeSnapshot* snapshot);

}  // namespace rip::usb
