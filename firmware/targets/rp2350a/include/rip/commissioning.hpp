#pragma once

#include <cstdint>

namespace rip::commissioning {

inline constexpr std::uint8_t kProtocolVersion = 1;

enum class Command : std::uint8_t {
    GetStatus = 0x01,
    TelemetryOn = 0x02,
    TelemetryOff = 0x03,
    SetMotorCommand = 0x12,
    SafeOff = 0x13,
    SetUserLed = 0x20,
};

enum class Status : std::uint8_t {
    Ok = 0,
    Invalid = 1,
    Denied = 2,
    Range = 3,
    Busy = 4,
};

struct Request {
    Command command{Command::GetStatus};
    std::uint8_t flags{0};
    std::uint32_t sequence{0};
    float value0{0.0f};
    float value1{0.0f};
    std::uint32_t duration_ms{0};
};

bool take_request(Request& request);
void queue_ack(const Request& request, Status status, std::uint32_t detail = 0,
               float value0 = 0.0f, float value1 = 0.0f);
void service();
void accept_output_report(const std::uint8_t* buffer, std::uint16_t size);

}  // namespace rip::commissioning
