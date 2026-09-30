#include "rip/commissioning.hpp"

#include <cstring>

#include "rip/platform.hpp"
#include "tusb.h"

namespace rip::commissioning {
namespace {

#pragma pack(push, 1)
struct HostCommandReport {
    std::uint8_t version;
    std::uint8_t message_type;
    std::uint8_t command;
    std::uint8_t flags;
    std::uint32_t sequence;
    float value0;
    float value1;
    std::uint32_t duration_ms;
    std::uint32_t reserved0;
    std::uint8_t reserved[40];
};

struct AckReport {
    std::uint8_t version;
    std::uint8_t message_type;
    std::uint8_t command;
    std::uint8_t status;
    std::uint32_t sequence;
    std::uint64_t timestamp_us;
    float value0;
    float value1;
    std::uint32_t detail;
    std::uint8_t reserved[36];
};
#pragma pack(pop)

static_assert(sizeof(HostCommandReport) == 64);
static_assert(sizeof(AckReport) == 64);

Request g_request{};
bool g_request_pending = false;
AckReport g_ack{};
bool g_ack_pending = false;

}  // namespace

bool take_request(Request& request) {
    if (!g_request_pending) return false;
    request = g_request;
    g_request_pending = false;
    return true;
}

void queue_ack(const Request& request, Status status, std::uint32_t detail,
               float value0, float value1) {
    g_ack = {};
    g_ack.version = kProtocolVersion;
    g_ack.message_type = 0x81;
    g_ack.command = static_cast<std::uint8_t>(request.command);
    g_ack.status = static_cast<std::uint8_t>(status);
    g_ack.sequence = request.sequence;
    g_ack.timestamp_us = platform::now_us();
    g_ack.value0 = value0;
    g_ack.value1 = value1;
    g_ack.detail = detail;
    g_ack_pending = true;
}

void service() {
    if (g_ack_pending && tud_hid_ready()) {
        if (tud_hid_report(0, &g_ack, sizeof(g_ack))) g_ack_pending = false;
    }
}

void accept_output_report(const std::uint8_t* buffer, std::uint16_t size) {
    if (size != sizeof(HostCommandReport) || g_request_pending) return;
    HostCommandReport report{};
    std::memcpy(&report, buffer, sizeof(report));
    if (report.version != kProtocolVersion || report.message_type != 0x01) return;

    g_request.command = static_cast<Command>(report.command);
    g_request.flags = report.flags;
    g_request.sequence = report.sequence;
    g_request.value0 = report.value0;
    g_request.value1 = report.value1;
    g_request.duration_ms = report.duration_ms;
    g_request_pending = true;
}

}  // namespace rip::commissioning

extern "C" void tud_hid_set_report_cb(std::uint8_t instance, std::uint8_t report_id,
                                       hid_report_type_t report_type,
                                       std::uint8_t const* buffer, std::uint16_t bufsize) {
    (void)instance;
    (void)report_id;
    if (report_type == HID_REPORT_TYPE_OUTPUT) {
        rip::commissioning::accept_output_report(buffer, bufsize);
    }
}
