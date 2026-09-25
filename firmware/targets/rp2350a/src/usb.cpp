#include "rip/usb.hpp"

#include <cstdio>
#include <cstring>

#include "tusb.h"

namespace rip::usb {
namespace {

const RuntimeSnapshot* g_snapshot = nullptr;
char g_line[96]{};
std::size_t g_line_len = 0;
std::uint16_t g_hid_sequence = 0;

#pragma pack(push, 1)
struct HidRuntimeReport {
    std::uint8_t schema;
    std::uint8_t flags;
    std::uint16_t sequence;
    std::uint64_t timestamp_us;
    std::uint32_t sample_index;
    std::uint16_t pendulum_adc_raw;
    std::int32_t arm_encoder_count;
    float theta;
    float theta_dot;
    float phi;
    float phi_dot;
    float arm_torque_nm;
    float normalized_command;
    std::uint32_t missed_opportunities;
    std::uint32_t deadline_overruns;
    std::uint32_t execution_time_us;
    std::uint8_t regime;
    std::uint8_t runtime_state;
    std::uint8_t authority_mode;
    std::uint8_t reserved[3];
};
#pragma pack(pop)
static_assert(sizeof(HidRuntimeReport) == 64);

void cdc_write(const char* text) {
    if (!tud_cdc_connected()) return;
    tud_cdc_write(text, std::strlen(text));
    tud_cdc_write_flush();
}

void handle_line() {
    g_line[g_line_len] = '\0';
    if (std::strcmp(g_line, "help") == 0) {
        cdc_write("commands: help version status\r\n");
    } else if (std::strcmp(g_line, "version") == 0) {
        cdc_write("rotary-rp2350a,pico-sdk,cxx17,feature-parity-port\r\n");
    } else if (std::strcmp(g_line, "status") == 0) {
        if (g_snapshot) log_status(*g_snapshot);
        else cdc_write("status,unavailable=1\r\n");
    } else if (g_line_len != 0) {
        cdc_write("error,unknown-command\r\n");
    }
    g_line_len = 0;
}

}  // namespace

void init() {
    tusb_rhport_init_t rh_init{};
    rh_init.role = TUSB_ROLE_DEVICE;
    rh_init.speed = TUSB_SPEED_FULL;
    (void)tud_rhport_init(0, &rh_init);
}

void set_snapshot_source(const RuntimeSnapshot* snapshot) { g_snapshot = snapshot; }

void task() {
    tud_task();
    while (tud_cdc_available()) {
        const int ch = tud_cdc_read_char();
        if (ch < 0) break;
        if (ch == '\r' || ch == '\n') {
            if (g_line_len != 0) handle_line();
        } else if (g_line_len + 1 < sizeof(g_line)) {
            g_line[g_line_len++] = static_cast<char>(ch);
        } else {
            g_line_len = 0;
        }
    }
}

void log(const char* text) { cdc_write(text); }

void log_status(const RuntimeSnapshot& snapshot) {
    char buffer[320];
    std::snprintf(buffer, sizeof(buffer),
                  "status,t_us=%llu,sample=%lu,adc=%u,enc=%ld,theta=%.6f,theta_dot=%.6f,"
                  "phi=%.6f,phi_dot=%.6f,regime=%s,runtime=%s,torque_nm=%.6f,cmd=%.6f,"
                  "missed=%lu,overrun=%lu,exec_us=%lu,motor_authority=%u\r\n",
                  static_cast<unsigned long long>(snapshot.timestamp_us),
                  static_cast<unsigned long>(snapshot.sample_index),
                  static_cast<unsigned>(snapshot.pendulum_adc_raw),
                  static_cast<long>(snapshot.arm_encoder_count),
                  static_cast<double>(snapshot.state.theta),
                  static_cast<double>(snapshot.state.theta_dot),
                  static_cast<double>(snapshot.state.phi),
                  static_cast<double>(snapshot.state.phi_dot), regime_name(snapshot.regime),
                  runtime_state_name(snapshot.runtime_state),
                  static_cast<double>(snapshot.demand.arm_torque_nm),
                  static_cast<double>(snapshot.command.command),
                  static_cast<unsigned long>(snapshot.missed_opportunities),
                  static_cast<unsigned long>(snapshot.deadline_overruns),
                  static_cast<unsigned long>(snapshot.execution_time_us),
                  snapshot.authority_mode == AuthorityMode::ClosedLoop ? 1u : 0u);
    cdc_write(buffer);
}

bool send_hid_snapshot(const RuntimeSnapshot& snapshot) {
    if (!tud_hid_ready()) return false;
    HidRuntimeReport report{};
    report.schema = 1;
    report.flags = snapshot.state.validity == StateValidity::Valid ? 1u : 0u;
    report.sequence = g_hid_sequence++;
    report.timestamp_us = snapshot.timestamp_us;
    report.sample_index = snapshot.sample_index;
    report.pendulum_adc_raw = snapshot.pendulum_adc_raw;
    report.arm_encoder_count = snapshot.arm_encoder_count;
    report.theta = snapshot.state.theta;
    report.theta_dot = snapshot.state.theta_dot;
    report.phi = snapshot.state.phi;
    report.phi_dot = snapshot.state.phi_dot;
    report.arm_torque_nm = snapshot.demand.arm_torque_nm;
    report.normalized_command = snapshot.command.command;
    report.missed_opportunities = snapshot.missed_opportunities;
    report.deadline_overruns = snapshot.deadline_overruns;
    report.execution_time_us = snapshot.execution_time_us;
    report.regime = static_cast<std::uint8_t>(snapshot.regime);
    report.runtime_state = static_cast<std::uint8_t>(snapshot.runtime_state);
    report.authority_mode = static_cast<std::uint8_t>(snapshot.authority_mode);
    return tud_hid_report(0, &report, sizeof(report));
}

}  // namespace rip::usb

extern "C" std::uint16_t tud_hid_get_report_cb(std::uint8_t instance, std::uint8_t report_id,
                                                hid_report_type_t report_type,
                                                std::uint8_t* buffer, std::uint16_t reqlen) {
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;
    return 0;
}

extern "C" void tud_hid_set_report_cb(std::uint8_t instance, std::uint8_t report_id,
                                       hid_report_type_t report_type,
                                       std::uint8_t const* buffer, std::uint16_t bufsize) {
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)bufsize;
}
