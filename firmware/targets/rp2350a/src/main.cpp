#include <cmath>
#include <cstdint>

#include "rip/commissioning.hpp"
#include "rip/config.hpp"
#include "rip/platform.hpp"
#include "rip/runtime.hpp"
#include "rip/usb.hpp"

namespace {

struct CommissioningMotor {
    bool active{false};
    float command{0.0f};
    std::uint64_t deadline_us{0};

    void clear() {
        active = false;
        command = 0.0f;
        deadline_us = 0;
    }
};

void service_commissioning(rip::ControlRuntime& runtime, CommissioningMotor& motor) {
    rip::commissioning::Request request{};
    while (rip::commissioning::take_request(request)) {
        const std::uint64_t now_us = rip::platform::now_us();
        switch (request.command) {
            case rip::commissioning::Command::GetStatus: {
                const std::uint32_t detail =
                    static_cast<std::uint32_t>(runtime.runtime_state()) |
                    (static_cast<std::uint32_t>(runtime.authority_mode()) << 8) |
                    (motor.active ? (1u << 16) : 0u);
                rip::commissioning::queue_ack(request, rip::commissioning::Status::Ok, detail,
                                               motor.command, 0.0f);
                break;
            }
            case rip::commissioning::Command::TelemetryOn:
                rip::usb::set_telemetry_enabled(true);
                rip::commissioning::queue_ack(request, rip::commissioning::Status::Ok);
                break;
            case rip::commissioning::Command::TelemetryOff:
                rip::usb::set_telemetry_enabled(false);
                rip::commissioning::queue_ack(request, rip::commissioning::Status::Ok);
                break;
            case rip::commissioning::Command::SetMotorCommand: {
                std::uint32_t lease_ms = request.duration_ms;
                if (lease_ms == 0) lease_ms = rip::config::kCommissioningDefaultLeaseMs;
                if (!std::isfinite(request.value0) ||
                    std::fabs(request.value0) > rip::config::kCommissioningMaxAbsCommand ||
                    lease_ms > rip::config::kCommissioningMaxLeaseMs) {
                    rip::commissioning::queue_ack(request, rip::commissioning::Status::Range);
                    break;
                }
                motor.active = true;
                motor.command = request.value0;
                motor.deadline_us = now_us + static_cast<std::uint64_t>(lease_ms) * 1000u;
                rip::commissioning::queue_ack(request, rip::commissioning::Status::Ok, 0,
                                               motor.command, 0.0f);
                break;
            }
            case rip::commissioning::Command::SafeOff:
                motor.clear();
                rip::platform::safe_off();
                rip::commissioning::queue_ack(request, rip::commissioning::Status::Ok);
                break;
            default:
                rip::commissioning::queue_ack(request, rip::commissioning::Status::Invalid);
                break;
        }
    }
    rip::commissioning::service();
}

rip::BoundedActuatorCommand direct_commissioning_command(float command) {
    rip::BoundedActuatorCommand out{};
    out.command = command;
    out.predicted_arm_torque_nm = command * rip::config::kActuatorTorquePerEffectiveCommandNm;
    return out;
}

}  // namespace

int main() {
    rip::platform::init();
    rip::platform::safe_off();
    rip::platform::watchdog_enable();
    rip::usb::init();

    const std::uint64_t started_at = rip::platform::now_us();
    rip::SensorTimingMonitor timing_monitor(
        rip::config::kSensorExpectedPeriodUs,
        rip::config::kSensorLateAfterUs,
        rip::config::kSensorTimeoutAfterUs,
        started_at);
    rip::ControlWatchdog control_watchdog(rip::config::kControlWatchdogTimeoutUs);
    rip::MeasurementAdapter adapter;
    rip::ControlRuntime runtime;
    CommissioningMotor commissioning_motor;

    rip::RuntimeSnapshot snapshot{};
    rip::usb::set_snapshot_source(&snapshot);
    rip::usb::log("boot,target=rp2350a,board=uno_rp2350,runtime=feature-parity,motor_command=0\r\n");

    std::uint32_t sample_index = 0;
    std::uint32_t telemetry_ticks = 0;

    while (true) {
        rip::usb::task();
        service_commissioning(runtime, commissioning_motor);

        const rip::platform::SchedulerEvidence scheduler = rip::platform::wait_next_opportunity();
        const std::uint64_t cycle_started = rip::platform::now_us();

        rip::RawObservation raw{};
        raw.sample_index = sample_index++;
        raw.pendulum.adc_raw = rip::platform::read_pendulum_adc();
        raw.arm_encoder.accumulated_count = rip::platform::read_arm_encoder_count();
        const std::uint64_t captured_at = rip::platform::now_us();
        raw.pendulum.captured_at.value = captured_at;
        raw.arm_encoder.captured_at.value = captured_at;
        raw.pendulum.quality = rip::MeasurementAvailable | rip::MeasurementIoOk |
                               rip::MeasurementTimingValid;
        raw.arm_encoder.quality = rip::MeasurementAvailable | rip::MeasurementIoOk |
                                  rip::MeasurementTimingValid;

        const rip::SensorTimingHealth timing = timing_monitor.on_event(captured_at);
        const rip::WatchdogHealth watchdog_health = control_watchdog.health(captured_at);

        rip::EstimatorMeasurement measurement{};
        const bool measurement_ok = adapter.convert(raw, measurement);

        rip::RuntimeObservation observation{};
        observation.measurement = measurement;
        observation.sensor_valid = measurement_ok;
        observation.sample_age_us = 0;
        observation.timing = timing;
        observation.watchdog = watchdog_health;

        const rip::ControlCycle cycle = runtime.step(observation);
        if (cycle.kind != rip::ControlCycle::Kind::Error) {
            control_watchdog.kick(captured_at);
        }

        if (commissioning_motor.active && captured_at > commissioning_motor.deadline_us) {
            commissioning_motor.clear();
        }

        rip::BoundedActuatorCommand applied_command{};
        if (commissioning_motor.active) {
            applied_command = direct_commissioning_command(commissioning_motor.command);
            rip::platform::apply_tb6612(rip::map_tb6612(applied_command));
        } else if (cycle.kind == rip::ControlCycle::Kind::Computed && cycle.authorized) {
            applied_command = cycle.bounded_command;
            rip::platform::apply_tb6612(rip::map_tb6612(applied_command));
        } else {
            rip::platform::safe_off();
        }

        snapshot.timestamp_us = captured_at;
        snapshot.sample_index = raw.sample_index;
        snapshot.pendulum_adc_raw = raw.pendulum.adc_raw;
        snapshot.arm_encoder_count = raw.arm_encoder.accumulated_count;
        snapshot.regime = runtime.regime();
        snapshot.runtime_state = runtime.runtime_state();
        snapshot.authority_mode = runtime.authority_mode();
        snapshot.missed_opportunities = scheduler.missed_opportunities;
        snapshot.deadline_overruns = scheduler.deadline_overruns;
        if (cycle.kind == rip::ControlCycle::Kind::Computed) {
            snapshot.state = cycle.state;
            snapshot.demand = cycle.demand;
        }
        snapshot.command = applied_command;

        const std::uint64_t cycle_finished = rip::platform::now_us();
        snapshot.execution_time_us = static_cast<std::uint32_t>(cycle_finished - cycle_started);

        ++telemetry_ticks;
        if (telemetry_ticks >= rip::config::kTelemetryPeriodTicks) {
            telemetry_ticks = 0;
            rip::usb::send_hid_snapshot(snapshot);
        }

        rip::platform::watchdog_feed();
        rip::usb::task();
        service_commissioning(runtime, commissioning_motor);
    }
}
