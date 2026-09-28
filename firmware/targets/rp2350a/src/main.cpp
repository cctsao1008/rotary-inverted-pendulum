#include <cmath>
#include <cstdint>

#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "rip/arm_servo.hpp"
#include "rip/board.hpp"
#include "rip/commissioning.hpp"
#include "rip/config.hpp"
#include "rip/platform.hpp"
#include "rip/runtime.hpp"
#include "rip/usb.hpp"

namespace {

constexpr float kArmServoMaxTargetSpeedRadS = 20.0f;
constexpr float kArmServoMaxCommand = 0.60f;
constexpr float kArmServoDefaultMaxCommand = 0.50f;
constexpr float kArmServoMaxPositionStepRad = 0.80f;
constexpr std::uint32_t kArmServoDefaultLeaseMs = 2000;
constexpr std::uint32_t kArmServoMaxLeaseMs = 5000;

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

struct ArmServoLease {
    std::uint64_t deadline_us{0};

    void arm(std::uint64_t now_us, std::uint32_t lease_ms) {
        deadline_us = now_us + static_cast<std::uint64_t>(lease_ms) * 1000u;
    }

    void clear() { deadline_us = 0; }

    bool expired(std::uint64_t now_us) const {
        return deadline_us != 0 && now_us > deadline_us;
    }
};

std::uint32_t servo_lease_ms(std::uint32_t requested) {
    return requested == 0 ? kArmServoDefaultLeaseMs : requested;
}

float servo_max_command(float requested) {
    return requested == 0.0f ? kArmServoDefaultMaxCommand : requested;
}

void stop_motor_paths(CommissioningMotor& motor, rip::ArmServo& servo, ArmServoLease& servo_lease) {
    motor.clear();
    servo.stop();
    servo_lease.clear();
    rip::platform::safe_off();
}

void service_commissioning(rip::ControlRuntime& runtime, CommissioningMotor& motor,
                           rip::ArmServo& servo, ArmServoLease& servo_lease,
                           bool& enter_usb_bootloader) {
    rip::commissioning::Request request{};
    while (rip::commissioning::take_request(request)) {
        const std::uint64_t now_us = rip::platform::now_us();
        switch (request.command) {
            case rip::commissioning::Command::GetStatus: {
                const std::uint32_t detail =
                    static_cast<std::uint32_t>(runtime.runtime_state()) |
                    (static_cast<std::uint32_t>(runtime.authority_mode()) << 8) |
                    (motor.active ? (1u << 16) : 0u) |
                    (servo.active() ? (1u << 17) : 0u) |
                    (static_cast<std::uint32_t>(servo.mode()) << 18);
                rip::commissioning::queue_ack(
                    request,
                    rip::commissioning::Status::Ok,
                    detail,
                    motor.active ? motor.command : servo.target(),
                    static_cast<float>(rip::platform::encoder_illegal_transition_count()));
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
                servo.stop();
                servo_lease.clear();
                motor.active = true;
                motor.command = request.value0;
                motor.deadline_us = now_us + static_cast<std::uint64_t>(lease_ms) * 1000u;
                rip::commissioning::queue_ack(request, rip::commissioning::Status::Ok, 0,
                                               motor.command, 0.0f);
                break;
            }
            case rip::commissioning::Command::SetArmVelocity: {
                const std::uint32_t lease_ms = servo_lease_ms(request.duration_ms);
                const float max_command = servo_max_command(request.value1);
                if (!std::isfinite(request.value0) ||
                    std::fabs(request.value0) > kArmServoMaxTargetSpeedRadS ||
                    !std::isfinite(max_command) || max_command <= 0.0f ||
                    max_command > kArmServoMaxCommand || lease_ms > kArmServoMaxLeaseMs) {
                    rip::commissioning::queue_ack(request, rip::commissioning::Status::Range);
                    break;
                }
                motor.clear();
                if (!servo.set_velocity(request.value0, max_command)) {
                    rip::commissioning::queue_ack(request, rip::commissioning::Status::Invalid);
                    break;
                }
                servo_lease.arm(now_us, lease_ms);
                rip::commissioning::queue_ack(request, rip::commissioning::Status::Ok, 0,
                                               request.value0, max_command);
                break;
            }
            case rip::commissioning::Command::SetArmPosition: {
                const std::uint32_t lease_ms = servo_lease_ms(request.duration_ms);
                const float max_command = servo_max_command(request.value1);
                const float delta = request.value0 - servo.state().position_rad;
                if (!std::isfinite(request.value0) || !std::isfinite(max_command) ||
                    max_command <= 0.0f || max_command > kArmServoMaxCommand ||
                    std::fabs(delta) > kArmServoMaxPositionStepRad ||
                    lease_ms > kArmServoMaxLeaseMs) {
                    rip::commissioning::queue_ack(request, rip::commissioning::Status::Range);
                    break;
                }
                motor.clear();
                if (!servo.set_position(request.value0, max_command)) {
                    rip::commissioning::queue_ack(request, rip::commissioning::Status::Invalid);
                    break;
                }
                servo_lease.arm(now_us, lease_ms);
                rip::commissioning::queue_ack(request, rip::commissioning::Status::Ok, 0,
                                               request.value0, max_command);
                break;
            }
            case rip::commissioning::Command::SafeOff:
                stop_motor_paths(motor, servo, servo_lease);
                rip::commissioning::queue_ack(request, rip::commissioning::Status::Ok);
                break;
            case rip::commissioning::Command::SetUserLed: {
                if (!std::isfinite(request.value0) ||
                    (request.value0 != 0.0f && request.value0 != 1.0f)) {
                    rip::commissioning::queue_ack(request, rip::commissioning::Status::Range);
                    break;
                }
                const bool on = request.value0 == 1.0f;
                rip::board::set_user_led(on);
                rip::commissioning::queue_ack(request, rip::commissioning::Status::Ok, 0,
                                               on ? 1.0f : 0.0f, 0.0f);
                break;
            }
            case rip::commissioning::Command::SetNeopixel: {
                if (!std::isfinite(request.value0)) {
                    rip::commissioning::queue_ack(request, rip::commissioning::Status::Range);
                    break;
                }
                const int color = static_cast<int>(request.value0);
                if (color < 0 || color > static_cast<int>(rip::board::NeopixelColor::White) ||
                    request.value0 != static_cast<float>(color)) {
                    rip::commissioning::queue_ack(request, rip::commissioning::Status::Range);
                    break;
                }
                rip::board::set_neopixel(static_cast<rip::board::NeopixelColor>(color));
                rip::commissioning::queue_ack(request, rip::commissioning::Status::Ok, 0,
                                               static_cast<float>(color), 0.0f);
                break;
            }
            case rip::commissioning::Command::EnterUsbBootloader:
                stop_motor_paths(motor, servo, servo_lease);
                enter_usb_bootloader = true;
                rip::commissioning::queue_ack(request, rip::commissioning::Status::Ok);
                break;
            default:
                rip::commissioning::queue_ack(request, rip::commissioning::Status::Invalid);
                break;
        }
    }
    rip::commissioning::service();
}

[[noreturn]] void enter_usb_bootloader() {
    rip::platform::safe_off();

    for (int i = 0; i < 50; ++i) {
        rip::usb::task();
        rip::commissioning::service();
        rip::platform::watchdog_feed();
        sleep_ms(1);
    }

    rom_reset_usb_boot(0, 1);
}

rip::BoundedActuatorCommand normalized_motor_command(float command) {
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
    rip::ArmServo arm_servo({});
    ArmServoLease arm_servo_lease;
    arm_servo.reset(rip::platform::read_arm_encoder_count(), started_at);
    bool usb_bootloader_requested = false;

    rip::RuntimeSnapshot snapshot{};
    rip::usb::set_snapshot_source(&snapshot);
    rip::usb::log("boot,target=rp2350a,board=uno_rp2350,runtime=feature-parity,motor_command=0\r\n");

    std::uint32_t sample_index = 0;
    std::uint32_t telemetry_ticks = 0;

    while (true) {
        rip::usb::task();
        service_commissioning(runtime, commissioning_motor, arm_servo, arm_servo_lease,
                              usb_bootloader_requested);
        if (usb_bootloader_requested) enter_usb_bootloader();

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

        const rip::ArmServoCommand servo_command =
            arm_servo.step(raw.arm_encoder.accumulated_count, captured_at);

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
        if (arm_servo.active() && arm_servo_lease.expired(captured_at)) {
            arm_servo.stop();
            arm_servo_lease.clear();
        }

        rip::BoundedActuatorCommand applied_command{};
        if (commissioning_motor.active) {
            applied_command = normalized_motor_command(commissioning_motor.command);
            rip::platform::apply_tb6612(rip::map_tb6612(applied_command));
        } else if (arm_servo.active()) {
            applied_command = normalized_motor_command(servo_command.normalized_command);
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
        // Arm servo state is encoder-only and remains meaningful even when the
        // pendulum ADC path has not been commissioned yet.
        snapshot.state.phi = arm_servo.state().position_rad;
        snapshot.state.phi_dot = arm_servo.state().velocity_rad_s;
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
        service_commissioning(runtime, commissioning_motor, arm_servo, arm_servo_lease,
                              usb_bootloader_requested);
        if (usb_bootloader_requested) enter_usb_bootloader();
    }
}
