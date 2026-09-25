#include <cstdint>

#include "rip/config.hpp"
#include "rip/platform.hpp"
#include "rip/runtime.hpp"
#include "rip/usb.hpp"

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

    // Keep the physical-authority policy aligned with main: the full sensing,
    // estimator, hybrid-control, actuator-model and TB6612 paths exist, but no
    // automatic closed-loop request is made at boot.

    rip::RuntimeSnapshot snapshot{};
    rip::usb::set_snapshot_source(&snapshot);
    rip::usb::log("boot,target=rp2350a,board=uno_rp2350,runtime=feature-parity,motor_authority=0\r\n");

    std::uint32_t sample_index = 0;
    std::uint32_t telemetry_ticks = 0;

    while (true) {
        rip::usb::task();
        const rip::platform::SchedulerEvidence scheduler = rip::platform::wait_next_opportunity();
        const std::uint64_t cycle_started = rip::platform::now_us();

        const rip::SensorTimingHealth timing = timing_monitor.on_event(cycle_started);
        control_watchdog.kick(cycle_started);

        rip::RawObservation raw{};
        raw.sample_index = sample_index++;
        raw.pendulum.captured_at.value = cycle_started;
        raw.pendulum.adc_raw = rip::platform::read_pendulum_adc();
        raw.pendulum.quality = rip::MeasurementAvailable | rip::MeasurementIoOk |
                               rip::MeasurementTimingValid;
        raw.arm_encoder.captured_at.value = cycle_started;
        raw.arm_encoder.accumulated_count = rip::platform::read_arm_encoder_count();
        raw.arm_encoder.quality = rip::MeasurementAvailable | rip::MeasurementIoOk |
                                  rip::MeasurementTimingValid;

        rip::EstimatorMeasurement measurement{};
        const bool measurement_ok = adapter.convert(raw, measurement);

        rip::RuntimeObservation observation{};
        observation.measurement = measurement;
        observation.sensor_valid = measurement_ok;
        observation.sample_age_us = 0;
        observation.timing = timing;
        observation.watchdog = control_watchdog.health(cycle_started);

        const rip::ControlCycle cycle = runtime.step(observation);

        // Physical output is reachable only through an AuthorizedActuation
        // result. With the main-equivalent boot policy this remains safe-off.
        if (cycle.kind == rip::ControlCycle::Kind::Computed && cycle.authorized) {
            rip::platform::apply_tb6612(rip::map_tb6612(cycle.bounded_command));
        } else {
            rip::platform::safe_off();
        }

        snapshot.timestamp_us = cycle_started;
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
            snapshot.command = cycle.bounded_command;
        }

        const std::uint64_t cycle_finished = rip::platform::now_us();
        snapshot.execution_time_us = static_cast<std::uint32_t>(cycle_finished - cycle_started);

        ++telemetry_ticks;
        if (telemetry_ticks >= rip::config::kTelemetryPeriodTicks) {
            telemetry_ticks = 0;
            rip::usb::send_hid_snapshot(snapshot);
        }

        rip::platform::watchdog_feed();
        rip::usb::task();
    }
}
