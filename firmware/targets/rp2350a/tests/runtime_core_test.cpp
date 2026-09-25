#include <cassert>
#include <cmath>
#include <limits>

#include "rip/config.hpp"
#include "rip/runtime.hpp"

namespace {

bool near(float a, float b, float eps = 1.0e-4f) { return std::fabs(a - b) < eps; }

rip::EstimatedState state(float theta, float theta_dot, float phi = 0.0f, float phi_dot = 0.0f) {
    rip::EstimatedState s{};
    s.timestamp.value = 1000;
    s.theta = theta;
    s.theta_dot = theta_dot;
    s.phi = phi;
    s.phi_dot = phi_dot;
    s.validity = rip::StateValidity::Valid;
    return s;
}

}  // namespace

int main() {
    // Measurement mapping keeps the STM32 target's current reference calibration.
    rip::MeasurementAdapter adapter;
    rip::RawObservation raw{};
    raw.pendulum.captured_at.value = 100;
    raw.pendulum.adc_raw = rip::config::kPendulumUprightAdc + 100;
    raw.pendulum.quality = rip::MeasurementAvailable | rip::MeasurementIoOk | rip::MeasurementTimingValid;
    raw.arm_encoder.captured_at.value = 101;
    raw.arm_encoder.accumulated_count = 260;
    raw.arm_encoder.quality = rip::MeasurementAvailable | rip::MeasurementIoOk | rip::MeasurementTimingValid;
    rip::EstimatorMeasurement measurement{};
    assert(adapter.convert(raw, measurement));
    assert(near(measurement.theta, 100.0f * rip::config::kPendulumRadiansPerCount));
    assert(near(measurement.phi, rip::config::kPi * 0.5f));
    assert(measurement.captured_at.value == 101);

    // Missing or stale source quality must not enter the estimator.
    auto invalid_raw = raw;
    invalid_raw.pendulum.quality = rip::MeasurementAvailable | rip::MeasurementIoOk;
    assert(!adapter.convert(invalid_raw, measurement));
    invalid_raw = raw;
    invalid_raw.arm_encoder.quality |= rip::MeasurementStale;
    assert(!adapter.convert(invalid_raw, measurement));

    // Estimator uses shortest circular delta for theta and continuous delta for phi.
    rip::BasicEstimator estimator;
    rip::EstimatorConfig estimator_cfg{20000, 1.0f};
    rip::EstimatedState estimated{};
    rip::EstimatorMeasurement a{179.0f * rip::config::kPi / 180.0f, 0.0f, {1000}};
    rip::EstimatorMeasurement b{-179.0f * rip::config::kPi / 180.0f, 0.1f, {11000}};
    assert(estimator.step(estimator_cfg, a, estimated) == rip::BasicEstimator::Result::Primed);
    assert(estimator.step(estimator_cfg, b, estimated) == rip::BasicEstimator::Result::Ready);
    assert(near(estimated.theta_dot, (2.0f * rip::config::kPi / 180.0f) / 0.01f, 1.0e-3f));
    assert(near(estimated.phi_dot, 10.0f, 1.0e-3f));

    // A gap beyond the configured estimator horizon re-primes rather than differentiating stale data.
    rip::BasicEstimator gap_estimator;
    rip::EstimatorMeasurement gap_a{0.0f, 0.0f, {1000}};
    rip::EstimatorMeasurement gap_b{0.5f, 0.5f, {22000}};
    assert(gap_estimator.step(estimator_cfg, gap_a, estimated) == rip::BasicEstimator::Result::Primed);
    assert(gap_estimator.step(estimator_cfg, gap_b, estimated) == rip::BasicEstimator::Result::Primed);

    // Capture is angle-driven, then requires the configured upright settle window.
    rip::HybridController controller(
        {rip::config::kPendulumMassKg, rip::config::kPendulumComLengthM,
         rip::config::kPendulumInertiaKgM2, rip::config::kGravityMps2,
         rip::config::kTargetEnergyJ, rip::config::kEnergyTorqueGain,
         rip::config::kMaxAbsTorqueNm, rip::config::kSwingKickTorqueNm,
         rip::config::kSwingKickBelowRateRadS},
        {rip::config::kCaptureEnterAngleRad, rip::config::kCaptureEnterRateRadS,
         rip::config::kBalanceEnterAngleRad, rip::config::kBalanceEnterRateRadS,
         rip::config::kBalanceExitAngleRad, rip::config::kBalanceExitRateRadS,
         rip::config::kCaptureExitAngleRad, rip::config::kCaptureExitRateRadS,
         rip::config::kCaptureSettleCycles});
    rip::GeneralizedDemand demand{};
    assert(controller.compute(state(0.2f, 20.0f), demand));
    assert(controller.regime() == rip::ControlRegime::Capture);
    for (std::uint16_t i = 0; i < rip::config::kCaptureSettleCycles; ++i) {
        assert(controller.compute(state(0.01f, 0.01f), demand));
    }
    assert(controller.regime() == rip::ControlRegime::Balance);
    assert(controller.compute(state(1.0f, 0.0f), demand));
    assert(controller.regime() == rip::ControlRegime::SwingUp);

    // Static actuator inverse and TB6612 sign mapping preserve canonical semantics.
    rip::ArmActuatorModel actuator(0.05f, 0.0f);
    rip::BoundedActuatorCommand command{};
    assert(actuator.command_for_demand({0.025f}, command));
    assert(near(command.command, 0.5f));
    assert(near(command.predicted_arm_torque_nm, 0.025f));
    const auto positive_frame = rip::map_tb6612(command);
    assert(positive_frame.mode == rip::Tb6612BridgeMode::DrivePositive);
    assert(near(positive_frame.duty_fraction, 0.5f));

    command.command = -0.25f;
    const auto negative_frame = rip::map_tb6612(command);
    assert(negative_frame.mode == rip::Tb6612BridgeMode::DriveNegative);
    assert(near(negative_frame.duty_fraction, 0.25f));

    const auto inverted_frame = rip::map_tb6612(command, false);
    assert(inverted_frame.mode == rip::Tb6612BridgeMode::DrivePositive);
    assert(near(inverted_frame.duty_fraction, 0.25f));

    command.command = 0.0f;
    const auto coast_frame = rip::map_tb6612(command);
    assert(coast_frame.mode == rip::Tb6612BridgeMode::Coast);
    assert(near(coast_frame.duty_fraction, 0.0f));

    command.command = std::numeric_limits<float>::quiet_NaN();
    const auto nonfinite_frame = rip::map_tb6612(command);
    assert(nonfinite_frame.mode == rip::Tb6612BridgeMode::Coast);
    assert(near(nonfinite_frame.duty_fraction, 0.0f));

    rip::BoundedActuatorCommand saturated{};
    assert(actuator.command_for_demand({0.10f}, saturated));
    assert(saturated.saturated);
    assert(near(saturated.command, 1.0f));

    // Closed-loop safety semantics remain unchanged from the production path.
    command.command = 0.5f;
    command.saturated = false;
    command.predicted_arm_torque_nm = actuator.predicted_torque(command.command);
    rip::CommandSafetyGate safety;
    safety.configure({0.8f, 10.0f});
    assert(safety.configured());
    rip::BoundedActuatorCommand safe{};
    assert(safety.constrain(command, {1000}, actuator, safe));
    assert(near(safe.command, 0.0f));
    assert(safety.constrain(command, {11000}, actuator, safe));
    assert(near(safe.command, 0.1f));

    rip::CommandSafetyGate invalid_safety;
    invalid_safety.configure({1.1f, 10.0f});
    assert(!invalid_safety.configured());

    // Timing/watchdog state machines are deterministic and hardware-independent.
    rip::SensorTimingMonitor timing(1000, 5000, 20000, 1000);
    assert(timing.on_event(2000) == rip::SensorTimingHealth::Startup);
    assert(timing.on_event(3000) == rip::SensorTimingHealth::Healthy);
    assert(timing.poll(9000) == rip::SensorTimingHealth::Late);
    assert(timing.poll(24000) == rip::SensorTimingHealth::Timeout);

    rip::ControlWatchdog watchdog(20000);
    assert(watchdog.health(1000) == rip::WatchdogHealth::Disarmed);
    watchdog.kick(1000);
    assert(watchdog.health(21000) == rip::WatchdogHealth::Healthy);
    assert(watchdog.health(21001) == rip::WatchdogHealth::Expired);
    watchdog.disarm();
    assert(watchdog.health(50000) == rip::WatchdogHealth::Disarmed);

    return 0;
}
