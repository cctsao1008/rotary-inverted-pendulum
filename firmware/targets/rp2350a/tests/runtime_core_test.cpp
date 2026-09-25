#include <cassert>
#include <cmath>

#include "rip/config.hpp"
#include "rip/maintenance.hpp"
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

    // Static actuator inverse and TB6612 sign mapping preserve canonical semantics.
    rip::ArmActuatorModel actuator(0.05f, 0.0f);
    rip::BoundedActuatorCommand command{};
    assert(actuator.command_for_demand({0.025f}, command));
    assert(near(command.command, 0.5f));
    assert(near(command.predicted_arm_torque_nm, 0.025f));
    const auto frame = rip::map_tb6612(command);
    assert(frame.mode == rip::Tb6612BridgeMode::DrivePositive);
    assert(near(frame.duty_fraction, 0.5f));

    // Safety gate starts from zero authority and then earns command through slew.
    rip::CommandSafetyGate safety;
    safety.configure({0.8f, 10.0f});
    rip::BoundedActuatorCommand safe{};
    assert(safety.constrain(command, {1000}, actuator, safe));
    assert(near(safe.command, 0.0f));
    assert(safety.constrain(command, {11000}, actuator, safe));
    assert(near(safe.command, 0.1f));

    // HID commissioning maintenance authority is explicit, slew bounded, and
    // expires to zero authority if the host stops refreshing its finite lease.
    rip::MaintenanceAuthority maintenance;
    assert(maintenance.enter(1000));
    assert(maintenance.active());
    assert(maintenance.set_command(0.10f, 101000, 250));
    assert(near(maintenance.command().command, 0.10f));
    maintenance.tick(351001);
    assert(!maintenance.active());
    assert(near(maintenance.command().command, 0.0f));
    assert(!maintenance.set_command(0.10f, 352000, 250));

    return 0;
}
