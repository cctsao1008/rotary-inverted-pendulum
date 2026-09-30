#include "rip/runtime.hpp"

#include <algorithm>
#include <cmath>

#include "rip/config.hpp"

namespace rip {
namespace {

constexpr std::uint16_t kQualSensorInvalid = 1u << 1;
constexpr std::uint16_t kQualEstimateNotReady = 1u << 2;
constexpr std::uint16_t kQualStateNonfinite = 1u << 3;

float wrap_pi(float value) {
    constexpr float two_pi = 2.0f * config::kPi;
    value = std::fmod(value + config::kPi, two_pi);
    if (value < 0.0f) value += two_pi;
    return value - config::kPi;
}

bool usable(std::uint8_t quality) {
    constexpr std::uint8_t required = MeasurementAvailable | MeasurementIoOk | MeasurementTimingValid;
    return (quality & required) == required && (quality & MeasurementStale) == 0;
}

float effective_command(float command, float deadzone) {
    const float magnitude = std::fabs(command);
    if (magnitude <= deadzone) return 0.0f;
    return std::copysign((magnitude - deadzone) / (1.0f - deadzone), command);
}

float inverse_effective_command(float effective, float deadzone) {
    if (effective == 0.0f) return 0.0f;
    return std::copysign(deadzone + (1.0f - deadzone) * std::fabs(effective), effective);
}

bool within(const EstimatedState& state, float angle, float rate) {
    return std::fabs(state.theta) <= angle && std::fabs(state.theta_dot) <= rate;
}

}  // namespace

bool EstimatedState::finite() const {
    return std::isfinite(theta) && std::isfinite(theta_dot) && std::isfinite(phi) &&
           std::isfinite(phi_dot);
}

bool MeasurementAdapter::convert(const RawObservation& raw, EstimatorMeasurement& out) const {
    if (!usable(raw.pendulum.quality) || !usable(raw.arm_encoder.quality)) return false;

    const std::int32_t adc_delta = static_cast<std::int32_t>(raw.pendulum.adc_raw) -
                                   static_cast<std::int32_t>(config::kPendulumUprightAdc);
    out.theta = wrap_pi(static_cast<float>(adc_delta) * config::kPendulumRadiansPerCount *
                        static_cast<float>(config::kPendulumDirection));
    out.phi = static_cast<float>(raw.arm_encoder.accumulated_count) *
              (2.0f * config::kPi / config::kArmEncoderCountsPerRevolution) *
              static_cast<float>(config::kArmEncoderDirection);
    out.captured_at.value = std::max(raw.pendulum.captured_at.value, raw.arm_encoder.captured_at.value);
    return true;
}

BasicEstimator::Result BasicEstimator::step(const EstimatorConfig& config,
                                            const EstimatorMeasurement& measurement,
                                            EstimatedState& out) {
    if (!std::isfinite(measurement.theta) || !std::isfinite(measurement.phi) ||
        config.max_gap_us == 0 || !std::isfinite(config.rate_filter_alpha) ||
        config.rate_filter_alpha < 0.0f || config.rate_filter_alpha > 1.0f) {
        return Result::Error;
    }

    if (!primed_) {
        primed_ = true;
        previous_ = measurement;
        filtered_theta_dot_ = 0.0f;
        filtered_phi_dot_ = 0.0f;
        return Result::Primed;
    }

    if (measurement.captured_at.value <= previous_.captured_at.value) return Result::Error;
    const std::uint64_t delta_us = measurement.captured_at.value - previous_.captured_at.value;
    if (delta_us > config.max_gap_us) {
        previous_ = measurement;
        filtered_theta_dot_ = 0.0f;
        filtered_phi_dot_ = 0.0f;
        return Result::Primed;
    }

    const float dt = static_cast<float>(delta_us) * 1.0e-6f;
    const float raw_theta_dot = wrap_pi(measurement.theta - previous_.theta) / dt;
    const float raw_phi_dot = (measurement.phi - previous_.phi) / dt;
    const float alpha = config.rate_filter_alpha;
    filtered_theta_dot_ += alpha * (raw_theta_dot - filtered_theta_dot_);
    filtered_phi_dot_ += alpha * (raw_phi_dot - filtered_phi_dot_);
    previous_ = measurement;

    out.timestamp = measurement.captured_at;
    out.theta = measurement.theta;
    out.theta_dot = filtered_theta_dot_;
    out.phi = measurement.phi;
    out.phi_dot = filtered_phi_dot_;
    out.validity = StateValidity::Valid;
    return out.finite() ? Result::Ready : Result::Error;
}

void BasicEstimator::reset() {
    primed_ = false;
    previous_ = {};
    filtered_theta_dot_ = 0.0f;
    filtered_phi_dot_ = 0.0f;
}

ControlRegime CapturePolicy::update(const EstimatedState& state) {
    if (state.validity != StateValidity::Valid || !state.finite()) return regime_;

    switch (regime_) {
        case ControlRegime::SwingUp:
            if (std::fabs(state.theta) <= config_.capture_enter_angle_rad) {
                settled_cycles_ = 0;
                regime_ = ControlRegime::Capture;
            }
            break;
        case ControlRegime::Capture:
            if (std::fabs(state.theta) > config_.capture_exit_angle_rad) {
                settled_cycles_ = 0;
                regime_ = ControlRegime::SwingUp;
            } else if (within(state, config_.balance_enter_angle_rad,
                              config_.balance_enter_rate_rad_s)) {
                if (settled_cycles_ < UINT16_MAX) ++settled_cycles_;
                if (settled_cycles_ >= config_.settle_cycles) {
                    settled_cycles_ = 0;
                    regime_ = ControlRegime::Balance;
                }
            } else {
                settled_cycles_ = 0;
            }
            break;
        case ControlRegime::Balance:
            if (std::fabs(state.theta) > config_.capture_exit_angle_rad) {
                settled_cycles_ = 0;
                regime_ = ControlRegime::SwingUp;
            } else if (!within(state, config_.balance_exit_angle_rad,
                               config_.balance_exit_rate_rad_s)) {
                settled_cycles_ = 0;
                regime_ = ControlRegime::Capture;
            }
            break;
    }
    return regime_;
}

void CapturePolicy::reset() {
    regime_ = ControlRegime::SwingUp;
    settled_cycles_ = 0;
}

HybridController::HybridController(EnergySwingUpConfig swing, CapturePolicyConfig capture)
    : swing_(swing), capture_(capture) {}

float HybridController::lqr_full_state(const EstimatedState& state) const {
    const float x[4] = {state.theta, state.theta_dot, state.phi, state.phi_dot};
    float u = 0.0f;
    for (int i = 0; i < 4; ++i) u -= config::kLqrGains[i] * x[i];
    return u;
}

float HybridController::lqr_capture_projection(const EstimatedState& state) const {
    return -(config::kLqrGains[0] * state.theta + config::kLqrGains[1] * state.theta_dot);
}

bool HybridController::swing_up(const EstimatedState& state, GeneralizedDemand& out) const {
    if (state.validity != StateValidity::Valid || !state.finite()) return false;
    const float potential = swing_.pendulum_mass_kg * swing_.gravity_m_s2 *
                            swing_.pendulum_com_length_m * (1.0f + std::cos(state.theta));
    const float kinetic = 0.5f * swing_.pendulum_inertia_kg_m2 * state.theta_dot * state.theta_dot;
    const float energy_error = swing_.target_energy_j - (potential + kinetic);
    float torque = swing_.energy_gain * energy_error * state.theta_dot * (-std::cos(state.theta));
    if (!std::isfinite(torque)) return false;

    if (energy_error > 0.0f && std::fabs(state.theta_dot) <= swing_.kick_below_rate_rad_s &&
        std::fabs(torque) < swing_.kick_torque_nm && swing_.kick_torque_nm > 0.0f) {
        torque = state.theta < 0.0f ? -swing_.kick_torque_nm : swing_.kick_torque_nm;
    }
    out.arm_torque_nm = std::clamp(torque, -swing_.max_abs_torque_nm, swing_.max_abs_torque_nm);
    return true;
}

bool HybridController::compute(const EstimatedState& state, GeneralizedDemand& out) {
    if (state.validity != StateValidity::Valid || !state.finite()) return false;
    const ControlRegime regime = capture_.update(state);
    switch (regime) {
        case ControlRegime::SwingUp:
            return swing_up(state, out);
        case ControlRegime::Capture:
            out.arm_torque_nm = lqr_capture_projection(state);
            break;
        case ControlRegime::Balance:
            out.arm_torque_nm = lqr_full_state(state);
            break;
    }
    return std::isfinite(out.arm_torque_nm);
}

void HybridController::reset() { capture_.reset(); }

ArmActuatorModel::ArmActuatorModel(float torque_per_effective_command_nm, float command_deadzone)
    : torque_span_nm_(torque_per_effective_command_nm), deadzone_(command_deadzone) {}

float ArmActuatorModel::predicted_torque(float normalized_command) const {
    return effective_command(normalized_command, deadzone_) * torque_span_nm_;
}

bool ArmActuatorModel::command_for_demand(GeneralizedDemand demand, BoundedActuatorCommand& out) const {
    if (!std::isfinite(demand.arm_torque_nm) || !(torque_span_nm_ > 0.0f) || deadzone_ < 0.0f ||
        deadzone_ >= 1.0f) return false;
    const float required_effective = demand.arm_torque_nm / torque_span_nm_;
    const float bounded_effective = std::clamp(required_effective, -1.0f, 1.0f);
    const float command = inverse_effective_command(bounded_effective, deadzone_);
    out.command = std::clamp(command, -1.0f, 1.0f);
    out.saturated = std::fabs(required_effective) > 1.0f;
    out.predicted_arm_torque_nm = predicted_torque(out.command);
    return true;
}

void CommandSafetyGate::configure(CommandSafetyLimits limits) {
    limits_ = limits;
    configured_ = std::isfinite(limits.max_abs_command) && limits.max_abs_command > 0.0f &&
                  limits.max_abs_command <= 1.0f && std::isfinite(limits.max_slew_per_s) &&
                  limits.max_slew_per_s > 0.0f;
    reset_history();
}

void CommandSafetyGate::reset_history() {
    last_command_ = 0.0f;
    has_timestamp_ = false;
    last_timestamp_ = {};
}

bool CommandSafetyGate::constrain(BoundedActuatorCommand requested, TimestampUs timestamp,
                                  const ArmActuatorModel& actuator, BoundedActuatorCommand& out) {
    if (!configured_) return false;
    float bounded = std::clamp(requested.command, -limits_.max_abs_command, limits_.max_abs_command);
    if (!has_timestamp_) {
        bounded = 0.0f;
    } else {
        if (timestamp.value < last_timestamp_.value) return false;
        const float max_delta = limits_.max_slew_per_s *
                                static_cast<float>(timestamp.value - last_timestamp_.value) * 1.0e-6f;
        bounded = std::clamp(bounded, last_command_ - max_delta, last_command_ + max_delta);
    }
    last_command_ = bounded;
    last_timestamp_ = timestamp;
    has_timestamp_ = true;
    out = requested;
    out.command = bounded;
    out.predicted_arm_torque_nm = actuator.predicted_torque(bounded);
    return true;
}

SensorTimingMonitor::SensorTimingMonitor(std::uint64_t expected_period_us,
                                         std::uint64_t late_after_us,
                                         std::uint64_t timeout_after_us,
                                         std::uint64_t started_at_us)
    : expected_period_us_(expected_period_us), late_after_us_(late_after_us),
      timeout_after_us_(timeout_after_us), started_at_us_(started_at_us) {}

SensorTimingHealth SensorTimingMonitor::classify(std::uint64_t elapsed_us) const {
    if (elapsed_us >= timeout_after_us_) return SensorTimingHealth::Timeout;
    if (elapsed_us >= late_after_us_) return SensorTimingHealth::Late;
    return SensorTimingHealth::Healthy;
}

SensorTimingHealth SensorTimingMonitor::on_event(std::uint64_t event_at_us) {
    if (has_event_) {
        cadence_verified_ = true;
        health_ = classify(event_at_us - last_event_at_us_);
    } else {
        health_ = SensorTimingHealth::Startup;
    }
    last_event_at_us_ = event_at_us;
    has_event_ = true;
    return health_;
}

SensorTimingHealth SensorTimingMonitor::poll(std::uint64_t now_us) {
    const std::uint64_t reference = has_event_ ? last_event_at_us_ : started_at_us_;
    const std::uint64_t elapsed = now_us - reference;
    health_ = (!cadence_verified_ && elapsed < timeout_after_us_) ? SensorTimingHealth::Startup
                                                                  : classify(elapsed);
    return health_;
}

void ControlWatchdog::kick(std::uint64_t now_us) {
    last_kick_us_ = now_us;
    armed_ = true;
}

void ControlWatchdog::disarm() { armed_ = false; }

WatchdogHealth ControlWatchdog::health(std::uint64_t now_us) const {
    if (!armed_) return WatchdogHealth::Disarmed;
    return now_us - last_kick_us_ <= timeout_us_ ? WatchdogHealth::Healthy : WatchdogHealth::Expired;
}

ControlRuntime::ControlRuntime()
    : estimator_config_{config::kEstimatorMaxGapUs, config::kEstimatorRateFilterAlpha},
      controller_({config::kPendulumMassKg, config::kPendulumComLengthM,
                   config::kPendulumInertiaKgM2, config::kGravityMps2, config::kTargetEnergyJ,
                   config::kEnergyTorqueGain, config::kMaxAbsTorqueNm, config::kSwingKickTorqueNm,
                   config::kSwingKickBelowRateRadS},
                  {config::kCaptureEnterAngleRad, config::kCaptureEnterRateRadS,
                   config::kBalanceEnterAngleRad, config::kBalanceEnterRateRadS,
                   config::kBalanceExitAngleRad, config::kBalanceExitRateRadS,
                   config::kCaptureExitAngleRad, config::kCaptureExitRateRadS,
                   config::kCaptureSettleCycles}),
      actuator_(config::kActuatorTorquePerEffectiveCommandNm, config::kActuatorCommandDeadzone) {}

RuntimeQualification ControlRuntime::qualify(const RuntimeObservation& observation,
                                             const EstimatedState& state) const {
    RuntimeQualification q{true, 0};
    if (!observation.sensor_valid) q.reasons |= kQualSensorInvalid;
    if (state.validity != StateValidity::Valid) q.reasons |= kQualEstimateNotReady;
    if (!state.finite()) q.reasons |= kQualStateNonfinite;
    q.allowed = q.reasons == 0;
    return q;
}

RuntimeState ControlRuntime::active_state_for(ControlRegime regime) const {
    switch (regime) {
        case ControlRegime::SwingUp: return RuntimeState::ActiveSwingUp;
        case ControlRegime::Capture: return RuntimeState::ActiveCapture;
        case ControlRegime::Balance: return RuntimeState::ActiveBalance;
    }
    return RuntimeState::Fault;
}

void ControlRuntime::update_authority(const RuntimeObservation& observation,
                                      const EstimatedState& state,
                                      const RuntimeQualification& qualification) {
    const bool active = runtime_state_ == RuntimeState::ActiveSwingUp ||
                        runtime_state_ == RuntimeState::ActiveCapture ||
                        runtime_state_ == RuntimeState::ActiveBalance;
    if (active) {
        const bool permit = observation.timing == SensorTimingHealth::Healthy &&
                            observation.watchdog == WatchdogHealth::Healthy &&
                            state.validity == StateValidity::Valid && qualification.allowed &&
                            authority_mode_ == AuthorityMode::ClosedLoop;
        if (!permit) {
            closed_loop_requested_ = false;
            authority_mode_ = AuthorityMode::Disarmed;
            runtime_state_ = RuntimeState::Ready;
            command_safety_.reset_history();
        }
        return;
    }

    if (runtime_state_ == RuntimeState::Ready && closed_loop_requested_ &&
        command_safety_.configured() && observation.timing == SensorTimingHealth::Healthy &&
        observation.watchdog == WatchdogHealth::Healthy && state.validity == StateValidity::Valid &&
        qualification.allowed && authority_mode_ == AuthorityMode::Disarmed) {
        authority_mode_ = AuthorityMode::ClosedLoop;
        runtime_state_ = active_state_for(requested_regime_);
    }
}

ControlCycle ControlRuntime::step(const RuntimeObservation& observation) {
    ControlCycle cycle{};
    EstimatedState state{};
    const auto estimate = estimator_.step(estimator_config_, observation.measurement, state);
    if (estimate == BasicEstimator::Result::Error) {
        cycle.kind = ControlCycle::Kind::Error;
        return cycle;
    }
    if (estimate == BasicEstimator::Result::Primed) {
        if (runtime_state_ == RuntimeState::ActiveSwingUp || runtime_state_ == RuntimeState::ActiveCapture ||
            runtime_state_ == RuntimeState::ActiveBalance) cancel_closed_loop();
        cycle.kind = ControlCycle::Kind::Primed;
        return cycle;
    }

    cycle.state = state;
    cycle.qualification = qualify(observation, state);
    update_authority(observation, state, cycle.qualification);
    if (!cycle.qualification.allowed) {
        cycle.kind = ControlCycle::Kind::Rejected;
        return cycle;
    }

    if (!controller_.compute(state, cycle.demand) ||
        !actuator_.command_for_demand(cycle.demand, cycle.bounded_command)) {
        cycle.kind = ControlCycle::Kind::Error;
        return cycle;
    }

    if (authority_mode_ == AuthorityMode::ClosedLoop) {
        BoundedActuatorCommand safe{};
        if (!command_safety_.constrain(cycle.bounded_command, state.timestamp, actuator_, safe)) {
            cancel_closed_loop();
            cycle.kind = ControlCycle::Kind::Error;
            return cycle;
        }
        cycle.bounded_command = safe;
    }

    cycle.authorized = authority_mode_ == AuthorityMode::ClosedLoop &&
                       (runtime_state_ == RuntimeState::ActiveSwingUp ||
                        runtime_state_ == RuntimeState::ActiveCapture ||
                        runtime_state_ == RuntimeState::ActiveBalance) &&
                       observation.timing == SensorTimingHealth::Healthy &&
                       observation.watchdog == WatchdogHealth::Healthy;
    cycle.authority.authority = cycle.authorized ? ActuationAuthority::ClosedLoop
                                                  : ActuationAuthority::Denied;
    cycle.kind = ControlCycle::Kind::Computed;
    return cycle;
}

void ControlRuntime::configure_command_safety(CommandSafetyLimits limits) {
    if (authority_mode_ != AuthorityMode::ClosedLoop) command_safety_.configure(limits);
}

bool ControlRuntime::request_closed_loop(ControlRegime regime) {
    if (runtime_state_ != RuntimeState::Ready || !command_safety_.configured()) return false;
    closed_loop_requested_ = true;
    requested_regime_ = regime;
    return true;
}

void ControlRuntime::cancel_closed_loop() {
    closed_loop_requested_ = false;
    authority_mode_ = AuthorityMode::Disarmed;
    if (runtime_state_ == RuntimeState::ActiveSwingUp || runtime_state_ == RuntimeState::ActiveCapture ||
        runtime_state_ == RuntimeState::ActiveBalance) runtime_state_ = RuntimeState::Ready;
    command_safety_.reset_history();
}

void ControlRuntime::disable() {
    cancel_closed_loop();
    runtime_state_ = RuntimeState::Disabled;
}

void ControlRuntime::enable_ready() {
    if (runtime_state_ == RuntimeState::Disabled) runtime_state_ = RuntimeState::Ready;
}

Tb6612ElectricalActuation map_tb6612(const BoundedActuatorCommand& command,
                                     bool positive_command_is_positive_drive) {
    Tb6612ElectricalActuation frame{};
    const float value = command.command;
    if (!std::isfinite(value) || value == 0.0f) return frame;
    const bool positive = value > 0.0f;
    frame.mode = (positive == positive_command_is_positive_drive) ? Tb6612BridgeMode::DrivePositive
                                                                  : Tb6612BridgeMode::DriveNegative;
    frame.duty_fraction = std::clamp(std::fabs(value), 0.0f, 1.0f);
    return frame;
}

const char* regime_name(ControlRegime regime) {
    switch (regime) {
        case ControlRegime::SwingUp: return "swing_up";
        case ControlRegime::Capture: return "capture";
        case ControlRegime::Balance: return "balance";
    }
    return "unknown";
}

const char* runtime_state_name(RuntimeState state) {
    switch (state) {
        case RuntimeState::Disabled: return "disabled";
        case RuntimeState::Ready: return "ready";
        case RuntimeState::ActiveSwingUp: return "active_swing_up";
        case RuntimeState::ActiveCapture: return "active_capture";
        case RuntimeState::ActiveBalance: return "active_balance";
        case RuntimeState::Fault: return "fault";
    }
    return "unknown";
}

}  // namespace rip
