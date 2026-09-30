#pragma once

#include <cstdint>

namespace rip {

enum class StateValidity : std::uint8_t { Invalid, Valid };
enum class ControlRegime : std::uint8_t { SwingUp, Capture, Balance };
enum class SensorTimingHealth : std::uint8_t { Startup, Healthy, Late, Timeout };
enum class WatchdogHealth : std::uint8_t { Disarmed, Healthy, Expired };
enum class RuntimeState : std::uint8_t { Disabled, Ready, ActiveSwingUp, ActiveCapture, ActiveBalance, Fault };
enum class AuthorityMode : std::uint8_t { Disarmed, ClosedLoop, Maintenance, Fault };
enum class ActuationAuthority : std::uint8_t { Denied, ClosedLoop };
enum class Tb6612BridgeMode : std::uint8_t { Coast, DrivePositive, DriveNegative, Brake };

struct TimestampUs {
    std::uint64_t value{0};
};

struct EstimatedState {
    TimestampUs timestamp{};
    float theta{0.0f};
    float theta_dot{0.0f};
    float phi{0.0f};
    float phi_dot{0.0f};
    StateValidity validity{StateValidity::Invalid};

    bool finite() const;
};

struct GeneralizedDemand {
    float arm_torque_nm{0.0f};
};

struct BoundedActuatorCommand {
    float command{0.0f};
    bool saturated{false};
    float predicted_arm_torque_nm{0.0f};
};

enum MeasurementQuality : std::uint8_t {
    MeasurementNone = 0,
    MeasurementAvailable = 1u << 0,
    MeasurementIoOk = 1u << 1,
    MeasurementTimingValid = 1u << 2,
    MeasurementStale = 1u << 3,
};

struct RawPendulumObservation {
    TimestampUs captured_at{};
    std::uint16_t adc_raw{0};
    std::uint8_t quality{MeasurementNone};
};

struct RawArmEncoderObservation {
    TimestampUs captured_at{};
    std::int32_t accumulated_count{0};
    std::uint8_t quality{MeasurementNone};
};

struct RawObservation {
    std::uint32_t sample_index{0};
    RawPendulumObservation pendulum{};
    RawArmEncoderObservation arm_encoder{};
};

struct EstimatorMeasurement {
    float theta{0.0f};
    float phi{0.0f};
    TimestampUs captured_at{};
};

class MeasurementAdapter {
  public:
    bool convert(const RawObservation& raw, EstimatorMeasurement& out) const;
};

struct EstimatorConfig {
    std::uint64_t max_gap_us{0};
    float rate_filter_alpha{1.0f};
};

class BasicEstimator {
  public:
    enum class Result : std::uint8_t { Primed, Ready, Error };
    Result step(const EstimatorConfig& config, const EstimatorMeasurement& measurement, EstimatedState& out);
    void reset();

  private:
    bool primed_{false};
    EstimatorMeasurement previous_{};
    float filtered_theta_dot_{0.0f};
    float filtered_phi_dot_{0.0f};
};

struct EnergySwingUpConfig {
    float pendulum_mass_kg;
    float pendulum_com_length_m;
    float pendulum_inertia_kg_m2;
    float gravity_m_s2;
    float target_energy_j;
    float energy_gain;
    float max_abs_torque_nm;
    float kick_torque_nm;
    float kick_below_rate_rad_s;
};

struct CapturePolicyConfig {
    float capture_enter_angle_rad;
    float capture_enter_rate_rad_s;
    float balance_enter_angle_rad;
    float balance_enter_rate_rad_s;
    float balance_exit_angle_rad;
    float balance_exit_rate_rad_s;
    float capture_exit_angle_rad;
    float capture_exit_rate_rad_s;
    std::uint16_t settle_cycles;
};

class CapturePolicy {
  public:
    explicit CapturePolicy(CapturePolicyConfig config) : config_(config) {}
    ControlRegime update(const EstimatedState& state);
    ControlRegime regime() const { return regime_; }
    void reset();

  private:
    CapturePolicyConfig config_;
    ControlRegime regime_{ControlRegime::SwingUp};
    std::uint16_t settled_cycles_{0};
};

class HybridController {
  public:
    HybridController(EnergySwingUpConfig swing, CapturePolicyConfig capture);
    bool compute(const EstimatedState& state, GeneralizedDemand& out);
    ControlRegime regime() const { return capture_.regime(); }
    void reset();

  private:
    float lqr_full_state(const EstimatedState& state) const;
    float lqr_capture_projection(const EstimatedState& state) const;
    bool swing_up(const EstimatedState& state, GeneralizedDemand& out) const;

    EnergySwingUpConfig swing_;
    CapturePolicy capture_;
};

class ArmActuatorModel {
  public:
    ArmActuatorModel(float torque_per_effective_command_nm, float command_deadzone);
    bool command_for_demand(GeneralizedDemand demand, BoundedActuatorCommand& out) const;
    float predicted_torque(float normalized_command) const;

  private:
    float torque_span_nm_;
    float deadzone_;
};

struct CommandSafetyLimits {
    float max_abs_command{1.0f};
    float max_slew_per_s{1.0f};
};

class CommandSafetyGate {
  public:
    void configure(CommandSafetyLimits limits);
    void reset_history();
    bool configured() const { return configured_; }
    bool constrain(BoundedActuatorCommand requested, TimestampUs timestamp,
                   const ArmActuatorModel& actuator, BoundedActuatorCommand& out);

  private:
    bool configured_{false};
    CommandSafetyLimits limits_{};
    float last_command_{0.0f};
    bool has_timestamp_{false};
    TimestampUs last_timestamp_{};
};

class SensorTimingMonitor {
  public:
    SensorTimingMonitor(std::uint64_t expected_period_us, std::uint64_t late_after_us,
                        std::uint64_t timeout_after_us, std::uint64_t started_at_us);
    SensorTimingHealth on_event(std::uint64_t event_at_us);
    SensorTimingHealth poll(std::uint64_t now_us);
    SensorTimingHealth health() const { return health_; }

  private:
    SensorTimingHealth classify(std::uint64_t elapsed_us) const;
    std::uint64_t expected_period_us_;
    std::uint64_t late_after_us_;
    std::uint64_t timeout_after_us_;
    std::uint64_t started_at_us_;
    std::uint64_t last_event_at_us_{0};
    bool has_event_{false};
    bool cadence_verified_{false};
    SensorTimingHealth health_{SensorTimingHealth::Startup};
};

class ControlWatchdog {
  public:
    explicit ControlWatchdog(std::uint64_t timeout_us) : timeout_us_(timeout_us) {}
    void kick(std::uint64_t now_us);
    void disarm();
    WatchdogHealth health(std::uint64_t now_us) const;

  private:
    std::uint64_t timeout_us_;
    std::uint64_t last_kick_us_{0};
    bool armed_{false};
};

struct RuntimeQualification {
    bool allowed{false};
    std::uint16_t reasons{0};
};

struct RuntimeObservation {
    EstimatorMeasurement measurement{};
    bool sensor_valid{false};
    std::uint64_t sample_age_us{0};
    SensorTimingHealth timing{SensorTimingHealth::Startup};
    WatchdogHealth watchdog{WatchdogHealth::Disarmed};
};

struct AuthorityDecision {
    ActuationAuthority authority{ActuationAuthority::Denied};
    std::uint16_t reasons{0};
};

struct ControlCycle {
    enum class Kind : std::uint8_t { Primed, Rejected, Computed, Error } kind{Kind::Error};
    RuntimeQualification qualification{};
    EstimatedState state{};
    GeneralizedDemand demand{};
    BoundedActuatorCommand bounded_command{};
    AuthorityDecision authority{};
    bool authorized{false};
};

class ControlRuntime {
  public:
    ControlRuntime();
    ControlCycle step(const RuntimeObservation& observation);

    RuntimeState runtime_state() const { return runtime_state_; }
    AuthorityMode authority_mode() const { return authority_mode_; }
    ControlRegime regime() const { return controller_.regime(); }
    void configure_command_safety(CommandSafetyLimits limits);
    bool request_closed_loop(ControlRegime regime);
    void cancel_closed_loop();
    void disable();
    void enable_ready();

  private:
    RuntimeQualification qualify(const RuntimeObservation& observation, const EstimatedState& state) const;
    void update_authority(const RuntimeObservation& observation, const EstimatedState& state,
                          const RuntimeQualification& qualification);
    RuntimeState active_state_for(ControlRegime regime) const;

    BasicEstimator estimator_{};
    EstimatorConfig estimator_config_{};
    HybridController controller_;
    ArmActuatorModel actuator_;
    CommandSafetyGate command_safety_{};
    RuntimeState runtime_state_{RuntimeState::Ready};
    AuthorityMode authority_mode_{AuthorityMode::Disarmed};
    bool closed_loop_requested_{false};
    ControlRegime requested_regime_{ControlRegime::SwingUp};
};

struct Tb6612ElectricalActuation {
    Tb6612BridgeMode mode{Tb6612BridgeMode::Coast};
    float duty_fraction{0.0f};
};

Tb6612ElectricalActuation map_tb6612(const BoundedActuatorCommand& command,
                                     bool positive_command_is_positive_drive = true);

struct RuntimeSnapshot {
    std::uint64_t timestamp_us{0};
    std::uint32_t sample_index{0};
    std::uint16_t pendulum_adc_raw{0};
    std::int32_t arm_encoder_count{0};
    EstimatedState state{};
    ControlRegime regime{ControlRegime::SwingUp};
    RuntimeState runtime_state{RuntimeState::Ready};
    AuthorityMode authority_mode{AuthorityMode::Disarmed};
    GeneralizedDemand demand{};
    BoundedActuatorCommand command{};
    std::uint32_t missed_opportunities{0};
    std::uint32_t deadline_overruns{0};
    std::uint32_t execution_time_us{0};
};

const char* regime_name(ControlRegime regime);
const char* runtime_state_name(RuntimeState state);

}  // namespace rip
