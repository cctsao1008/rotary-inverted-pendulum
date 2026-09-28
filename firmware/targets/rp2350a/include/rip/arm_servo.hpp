#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace rip {

enum class ArmServoMode : std::uint8_t { Off = 0, Velocity = 1, Position = 2 };

struct ArmServoState {
    float position_rad{0.0f};
    float velocity_rad_s{0.0f};
    bool velocity_valid{false};
};

struct ArmServoCommand {
    float normalized_command{0.0f};
    float velocity_target_rad_s{0.0f};
    float position_error_rad{0.0f};
    bool saturated{false};
};

struct ArmServoConfig {
    float encoder_counts_per_rev{1040.0f};
    int encoder_direction{1};
    std::uint32_t velocity_window_us{10000};
    float velocity_filter_alpha{0.25f};

    // 2026-09-26 unloaded running-region fits from the hardware specimen.
    float positive_speed_gain_rad_s_per_command{56.19f};
    float positive_speed_intercept_rad_s{-3.68f};
    float negative_speed_gain_rad_s_per_command{54.10f};
    float negative_speed_intercept_rad_s{4.03f};

    // Inner velocity loop. The feedforward owns most of the steady command;
    // PI closes specimen/load/supply error and supplies start authority.
    float velocity_kp{0.018f};
    float velocity_ki{0.67f};
    float integrator_limit{0.35f};

    // Position mode is deliberately cascaded through the same velocity loop
    // instead of maintaining a second direct-PWM tuning path.
    float position_kp_rad_s_per_rad{10.0f};
    float position_max_velocity_rad_s{8.0f};
    float position_tolerance_rad{0.01f};
    float settle_velocity_rad_s{0.8f};

    // Static start was history/rotor-position dependent. This is only a
    // near-zero-speed minimum authority; the PI loop may request more.
    float stiction_command{0.18f};
    float stiction_velocity_rad_s{1.0f};
};

class ArmServo {
  public:
    explicit ArmServo(ArmServoConfig config);

    void reset(std::int32_t encoder_count, std::uint64_t timestamp_us);
    void stop();
    bool set_velocity(float target_rad_s, float max_abs_command);
    bool set_position(float target_rad, float max_abs_command);

    ArmServoCommand step(std::int32_t encoder_count, std::uint64_t timestamp_us);

    ArmServoMode mode() const { return mode_; }
    bool active() const { return mode_ != ArmServoMode::Off; }
    float target() const { return target_; }
    float max_abs_command() const { return max_abs_command_; }
    const ArmServoState& state() const { return state_; }

  private:
    struct RateSample {
        std::int32_t count{0};
        std::uint64_t timestamp_us{0};
    };

    static constexpr std::size_t kRateHistory = 16;

    float position_from_count(std::int32_t encoder_count) const;
    float velocity_feedforward(float target_rad_s) const;
    void update_rate(std::int32_t encoder_count, std::uint64_t timestamp_us);
    float apply_stiction_floor(float command, float intent) const;
    ArmServoCommand velocity_step(float target_rad_s);
    ArmServoCommand position_step();

    ArmServoConfig config_{};
    ArmServoMode mode_{ArmServoMode::Off};
    ArmServoState state_{};
    float target_{0.0f};
    float max_abs_command_{0.0f};
    float velocity_integrator_{0.0f};

    std::array<RateSample, kRateHistory> rate_history_{};
    std::size_t rate_head_{0};
    std::size_t rate_count_{0};
};

}  // namespace rip
