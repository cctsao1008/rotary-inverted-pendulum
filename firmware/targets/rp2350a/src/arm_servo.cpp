#include "rip/arm_servo.hpp"

#include <algorithm>
#include <cmath>

namespace rip {
namespace {

constexpr float kPi = 3.14159265358979323846f;

bool finite_positive(float value) {
    return std::isfinite(value) && value > 0.0f;
}

}  // namespace

ArmServo::ArmServo(ArmServoConfig config) : config_(config) {}

void ArmServo::reset(std::int32_t encoder_count, std::uint64_t timestamp_us) {
    mode_ = ArmServoMode::Off;
    target_ = 0.0f;
    max_abs_command_ = 0.0f;
    velocity_integrator_ = 0.0f;
    state_ = {};
    state_.position_rad = position_from_count(encoder_count);
    rate_history_ = {};
    rate_head_ = 0;
    rate_count_ = 1;
    rate_history_[0] = {encoder_count, timestamp_us};
}

void ArmServo::stop() {
    mode_ = ArmServoMode::Off;
    target_ = 0.0f;
    max_abs_command_ = 0.0f;
    velocity_integrator_ = 0.0f;
}

bool ArmServo::set_velocity(float target_rad_s, float max_abs_command) {
    if (!std::isfinite(target_rad_s) || !finite_positive(max_abs_command) || max_abs_command > 1.0f) {
        return false;
    }
    mode_ = ArmServoMode::Velocity;
    target_ = target_rad_s;
    max_abs_command_ = max_abs_command;
    velocity_integrator_ = 0.0f;
    return true;
}

bool ArmServo::set_position(float target_rad, float max_abs_command) {
    if (!std::isfinite(target_rad) || !finite_positive(max_abs_command) || max_abs_command > 1.0f) {
        return false;
    }
    mode_ = ArmServoMode::Position;
    target_ = target_rad;
    max_abs_command_ = max_abs_command;
    velocity_integrator_ = 0.0f;
    return true;
}

float ArmServo::position_from_count(std::int32_t encoder_count) const {
    if (!finite_positive(config_.encoder_counts_per_rev) || config_.encoder_direction == 0) return 0.0f;
    const float radians_per_count = 2.0f * kPi / config_.encoder_counts_per_rev;
    return static_cast<float>(encoder_count) * radians_per_count *
           static_cast<float>(config_.encoder_direction > 0 ? 1 : -1);
}

void ArmServo::update_rate(std::int32_t encoder_count, std::uint64_t timestamp_us) {
    state_.position_rad = position_from_count(encoder_count);

    const std::size_t next = (rate_head_ + 1u) % kRateHistory;
    rate_head_ = next;
    rate_history_[rate_head_] = {encoder_count, timestamp_us};
    if (rate_count_ < kRateHistory) ++rate_count_;

    const RateSample* oldest = nullptr;
    std::uint64_t best_span = 0;
    for (std::size_t i = 0; i < rate_count_; ++i) {
        const std::size_t index = (rate_head_ + kRateHistory - i) % kRateHistory;
        const RateSample& sample = rate_history_[index];
        if (timestamp_us <= sample.timestamp_us) continue;
        const std::uint64_t span = timestamp_us - sample.timestamp_us;
        if (span >= config_.velocity_window_us) {
            oldest = &sample;
            best_span = span;
            break;
        }
    }

    if (oldest == nullptr || best_span == 0) {
        state_.velocity_valid = false;
        return;
    }

    const std::int32_t delta_count = encoder_count - oldest->count;
    const float delta_position = position_from_count(delta_count);
    const float raw_velocity = delta_position / (static_cast<float>(best_span) * 1.0e-6f);
    if (!std::isfinite(raw_velocity)) {
        state_.velocity_valid = false;
        return;
    }

    const float alpha = std::clamp(config_.velocity_filter_alpha, 0.0f, 1.0f);
    if (!state_.velocity_valid) {
        state_.velocity_rad_s = raw_velocity;
    } else {
        state_.velocity_rad_s += alpha * (raw_velocity - state_.velocity_rad_s);
    }
    state_.velocity_valid = true;
}

float ArmServo::velocity_feedforward(float target_rad_s) const {
    if (std::fabs(target_rad_s) < 1.0e-4f) return 0.0f;

    float command = 0.0f;
    if (target_rad_s > 0.0f && finite_positive(config_.positive_speed_gain_rad_s_per_command)) {
        command = (target_rad_s - config_.positive_speed_intercept_rad_s) /
                  config_.positive_speed_gain_rad_s_per_command;
    } else if (target_rad_s < 0.0f && finite_positive(config_.negative_speed_gain_rad_s_per_command)) {
        command = (target_rad_s - config_.negative_speed_intercept_rad_s) /
                  config_.negative_speed_gain_rad_s_per_command;
    }
    return command;
}

float ArmServo::apply_stiction_floor(float command, float intent) const {
    if (std::fabs(intent) < 1.0e-4f || std::fabs(state_.velocity_rad_s) > config_.stiction_velocity_rad_s) {
        return command;
    }
    if (std::fabs(command) >= config_.stiction_command) return command;
    return std::copysign(config_.stiction_command, intent);
}

ArmServoCommand ArmServo::velocity_step(float target_rad_s) {
    ArmServoCommand out{};
    out.velocity_target_rad_s = target_rad_s;
    if (!state_.velocity_valid) return out;

    if (std::fabs(target_rad_s) < 1.0e-4f &&
        std::fabs(state_.velocity_rad_s) <= config_.settle_velocity_rad_s) {
        velocity_integrator_ = 0.0f;
        return out;
    }

    const float error = target_rad_s - state_.velocity_rad_s;
    const float feedforward = velocity_feedforward(target_rad_s);
    const float proportional = config_.velocity_kp * error;
    float candidate = feedforward + proportional + velocity_integrator_;

    const float unclamped = candidate;
    candidate = std::clamp(candidate, -max_abs_command_, max_abs_command_);
    const bool saturated = candidate != unclamped;

    const bool integration_reduces_saturation =
        !saturated || (unclamped > max_abs_command_ && error < 0.0f) ||
        (unclamped < -max_abs_command_ && error > 0.0f);
    if (integration_reduces_saturation) {
        velocity_integrator_ += config_.velocity_ki * error * 0.001f;
        velocity_integrator_ = std::clamp(velocity_integrator_, -config_.integrator_limit,
                                          config_.integrator_limit);
        candidate = feedforward + proportional + velocity_integrator_;
    }

    candidate = apply_stiction_floor(candidate, target_rad_s);
    const float bounded = std::clamp(candidate, -max_abs_command_, max_abs_command_);
    out.normalized_command = bounded;
    out.saturated = saturated || bounded != candidate;
    return out;
}

ArmServoCommand ArmServo::position_step() {
    ArmServoCommand out{};
    if (!state_.velocity_valid) return out;

    const float error = target_ - state_.position_rad;
    out.position_error_rad = error;
    if (std::fabs(error) <= config_.position_tolerance_rad &&
        std::fabs(state_.velocity_rad_s) <= config_.settle_velocity_rad_s) {
        velocity_integrator_ = 0.0f;
        return out;
    }

    float command = config_.position_kp_command_per_rad * error -
                    config_.position_kd_command_per_rad_s * state_.velocity_rad_s;
    command = apply_stiction_floor(command, error);
    const float bounded = std::clamp(command, -max_abs_command_, max_abs_command_);
    out.normalized_command = bounded;
    out.saturated = bounded != command;
    return out;
}

ArmServoCommand ArmServo::step(std::int32_t encoder_count, std::uint64_t timestamp_us) {
    update_rate(encoder_count, timestamp_us);
    if (mode_ == ArmServoMode::Velocity) return velocity_step(target_);
    if (mode_ == ArmServoMode::Position) return position_step();
    return {};
}

}  // namespace rip
