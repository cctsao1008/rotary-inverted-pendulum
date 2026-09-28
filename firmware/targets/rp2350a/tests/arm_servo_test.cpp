#include <cassert>
#include <cmath>
#include <cstdint>

#include "rip/arm_servo.hpp"

namespace {

bool near(float actual, float expected, float tolerance) {
    return std::fabs(actual - expected) <= tolerance;
}

void prime_stationary(rip::ArmServo& servo, std::int32_t count = 0) {
    servo.reset(count, 0);
    for (std::uint64_t i = 1; i <= 20; ++i) {
        (void)servo.step(count, i * 1000u);
    }
    assert(servo.state().velocity_valid);
    assert(near(servo.state().velocity_rad_s, 0.0f, 0.01f));
}

void rate_estimator_tracks_quantized_encoder_motion() {
    rip::ArmServo servo({});
    servo.reset(0, 0);
    for (std::uint64_t i = 1; i <= 40; ++i) {
        (void)servo.step(static_cast<std::int32_t>(i), i * 1000u);
    }
    assert(servo.state().velocity_valid);
    const float expected = 2.0f * 3.14159265358979323846f / 1040.0f / 0.001f;
    assert(near(servo.state().velocity_rad_s, expected, 0.15f));
}

void velocity_servo_has_signed_feedforward_and_stiction_authority() {
    rip::ArmServo servo({});
    prime_stationary(servo);

    assert(servo.set_velocity(10.0f, 0.5f));
    const auto positive = servo.step(0, 21000u);
    assert(positive.normalized_command >= 0.18f);
    assert(positive.normalized_command <= 0.5f);

    assert(servo.set_velocity(-10.0f, 0.5f));
    const auto negative = servo.step(0, 22000u);
    assert(negative.normalized_command <= -0.18f);
    assert(negative.normalized_command >= -0.5f);
}

void position_servo_stops_inside_encoder_scale_tolerance() {
    rip::ArmServo servo({});
    prime_stationary(servo);

    assert(servo.set_position(0.20f, 0.35f));
    auto command = servo.step(0, 21000u);
    assert(command.normalized_command >= 0.18f);

    const float radians_per_count = 2.0f * 3.14159265358979323846f / 1040.0f;
    const auto target_count = static_cast<std::int32_t>(std::lround(0.20f / radians_per_count));
    servo.reset(target_count, 30000u);
    for (std::uint64_t i = 31; i <= 50; ++i) {
        (void)servo.step(target_count, i * 1000u);
    }
    assert(servo.set_position(servo.state().position_rad, 0.35f));
    command = servo.step(target_count, 51000u);
    assert(near(command.normalized_command, 0.0f, 1.0e-6f));
}

}  // namespace

int main() {
    rate_estimator_tracks_quantized_encoder_motion();
    velocity_servo_has_signed_feedforward_and_stiction_authority();
    position_servo_stops_inside_encoder_scale_tolerance();
    return 0;
}
