#pragma once

#include <cstdint>

namespace rip::config {

inline constexpr std::uint32_t kControlTickHz = 1000;
inline constexpr std::uint64_t kControlPeriodUs = 1000;
inline constexpr std::uint32_t kMotorPwmHz = 20000;
inline constexpr std::uint32_t kHardwareWatchdogMs = 100;

inline constexpr std::uint16_t kPendulumUprightAdc = 2928;
inline constexpr float kPi = 3.14159265358979323846f;
inline constexpr float kPendulumRadiansPerCount = 2.0f * kPi / 4096.0f;
inline constexpr std::int8_t kPendulumDirection = 1;
inline constexpr float kArmEncoderCountsPerRevolution = 1040.0f;
inline constexpr std::int8_t kArmEncoderDirection = 1;

inline constexpr std::uint64_t kEstimatorMaxGapUs = 20000;
// At 1 kHz, a one-sample derivative quantizes one arm-encoder count to about
// 6.04 rad/s and one pendulum ADC count to about 1.53 rad/s. A 0.10 IIR
// coefficient gives the dirty-derivative estimator a roughly 9.5 ms time
// constant (~16.8 Hz pole) while materially suppressing those impulses.
inline constexpr float kEstimatorRateFilterAlpha = 0.10f;
inline constexpr std::uint64_t kSensorExpectedPeriodUs = 1000;
inline constexpr std::uint64_t kSensorLateAfterUs = 5000;
inline constexpr std::uint64_t kSensorTimeoutAfterUs = 20000;
inline constexpr std::uint64_t kControlWatchdogTimeoutUs = 20000;

inline constexpr float kPendulumMassKg = 0.04f;
inline constexpr float kPendulumComLengthM = 0.129f;
inline constexpr float kPendulumInertiaKgM2 = 0.0001f;
inline constexpr float kGravityMps2 = 9.81f;
inline constexpr float kTargetEnergyJ = 0.025f;
inline constexpr float kEnergyTorqueGain = 0.175f;
inline constexpr float kMaxAbsTorqueNm = 0.05f;
inline constexpr float kSwingKickTorqueNm = 0.01f;
inline constexpr float kSwingKickBelowRateRadS = 0.05f;

inline constexpr float kCaptureEnterAngleRad = 20.0f * kPi / 180.0f;
inline constexpr float kCaptureEnterRateRadS = 3.0f;
inline constexpr float kBalanceEnterAngleRad = 8.0f * kPi / 180.0f;
inline constexpr float kBalanceEnterRateRadS = 1.0f;
inline constexpr float kBalanceExitAngleRad = 12.0f * kPi / 180.0f;
inline constexpr float kBalanceExitRateRadS = 2.0f;
inline constexpr float kCaptureExitAngleRad = 30.0f * kPi / 180.0f;
inline constexpr float kCaptureExitRateRadS = 4.0f;
inline constexpr std::uint16_t kCaptureSettleCycles = 20;

inline constexpr float kLqrGains[4] = {-0.18355f, -0.01585f, -0.01120f, -0.00745f};
// Torque span remains the pre-commissioning baseline; no torque/current
// measurement has yet established a replacement value.
inline constexpr float kActuatorTorquePerEffectiveCommandNm = 0.05f;
// 2026-09-26 unloaded motor characterization gave running-region command
// intercepts of about +0.065 and -0.074. Use the symmetric midpoint as the
// kinetic-friction inverse-map deadzone rather than the previous zero value.
inline constexpr float kActuatorCommandDeadzone = 0.07f;
// Starting from rest is strongly hysteretic and position-dependent. The most
// conservative observed positive breakaway was +0.23, so automatic control
// must not claim sub-breakaway authority while the arm is effectively stopped.
// This is a fail-closed floor, not a claim that 0.23 is a universal plant
// constant; mechanism-level commissioning can refine it later.
inline constexpr float kActuatorStaticStartCommand = 0.23f;
inline constexpr float kActuatorMovingRateThresholdRadS = 0.50f;

inline constexpr float kCommissioningMaxAbsCommand = 1.0f;
inline constexpr std::uint32_t kCommissioningDefaultLeaseMs = 250;
inline constexpr std::uint32_t kCommissioningMaxLeaseMs = 1000;

inline constexpr std::uint32_t kTelemetryRateHz = 100;
inline constexpr std::uint32_t kTelemetryPeriodTicks = kControlTickHz / kTelemetryRateHz;

}  // namespace rip::config
