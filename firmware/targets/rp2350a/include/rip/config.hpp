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
inline constexpr float kEstimatorRateFilterAlpha = 1.0f;
inline constexpr std::uint64_t kSensorExpectedPeriodUs = 1000;
inline constexpr std::uint64_t kSensorLateAfterUs = 5000;
inline constexpr std::uint64_t kSensorTimeoutAfterUs = 20000;
inline constexpr std::uint64_t kControlWatchdogTimeoutUs = 20000;

// Reference-backed live-shadow controller parameters retained from the STM32 target.
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
inline constexpr float kActuatorTorquePerEffectiveCommandNm = 0.05f;
inline constexpr float kActuatorCommandDeadzone = 0.0f;

// HID commissioning authority is bounded independently from closed-loop control.
// A host command must be refreshed before its lease expires or firmware returns
// the bridge to safe-off automatically.
inline constexpr float kMaintenanceMaxAbsCommand = 0.50f;
inline constexpr float kMaintenanceMaxSlewPerSec = 2.0f;
inline constexpr std::uint32_t kMaintenanceDefaultLeaseMs = 250;
inline constexpr std::uint32_t kMaintenanceMaxLeaseMs = 500;

// 100 Hz is intentionally well below the 1 kHz control loop while retaining
// enough temporal resolution for motor/encoder/ADC characterization.
inline constexpr std::uint32_t kTelemetryRateHz = 100;
inline constexpr std::uint32_t kTelemetryPeriodTicks = kControlTickHz / kTelemetryRateHz;

}  // namespace rip::config
