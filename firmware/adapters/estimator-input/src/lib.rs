#![no_std]
#![forbid(unsafe_code)]

use rip_measurement_model::{EncoderScale, PendulumCalibration};
use rip_plant_observation::{MeasurementQuality, RawObservation};
use rip_state_estimator::EstimatorMeasurement;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdapterError {
    PendulumEvidenceUnavailable,
    ArmEncoderEvidenceUnavailable,
}

/// Converts one wrapping 16-bit hardware encoder counter into the Plant-owned
/// accumulated-count observation semantic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EncoderCounterAccumulator {
    previous_counter: u16,
    accumulated_count: i32,
}

impl EncoderCounterAccumulator {
    pub const fn new(initial_counter: u16) -> Self {
        Self {
            previous_counter: initial_counter,
            accumulated_count: 0,
        }
    }

    pub fn update(&mut self, counter: u16) -> i32 {
        let delta = counter.wrapping_sub(self.previous_counter) as i16 as i32;
        self.previous_counter = counter;
        self.accumulated_count = self.accumulated_count.saturating_add(delta);
        self.accumulated_count
    }

    pub const fn accumulated_count(self) -> i32 {
        self.accumulated_count
    }
}

/// Firmware semantic adapter from Plant-owned raw evidence to the
/// Supervisor-owned estimator input representation.
#[derive(Debug, Clone, Copy)]
pub struct EstimatorInputAdapter {
    pendulum: PendulumCalibration,
    arm_encoder: EncoderScale,
}

impl EstimatorInputAdapter {
    pub const fn new(pendulum: PendulumCalibration, arm_encoder: EncoderScale) -> Self {
        Self {
            pendulum,
            arm_encoder,
        }
    }

    pub fn measurement(
        &self,
        observation: RawObservation,
    ) -> Result<EstimatorMeasurement, AdapterError> {
        if !usable(observation.pendulum.quality) {
            return Err(AdapterError::PendulumEvidenceUnavailable);
        }
        if !usable(observation.arm_encoder.quality) {
            return Err(AdapterError::ArmEncoderEvidenceUnavailable);
        }

        let captured_at = core::cmp::max(
            observation.pendulum.captured_at,
            observation.arm_encoder.captured_at,
        );

        Ok(EstimatorMeasurement {
            theta: self.pendulum.angle(observation.pendulum.adc_raw),
            phi: self
                .arm_encoder
                .angle(observation.arm_encoder.accumulated_count),
            captured_at,
        })
    }
}

fn usable(quality: MeasurementQuality) -> bool {
    quality.contains(MeasurementQuality::AVAILABLE)
        && quality.contains(MeasurementQuality::IO_OK)
        && quality.contains(MeasurementQuality::TIMING_VALID)
        && !quality.contains(MeasurementQuality::STALE)
}

#[cfg(test)]
mod tests {
    use super::*;
    use rip_plant_observation::{RawArmEncoderObservation, RawPendulumObservation};
    use rip_robot_domain::TimestampUs;

    fn good_quality() -> MeasurementQuality {
        MeasurementQuality::AVAILABLE | MeasurementQuality::IO_OK | MeasurementQuality::TIMING_VALID
    }

    #[test]
    fn encoder_accumulator_handles_forward_counter_wrap() {
        let mut accumulator = EncoderCounterAccumulator::new(65_530);
        assert_eq!(accumulator.update(4), 10);
    }

    #[test]
    fn encoder_accumulator_handles_reverse_counter_wrap() {
        let mut accumulator = EncoderCounterAccumulator::new(4);
        assert_eq!(accumulator.update(65_530), -10);
    }

    #[test]
    fn raw_observation_promotes_to_estimator_measurement() {
        let pendulum = PendulumCalibration::new(2_000, 0.01, 1).unwrap();
        let encoder = EncoderScale::new(1_000.0, 1).unwrap();
        let adapter = EstimatorInputAdapter::new(pendulum, encoder);
        let observation = RawObservation {
            sample_index: 7,
            pendulum: RawPendulumObservation {
                captured_at: TimestampUs(100),
                adc_raw: 2_010,
                quality: good_quality(),
            },
            arm_encoder: RawArmEncoderObservation {
                captured_at: TimestampUs(101),
                accumulated_count: 250,
                quality: good_quality(),
            },
        };

        let measurement = adapter.measurement(observation).unwrap();
        assert!((measurement.theta.0 - 0.1).abs() < 1.0e-6);
        assert!((measurement.phi.0 - core::f32::consts::FRAC_PI_2).abs() < 1.0e-6);
        assert_eq!(measurement.captured_at, TimestampUs(101));
    }

    #[test]
    fn stale_sensor_evidence_is_not_promoted() {
        let pendulum = PendulumCalibration::new(2_000, 0.01, 1).unwrap();
        let encoder = EncoderScale::new(1_000.0, 1).unwrap();
        let adapter = EstimatorInputAdapter::new(pendulum, encoder);
        let observation = RawObservation {
            sample_index: 0,
            pendulum: RawPendulumObservation {
                captured_at: TimestampUs(100),
                adc_raw: 2_000,
                quality: good_quality() | MeasurementQuality::STALE,
            },
            arm_encoder: RawArmEncoderObservation {
                captured_at: TimestampUs(100),
                accumulated_count: 0,
                quality: good_quality(),
            },
        };

        assert_eq!(
            adapter.measurement(observation),
            Err(AdapterError::PendulumEvidenceUnavailable)
        );
    }
}
