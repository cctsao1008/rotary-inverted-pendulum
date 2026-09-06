use core::f32::consts::PI;

use crate::{ControlState, EstimatorInput};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EstimatorConfig {
    pub max_gap_us: u32,
    pub rate_filter_alpha: f32,
}

impl EstimatorConfig {
    pub fn is_valid(&self) -> bool {
        self.max_gap_us > 0
            && self.rate_filter_alpha.is_finite()
            && (0.0..=1.0).contains(&self.rate_filter_alpha)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EstimatorError {
    InvalidConfig,
    NonFiniteSample,
    InvalidTimestamp,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Estimate {
    Primed,
    Ready(ControlState),
}

#[derive(Debug, Clone, Copy)]
pub struct BasicEstimator {
    previous: Option<EstimatorInput>,
    pendulum_rate_rad_s: f32,
    arm_rate_rad_s: f32,
}

impl BasicEstimator {
    pub const fn new() -> Self {
        Self {
            previous: None,
            pendulum_rate_rad_s: 0.0,
            arm_rate_rad_s: 0.0,
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    pub fn step(
        &mut self,
        config: &EstimatorConfig,
        input: EstimatorInput,
    ) -> Result<Estimate, EstimatorError> {
        if !config.is_valid() {
            return Err(EstimatorError::InvalidConfig);
        }

        if !input.is_finite() {
            return Err(EstimatorError::NonFiniteSample);
        }

        let previous = match self.previous {
            Some(previous) => previous,
            None => {
                self.prime(input);
                return Ok(Estimate::Primed);
            }
        };

        let elapsed_us = input.timestamp_us.wrapping_sub(previous.timestamp_us);

        if elapsed_us == 0 || elapsed_us > i32::MAX as u32 {
            return Err(EstimatorError::InvalidTimestamp);
        }

        if elapsed_us > config.max_gap_us {
            self.prime(input);
            return Ok(Estimate::Primed);
        }

        let dt_s = elapsed_us as f32 * 1.0e-6;
        let pendulum_delta = shortest_circular_delta(
            input.pendulum_angle_rad - previous.pendulum_angle_rad,
        );
        let pendulum_rate_raw = pendulum_delta / dt_s;
        let arm_rate_raw = (input.arm_angle_rad - previous.arm_angle_rad) / dt_s;

        if !pendulum_rate_raw.is_finite() || !arm_rate_raw.is_finite() {
            return Err(EstimatorError::NonFiniteSample);
        }

        let alpha = config.rate_filter_alpha;
        self.pendulum_rate_rad_s =
            alpha * pendulum_rate_raw + (1.0 - alpha) * self.pendulum_rate_rad_s;
        self.arm_rate_rad_s = alpha * arm_rate_raw + (1.0 - alpha) * self.arm_rate_rad_s;
        self.previous = Some(input);

        Ok(Estimate::Ready(ControlState::new(
            input.pendulum_angle_rad,
            self.pendulum_rate_rad_s,
            input.arm_angle_rad,
            self.arm_rate_rad_s,
            input.timestamp_us,
        )))
    }

    fn prime(&mut self, input: EstimatorInput) {
        self.previous = Some(input);
        self.pendulum_rate_rad_s = 0.0;
        self.arm_rate_rad_s = 0.0;
    }
}

impl Default for BasicEstimator {
    fn default() -> Self {
        Self::new()
    }
}

fn shortest_circular_delta(mut delta: f32) -> f32 {
    let tau = 2.0 * PI;

    if delta > PI {
        delta -= tau;
    } else if delta < -PI {
        delta += tau;
    }

    delta
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> EstimatorConfig {
        EstimatorConfig {
            max_gap_us: 20_000,
            rate_filter_alpha: 1.0,
        }
    }

    #[test]
    fn first_sample_only_primes_history() {
        let mut estimator = BasicEstimator::new();
        let result = estimator
            .step(&config(), EstimatorInput::new(0.1, 0.2, 1_000))
            .unwrap();
        assert_eq!(result, Estimate::Primed);
    }

    #[test]
    fn pendulum_rate_uses_shortest_circular_delta() {
        let mut estimator = BasicEstimator::new();
        estimator
            .step(&config(), EstimatorInput::new(3.13, 0.0, 1_000))
            .unwrap();

        let estimate = estimator
            .step(&config(), EstimatorInput::new(-3.13, 0.0, 11_000))
            .unwrap();

        match estimate {
            Estimate::Ready(state) => assert!(state.pendulum_rate_rad_s.abs() < 10.0),
            Estimate::Primed => panic!("second valid sample must produce an estimate"),
        }
    }

    #[test]
    fn long_gap_reseeds_estimator() {
        let mut estimator = BasicEstimator::new();
        estimator
            .step(&config(), EstimatorInput::new(0.0, 0.0, 1_000))
            .unwrap();

        let result = estimator
            .step(&config(), EstimatorInput::new(0.1, 0.2, 50_000))
            .unwrap();
        assert_eq!(result, Estimate::Primed);
    }

    #[test]
    fn backward_timestamp_is_rejected() {
        let mut estimator = BasicEstimator::new();
        estimator
            .step(&config(), EstimatorInput::new(0.0, 0.0, 10_000))
            .unwrap();

        let result = estimator.step(&config(), EstimatorInput::new(0.1, 0.2, 9_000));
        assert_eq!(result, Err(EstimatorError::InvalidTimestamp));
    }
}
