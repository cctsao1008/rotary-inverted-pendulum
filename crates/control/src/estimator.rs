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
    theta_dot_rad_s: f32,
    phi_dot_rad_s: f32,
}

impl BasicEstimator {
    pub const fn new() -> Self {
        Self {
            previous: None,
            theta_dot_rad_s: 0.0,
            phi_dot_rad_s: 0.0,
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
        let theta_dot_raw = shortest_circular_delta(input.theta_rad - previous.theta_rad) / dt_s;
        let phi_dot_raw = (input.phi_rad - previous.phi_rad) / dt_s;
        if !theta_dot_raw.is_finite() || !phi_dot_raw.is_finite() {
            return Err(EstimatorError::NonFiniteSample);
        }

        let alpha = config.rate_filter_alpha;
        self.theta_dot_rad_s = alpha * theta_dot_raw + (1.0 - alpha) * self.theta_dot_rad_s;
        self.phi_dot_rad_s = alpha * phi_dot_raw + (1.0 - alpha) * self.phi_dot_rad_s;
        self.previous = Some(input);

        Ok(Estimate::Ready(ControlState::new(
            input.theta_rad,
            self.theta_dot_rad_s,
            input.phi_rad,
            self.phi_dot_rad_s,
            input.timestamp_us,
        )))
    }

    fn prime(&mut self, input: EstimatorInput) {
        self.previous = Some(input);
        self.theta_dot_rad_s = 0.0;
        self.phi_dot_rad_s = 0.0;
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
    fn first_sample_primes_history() {
        let mut estimator = BasicEstimator::new();
        assert_eq!(
            estimator
                .step(&config(), EstimatorInput::new(0.1, 0.2, 1_000))
                .unwrap(),
            Estimate::Primed
        );
    }

    #[test]
    fn theta_rate_uses_shortest_circular_delta() {
        let mut estimator = BasicEstimator::new();
        estimator
            .step(&config(), EstimatorInput::new(3.13, 0.0, 1_000))
            .unwrap();
        match estimator
            .step(&config(), EstimatorInput::new(-3.13, 0.0, 11_000))
            .unwrap()
        {
            Estimate::Ready(state) => assert!(state.theta_dot_rad_s.abs() < 10.0),
            Estimate::Primed => panic!("second sample must produce an estimate"),
        }
    }
}
