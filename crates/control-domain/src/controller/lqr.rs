use crate::{ControlState, NormalizedEffort};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LqrConfigError {
    NonFiniteGain,
    InvalidOutputLimit,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LqrError {
    InvalidState,
    Numeric,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LqrController {
    gains: [f32; 4],
    max_abs_effort: f32,
}

impl LqrController {
    pub fn new(gains: [f32; 4], max_abs_effort: f32) -> Result<Self, LqrConfigError> {
        if gains.iter().any(|gain| !gain.is_finite()) {
            return Err(LqrConfigError::NonFiniteGain);
        }

        if !max_abs_effort.is_finite() || !(0.0 < max_abs_effort && max_abs_effort <= 1.0) {
            return Err(LqrConfigError::InvalidOutputLimit);
        }

        Ok(Self {
            gains,
            max_abs_effort,
        })
    }

    pub fn compute(&self, state: &ControlState) -> Result<NormalizedEffort, LqrError> {
        if !state.is_finite() {
            return Err(LqrError::InvalidState);
        }

        let state_vector = state.as_vector();
        let mut feedback = 0.0_f32;

        for (gain, value) in self.gains.iter().zip(state_vector.iter()) {
            feedback += gain * value;
        }

        if !feedback.is_finite() {
            return Err(LqrError::Numeric);
        }

        let raw = -feedback;
        let command = if raw > self.max_abs_effort {
            self.max_abs_effort
        } else if raw < -self.max_abs_effort {
            -self.max_abs_effort
        } else {
            raw
        };

        NormalizedEffort::try_new(command).map_err(|_| LqrError::Numeric)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lqr_uses_established_state_order_and_saturates() {
        let controller = LqrController::new([2.0, 3.0, 5.0, 7.0], 0.5).unwrap();
        let state = ControlState::new(1.0, 2.0, 3.0, 4.0, 10);

        let output = controller.compute(&state).unwrap();
        assert!((output.value() + 0.5).abs() < f32::EPSILON);
    }
}
