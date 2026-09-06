#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EffortError {
    NonFinite,
    OutOfRange,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlEffort(f32);

impl ControlEffort {
    pub const fn zero() -> Self {
        Self(0.0)
    }

    pub fn try_new(value: f32) -> Result<Self, EffortError> {
        if !value.is_finite() {
            return Err(EffortError::NonFinite);
        }
        if !(-1.0..=1.0).contains(&value) {
            return Err(EffortError::OutOfRange);
        }
        Ok(Self(value))
    }

    pub const fn value(self) -> f32 {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn effort_is_bounded() {
        assert_eq!(ControlEffort::try_new(1.1), Err(EffortError::OutOfRange));
        assert_eq!(ControlEffort::try_new(f32::NAN), Err(EffortError::NonFinite));
    }
}
