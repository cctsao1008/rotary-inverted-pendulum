#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EffortError {
    NonFinite,
    OutOfRange,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NormalizedEffort(f32);

impl NormalizedEffort {
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorDirection {
    Stopped,
    Left,
    Right,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorCommand {
    direction: MotorDirection,
    duty_percent: f32,
    brake: bool,
}

impl MotorCommand {
    pub const fn safe_off() -> Self {
        Self {
            direction: MotorDirection::Stopped,
            duty_percent: 0.0,
            brake: false,
        }
    }

    pub const fn direction(self) -> MotorDirection {
        self.direction
    }

    pub const fn duty_percent(self) -> f32 {
        self.duty_percent
    }

    pub const fn brake(self) -> bool {
        self.brake
    }
}

pub struct ActuatorMapper;

impl ActuatorMapper {
    pub fn map(effort: NormalizedEffort) -> MotorCommand {
        let value = effort.value();

        if value > 0.0 {
            MotorCommand {
                direction: MotorDirection::Right,
                duty_percent: value * 100.0,
                brake: false,
            }
        } else if value < 0.0 {
            MotorCommand {
                direction: MotorDirection::Left,
                duty_percent: -value * 100.0,
                brake: false,
            }
        } else {
            MotorCommand::safe_off()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalized_effort_rejects_invalid_values() {
        assert_eq!(NormalizedEffort::try_new(1.01), Err(EffortError::OutOfRange));
        assert_eq!(NormalizedEffort::try_new(f32::NAN), Err(EffortError::NonFinite));
    }

    #[test]
    fn mapper_preserves_effort_sign() {
        let command = ActuatorMapper::map(NormalizedEffort::try_new(-0.25).unwrap());
        assert_eq!(command.direction(), MotorDirection::Left);
        assert!((command.duty_percent() - 25.0).abs() < f32::EPSILON);
    }
}
