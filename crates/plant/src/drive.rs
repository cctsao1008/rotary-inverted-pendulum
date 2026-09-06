use rip_control::ControlEffort;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveDirection {
    Stopped,
    Left,
    Right,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DriveCommand {
    pub direction: DriveDirection,
    pub duty_percent: f32,
    pub brake: bool,
}

impl DriveCommand {
    pub const fn safe_off() -> Self {
        Self {
            direction: DriveDirection::Stopped,
            duty_percent: 0.0,
            brake: false,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveMapError {
    InvalidDutyLimit,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DriveMap {
    max_duty_percent: f32,
    positive_is_right: bool,
}

impl DriveMap {
    pub fn new(max_duty_percent: f32, positive_is_right: bool) -> Result<Self, DriveMapError> {
        if !max_duty_percent.is_finite() || !(0.0 < max_duty_percent && max_duty_percent <= 100.0) {
            return Err(DriveMapError::InvalidDutyLimit);
        }
        Ok(Self {
            max_duty_percent,
            positive_is_right,
        })
    }

    pub fn map(&self, effort: ControlEffort) -> DriveCommand {
        let value = effort.value();
        if value == 0.0 {
            return DriveCommand::safe_off();
        }

        let positive = value > 0.0;
        let direction = match (positive, self.positive_is_right) {
            (true, true) | (false, false) => DriveDirection::Right,
            _ => DriveDirection::Left,
        };

        DriveCommand {
            direction,
            duty_percent: value.abs() * self.max_duty_percent,
            brake: false,
        }
    }
}
