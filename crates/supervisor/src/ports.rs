use rip_plant::DriveCommand;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Observation {
    pub theta_rad: f32,
    pub phi_rad: f32,
    pub timestamp_us: u32,
    pub sample_age_us: u32,
    pub valid: bool,
}

pub trait SensorSource {
    type Error;

    fn observe(&mut self) -> Result<Observation, Self::Error>;
}

pub trait MotorSink {
    fn apply(&mut self, command: DriveCommand);
    fn safe_off(&mut self);
}

impl<T> MotorSink for &mut T
where
    T: MotorSink + ?Sized,
{
    fn apply(&mut self, command: DriveCommand) {
        (**self).apply(command);
    }

    fn safe_off(&mut self) {
        (**self).safe_off();
    }
}

pub trait TelemetrySink {
    fn write(&mut self, bytes: &[u8]);
}
