#![no_std]
#![forbid(unsafe_code)]

use rip_control_domain::MotorCommand;

pub trait PendulumSensor {
    type Error;

    fn angle_rad(&mut self) -> Result<f32, Self::Error>;
}

pub trait ArmEncoder {
    type Error;

    fn angle_rad(&mut self) -> Result<f32, Self::Error>;
}

pub trait BusVoltageSensor {
    type Error;

    fn millivolts(&mut self) -> Result<u32, Self::Error>;
}

pub trait MonotonicClock {
    fn now_micros(&self) -> u32;
}

pub trait MotorActuator {
    fn apply(&mut self, command: MotorCommand);
    fn safe_off(&mut self);
}

impl<T> MotorActuator for &mut T
where
    T: MotorActuator + ?Sized,
{
    fn apply(&mut self, command: MotorCommand) {
        (**self).apply(command);
    }

    fn safe_off(&mut self) {
        (**self).safe_off();
    }
}

pub trait TelemetrySink {
    fn write(&mut self, bytes: &[u8]);
}
