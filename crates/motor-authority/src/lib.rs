#![no_std]
#![forbid(unsafe_code)]

use core::marker::PhantomData;

use rip_control_domain::MotorCommand;
use rip_hardware_contract::MotorActuator;

pub struct Disarmed;
pub struct Maintenance;
pub struct Control;
pub struct Fault;

pub struct MotorAuthority<M, S> {
    motor: M,
    last_command: MotorCommand,
    state: PhantomData<S>,
}

impl<M> MotorAuthority<M, Disarmed>
where
    M: MotorActuator,
{
    pub fn new(mut motor: M) -> Self {
        motor.safe_off();
        Self {
            motor,
            last_command: MotorCommand::safe_off(),
            state: PhantomData,
        }
    }

    pub fn acquire_maintenance(self) -> MotorAuthority<M, Maintenance> {
        self.transition()
    }

    pub fn acquire_control(self) -> MotorAuthority<M, Control> {
        self.transition()
    }
}

impl<M> MotorAuthority<M, Maintenance>
where
    M: MotorActuator,
{
    pub fn apply(&mut self, command: MotorCommand) {
        self.motor.apply(command);
        self.last_command = command;
    }

    pub fn release(mut self) -> MotorAuthority<M, Disarmed> {
        self.safe_off();
        self.transition()
    }
}

impl<M> MotorAuthority<M, Control>
where
    M: MotorActuator,
{
    pub fn apply(&mut self, command: MotorCommand) {
        self.motor.apply(command);
        self.last_command = command;
    }

    pub fn release(mut self) -> MotorAuthority<M, Disarmed> {
        self.safe_off();
        self.transition()
    }
}

impl<M> MotorAuthority<M, Fault>
where
    M: MotorActuator,
{
    pub fn clear(mut self) -> MotorAuthority<M, Disarmed> {
        self.safe_off();
        self.transition()
    }
}

impl<M, S> MotorAuthority<M, S>
where
    M: MotorActuator,
{
    pub const fn last_command(&self) -> MotorCommand {
        self.last_command
    }

    pub fn into_fault(mut self) -> MotorAuthority<M, Fault> {
        self.safe_off();
        self.transition()
    }

    fn safe_off(&mut self) {
        self.motor.safe_off();
        self.last_command = MotorCommand::safe_off();
    }

    fn transition<T>(self) -> MotorAuthority<M, T> {
        MotorAuthority {
            motor: self.motor,
            last_command: self.last_command,
            state: PhantomData,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rip_control_domain::{ActuatorMapper, NormalizedEffort};

    struct FakeMotor {
        last: MotorCommand,
        apply_count: u32,
        safe_off_count: u32,
    }

    impl FakeMotor {
        fn new() -> Self {
            Self {
                last: MotorCommand::safe_off(),
                apply_count: 0,
                safe_off_count: 0,
            }
        }
    }

    impl MotorActuator for FakeMotor {
        fn apply(&mut self, command: MotorCommand) {
            self.last = command;
            self.apply_count += 1;
        }

        fn safe_off(&mut self) {
            self.last = MotorCommand::safe_off();
            self.safe_off_count += 1;
        }
    }

    #[test]
    fn maintenance_authority_owns_motor_until_release() {
        let mut motor = FakeMotor::new();
        {
            let authority = MotorAuthority::new(&mut motor);
            let mut authority = authority.acquire_maintenance();
            let command = ActuatorMapper::map(NormalizedEffort::try_new(0.2).unwrap());
            authority.apply(command);
            let authority = authority.release();
            drop(authority);
        }

        assert_eq!(motor.apply_count, 1);
        assert_eq!(motor.safe_off_count, 2);
        assert_eq!(motor.last, MotorCommand::safe_off());
    }

    #[test]
    fn fault_transition_forces_safe_off() {
        let mut motor = FakeMotor::new();
        {
            let authority = MotorAuthority::new(&mut motor);
            let mut authority = authority.acquire_control();
            let command = ActuatorMapper::map(NormalizedEffort::try_new(-0.3).unwrap());
            authority.apply(command);
            let authority = authority.into_fault();
            drop(authority);
        }

        assert_eq!(motor.apply_count, 1);
        assert_eq!(motor.last, MotorCommand::safe_off());
        assert_eq!(motor.safe_off_count, 2);
    }
}
