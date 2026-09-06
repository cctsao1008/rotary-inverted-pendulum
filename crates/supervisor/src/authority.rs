use rip_plant::DriveCommand;

use crate::MotorSink;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuthorityState {
    Disarmed,
    Maintenance,
    Control,
    Fault,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuthorityError {
    Busy,
    Faulted,
    NotMaintenance,
    NotControl,
}

pub struct MotorAuthority<M> {
    motor: M,
    state: AuthorityState,
    last_command: DriveCommand,
}

impl<M> MotorAuthority<M>
where
    M: MotorSink,
{
    pub fn new(mut motor: M) -> Self {
        motor.safe_off();
        Self {
            motor,
            state: AuthorityState::Disarmed,
            last_command: DriveCommand::safe_off(),
        }
    }

    pub const fn state(&self) -> AuthorityState {
        self.state
    }

    pub fn enter_maintenance(&mut self) -> Result<(), AuthorityError> {
        self.enter(AuthorityState::Maintenance)
    }

    pub fn enter_control(&mut self) -> Result<(), AuthorityError> {
        self.enter(AuthorityState::Control)
    }

    pub fn maintenance_access(&mut self) -> Result<MaintenanceAccess<'_, M>, AuthorityError> {
        if self.state != AuthorityState::Maintenance {
            return Err(if self.state == AuthorityState::Fault {
                AuthorityError::Faulted
            } else {
                AuthorityError::NotMaintenance
            });
        }
        Ok(MaintenanceAccess { authority: self })
    }

    pub fn control_access(&mut self) -> Result<ControlAccess<'_, M>, AuthorityError> {
        if self.state != AuthorityState::Control {
            return Err(if self.state == AuthorityState::Fault {
                AuthorityError::Faulted
            } else {
                AuthorityError::NotControl
            });
        }
        Ok(ControlAccess { authority: self })
    }

    pub fn release(&mut self) {
        self.safe_off();
        if self.state != AuthorityState::Fault {
            self.state = AuthorityState::Disarmed;
        }
    }

    pub fn enter_fault(&mut self) {
        self.safe_off();
        self.state = AuthorityState::Fault;
    }

    pub fn clear_fault(&mut self) {
        self.safe_off();
        self.state = AuthorityState::Disarmed;
    }

    fn enter(&mut self, target: AuthorityState) -> Result<(), AuthorityError> {
        match self.state {
            AuthorityState::Disarmed => {
                self.state = target;
                Ok(())
            }
            AuthorityState::Fault => Err(AuthorityError::Faulted),
            _ => Err(AuthorityError::Busy),
        }
    }

    fn apply(&mut self, command: DriveCommand) {
        self.motor.apply(command);
        self.last_command = command;
    }

    fn safe_off(&mut self) {
        self.motor.safe_off();
        self.last_command = DriveCommand::safe_off();
    }
}

pub struct MaintenanceAccess<'a, M>
where
    M: MotorSink,
{
    authority: &'a mut MotorAuthority<M>,
}

impl<M> MaintenanceAccess<'_, M>
where
    M: MotorSink,
{
    pub fn apply(&mut self, command: DriveCommand) {
        self.authority.apply(command);
    }
}

pub struct ControlAccess<'a, M>
where
    M: MotorSink,
{
    authority: &'a mut MotorAuthority<M>,
}

impl<M> ControlAccess<'_, M>
where
    M: MotorSink,
{
    pub fn apply(&mut self, command: DriveCommand) {
        self.authority.apply(command);
    }
}
