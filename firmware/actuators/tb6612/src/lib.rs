#![no_std]
#![forbid(unsafe_code)]

use rip_actuator_model::BoundedActuatorCommand;
use rip_runtime_state::{AuthorizedActuation, MaintenanceActuation};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tb6612BridgeMode {
    Coast,
    DrivePositive,
    DriveNegative,
    Brake,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tb6612ElectricalActuation {
    pub mode: Tb6612BridgeMode,
    pub duty_fraction: f32,
}

impl Tb6612ElectricalActuation {
    pub const fn safe_off() -> Self {
        Self {
            mode: Tb6612BridgeMode::Coast,
            duty_fraction: 0.0,
        }
    }
}

/// Electrical-semantic mapper for one TB6612 motor channel.
///
/// The mapper accepts only Supervisor-authorized proof types. Raw bounded plant
/// commands cannot be promoted to electrical output through a public API here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Tb6612Mapper {
    positive_command_is_positive_drive: bool,
}

impl Tb6612Mapper {
    pub const fn new(positive_command_is_positive_drive: bool) -> Self {
        Self {
            positive_command_is_positive_drive,
        }
    }

    pub fn closed_loop_frame(self, actuation: AuthorizedActuation) -> Tb6612ElectricalActuation {
        self.map_command(actuation.command())
    }

    pub fn maintenance_frame(self, actuation: MaintenanceActuation) -> Tb6612ElectricalActuation {
        self.map_command(actuation.command())
    }

    fn map_command(self, command: BoundedActuatorCommand) -> Tb6612ElectricalActuation {
        let value = command.command.get();
        if value == 0.0 {
            return Tb6612ElectricalActuation::safe_off();
        }

        let positive = value > 0.0;
        let mode = match (positive, self.positive_command_is_positive_drive) {
            (true, true) | (false, false) => Tb6612BridgeMode::DrivePositive,
            _ => Tb6612BridgeMode::DriveNegative,
        };

        Tb6612ElectricalActuation {
            mode,
            duty_fraction: value.abs(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rip_hybrid_control::ControlRegime;
    use rip_robot_domain::{NormalizedCommand, StateValidity, TorqueNm};
    use rip_runtime_state::{
        AuthorityContext, RuntimeAuthority, RuntimeState, SensorTimingHealth, WatchdogHealth,
    };

    #[test]
    fn electrical_mapping_requires_closed_loop_authorization() {
        let command = BoundedActuatorCommand {
            command: NormalizedCommand::new(0.5).unwrap(),
            saturated: false,
            predicted_arm_torque: TorqueNm(0.1),
        };
        let mut authority = RuntimeAuthority::new();
        authority.enter_closed_loop().unwrap();
        let authorized = authority
            .evaluate(
                AuthorityContext {
                    runtime_state: RuntimeState::Active(ControlRegime::Balance),
                    timing: SensorTimingHealth::Healthy,
                    watchdog: WatchdogHealth::Healthy,
                    estimate_validity: StateValidity::Valid,
                    runtime_qualified: true,
                },
                command,
            )
            .authorized()
            .unwrap();

        let frame = Tb6612Mapper::new(true).closed_loop_frame(authorized);
        assert_eq!(frame.mode, Tb6612BridgeMode::DrivePositive);
        assert_eq!(frame.duty_fraction, 0.5);
    }
}
