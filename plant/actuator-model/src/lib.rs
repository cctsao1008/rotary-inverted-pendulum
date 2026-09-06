#![no_std]
#![forbid(unsafe_code)]

use rip_robot_domain::{GeneralizedDemand, NormalizedCommand, TorqueNm};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArmActuatorParameters {
    pub torque_per_effective_command_nm: f32,
    pub command_deadzone: f32,
}

impl ArmActuatorParameters {
    pub fn new(torque_per_effective_command_nm: f32, command_deadzone: f32) -> Option<Self> {
        let candidate = Self {
            torque_per_effective_command_nm,
            command_deadzone,
        };
        candidate.is_valid().then_some(candidate)
    }

    pub fn is_valid(self) -> bool {
        self.torque_per_effective_command_nm.is_finite()
            && self.torque_per_effective_command_nm > 0.0
            && self.command_deadzone.is_finite()
            && (0.0..1.0).contains(&self.command_deadzone)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoundedActuatorCommand {
    pub command: NormalizedCommand,
    pub saturated: bool,
    pub predicted_arm_torque: TorqueNm,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActuatorModelError {
    InvalidParameters,
    NonFiniteDemand,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArmActuatorModel {
    parameters: ArmActuatorParameters,
}

impl ArmActuatorModel {
    pub fn new(parameters: ArmActuatorParameters) -> Option<Self> {
        parameters.is_valid().then_some(Self { parameters })
    }

    pub const fn parameters(self) -> ArmActuatorParameters {
        self.parameters
    }

    pub fn command_for_demand(
        self,
        demand: GeneralizedDemand,
    ) -> Result<BoundedActuatorCommand, ActuatorModelError> {
        if !self.parameters.is_valid() {
            return Err(ActuatorModelError::InvalidParameters);
        }
        if !demand.arm_torque.0.is_finite() {
            return Err(ActuatorModelError::NonFiniteDemand);
        }

        let required_effective =
            demand.arm_torque.0 / self.parameters.torque_per_effective_command_nm;
        let saturated = required_effective.abs() > 1.0;
        let bounded_effective = required_effective.clamp(-1.0, 1.0);
        let raw_command =
            inverse_effective_command(bounded_effective, self.parameters.command_deadzone);
        let command = NormalizedCommand::new(raw_command).expect("bounded inverse command");
        let predicted_arm_torque = TorqueNm(
            effective_command(command.get(), self.parameters.command_deadzone)
                * self.parameters.torque_per_effective_command_nm,
        );

        Ok(BoundedActuatorCommand {
            command,
            saturated,
            predicted_arm_torque,
        })
    }
}

fn effective_command(command: f32, deadzone: f32) -> f32 {
    let magnitude = command.abs();
    if magnitude <= deadzone {
        0.0
    } else {
        command.signum() * (magnitude - deadzone) / (1.0 - deadzone)
    }
}

fn inverse_effective_command(effective: f32, deadzone: f32) -> f32 {
    if effective == 0.0 {
        0.0
    } else {
        effective.signum() * (deadzone + (1.0 - deadzone) * effective.abs())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn model() -> ArmActuatorModel {
        ArmActuatorModel::new(ArmActuatorParameters::new(0.2, 0.1).unwrap()).unwrap()
    }

    #[test]
    fn inverse_model_recovers_requested_torque_inside_authority() {
        let command = model()
            .command_for_demand(GeneralizedDemand {
                arm_torque: TorqueNm(0.08),
            })
            .unwrap();

        assert!(!command.saturated);
        assert!((command.predicted_arm_torque.0 - 0.08).abs() < 1.0e-5);
    }

    #[test]
    fn excessive_torque_is_explicitly_saturated() {
        let command = model()
            .command_for_demand(GeneralizedDemand {
                arm_torque: TorqueNm(1.0),
            })
            .unwrap();

        assert!(command.saturated);
        assert_eq!(command.command.get(), 1.0);
    }
}
