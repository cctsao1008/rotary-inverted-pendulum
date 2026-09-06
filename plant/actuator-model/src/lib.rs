#![no_std]
#![forbid(unsafe_code)]

use rip_robot_domain::{AngularRateRadPerSec, GeneralizedDemand, NormalizedCommand, TorqueNm};

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
        positive(self.torque_per_effective_command_nm) && valid_deadzone(self.command_deadzone)
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
    NonFiniteState,
}

/// Compact static inverse actuator model used when only an effective torque span
/// and command deadzone are available.
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

/// Reference-backed DC motor + gearbox parameters for a speed-aware actuator model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DcMotorActuatorParameters {
    pub supply_voltage_v: f32,
    pub armature_resistance_ohm: f32,
    pub torque_constant_nm_per_a: f32,
    pub back_emf_v_per_rad_s: f32,
    pub gear_ratio_motor_per_arm: f32,
    pub gear_efficiency: f32,
    pub max_abs_current_a: f32,
    pub command_deadzone: f32,
}

impl DcMotorActuatorParameters {
    pub fn is_valid(self) -> bool {
        positive(self.supply_voltage_v)
            && positive(self.armature_resistance_ohm)
            && positive(self.torque_constant_nm_per_a)
            && nonnegative(self.back_emf_v_per_rad_s)
            && positive(self.gear_ratio_motor_per_arm)
            && self.gear_efficiency.is_finite()
            && self.gear_efficiency > 0.0
            && self.gear_efficiency <= 1.0
            && positive(self.max_abs_current_a)
            && valid_deadzone(self.command_deadzone)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DcMotorActuatorModel {
    parameters: DcMotorActuatorParameters,
}

impl DcMotorActuatorModel {
    pub fn new(parameters: DcMotorActuatorParameters) -> Option<Self> {
        parameters.is_valid().then_some(Self { parameters })
    }

    pub const fn parameters(self) -> DcMotorActuatorParameters {
        self.parameters
    }

    /// Convert requested arm torque into a realizable normalized drive command.
    ///
    /// The model accounts for gearbox ratio/efficiency, armature resistance,
    /// motor torque constant, back-EMF at the measured arm speed, supply-voltage
    /// authority, command deadzone, and an explicit current limit.
    pub fn command_for_demand(
        self,
        demand: GeneralizedDemand,
        arm_rate: AngularRateRadPerSec,
    ) -> Result<BoundedActuatorCommand, ActuatorModelError> {
        let p = self.parameters;
        if !p.is_valid() {
            return Err(ActuatorModelError::InvalidParameters);
        }
        if !demand.arm_torque.0.is_finite() {
            return Err(ActuatorModelError::NonFiniteDemand);
        }
        if !arm_rate.0.is_finite() {
            return Err(ActuatorModelError::NonFiniteState);
        }

        let motor_rate = arm_rate.0 * p.gear_ratio_motor_per_arm;
        let required_motor_torque =
            demand.arm_torque.0 / (p.gear_ratio_motor_per_arm * p.gear_efficiency);
        let required_current = required_motor_torque / p.torque_constant_nm_per_a;
        let current_limited = required_current.clamp(-p.max_abs_current_a, p.max_abs_current_a);
        let required_voltage = current_limited * p.armature_resistance_ohm
            + p.back_emf_v_per_rad_s * motor_rate;
        let required_effective_command = required_voltage / p.supply_voltage_v;

        if !required_effective_command.is_finite() {
            return Err(ActuatorModelError::NonFiniteDemand);
        }

        let saturated = required_current.abs() > p.max_abs_current_a
            || required_effective_command.abs() > 1.0;
        let bounded_effective = required_effective_command.clamp(-1.0, 1.0);
        let command_value = inverse_effective_command(bounded_effective, p.command_deadzone);
        let command = NormalizedCommand::new(command_value).expect("bounded inverse command");

        let predicted_arm_torque = if command.get() == 0.0 {
            TorqueNm(0.0)
        } else {
            let applied_voltage =
                effective_command(command.get(), p.command_deadzone) * p.supply_voltage_v;
            let predicted_current = ((applied_voltage - p.back_emf_v_per_rad_s * motor_rate)
                / p.armature_resistance_ohm)
                .clamp(-p.max_abs_current_a, p.max_abs_current_a);
            TorqueNm(
                predicted_current
                    * p.torque_constant_nm_per_a
                    * p.gear_ratio_motor_per_arm
                    * p.gear_efficiency,
            )
        };

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

fn positive(value: f32) -> bool {
    value.is_finite() && value > 0.0
}

fn nonnegative(value: f32) -> bool {
    value.is_finite() && value >= 0.0
}

fn valid_deadzone(value: f32) -> bool {
    value.is_finite() && (0.0..1.0).contains(&value)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn model() -> ArmActuatorModel {
        ArmActuatorModel::new(ArmActuatorParameters::new(0.2, 0.1).unwrap()).unwrap()
    }

    fn dc_model() -> DcMotorActuatorModel {
        DcMotorActuatorModel::new(DcMotorActuatorParameters {
            supply_voltage_v: 12.0,
            armature_resistance_ohm: 8.4,
            torque_constant_nm_per_a: 0.042,
            back_emf_v_per_rad_s: 0.042,
            gear_ratio_motor_per_arm: 1.0,
            gear_efficiency: 1.0,
            max_abs_current_a: 1.2,
            command_deadzone: 0.0,
        })
        .unwrap()
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

    #[test]
    fn dc_motor_model_accounts_for_back_emf() {
        let low_speed = dc_model()
            .command_for_demand(
                GeneralizedDemand {
                    arm_torque: TorqueNm(0.02),
                },
                AngularRateRadPerSec(0.0),
            )
            .unwrap();
        let high_speed = dc_model()
            .command_for_demand(
                GeneralizedDemand {
                    arm_torque: TorqueNm(0.02),
                },
                AngularRateRadPerSec(20.0),
            )
            .unwrap();

        assert!(high_speed.command.get() > low_speed.command.get());
    }

    #[test]
    fn dc_motor_model_marks_current_limit_as_saturation() {
        let command = dc_model()
            .command_for_demand(
                GeneralizedDemand {
                    arm_torque: TorqueNm(1.0),
                },
                AngularRateRadPerSec(0.0),
            )
            .unwrap();
        assert!(command.saturated);
        assert!(command.predicted_arm_torque.0 <= 0.042 * 1.2 + 1.0e-6);
    }
}
