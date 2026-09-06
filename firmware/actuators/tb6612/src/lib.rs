#![no_std]
#![forbid(unsafe_code)]

use rip_actuation_interface::ActuationSink;
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

    pub fn is_valid(self) -> bool {
        self.duty_fraction.is_finite() && (0.0..=1.0).contains(&self.duty_fraction)
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

/// Minimal Firmware boundary required from a PWM realization.
pub trait DutyOutput {
    type Error;

    fn set_duty_fraction(&mut self, duty_fraction: f32) -> Result<(), Self::Error>;
}

/// Minimal Firmware boundary required from one digital direction output.
pub trait LogicOutput {
    type Error;

    fn set_level(&mut self, high: bool) -> Result<(), Self::Error>;
}

#[derive(Debug, PartialEq)]
pub enum Tb6612OutputError<PwmError, In1Error, In2Error> {
    InvalidFrame,
    Pwm(PwmError),
    In1(In1Error),
    In2(In2Error),
}

/// Hardware-facing TB6612 sink with an explicit break-before-make sequence.
///
/// Direction pins are never changed while a non-zero PWM request remains
/// applied. Every transition first removes PWM authority, updates the bridge
/// mode, then applies the requested duty.
pub struct Tb6612Output<Pwm, In1, In2> {
    mapper: Tb6612Mapper,
    pwm: Pwm,
    in1: In1,
    in2: In2,
}

impl<Pwm, In1, In2> Tb6612Output<Pwm, In1, In2>
where
    Pwm: DutyOutput,
    In1: LogicOutput,
    In2: LogicOutput,
{
    pub const fn new(mapper: Tb6612Mapper, pwm: Pwm, in1: In1, in2: In2) -> Self {
        Self {
            mapper,
            pwm,
            in1,
            in2,
        }
    }

    pub fn apply_frame(
        &mut self,
        frame: Tb6612ElectricalActuation,
    ) -> Result<(), Tb6612OutputError<Pwm::Error, In1::Error, In2::Error>> {
        if !frame.is_valid() {
            return Err(Tb6612OutputError::InvalidFrame);
        }

        self.pwm
            .set_duty_fraction(0.0)
            .map_err(Tb6612OutputError::Pwm)?;

        let (in1, in2, duty) = match frame.mode {
            Tb6612BridgeMode::Coast => (false, false, 0.0),
            Tb6612BridgeMode::DrivePositive => (true, false, frame.duty_fraction),
            Tb6612BridgeMode::DriveNegative => (false, true, frame.duty_fraction),
            Tb6612BridgeMode::Brake => (true, true, 0.0),
        };

        self.in1
            .set_level(in1)
            .map_err(Tb6612OutputError::In1)?;
        self.in2
            .set_level(in2)
            .map_err(Tb6612OutputError::In2)?;
        self.pwm
            .set_duty_fraction(duty)
            .map_err(Tb6612OutputError::Pwm)?;
        Ok(())
    }

    pub fn into_parts(self) -> (Pwm, In1, In2) {
        (self.pwm, self.in1, self.in2)
    }
}

impl<Pwm, In1, In2> ActuationSink for Tb6612Output<Pwm, In1, In2>
where
    Pwm: DutyOutput,
    In1: LogicOutput,
    In2: LogicOutput,
{
    type Error = Tb6612OutputError<Pwm::Error, In1::Error, In2::Error>;

    fn apply_closed_loop(&mut self, actuation: AuthorizedActuation) -> Result<(), Self::Error> {
        self.apply_frame(self.mapper.closed_loop_frame(actuation))
    }

    fn apply_maintenance(&mut self, actuation: MaintenanceActuation) -> Result<(), Self::Error> {
        self.apply_frame(self.mapper.maintenance_frame(actuation))
    }

    fn safe_off(&mut self) -> Result<(), Self::Error> {
        self.apply_frame(Tb6612ElectricalActuation::safe_off())
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

    #[derive(Default)]
    struct MockPwm {
        duty: f32,
        writes: u32,
    }

    impl DutyOutput for MockPwm {
        type Error = ();

        fn set_duty_fraction(&mut self, duty_fraction: f32) -> Result<(), Self::Error> {
            self.duty = duty_fraction;
            self.writes += 1;
            Ok(())
        }
    }

    #[derive(Default)]
    struct MockPin {
        high: bool,
    }

    impl LogicOutput for MockPin {
        type Error = ();

        fn set_level(&mut self, high: bool) -> Result<(), Self::Error> {
            self.high = high;
            Ok(())
        }
    }

    fn command(value: f32) -> BoundedActuatorCommand {
        BoundedActuatorCommand {
            command: NormalizedCommand::new(value).unwrap(),
            saturated: false,
            predicted_arm_torque: TorqueNm(0.1 * value),
        }
    }

    fn authorized(value: f32) -> AuthorizedActuation {
        let mut authority = RuntimeAuthority::new();
        authority.enter_closed_loop().unwrap();
        authority
            .evaluate(
                AuthorityContext {
                    runtime_state: RuntimeState::Active(ControlRegime::Balance),
                    timing: SensorTimingHealth::Healthy,
                    watchdog: WatchdogHealth::Healthy,
                    estimate_validity: StateValidity::Valid,
                    runtime_qualified: true,
                },
                command(value),
            )
            .authorized()
            .unwrap()
    }

    #[test]
    fn electrical_mapping_requires_closed_loop_authorization() {
        let frame = Tb6612Mapper::new(true).closed_loop_frame(authorized(0.5));
        assert_eq!(frame.mode, Tb6612BridgeMode::DrivePositive);
        assert_eq!(frame.duty_fraction, 0.5);
    }

    #[test]
    fn hardware_sink_applies_guarded_drive_and_safe_off() {
        let mut output = Tb6612Output::new(
            Tb6612Mapper::new(true),
            MockPwm::default(),
            MockPin::default(),
            MockPin::default(),
        );

        output.apply_closed_loop(authorized(-0.4)).unwrap();
        assert_eq!(output.pwm.duty, 0.4);
        assert!(!output.in1.high);
        assert!(output.in2.high);
        assert_eq!(output.pwm.writes, 2);

        output.safe_off().unwrap();
        assert_eq!(output.pwm.duty, 0.0);
        assert!(!output.in1.high);
        assert!(!output.in2.high);
    }

    #[test]
    fn invalid_frame_is_rejected_before_gpio_changes() {
        let mut output = Tb6612Output::new(
            Tb6612Mapper::new(true),
            MockPwm::default(),
            MockPin::default(),
            MockPin::default(),
        );
        let error = output
            .apply_frame(Tb6612ElectricalActuation {
                mode: Tb6612BridgeMode::DrivePositive,
                duty_fraction: 1.2,
            })
            .unwrap_err();
        assert_eq!(error, Tb6612OutputError::InvalidFrame);
        assert_eq!(output.pwm.writes, 0);
    }
}
