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
    mode: Tb6612BridgeMode,
    duty_fraction: f32,
}

impl Tb6612ElectricalActuation {
    pub const fn safe_off() -> Self {
        Self {
            mode: Tb6612BridgeMode::Coast,
            duty_fraction: 0.0,
        }
    }

    pub const fn mode(self) -> Tb6612BridgeMode {
        self.mode
    }

    pub const fn duty_fraction(self) -> f32 {
        self.duty_fraction
    }

    pub fn is_valid(self) -> bool {
        self.duty_fraction.is_finite() && (0.0..=1.0).contains(&self.duty_fraction)
    }
}

/// Electrical-semantic mapper for one TB6612 motor channel.
///
/// Drive frames are created only inside this crate from Supervisor-owned proof
/// types. External code may inspect a frame but cannot construct an arbitrary
/// drive request and bypass authority.
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

    fn closed_loop_frame(self, actuation: AuthorizedActuation) -> Tb6612ElectricalActuation {
        self.map_command(actuation.command())
    }

    fn maintenance_frame(self, actuation: MaintenanceActuation) -> Tb6612ElectricalActuation {
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

/// Target/backend boundary for emitting one actuator-specific TB6612 frame.
///
/// A target may implement this trait with concrete PWM/GPIO resources, but the
/// runtime backend instance is intended to be owned by `Tb6612Output`. Since
/// drive frames cannot be publicly constructed, arbitrary frame-to-hardware
/// actuation is not a public route.
pub trait Tb6612FrameIo {
    type Error;

    fn apply_frame(&mut self, frame: Tb6612ElectricalActuation) -> Result<(), Self::Error>;
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

pub type Tb6612OutputResult<PwmError, In1Error, In2Error> =
    Result<(), Tb6612OutputError<PwmError, In1Error, In2Error>>;

/// Generic PWM/direction backend with an explicit break-before-make sequence.
///
/// Direction pins are never changed while a non-zero PWM request remains
/// applied. Every transition first removes PWM authority, updates the bridge
/// mode, then applies the requested duty.
pub struct Tb6612PwmDirIo<Pwm, In1, In2> {
    pwm: Pwm,
    in1: In1,
    in2: In2,
}

impl<Pwm, In1, In2> Tb6612PwmDirIo<Pwm, In1, In2>
where
    Pwm: DutyOutput,
    In1: LogicOutput,
    In2: LogicOutput,
{
    pub const fn new(pwm: Pwm, in1: In1, in2: In2) -> Self {
        Self { pwm, in1, in2 }
    }

    pub fn into_parts(self) -> (Pwm, In1, In2) {
        (self.pwm, self.in1, self.in2)
    }
}

impl<Pwm, In1, In2> Tb6612FrameIo for Tb6612PwmDirIo<Pwm, In1, In2>
where
    Pwm: DutyOutput,
    In1: LogicOutput,
    In2: LogicOutput,
{
    type Error = Tb6612OutputError<Pwm::Error, In1::Error, In2::Error>;

    fn apply_frame(&mut self, frame: Tb6612ElectricalActuation) -> Result<(), Self::Error> {
        if !frame.is_valid() {
            return Err(Tb6612OutputError::InvalidFrame);
        }

        self.pwm
            .set_duty_fraction(0.0)
            .map_err(Tb6612OutputError::Pwm)?;

        let (in1, in2, duty) = match frame.mode() {
            Tb6612BridgeMode::Coast => (false, false, 0.0),
            Tb6612BridgeMode::DrivePositive => (true, false, frame.duty_fraction()),
            Tb6612BridgeMode::DriveNegative => (false, true, frame.duty_fraction()),
            Tb6612BridgeMode::Brake => (true, true, 0.0),
        };

        self.in1.set_level(in1).map_err(Tb6612OutputError::In1)?;
        self.in2.set_level(in2).map_err(Tb6612OutputError::In2)?;
        self.pwm
            .set_duty_fraction(duty)
            .map_err(Tb6612OutputError::Pwm)?;
        Ok(())
    }
}

/// Firmware `ActuationSink` for one TB6612 channel.
///
/// The sink owns the frame backend and exposes no public frame-application
/// method. Closed-loop and maintenance paths therefore enter through distinct
/// Supervisor proof types; `safe_off` is the only unqualified output action.
pub struct Tb6612Output<Io> {
    mapper: Tb6612Mapper,
    io: Io,
}

impl<Io> Tb6612Output<Io>
where
    Io: Tb6612FrameIo,
{
    pub const fn new(mapper: Tb6612Mapper, io: Io) -> Self {
        Self { mapper, io }
    }

    pub fn into_inner(self) -> Io {
        self.io
    }
}

impl<Io> ActuationSink for Tb6612Output<Io>
where
    Io: Tb6612FrameIo,
{
    type Error = Io::Error;

    fn apply_closed_loop(&mut self, actuation: AuthorizedActuation) -> Result<(), Self::Error> {
        let frame = self.mapper.closed_loop_frame(actuation);
        self.io.apply_frame(frame)
    }

    fn apply_maintenance(&mut self, actuation: MaintenanceActuation) -> Result<(), Self::Error> {
        let frame = self.mapper.maintenance_frame(actuation);
        self.io.apply_frame(frame)
    }

    fn safe_off(&mut self) -> Result<(), Self::Error> {
        self.io.apply_frame(Tb6612ElectricalActuation::safe_off())
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
        assert_eq!(frame.mode(), Tb6612BridgeMode::DrivePositive);
        assert_eq!(frame.duty_fraction(), 0.5);
    }

    #[test]
    fn sink_owns_backend_and_applies_guarded_drive_and_safe_off() {
        let io = Tb6612PwmDirIo::new(MockPwm::default(), MockPin::default(), MockPin::default());
        let mut output = Tb6612Output::new(Tb6612Mapper::new(true), io);

        output.apply_closed_loop(authorized(-0.4)).unwrap();
        output.safe_off().unwrap();

        let io = output.into_inner();
        let (pwm, in1, in2) = io.into_parts();
        assert_eq!(pwm.duty, 0.0);
        assert_eq!(pwm.writes, 4);
        assert!(!in1.high);
        assert!(!in2.high);
    }

    #[test]
    fn pwm_dir_backend_rejects_invalid_internal_frame_before_gpio_changes() {
        let mut io =
            Tb6612PwmDirIo::new(MockPwm::default(), MockPin::default(), MockPin::default());
        let error = io
            .apply_frame(Tb6612ElectricalActuation {
                mode: Tb6612BridgeMode::DrivePositive,
                duty_fraction: 1.2,
            })
            .unwrap_err();
        assert_eq!(error, Tb6612OutputError::InvalidFrame);
        assert_eq!(io.pwm.writes, 0);
    }

    #[test]
    fn safe_off_is_the_only_publicly_constructible_frame() {
        let frame = Tb6612ElectricalActuation::safe_off();
        assert_eq!(frame.mode(), Tb6612BridgeMode::Coast);
        assert_eq!(frame.duty_fraction(), 0.0);
    }
}
