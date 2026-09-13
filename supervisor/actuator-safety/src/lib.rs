#![no_std]
#![forbid(unsafe_code)]

use rip_robot_domain::{NormalizedCommand, TimestampUs};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SafetyProfileKind {
    Commissioning,
    Simulation,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CommandSafetyLimits {
    max_abs_command: f32,
    max_slew_per_s: f32,
}

impl CommandSafetyLimits {
    pub fn new(max_abs_command: f32, max_slew_per_s: f32) -> Option<Self> {
        let limits = Self {
            max_abs_command,
            max_slew_per_s,
        };
        limits.is_valid().then_some(limits)
    }

    pub fn is_valid(self) -> bool {
        self.max_abs_command.is_finite()
            && self.max_abs_command > 0.0
            && self.max_abs_command <= 1.0
            && self.max_slew_per_s.is_finite()
            && self.max_slew_per_s > 0.0
    }

    pub const fn max_abs_command(self) -> f32 {
        self.max_abs_command
    }

    pub const fn max_slew_per_s(self) -> f32 {
        self.max_slew_per_s
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CommandSafetyProfile {
    kind: SafetyProfileKind,
    limits: CommandSafetyLimits,
}

impl CommandSafetyProfile {
    pub const fn new(kind: SafetyProfileKind, limits: CommandSafetyLimits) -> Self {
        Self { kind, limits }
    }

    pub const fn kind(self) -> SafetyProfileKind {
        self.kind
    }

    pub const fn limits(self) -> CommandSafetyLimits {
        self.limits
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CommandConstraintReasons(u8);

impl CommandConstraintReasons {
    pub const NONE: Self = Self(0);
    pub const MAGNITUDE: Self = Self(1 << 0);
    pub const SLEW: Self = Self(1 << 1);

    pub const fn bits(self) -> u8 {
        self.0
    }

    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    const fn with(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CommandSafetyOutcome {
    pub command: NormalizedCommand,
    pub profile: SafetyProfileKind,
    pub reasons: CommandConstraintReasons,
}

impl CommandSafetyOutcome {
    pub const fn constrained(self) -> bool {
        !self.reasons.is_empty()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandSafetyError {
    Unconfigured,
    NonMonotonicTimestamp,
}

/// Stateful Supervisor-side limiter for automatic closed-loop command authority.
///
/// The gate is intentionally unconfigured by default. A caller must install a
/// named profile before any command can pass. The first command after configure
/// or reset starts from zero authority, so a non-zero request must earn command
/// magnitude through the configured slew limit on subsequent timestamps.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CommandSafetyGate {
    profile: Option<CommandSafetyProfile>,
    last_command: NormalizedCommand,
    last_timestamp: Option<TimestampUs>,
}

impl CommandSafetyGate {
    pub const fn new() -> Self {
        Self {
            profile: None,
            last_command: NormalizedCommand::ZERO,
            last_timestamp: None,
        }
    }

    pub fn configure(&mut self, profile: CommandSafetyProfile) {
        self.profile = Some(profile);
        self.reset_history();
    }

    pub const fn profile(self) -> Option<CommandSafetyProfile> {
        self.profile
    }

    pub const fn is_configured(self) -> bool {
        self.profile.is_some()
    }

    /// Reset accumulated slew history while preserving the selected profile.
    ///
    /// Callers should use this on authority release, disable, fault entry, or
    /// any other transition that returns the physical output to safe-off.
    pub fn reset_history(&mut self) {
        self.last_command = NormalizedCommand::ZERO;
        self.last_timestamp = None;
    }

    pub fn constrain(
        &mut self,
        requested: NormalizedCommand,
        timestamp: TimestampUs,
    ) -> Result<CommandSafetyOutcome, CommandSafetyError> {
        let profile = self.profile.ok_or(CommandSafetyError::Unconfigured)?;
        let limits = profile.limits();

        let magnitude_bounded = requested
            .get()
            .clamp(-limits.max_abs_command(), limits.max_abs_command());
        let mut reasons = CommandConstraintReasons::NONE;
        if magnitude_bounded != requested.get() {
            reasons = reasons.with(CommandConstraintReasons::MAGNITUDE);
        }

        let previous = self.last_command.get();
        let slew_bounded = match self.last_timestamp {
            None => {
                if magnitude_bounded != 0.0 {
                    reasons = reasons.with(CommandConstraintReasons::SLEW);
                }
                0.0
            }
            Some(previous_timestamp) => {
                if timestamp < previous_timestamp {
                    return Err(CommandSafetyError::NonMonotonicTimestamp);
                }
                let elapsed_us = timestamp.0 - previous_timestamp.0;
                let max_delta = limits.max_slew_per_s() * elapsed_us as f32 * 1.0e-6;
                let bounded = magnitude_bounded.clamp(previous - max_delta, previous + max_delta);
                if bounded != magnitude_bounded {
                    reasons = reasons.with(CommandConstraintReasons::SLEW);
                }
                bounded
            }
        };

        let command = NormalizedCommand::new(slew_bounded)
            .expect("validated safety limits preserve normalized command range");
        self.last_command = command;
        self.last_timestamp = Some(timestamp);

        Ok(CommandSafetyOutcome {
            command,
            profile: profile.kind(),
            reasons,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn command(value: f32) -> NormalizedCommand {
        NormalizedCommand::new(value).unwrap()
    }

    fn profile(max_abs_command: f32, max_slew_per_s: f32) -> CommandSafetyProfile {
        CommandSafetyProfile::new(
            SafetyProfileKind::Simulation,
            CommandSafetyLimits::new(max_abs_command, max_slew_per_s).unwrap(),
        )
    }

    #[test]
    fn invalid_limit_configuration_is_rejected() {
        assert!(CommandSafetyLimits::new(0.0, 1.0).is_none());
        assert!(CommandSafetyLimits::new(1.1, 1.0).is_none());
        assert!(CommandSafetyLimits::new(0.5, 0.0).is_none());
        assert!(CommandSafetyLimits::new(f32::NAN, 1.0).is_none());
        assert!(CommandSafetyLimits::new(0.5, f32::INFINITY).is_none());
    }

    #[test]
    fn unconfigured_gate_fails_closed() {
        let mut gate = CommandSafetyGate::new();
        assert_eq!(
            gate.constrain(command(0.2), TimestampUs(1_000)),
            Err(CommandSafetyError::Unconfigured)
        );
    }

    #[test]
    fn first_nonzero_request_starts_from_zero_authority() {
        let mut gate = CommandSafetyGate::new();
        gate.configure(profile(1.0, 10.0));

        let outcome = gate.constrain(command(0.5), TimestampUs(1_000)).unwrap();
        assert_eq!(outcome.command, NormalizedCommand::ZERO);
        assert!(outcome.reasons.contains(CommandConstraintReasons::SLEW));
    }

    #[test]
    fn magnitude_limit_is_independent_of_controller_request() {
        let mut gate = CommandSafetyGate::new();
        gate.configure(profile(0.25, 10.0));
        gate.constrain(command(1.0), TimestampUs(0)).unwrap();

        let outcome = gate
            .constrain(command(1.0), TimestampUs(1_000_000))
            .unwrap();
        assert_eq!(outcome.command, command(0.25));
        assert!(outcome
            .reasons
            .contains(CommandConstraintReasons::MAGNITUDE));
        assert!(!outcome.reasons.contains(CommandConstraintReasons::SLEW));
    }

    #[test]
    fn slew_limit_applies_at_exact_timestamp_delta() {
        let mut gate = CommandSafetyGate::new();
        gate.configure(profile(1.0, 0.5));
        gate.constrain(command(1.0), TimestampUs(0)).unwrap();

        let first = gate
            .constrain(command(1.0), TimestampUs(200_000))
            .unwrap();
        let second = gate
            .constrain(command(1.0), TimestampUs(400_000))
            .unwrap();

        assert!((first.command.get() - 0.1).abs() < 1.0e-6);
        assert!((second.command.get() - 0.2).abs() < 1.0e-6);
        assert!(first.reasons.contains(CommandConstraintReasons::SLEW));
        assert!(second.reasons.contains(CommandConstraintReasons::SLEW));
    }

    #[test]
    fn sign_reversal_cannot_jump_through_zero() {
        let mut gate = CommandSafetyGate::new();
        gate.configure(profile(1.0, 0.5));
        gate.constrain(command(1.0), TimestampUs(0)).unwrap();
        gate.constrain(command(1.0), TimestampUs(400_000)).unwrap();

        let reversed = gate
            .constrain(command(-1.0), TimestampUs(600_000))
            .unwrap();
        assert!((reversed.command.get() - 0.1).abs() < 1.0e-6);
        assert!(reversed
            .reasons
            .contains(CommandConstraintReasons::SLEW));
    }

    #[test]
    fn reset_history_requires_slew_ramp_to_restart_from_zero() {
        let mut gate = CommandSafetyGate::new();
        gate.configure(profile(1.0, 10.0));
        gate.constrain(command(0.5), TimestampUs(0)).unwrap();
        let moving = gate
            .constrain(command(0.5), TimestampUs(100_000))
            .unwrap();
        assert_eq!(moving.command, command(0.5));

        gate.reset_history();
        let restarted = gate
            .constrain(command(0.5), TimestampUs(200_000))
            .unwrap();
        assert_eq!(restarted.command, NormalizedCommand::ZERO);
        assert!(restarted
            .reasons
            .contains(CommandConstraintReasons::SLEW));
    }

    #[test]
    fn non_monotonic_timestamp_fails_closed_without_advancing_history() {
        let mut gate = CommandSafetyGate::new();
        gate.configure(profile(1.0, 1.0));
        gate.constrain(command(0.5), TimestampUs(100_000)).unwrap();
        gate.constrain(command(0.5), TimestampUs(200_000)).unwrap();

        assert_eq!(
            gate.constrain(command(0.5), TimestampUs(150_000)),
            Err(CommandSafetyError::NonMonotonicTimestamp)
        );

        let next = gate
            .constrain(command(0.5), TimestampUs(300_000))
            .unwrap();
        assert!((next.command.get() - 0.2).abs() < 1.0e-6);
    }

    #[test]
    fn profile_identity_is_preserved_in_outcome() {
        let limits = CommandSafetyLimits::new(0.2, 0.4).unwrap();
        let mut gate = CommandSafetyGate::new();
        gate.configure(CommandSafetyProfile::new(
            SafetyProfileKind::Commissioning,
            limits,
        ));

        let outcome = gate.constrain(command(0.0), TimestampUs(0)).unwrap();
        assert_eq!(outcome.profile, SafetyProfileKind::Commissioning);
    }
}
