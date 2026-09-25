#![no_std]
#![forbid(unsafe_code)]

use rip_board_uno_rp2350::{ARM_ENCODER, ARM_MOTOR, PENDULUM_ADC};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActuatorRole {
    RotaryArm,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SensorRole {
    PendulumAngle,
    RotaryArmPosition,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShieldMotorChannel {
    A,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShieldSignal {
    MaPlus,
    MaMinus,
    Encoder1A,
    Encoder1B,
    Vcc50,
    Ground,
    A0,
    Board3V3,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HarnessPin {
    pub mechanism_pin: u8,
    pub shield_signal: ShieldSignal,
}

pub const ROTARY_ARM_ACTUATOR: ActuatorRole = ActuatorRole::RotaryArm;
pub const ROTARY_ARM_CHANNEL: ShieldMotorChannel = ShieldMotorChannel::A;
pub const PENDULUM_SENSOR_ROLE: SensorRole = SensorRole::PendulumAngle;
pub const ARM_ENCODER_ROLE: SensorRole = SensorRole::RotaryArmPosition;

// Original mechanism six-pin motor/encoder harness -> UNO Balance JP2.
// The connector numbering reverses end-to-end: 1->6, 2->5, ... 6->1.
pub const ARM_MOTOR_ENCODER_HARNESS: [HarnessPin; 6] = [
    HarnessPin {
        mechanism_pin: 1,
        shield_signal: ShieldSignal::MaPlus,
    },
    HarnessPin {
        mechanism_pin: 2,
        shield_signal: ShieldSignal::Vcc50,
    },
    HarnessPin {
        mechanism_pin: 3,
        shield_signal: ShieldSignal::Encoder1A,
    },
    HarnessPin {
        mechanism_pin: 4,
        shield_signal: ShieldSignal::Encoder1B,
    },
    HarnessPin {
        mechanism_pin: 5,
        shield_signal: ShieldSignal::Ground,
    },
    HarnessPin {
        mechanism_pin: 6,
        shield_signal: ShieldSignal::MaMinus,
    },
];

// Pendulum potentiometer remains a 3.3 V ratiometric sensor. Do not source it
// from the shield VCC50 pin.
pub const PENDULUM_SENSOR_HARNESS: [HarnessPin; 3] = [
    HarnessPin {
        mechanism_pin: 1,
        shield_signal: ShieldSignal::Ground,
    },
    HarnessPin {
        mechanism_pin: 2,
        shield_signal: ShieldSignal::Board3V3,
    },
    HarnessPin {
        mechanism_pin: 3,
        shield_signal: ShieldSignal::A0,
    },
];

pub const TELEMETRY_DEFAULT_ENABLED: bool = false;
pub const CONTROL_OUTPUT_DEFAULT_ENABLED: bool = false;

pub const fn board_mapping_is_consistent() -> bool {
    ARM_MOTOR.pwm.gpio == 10
        && ARM_MOTOR.direction_1.gpio == 13
        && ARM_MOTOR.direction_2.gpio == 12
        && ARM_ENCODER.channel_a.gpio == 9
        && ARM_ENCODER.channel_b.gpio == 2
        && PENDULUM_ADC.gpio == 26
        && PENDULUM_ADC.channel == 0
}

const _: () = assert!(board_mapping_is_consistent());

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mechanism_harness_selects_motor_a_and_encoder1() {
        assert_eq!(ROTARY_ARM_CHANNEL, ShieldMotorChannel::A);
        assert_eq!(ARM_MOTOR_ENCODER_HARNESS[0].shield_signal, ShieldSignal::MaPlus);
        assert_eq!(ARM_MOTOR_ENCODER_HARNESS[2].shield_signal, ShieldSignal::Encoder1A);
        assert_eq!(ARM_MOTOR_ENCODER_HARNESS[3].shield_signal, ShieldSignal::Encoder1B);
        assert_eq!(ARM_MOTOR_ENCODER_HARNESS[5].shield_signal, ShieldSignal::MaMinus);
    }

    #[test]
    fn pendulum_sensor_uses_board_3v3_not_vcc50() {
        assert_eq!(
            PENDULUM_SENSOR_HARNESS[1].shield_signal,
            ShieldSignal::Board3V3
        );
        assert_eq!(PENDULUM_SENSOR_HARNESS[2].shield_signal, ShieldSignal::A0);
    }

    #[test]
    fn control_output_is_safe_by_default() {
        assert!(!CONTROL_OUTPUT_DEFAULT_ENABLED);
    }
}
