#![no_std]
#![forbid(unsafe_code)]

use rip_board_forest_s1_d1::{ARM_ENCODER, MOTOR_D1, MOTOR_D2, PENDULUM_ADC};

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
pub enum MotorPopulation {
    Installed(ActuatorRole),
    NotInstalled,
}

pub const D1_POPULATION: MotorPopulation = MotorPopulation::NotInstalled;
pub const D2_POPULATION: MotorPopulation = MotorPopulation::Installed(ActuatorRole::RotaryArm);
pub const PENDULUM_SENSOR_ROLE: SensorRole = SensorRole::PendulumAngle;
pub const ARM_ENCODER_ROLE: SensorRole = SensorRole::RotaryArmPosition;

pub const TELEMETRY_DEFAULT_ENABLED: bool = false;
pub const TELEMETRY_RATE_HZ: u32 = 10;
pub const UI_REFRESH_HZ: u32 = 10;
pub const OLED_BACKGROUND_FLUSH_BYTES: usize = 8;
pub const OLED_PRESENT: bool = true;

pub const fn telemetry_period_ticks(control_tick_hz: u32) -> u32 {
    control_tick_hz / TELEMETRY_RATE_HZ
}

pub const fn ui_period_ticks(control_tick_hz: u32) -> u32 {
    control_tick_hz / UI_REFRESH_HZ
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reference_assembly_maps_the_installed_motor_to_d2() {
        assert_eq!(D2_POPULATION, MotorPopulation::Installed(ActuatorRole::RotaryArm));
        assert_eq!(D1_POPULATION, MotorPopulation::NotInstalled);
        assert_eq!(MOTOR_D2.pwm_pin.index, 1);
        assert_eq!(MOTOR_D1.pwm_pin.index, 0);
    }

    #[test]
    fn reference_sensors_match_board_wiring() {
        assert_eq!(PENDULUM_ADC.channel, 7);
        assert_eq!(ARM_ENCODER.channel_a_pin.index, 0);
        assert_eq!(telemetry_period_ticks(1_000), 100);
        assert_eq!(ui_period_ticks(1_000), 100);
    }
}
