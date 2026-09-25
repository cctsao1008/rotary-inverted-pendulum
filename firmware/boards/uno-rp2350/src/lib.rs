#![no_std]
#![forbid(unsafe_code)]

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArduinoPin {
    D2,
    D9,
    D10,
    D12,
    D13,
    A0,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DigitalPin {
    pub arduino: ArduinoPin,
    pub gpio: u8,
}

impl DigitalPin {
    pub const fn new(arduino: ArduinoPin, gpio: u8) -> Self {
        Self { arduino, gpio }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AdcWiring {
    pub arduino: ArduinoPin,
    pub gpio: u8,
    pub channel: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EncoderWiring {
    pub channel_a: DigitalPin,
    pub channel_b: DigitalPin,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MotorWiring {
    pub pwm: DigitalPin,
    pub direction_1: DigitalPin,
    pub direction_2: DigitalPin,
}

pub const MCU: &str = "RP2350A";
pub const XTAL_FREQ_HZ: u32 = 12_000_000;
pub const CONTROL_TICK_HZ: u32 = 1_000;
pub const MOTOR_PWM_HZ: u32 = 20_000;

pub const ARM_MOTOR: MotorWiring = MotorWiring {
    pwm: DigitalPin::new(ArduinoPin::D10, 10),
    direction_1: DigitalPin::new(ArduinoPin::D13, 13),
    direction_2: DigitalPin::new(ArduinoPin::D12, 12),
};

pub const ARM_ENCODER: EncoderWiring = EncoderWiring {
    channel_a: DigitalPin::new(ArduinoPin::D9, 9),
    channel_b: DigitalPin::new(ArduinoPin::D2, 2),
};

pub const PENDULUM_ADC: AdcWiring = AdcWiring {
    arduino: ArduinoPin::A0,
    gpio: 26,
    channel: 0,
};

pub const ENCODER_SUPPLY_MV: u32 = 5_000;
pub const PENDULUM_SENSOR_SUPPLY_MV: u32 = 3_300;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rotary_arm_mapping_matches_uno_balance_channel_a() {
        assert_eq!(ARM_MOTOR.pwm.gpio, 10);
        assert_eq!(ARM_MOTOR.direction_1.gpio, 13);
        assert_eq!(ARM_MOTOR.direction_2.gpio, 12);
        assert_eq!(ARM_ENCODER.channel_a.gpio, 9);
        assert_eq!(ARM_ENCODER.channel_b.gpio, 2);
    }

    #[test]
    fn pendulum_angle_uses_adc0_on_gpio26() {
        assert_eq!(PENDULUM_ADC.gpio, 26);
        assert_eq!(PENDULUM_ADC.channel, 0);
        assert_eq!(PENDULUM_SENSOR_SUPPLY_MV, 3_300);
    }

    #[test]
    fn active_rotary_signals_do_not_share_gpio() {
        let pins = [
            ARM_MOTOR.pwm.gpio,
            ARM_MOTOR.direction_1.gpio,
            ARM_MOTOR.direction_2.gpio,
            ARM_ENCODER.channel_a.gpio,
            ARM_ENCODER.channel_b.gpio,
            PENDULUM_ADC.gpio,
        ];

        for (index, pin) in pins.iter().enumerate() {
            assert!(!pins[index + 1..].contains(pin));
        }
    }
}
