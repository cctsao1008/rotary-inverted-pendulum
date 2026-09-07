#![no_std]
#![forbid(unsafe_code)]

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Port {
    A,
    B,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pin {
    pub port: Port,
    pub index: u8,
}

impl Pin {
    pub const fn new(port: Port, index: u8) -> Self {
        Self { port, index }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimerChannel {
    Tim2Ch1,
    Tim2Ch2,
    Tim3Ch3,
    Tim3Ch4,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AdcWiring {
    pub pin: Pin,
    pub channel: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EncoderWiring {
    pub channel_a_pin: Pin,
    pub channel_a_timer: TimerChannel,
    pub channel_b_pin: Pin,
    pub channel_b_timer: TimerChannel,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MotorWiring {
    pub pwm_pin: Pin,
    pub pwm_timer: TimerChannel,
    pub direction_1_pin: Pin,
    pub direction_2_pin: Pin,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SerialWiring {
    pub tx_pin: Pin,
    pub rx_pin: Pin,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OledWiring {
    pub clock_pin: Pin,
    pub data_pin: Pin,
    pub reset_pin: Pin,
    pub dc_pin: Pin,
}

pub const MCU: &str = "STM32F103C8T6";
pub const HSE_MHZ: u32 = 8;
pub const SYSTEM_CLOCK_MHZ: u32 = 72;
pub const SYSTEM_CLOCK_HZ: u32 = SYSTEM_CLOCK_MHZ * 1_000_000;

pub const PENDULUM_ADC: AdcWiring = AdcWiring {
    pin: Pin::new(Port::A, 7),
    channel: 7,
};
pub const BATTERY_ADC: AdcWiring = AdcWiring {
    pin: Pin::new(Port::A, 6),
    channel: 6,
};

pub const ARM_ENCODER: EncoderWiring = EncoderWiring {
    channel_a_pin: Pin::new(Port::A, 0),
    channel_a_timer: TimerChannel::Tim2Ch1,
    channel_b_pin: Pin::new(Port::A, 1),
    channel_b_timer: TimerChannel::Tim2Ch2,
};

pub const MOTOR_D1: MotorWiring = MotorWiring {
    pwm_pin: Pin::new(Port::B, 0),
    pwm_timer: TimerChannel::Tim3Ch3,
    direction_1_pin: Pin::new(Port::B, 14),
    direction_2_pin: Pin::new(Port::B, 15),
};

pub const MOTOR_D2: MotorWiring = MotorWiring {
    pwm_pin: Pin::new(Port::B, 1),
    pwm_timer: TimerChannel::Tim3Ch4,
    direction_1_pin: Pin::new(Port::B, 13),
    direction_2_pin: Pin::new(Port::B, 12),
};

pub const MAIN_UART: SerialWiring = SerialWiring {
    tx_pin: Pin::new(Port::A, 9),
    rx_pin: Pin::new(Port::A, 10),
};

pub const OLED: OledWiring = OledWiring {
    clock_pin: Pin::new(Port::B, 5),
    data_pin: Pin::new(Port::B, 4),
    reset_pin: Pin::new(Port::B, 3),
    dc_pin: Pin::new(Port::A, 15),
};
pub const OLED_REQUIRES_JTAG_DISABLE: bool = true;
const _: () = assert!(OLED_REQUIRES_JTAG_DISABLE);
pub const SWDIO: Pin = Pin::new(Port::A, 13);
pub const SWCLK: Pin = Pin::new(Port::A, 14);

pub const KEY_M: Pin = Pin::new(Port::A, 3);
pub const KEY_X: Pin = Pin::new(Port::A, 2);
pub const KEY_PLUS: Pin = Pin::new(Port::A, 11);
pub const KEY_MINUS: Pin = Pin::new(Port::A, 12);
pub const KEY_USER: Pin = Pin::new(Port::A, 5);
pub const USER_LED: Pin = Pin::new(Port::A, 4);

pub const CONTROL_TICK_HZ: u32 = 1_000;
pub const MOTOR_PWM_HZ: u32 = 20_000;
pub const UART_BAUD: u32 = 115_200;
pub const OLED_SOFTWARE_SPI_HALF_PERIOD_PADDING: u8 = 6;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn debug_port_and_oled_wiring_do_not_claim_swd_pins() {
        assert_ne!(OLED.clock_pin, SWDIO);
        assert_ne!(OLED.data_pin, SWCLK);
    }
}
