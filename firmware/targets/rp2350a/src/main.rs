#![no_std]
#![no_main]

use embedded_hal::digital::OutputPin;
use panic_halt as _;
use rip_assembly_rp2350_uno_balance::{board_mapping_is_consistent, CONTROL_OUTPUT_DEFAULT_ENABLED};
use rip_board_uno_rp2350::XTAL_FREQ_HZ;
use rp235x_hal as hal;

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

const _: () = assert!(board_mapping_is_consistent());
const _: () = assert!(!CONTROL_OUTPUT_DEFAULT_ENABLED);

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let _clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Safe commissioning baseline: the TB6612 channel-A command pins are held low.
    // PWM is intentionally left as a GPIO until bounded motor actuation is enabled.
    let mut motor_pwm = pins.gpio10.into_push_pull_output();
    let mut motor_in1 = pins.gpio13.into_push_pull_output();
    let mut motor_in2 = pins.gpio12.into_push_pull_output();
    motor_pwm.set_low().unwrap();
    motor_in1.set_low().unwrap();
    motor_in2.set_low().unwrap();

    // Reserve the mechanism inputs with no internal bias. The encoder electrical
    // level must be verified during preflight before closed-loop operation.
    let _arm_encoder_a = pins.gpio9.into_floating_input();
    let _arm_encoder_b = pins.gpio2.into_floating_input();

    let _adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);
    let _pendulum_adc = hal::adc::AdcPin::new(pins.gpio26).unwrap();

    loop {
        cortex_m::asm::wfi();
    }
}

#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 3] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"Rotary Inverted Pendulum RP2350A safe-idle target"),
];
