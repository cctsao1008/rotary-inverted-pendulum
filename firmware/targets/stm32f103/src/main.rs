#![no_std]
#![no_main]
#![deny(unsafe_code)]

use core::f32::consts::PI;
use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};

use cortex_m_rt::entry;
use panic_halt as _;
use rip_estimator_input_adapter::{EncoderCounterAccumulator, EstimatorInputAdapter};
use rip_measurement_model::{EncoderScale, PendulumCalibration};
use rip_plant_observation::{
    MeasurementQuality, RawArmEncoderObservation, RawObservation, RawPendulumObservation,
};
use rip_robot_domain::{EstimatedState, TimestampUs};
use rip_state_estimator::{BasicEstimator, Estimate, EstimatorConfig};
use stm32f1xx_hal::{
    adc, pac,
    prelude::*,
    rcc,
    time::{Instant, MonoTimer},
    timer::{pwm_input::QeiOptions, Timer},
};

const PENDULUM_UPRIGHT_ADC: u16 = 2_928;
const PENDULUM_RADIANS_PER_COUNT: f32 = 2.0 * PI / 4_096.0;
const PENDULUM_DIRECTION: i8 = 1;
const ARM_ENCODER_COUNTS_PER_REVOLUTION: f32 = 1_040.0;
const ARM_ENCODER_DIRECTION: i8 = 1;
const ESTIMATOR_MAX_GAP_US: u64 = 20_000;
const ESTIMATOR_RATE_FILTER_ALPHA: f32 = 1.0;

/// Debugger-visible observe-only snapshot. No actuator backend is linked into
/// this target, so these values cannot grant physical-output authority.
static SHADOW_SAMPLE_INDEX: AtomicU32 = AtomicU32::new(0);
static SHADOW_TIMESTAMP_US_LOW: AtomicU32 = AtomicU32::new(0);
static SHADOW_PENDULUM_ADC: AtomicU32 = AtomicU32::new(0);
static SHADOW_ARM_ENCODER_COUNT: AtomicI32 = AtomicI32::new(0);
static SHADOW_THETA_MRAD: AtomicI32 = AtomicI32::new(0);
static SHADOW_THETA_DOT_MRAD_S: AtomicI32 = AtomicI32::new(0);
static SHADOW_PHI_MRAD: AtomicI32 = AtomicI32::new(0);
static SHADOW_PHI_DOT_MRAD_S: AtomicI32 = AtomicI32::new(0);

struct MicrosecondTimebase {
    timer: MonoTimer,
    last: Instant,
    ticks_per_us: u32,
    remainder_ticks: u32,
    elapsed_us: u64,
}

impl MicrosecondTimebase {
    fn new(timer: MonoTimer) -> Self {
        let ticks_per_us = timer.frequency().raw() / 1_000_000;
        assert!(ticks_per_us > 0);
        Self {
            last: timer.now(),
            timer,
            ticks_per_us,
            remainder_ticks: 0,
            elapsed_us: 0,
        }
    }

    fn now(&mut self) -> TimestampUs {
        let elapsed_ticks = self.last.elapsed();
        self.last = self.timer.now();

        let total_ticks = self.remainder_ticks as u64 + elapsed_ticks as u64;
        let ticks_per_us = self.ticks_per_us as u64;
        self.elapsed_us = self.elapsed_us.wrapping_add(total_ticks / ticks_per_us);
        self.remainder_ticks = (total_ticks % ticks_per_us) as u32;
        TimestampUs(self.elapsed_us)
    }
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.freeze(
        rcc::Config::hse(8.MHz())
            .sysclk(72.MHz())
            .pclk1(36.MHz())
            .adcclk(12.MHz()),
        &mut flash.acr,
    );

    let mut gpioa = dp.GPIOA.split(&mut rcc);
    let mut pendulum_pin = gpioa.pa7.into_analog(&mut gpioa.crl);

    let mut adc1 = adc::Adc::new(dp.ADC1, &mut rcc);
    let qei = Timer::new(dp.TIM2, &mut rcc).qei((gpioa.pa0, gpioa.pa1), QeiOptions::default());

    let monotonic = MonoTimer::new(cp.DWT, cp.DCB, &rcc.clocks);
    let mut timebase = MicrosecondTimebase::new(monotonic);
    let mut delay = cp.SYST.delay(&rcc.clocks);

    let pendulum_calibration = PendulumCalibration::new(
        PENDULUM_UPRIGHT_ADC,
        PENDULUM_RADIANS_PER_COUNT,
        PENDULUM_DIRECTION,
    )
    .unwrap();
    let encoder_scale =
        EncoderScale::new(ARM_ENCODER_COUNTS_PER_REVOLUTION, ARM_ENCODER_DIRECTION).unwrap();
    let adapter = EstimatorInputAdapter::new(pendulum_calibration, encoder_scale);
    let mut encoder_accumulator = EncoderCounterAccumulator::new(qei.count());
    let mut estimator = BasicEstimator::new();
    let estimator_config = EstimatorConfig {
        max_gap_us: ESTIMATOR_MAX_GAP_US,
        rate_filter_alpha: ESTIMATOR_RATE_FILTER_ALPHA,
    };
    let quality = MeasurementQuality::AVAILABLE
        | MeasurementQuality::IO_OK
        | MeasurementQuality::TIMING_VALID;
    let mut sample_index = 0_u32;

    loop {
        let adc_raw: u16 = match adc1.read(&mut pendulum_pin) {
            Ok(value) => value,
            Err(_) => {
                delay.delay_ms(1_u16);
                continue;
            }
        };
        let accumulated_count = encoder_accumulator.update(qei.count());
        let captured_at = timebase.now();

        let raw = RawObservation {
            sample_index,
            pendulum: RawPendulumObservation {
                captured_at,
                adc_raw,
                quality,
            },
            arm_encoder: RawArmEncoderObservation {
                captured_at,
                accumulated_count,
                quality,
            },
        };
        sample_index = sample_index.wrapping_add(1);
        publish_raw(raw);

        if let Ok(measurement) = adapter.measurement(raw) {
            if let Ok(Estimate::Ready(state)) = estimator.step(estimator_config, measurement) {
                publish_state(state);
            }
        }

        delay.delay_ms(1_u16);
    }
}

fn publish_raw(raw: RawObservation) {
    SHADOW_SAMPLE_INDEX.store(raw.sample_index, Ordering::Relaxed);
    SHADOW_TIMESTAMP_US_LOW.store(raw.pendulum.captured_at.0 as u32, Ordering::Relaxed);
    SHADOW_PENDULUM_ADC.store(raw.pendulum.adc_raw as u32, Ordering::Relaxed);
    SHADOW_ARM_ENCODER_COUNT.store(raw.arm_encoder.accumulated_count, Ordering::Relaxed);
}

fn publish_state(state: EstimatedState) {
    SHADOW_THETA_MRAD.store(scale_milli(state.theta.0), Ordering::Relaxed);
    SHADOW_THETA_DOT_MRAD_S.store(scale_milli(state.theta_dot.0), Ordering::Relaxed);
    SHADOW_PHI_MRAD.store(scale_milli(state.phi.0), Ordering::Relaxed);
    SHADOW_PHI_DOT_MRAD_S.store(scale_milli(state.phi_dot.0), Ordering::Relaxed);
}

fn scale_milli(value: f32) -> i32 {
    let scaled = value * 1_000.0;
    if scaled >= i32::MAX as f32 {
        i32::MAX
    } else if scaled <= i32::MIN as f32 {
        i32::MIN
    } else {
        scaled as i32
    }
}
