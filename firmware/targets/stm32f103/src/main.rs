#![no_std]
#![no_main]
#![deny(unsafe_code)]

use core::f32::consts::PI;
use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};

use cortex_m_rt::entry;
use panic_halt as _;
use rip_actuator_model::{ArmActuatorModel, ArmActuatorParameters};
use rip_control_runtime::{
    ControlCycle, ControlRuntime, RuntimeObservation, RuntimeObservationSource,
};
use rip_estimator_input_adapter::{EncoderCounterAccumulator, EstimatorInputAdapter};
use rip_hybrid_control::{
    CapturePolicy, CapturePolicyConfig, ControlRegime, EnergySwingUpConfig,
    EnergySwingUpController, HybridController,
};
use rip_measurement_model::{EncoderScale, PendulumCalibration};
use rip_plant_observation::{
    MeasurementQuality, RawArmEncoderObservation, RawObservation, RawPendulumObservation,
};
use rip_robot_domain::{EstimatedState, TimestampUs};
use rip_runtime_state::{
    ControlWatchdog, RuntimeLimits, SensorTimingHealth, SensorTimingLimits, SensorTimingMonitor,
    WatchdogHealth,
};
use rip_state_estimator::EstimatorConfig;
use rip_state_feedback::LqrController;
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

const SENSOR_EXPECTED_PERIOD_US: u64 = 1_000;
const SENSOR_LATE_AFTER_US: u64 = 5_000;
const SENSOR_TIMEOUT_AFTER_US: u64 = 20_000;
const CONTROL_WATCHDOG_TIMEOUT_US: u64 = 20_000;

// Reference-backed live-shadow controller parameters from the QNET RIP model
// in Abdullah et al. (2021). These are not Forest D1 specimen calibration.
const SHADOW_LQR_TORQUE_GAINS: [f32; 4] = [0.183_55, 0.015_85, 0.011_20, 0.007_66];
const SHADOW_PENDULUM_MASS_KG: f32 = 0.04;
const SHADOW_PENDULUM_COM_LENGTH_M: f32 = 0.129;
const SHADOW_PENDULUM_INERTIA_KG_M2: f32 = 0.0001;
const SHADOW_TARGET_ENERGY_J: f32 = 0.025;
// EBC ku=35 in the reference voltage domain and Kt/Rm=0.005 Nm/V.
const SHADOW_ENERGY_TORQUE_GAIN: f32 = 0.175;
const SHADOW_MAX_ABS_TORQUE_NM: f32 = 0.05;
const SHADOW_SWING_KICK_TORQUE_NM: f32 = 0.01;
const SHADOW_SWING_KICK_BELOW_RATE_RAD_S: f32 = 0.05;

// Project live-shadow capture policy. The wider Capture window gives explicit
// hysteresis around the smaller Balance admission region.
const SHADOW_CAPTURE_ENTER_ANGLE_RAD: f32 = 20.0 * PI / 180.0;
const SHADOW_CAPTURE_ENTER_RATE_RAD_S: f32 = 3.0;
const SHADOW_BALANCE_ENTER_ANGLE_RAD: f32 = 8.0 * PI / 180.0;
const SHADOW_BALANCE_ENTER_RATE_RAD_S: f32 = 1.0;
const SHADOW_BALANCE_EXIT_ANGLE_RAD: f32 = 12.0 * PI / 180.0;
const SHADOW_BALANCE_EXIT_RATE_RAD_S: f32 = 2.0;
const SHADOW_CAPTURE_EXIT_ANGLE_RAD: f32 = 30.0 * PI / 180.0;
const SHADOW_CAPTURE_EXIT_RATE_RAD_S: f32 = 4.0;
const SHADOW_CAPTURE_SETTLE_CYCLES: u16 = 20;

// QNET reference nominal: Kt/Rm = 0.005 Nm/V and reported +/-10 V control
// saturation gives a zero-speed static torque span of 0.05 Nm.
const SHADOW_ACTUATOR_TORQUE_PER_EFFECTIVE_COMMAND_NM: f32 = 0.05;
const SHADOW_ACTUATOR_COMMAND_DEADZONE: f32 = 0.0;

const CYCLE_IDLE: u32 = 0;
const CYCLE_PRIMED: u32 = 1;
const CYCLE_REJECTED: u32 = 2;
const CYCLE_COMPUTED: u32 = 3;
const CYCLE_ERROR: u32 = 4;

/// Debugger-visible live-shadow snapshot.
///
/// The concrete STM32 motor channel is configured into hard safe-off at boot:
/// PB1/TIM3_CH4 duty=0 and PB13/PB12 low. No ActuationSink is connected to
/// those peripherals, so sensing, estimation, hybrid control, actuator-model,
/// and authority computation terminate in debugger-visible data only.
static SHADOW_SAMPLE_INDEX: AtomicU32 = AtomicU32::new(0);
static SHADOW_TIMESTAMP_US_LOW: AtomicU32 = AtomicU32::new(0);
static SHADOW_PENDULUM_ADC: AtomicU32 = AtomicU32::new(0);
static SHADOW_ARM_ENCODER_COUNT: AtomicI32 = AtomicI32::new(0);
static SHADOW_THETA_MRAD: AtomicI32 = AtomicI32::new(0);
static SHADOW_THETA_DOT_MRAD_S: AtomicI32 = AtomicI32::new(0);
static SHADOW_PHI_MRAD: AtomicI32 = AtomicI32::new(0);
static SHADOW_PHI_DOT_MRAD_S: AtomicI32 = AtomicI32::new(0);
static SHADOW_CYCLE: AtomicU32 = AtomicU32::new(CYCLE_IDLE);
static SHADOW_CONTROL_REGIME: AtomicU32 = AtomicU32::new(0);
static SHADOW_DEMAND_TORQUE_UNM: AtomicI32 = AtomicI32::new(0);
static SHADOW_BOUNDED_COMMAND_PPM: AtomicI32 = AtomicI32::new(0);
static SHADOW_PREDICTED_TORQUE_UNM: AtomicI32 = AtomicI32::new(0);
static SHADOW_ACTUATOR_SATURATED: AtomicU32 = AtomicU32::new(0);
static SHADOW_QUALIFICATION_REASONS: AtomicU32 = AtomicU32::new(0);
static SHADOW_AUTHORITY_REASONS: AtomicU32 = AtomicU32::new(0);
static SHADOW_AUTHORIZED: AtomicU32 = AtomicU32::new(0);
static SHADOW_SENSOR_TIMING_HEALTH: AtomicU32 = AtomicU32::new(0);
static SHADOW_WATCHDOG_HEALTH: AtomicU32 = AtomicU32::new(0);

struct PendingObservationSource {
    pending: Option<RuntimeObservation>,
}

impl PendingObservationSource {
    const fn new() -> Self {
        Self { pending: None }
    }

    fn submit(&mut self, observation: RuntimeObservation) {
        self.pending = Some(observation);
    }
}

impl RuntimeObservationSource for PendingObservationSource {
    type Error = ();

    fn observe(&mut self) -> Result<RuntimeObservation, Self::Error> {
        self.pending.take().ok_or(())
    }
}

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
    let mut gpiob = dp.GPIOB.split(&mut rcc);
    let mut pendulum_pin = gpioa.pa7.into_analog(&mut gpioa.crl);

    let mut adc1 = adc::Adc::new(dp.ADC1, &mut rcc);
    let qei = Timer::new(dp.TIM2, &mut rcc).qei((gpioa.pa0, gpioa.pa1), QeiOptions::default());

    // D2 is the installed rotary-arm motor channel. Bind the concrete pins now,
    // but expose no runtime actuation object: hard safe-off is the only physical
    // state reachable from this executable.
    let mut motor_in1 = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);
    let mut motor_in2 = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
    motor_in1.set_low();
    motor_in2.set_low();
    let (_motor_pwm_manager, (.., motor_pwm_channel)) = dp.TIM3.pwm_hz(20.kHz(), &mut rcc);
    let mut motor_pwm = motor_pwm_channel.with(gpiob.pb1);
    motor_pwm.set_duty(0);
    motor_pwm.enable();

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

    let estimator_config = EstimatorConfig {
        max_gap_us: ESTIMATOR_MAX_GAP_US,
        rate_filter_alpha: ESTIMATOR_RATE_FILTER_ALPHA,
    };
    let balance_controller = LqrController::new(SHADOW_LQR_TORQUE_GAINS).unwrap();
    let swing_up_controller = EnergySwingUpController::new(EnergySwingUpConfig {
        pendulum_mass_kg: SHADOW_PENDULUM_MASS_KG,
        pendulum_com_length_m: SHADOW_PENDULUM_COM_LENGTH_M,
        pendulum_inertia_kg_m2: SHADOW_PENDULUM_INERTIA_KG_M2,
        gravity_m_s2: 9.81,
        target_energy_j: SHADOW_TARGET_ENERGY_J,
        energy_gain: SHADOW_ENERGY_TORQUE_GAIN,
        max_abs_torque_nm: SHADOW_MAX_ABS_TORQUE_NM,
        kick_torque_nm: SHADOW_SWING_KICK_TORQUE_NM,
        kick_below_rate_rad_s: SHADOW_SWING_KICK_BELOW_RATE_RAD_S,
    })
    .unwrap();
    let capture_policy = CapturePolicy::new(CapturePolicyConfig {
        capture_enter_angle_rad: SHADOW_CAPTURE_ENTER_ANGLE_RAD,
        capture_enter_rate_rad_s: SHADOW_CAPTURE_ENTER_RATE_RAD_S,
        balance_enter_angle_rad: SHADOW_BALANCE_ENTER_ANGLE_RAD,
        balance_enter_rate_rad_s: SHADOW_BALANCE_ENTER_RATE_RAD_S,
        balance_exit_angle_rad: SHADOW_BALANCE_EXIT_ANGLE_RAD,
        balance_exit_rate_rad_s: SHADOW_BALANCE_EXIT_RATE_RAD_S,
        capture_exit_angle_rad: SHADOW_CAPTURE_EXIT_ANGLE_RAD,
        capture_exit_rate_rad_s: SHADOW_CAPTURE_EXIT_RATE_RAD_S,
        settle_cycles: SHADOW_CAPTURE_SETTLE_CYCLES,
    })
    .unwrap();
    let controller = HybridController::new(swing_up_controller, balance_controller, capture_policy);

    let actuator_model = ArmActuatorModel::new(
        ArmActuatorParameters::new(
            SHADOW_ACTUATOR_TORQUE_PER_EFFECTIVE_COMMAND_NM,
            SHADOW_ACTUATOR_COMMAND_DEADZONE,
        )
        .unwrap(),
    )
    .unwrap();
    let mut runtime = ControlRuntime::new(
        PendingObservationSource::new(),
        estimator_config,
        RuntimeLimits::observe_only(),
        controller,
        actuator_model,
    );

    // RuntimeAuthority remains disarmed and RuntimeState remains Ready. Hybrid
    // regime selection and authority evaluation execute on every computed cycle,
    // but this target cannot produce physical closed-loop output.
    let timing_limits = SensorTimingLimits::new(
        SENSOR_EXPECTED_PERIOD_US,
        SENSOR_LATE_AFTER_US,
        SENSOR_TIMEOUT_AFTER_US,
    )
    .unwrap();
    let mut timing_monitor = SensorTimingMonitor::new(timing_limits, 0);
    let mut watchdog = ControlWatchdog::new(CONTROL_WATCHDOG_TIMEOUT_US).unwrap();

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

        let timing = timing_monitor.on_event(captured_at.0);
        let watchdog_health = watchdog.health(captured_at.0);
        publish_runtime_health(timing, watchdog_health);

        if let Ok(measurement) = adapter.measurement(raw) {
            runtime.source_mut().submit(RuntimeObservation {
                measurement,
                sensor_valid: true,
                sample_age_us: 0,
                timing,
                watchdog: watchdog_health,
            });

            match runtime.step() {
                Ok(cycle) => {
                    publish_cycle(cycle);
                    publish_regime(runtime.controller().regime());
                    watchdog.kick(captured_at.0);
                }
                Err(_) => publish_cycle_error(),
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

fn publish_cycle(cycle: ControlCycle) {
    match cycle {
        ControlCycle::Primed => {
            SHADOW_CYCLE.store(CYCLE_PRIMED, Ordering::Relaxed);
            SHADOW_QUALIFICATION_REASONS.store(0, Ordering::Relaxed);
            clear_computed_snapshot();
        }
        ControlCycle::Rejected { qualification } => {
            SHADOW_CYCLE.store(CYCLE_REJECTED, Ordering::Relaxed);
            SHADOW_QUALIFICATION_REASONS
                .store(qualification.reasons.bits() as u32, Ordering::Relaxed);
            clear_computed_snapshot();
        }
        ControlCycle::Computed {
            state,
            demand,
            bounded_command,
            authority,
            authorized,
        } => {
            SHADOW_CYCLE.store(CYCLE_COMPUTED, Ordering::Relaxed);
            SHADOW_QUALIFICATION_REASONS.store(0, Ordering::Relaxed);
            publish_state(state);
            SHADOW_DEMAND_TORQUE_UNM.store(scale_micro(demand.arm_torque.0), Ordering::Relaxed);
            SHADOW_BOUNDED_COMMAND_PPM.store(
                scale_micro(bounded_command.command.get()),
                Ordering::Relaxed,
            );
            SHADOW_PREDICTED_TORQUE_UNM.store(
                scale_micro(bounded_command.predicted_arm_torque.0),
                Ordering::Relaxed,
            );
            SHADOW_ACTUATOR_SATURATED.store(bounded_command.saturated as u32, Ordering::Relaxed);
            SHADOW_AUTHORITY_REASONS.store(authority.reasons.bits() as u32, Ordering::Relaxed);
            SHADOW_AUTHORIZED.store(authorized.is_some() as u32, Ordering::Relaxed);
        }
    }
}

fn publish_cycle_error() {
    SHADOW_CYCLE.store(CYCLE_ERROR, Ordering::Relaxed);
    SHADOW_QUALIFICATION_REASONS.store(0, Ordering::Relaxed);
    clear_computed_snapshot();
}

fn clear_computed_snapshot() {
    SHADOW_DEMAND_TORQUE_UNM.store(0, Ordering::Relaxed);
    SHADOW_BOUNDED_COMMAND_PPM.store(0, Ordering::Relaxed);
    SHADOW_PREDICTED_TORQUE_UNM.store(0, Ordering::Relaxed);
    SHADOW_ACTUATOR_SATURATED.store(0, Ordering::Relaxed);
    SHADOW_AUTHORITY_REASONS.store(0, Ordering::Relaxed);
    SHADOW_AUTHORIZED.store(0, Ordering::Relaxed);
}

fn publish_state(state: EstimatedState) {
    SHADOW_THETA_MRAD.store(scale_milli(state.theta.0), Ordering::Relaxed);
    SHADOW_THETA_DOT_MRAD_S.store(scale_milli(state.theta_dot.0), Ordering::Relaxed);
    SHADOW_PHI_MRAD.store(scale_milli(state.phi.0), Ordering::Relaxed);
    SHADOW_PHI_DOT_MRAD_S.store(scale_milli(state.phi_dot.0), Ordering::Relaxed);
}

fn publish_regime(regime: ControlRegime) {
    let code = match regime {
        ControlRegime::SwingUp => 0,
        ControlRegime::Capture => 1,
        ControlRegime::Balance => 2,
    };
    SHADOW_CONTROL_REGIME.store(code, Ordering::Relaxed);
}

fn publish_runtime_health(timing: SensorTimingHealth, watchdog: WatchdogHealth) {
    SHADOW_SENSOR_TIMING_HEALTH.store(sensor_timing_code(timing), Ordering::Relaxed);
    SHADOW_WATCHDOG_HEALTH.store(watchdog_code(watchdog), Ordering::Relaxed);
}

const fn sensor_timing_code(health: SensorTimingHealth) -> u32 {
    match health {
        SensorTimingHealth::Startup => 0,
        SensorTimingHealth::Healthy => 1,
        SensorTimingHealth::Late => 2,
        SensorTimingHealth::Timeout => 3,
    }
}

const fn watchdog_code(health: WatchdogHealth) -> u32 {
    match health {
        WatchdogHealth::Disarmed => 0,
        WatchdogHealth::Healthy => 1,
        WatchdogHealth::Expired => 2,
    }
}

fn scale_milli(value: f32) -> i32 {
    scale(value, 1_000.0)
}

fn scale_micro(value: f32) -> i32 {
    scale(value, 1_000_000.0)
}

fn scale(value: f32, factor: f32) -> i32 {
    let scaled = value * factor;
    if scaled >= i32::MAX as f32 {
        i32::MAX
    } else if scaled <= i32::MIN as f32 {
        i32::MIN
    } else {
        scaled as i32
    }
}
