#![no_std]
#![no_main]
#![deny(unsafe_code)]

use core::f32::consts::PI;

use cortex_m::asm;
use cortex_m_rt::entry;
use panic_halt as _;
use rip_actuator_model::{ArmActuatorModel, ArmActuatorParameters};
use rip_arm_encoder_sensor::EncoderCounterAccumulator;
use rip_assembly_forest_d1_reference::{
    telemetry_period_ticks, ui_period_ticks, OLED_BACKGROUND_FLUSH_BYTES, TELEMETRY_DEFAULT_ENABLED,
};
use rip_board_forest_s1_d1::{
    CONTROL_TICK_HZ, HSE_MHZ, MOTOR_PWM_HZ, OLED_SOFTWARE_SPI_HALF_PERIOD_PADDING, SYSTEM_CLOCK_HZ,
    SYSTEM_CLOCK_MHZ, UART_BAUD,
};
use rip_control_runtime::{ControlRuntime, RuntimeObservation, RuntimeObservationSource};
use rip_estimator_input_adapter::EstimatorInputAdapter;
use rip_hybrid_control::{
    CapturePolicy, CapturePolicyConfig, EnergySwingUpConfig, EnergySwingUpController,
    HybridController,
};
use rip_measurement_model::{EncoderScale, PendulumCalibration};
use rip_oled_ui::{OledIo, OledRenderer, Ssd1315};
use rip_pendulum_adc_sensor::PendulumAdcSensor;
use rip_plant_observation::{MeasurementQuality, RawObservation};
use rip_robot_domain::TimestampUs;
use rip_runtime_observation_record::{
    publish_cycle, publish_cycle_error, publish_raw, publish_regime, publish_runtime_health,
    snapshot as runtime_record_snapshot,
};
use rip_runtime_state::{ControlWatchdog, RuntimeLimits, SensorTimingLimits, SensorTimingMonitor};
use rip_software_spi::{OutputLine, SoftwareSpi};
use rip_state_estimator::EstimatorConfig;
use rip_state_feedback::{LqrController, QNET_REFERENCE_TORQUE_GAINS};
use rip_status_view::{
    KeyService, LocalUi, StatusView, KEY_MINUS_MASK, KEY_M_MASK, KEY_PLUS_MASK, KEY_USER_MASK,
    KEY_X_MASK,
};
use rip_telemetry::{
    TelemetryPublisher, TelemetrySnapshot, TelemetryTransport, TelemetryTxOutcome,
    TELEMETRY_PACKET_LEN,
};
use rip_timing_evidence::{snapshot as timing_evidence_snapshot, RuntimeTimingCharacterizer};
use stm32f1xx_hal::{
    adc, pac,
    prelude::*,
    rcc,
    serial::{Config as SerialConfig, Tx1},
    time::{Instant, MonoTimer},
    timer::{pwm_input::QeiOptions, Tim3NoRemap, Timer},
    watchdog::IndependentWatchdog,
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
const HARDWARE_WATCHDOG_TIMEOUT_MS: u32 = 100;
const UART_BACKGROUND_SERVICE_BYTES: usize = 8;

// Reference-backed live-shadow controller parameters from the QNET RIP model
// in Abdullah et al. (2021). These are not Forest D1 specimen calibration.
const SHADOW_PENDULUM_MASS_KG: f32 = 0.04;
const SHADOW_PENDULUM_COM_LENGTH_M: f32 = 0.129;
const SHADOW_PENDULUM_INERTIA_KG_M2: f32 = 0.0001;
const SHADOW_TARGET_ENERGY_J: f32 = 0.025;
const SHADOW_ENERGY_TORQUE_GAIN: f32 = 0.175;
const SHADOW_MAX_ABS_TORQUE_NM: f32 = 0.05;
const SHADOW_SWING_KICK_TORQUE_NM: f32 = 0.01;
const SHADOW_SWING_KICK_BELOW_RATE_RAD_S: f32 = 0.05;

const SHADOW_CAPTURE_ENTER_ANGLE_RAD: f32 = 20.0 * PI / 180.0;
const SHADOW_CAPTURE_ENTER_RATE_RAD_S: f32 = 3.0;
const SHADOW_BALANCE_ENTER_ANGLE_RAD: f32 = 8.0 * PI / 180.0;
const SHADOW_BALANCE_ENTER_RATE_RAD_S: f32 = 1.0;
const SHADOW_BALANCE_EXIT_ANGLE_RAD: f32 = 12.0 * PI / 180.0;
const SHADOW_BALANCE_EXIT_RATE_RAD_S: f32 = 2.0;
const SHADOW_CAPTURE_EXIT_ANGLE_RAD: f32 = 30.0 * PI / 180.0;
const SHADOW_CAPTURE_EXIT_RATE_RAD_S: f32 = 4.0;
const SHADOW_CAPTURE_SETTLE_CYCLES: u16 = 20;

const SHADOW_ACTUATOR_TORQUE_PER_EFFECTIVE_COMMAND_NM: f32 = 0.05;
const SHADOW_ACTUATOR_COMMAND_DEADZONE: f32 = 0.0;

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
        let total_ticks = u64::from(self.remainder_ticks) + u64::from(elapsed_ticks);
        let ticks_per_us = u64::from(self.ticks_per_us);
        self.elapsed_us = self.elapsed_us.wrapping_add(total_ticks / ticks_per_us);
        self.remainder_ticks = (total_ticks % ticks_per_us) as u32;
        TimestampUs(self.elapsed_us)
    }

    fn mark(&self) -> Instant {
        self.timer.now()
    }

    const fn ticks_per_us(&self) -> u32 {
        self.ticks_per_us
    }
}

struct UartTelemetryTransport {
    tx: Tx1,
    pending: [u8; TELEMETRY_PACKET_LEN],
    pending_len: usize,
    cursor: usize,
    tx_errors: u32,
}

impl UartTelemetryTransport {
    const fn new(tx: Tx1) -> Self {
        Self {
            tx,
            pending: [0; TELEMETRY_PACKET_LEN],
            pending_len: 0,
            cursor: 0,
            tx_errors: 0,
        }
    }

    fn service(&mut self, max_bytes: usize) {
        let mut serviced = 0;
        while self.cursor < self.pending_len && serviced < max_bytes {
            match self.tx.write_u8(self.pending[self.cursor]) {
                Ok(()) => {
                    self.cursor += 1;
                    serviced += 1;
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => {
                    self.tx_errors = self.tx_errors.wrapping_add(1);
                    self.cursor = self.pending_len;
                    break;
                }
            }
        }
        if self.cursor >= self.pending_len {
            self.pending_len = 0;
            self.cursor = 0;
        }
    }

    const fn tx_errors(&self) -> u32 {
        self.tx_errors
    }
}

impl TelemetryTransport for UartTelemetryTransport {
    type Error = ();

    fn try_send(&mut self, packet: &[u8]) -> Result<TelemetryTxOutcome, Self::Error> {
        if self.pending_len != 0 {
            return Ok(TelemetryTxOutcome::Busy);
        }
        if packet.len() != TELEMETRY_PACKET_LEN {
            return Err(());
        }
        self.pending.copy_from_slice(packet);
        self.pending_len = packet.len();
        self.cursor = 0;
        Ok(TelemetryTxOutcome::Sent)
    }
}

struct OledGpioIo<C, D, R, DC> {
    spi: SoftwareSpi<C, D>,
    reset: R,
    dc: DC,
}

impl<C, D, R, DC> OledGpioIo<C, D, R, DC>
where
    C: OutputLine,
    D: OutputLine,
    R: OutputLine,
    DC: OutputLine,
{
    fn new(clock: C, data: D, mut reset: R, mut dc: DC) -> Self {
        OutputLine::set_high(&mut reset);
        OutputLine::set_low(&mut dc);
        Self {
            spi: SoftwareSpi::new(clock, data, OLED_SOFTWARE_SPI_HALF_PERIOD_PADDING),
            reset,
            dc,
        }
    }
}

impl<C, D, R, DC> OledIo for OledGpioIo<C, D, R, DC>
where
    C: OutputLine,
    D: OutputLine,
    R: OutputLine,
    DC: OutputLine,
{
    fn reset(&mut self) {
        OutputLine::set_low(&mut self.reset);
        asm::delay(SYSTEM_CLOCK_HZ / 10);
        OutputLine::set_high(&mut self.reset);
        asm::delay(SYSTEM_CLOCK_HZ / 100);
    }

    fn write_command(&mut self, command: u8) {
        OutputLine::set_low(&mut self.dc);
        self.spi.write_byte(command);
    }

    fn write_data(&mut self, data: &[u8]) {
        OutputLine::set_high(&mut self.dc);
        self.spi.write(data);
    }
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.freeze(
        rcc::Config::hse(HSE_MHZ.MHz())
            .sysclk(SYSTEM_CLOCK_MHZ.MHz())
            .pclk1(36.MHz())
            .adcclk(12.MHz()),
        &mut flash.acr,
    );
    let mut afio = dp.AFIO.constrain(&mut rcc);

    let mut gpioa = dp.GPIOA.split(&mut rcc);
    let mut gpiob = dp.GPIOB.split(&mut rcc);
    let (pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    let mut pendulum_pin = gpioa.pa7.into_analog(&mut gpioa.crl);
    let mut adc1 = adc::Adc::new(dp.ADC1, &mut rcc);
    let qei = Timer::new(dp.TIM2, &mut rcc).qei((gpioa.pa0, gpioa.pa1), QeiOptions::default());

    // D2 is the installed rotary-arm channel. The target binds the concrete
    // pins only to establish hard safe-off; no runtime ActuationSink receives
    // ownership of these peripherals.
    let mut motor_in1 = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);
    let mut motor_in2 = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
    motor_in1.set_low();
    motor_in2.set_low();
    let motor_pwm_pin = gpiob.pb1.into_alternate_push_pull(&mut gpiob.crl);
    let mut motor_pwm = dp
        .TIM3
        .pwm_hz::<Tim3NoRemap, _, _>(motor_pwm_pin, &mut afio.mapr, MOTOR_PWM_HZ.Hz(), &mut rcc)
        .split();
    motor_pwm.set_duty(0);
    motor_pwm.enable();

    let uart_tx_pin = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let uart_rx_pin = gpioa.pa10;
    let serial = dp.USART1.serial(
        (uart_tx_pin, uart_rx_pin),
        SerialConfig::default().baudrate(UART_BAUD.bps()),
        &mut rcc,
    );
    let (uart_tx, _uart_rx) = serial.split();
    let mut telemetry = TelemetryPublisher::new(UartTelemetryTransport::new(uart_tx));

    let oled_clock = gpiob.pb5.into_push_pull_output(&mut gpiob.crl);
    let oled_data = pb4.into_push_pull_output(&mut gpiob.crl);
    let oled_reset = pb3.into_push_pull_output(&mut gpiob.crl);
    let oled_dc = pa15.into_push_pull_output(&mut gpioa.crh);
    let mut local_ui = LocalUi::new();
    let oled_io = OledGpioIo::new(oled_clock, oled_data, oled_reset, oled_dc);
    let mut oled = Ssd1315::new(oled_io, local_ui.contrast());
    oled.init();

    let key_m = gpioa.pa3.into_pull_up_input(&mut gpioa.crl);
    let key_x = gpioa.pa2.into_pull_up_input(&mut gpioa.crl);
    let key_plus = gpioa.pa11.into_pull_up_input(&mut gpioa.crh);
    let key_minus = gpioa.pa12.into_pull_up_input(&mut gpioa.crh);
    let key_user = gpioa.pa5.into_pull_up_input(&mut gpioa.crl);
    let mut key_service = KeyService::new(0, 0);

    let mut control_tick = Timer::new(dp.TIM1, &mut rcc).counter_hz();
    control_tick.start(CONTROL_TICK_HZ.Hz()).unwrap();

    let monotonic = MonoTimer::new(cp.DWT, cp.DCB, &rcc.clocks);
    let mut timebase = MicrosecondTimebase::new(monotonic);
    let mut timing_characterizer =
        RuntimeTimingCharacterizer::new(SENSOR_EXPECTED_PERIOD_US as u32);

    let mut hardware_watchdog = IndependentWatchdog::new(dp.IWDG);
    hardware_watchdog.stop_on_debug(&dp.DBGMCU, true);
    hardware_watchdog.start(HARDWARE_WATCHDOG_TIMEOUT_MS.millis());

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
    let balance_controller = LqrController::new(QNET_REFERENCE_TORQUE_GAINS).unwrap();
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
    let telemetry_period = telemetry_period_ticks(CONTROL_TICK_HZ);
    let ui_period = ui_period_ticks(CONTROL_TICK_HZ);
    let mut telemetry_phase = 0_u32;
    let mut ui_phase = 0_u32;
    let mut telemetry_enabled = TELEMETRY_DEFAULT_ENABLED;
    let mut sample_index = 0_u32;

    loop {
        // Exactly one fresh acquisition/control opportunity is admitted per
        // observed TIM1 update flag. Missed periods coalesce and are not replayed.
        while control_tick.wait().is_err() {}
        let cycle_started = timebase.mark();
        let tick_phase_us = control_tick.now().ticks();
        let admitted_at = timebase.now();
        timing_characterizer.observe_admission(admitted_at.0, tick_phase_us);

        let adc_raw: u16 = match adc1.read(&mut pendulum_pin) {
            Ok(value) => value,
            Err(_) => {
                timing_characterizer.record_adc_error();
                timing_characterizer.finish_cycle(cycle_started.elapsed(), timebase.ticks_per_us());
                hardware_watchdog.feed();
                continue;
            }
        };
        let encoder_counter = qei.count();
        let captured_at = timebase.now();
        let raw = RawObservation {
            sample_index,
            pendulum: PendulumAdcSensor::observation(adc_raw, captured_at, quality),
            arm_encoder: encoder_accumulator.observation(encoder_counter, captured_at, quality),
        };
        sample_index = sample_index.wrapping_add(1);
        publish_raw(raw);

        let timing = timing_monitor.on_event(captured_at.0);
        timing_characterizer.observe_supervisor_health(timing);
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
                Err(_) => {
                    timing_characterizer.record_runtime_error();
                    publish_cycle_error();
                }
            }
        }

        // Critical-path accounting ends before UART formatting/transmission and
        // before blocking software-SPI OLED service.
        timing_characterizer.finish_cycle(cycle_started.elapsed(), timebase.ticks_per_us());
        hardware_watchdog.feed();

        let mut raw_key_mask = 0_u32;
        if key_m.is_low() {
            raw_key_mask |= KEY_M_MASK;
        }
        if key_x.is_low() {
            raw_key_mask |= KEY_X_MASK;
        }
        if key_plus.is_low() {
            raw_key_mask |= KEY_PLUS_MASK;
        }
        if key_minus.is_low() {
            raw_key_mask |= KEY_MINUS_MASK;
        }
        if key_user.is_low() {
            raw_key_mask |= KEY_USER_MASK;
        }
        let key_events = key_service.update(raw_key_mask, sample_index);
        if key_events.pressed & KEY_M_MASK != 0 {
            local_ui.next_page();
        }
        if key_events.pressed & KEY_X_MASK != 0 {
            local_ui.previous_page();
        }
        if (key_events.pressed | key_events.repeat) & KEY_PLUS_MASK != 0 {
            oled.set_contrast(local_ui.increase_contrast());
        }
        if (key_events.pressed | key_events.repeat) & KEY_MINUS_MASK != 0 {
            oled.set_contrast(local_ui.decrease_contrast());
        }
        if key_events.pressed & KEY_USER_MASK != 0 {
            telemetry_enabled = !telemetry_enabled;
        }

        telemetry_phase = telemetry_phase.saturating_add(1);
        if telemetry_phase >= telemetry_period {
            telemetry_phase = 0;
            if telemetry_enabled {
                let snapshot = TelemetrySnapshot::from_records(
                    runtime_record_snapshot(),
                    timing_evidence_snapshot(),
                );
                let _ = telemetry.publish_latest(snapshot);
            }
        }

        ui_phase = ui_phase.saturating_add(1);
        if ui_phase >= ui_period && oled.is_idle() {
            ui_phase = 0;
            let view = StatusView::from_records(
                runtime_record_snapshot(),
                timing_evidence_snapshot(),
                telemetry_enabled,
                false,
            );
            let frame = OledRenderer::render(view, local_ui.page());
            oled.write_frame(&frame);
        }

        // Background transports are bounded and never replay stale UI/telemetry.
        telemetry
            .transport_mut()
            .service(UART_BACKGROUND_SERVICE_BYTES);
        oled.service(OLED_BACKGROUND_FLUSH_BYTES);

        let _ = telemetry.transport().tx_errors();
    }
}
