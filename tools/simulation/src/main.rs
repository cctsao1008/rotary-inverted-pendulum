use std::env;
use std::f32::consts::PI;
use std::process;

use rip_actuator_model::{ArmActuatorModel, ArmActuatorParameters};
use rip_control_runtime::{
    ControlCycle, ControlRuntime, RuntimeObservation, RuntimeObservationSource,
};
use rip_dynamics_model::{FurutaParameters, FurutaPlant, FurutaState};
use rip_estimator_input_adapter::EstimatorInputAdapter;
use rip_hybrid_control::{
    CapturePolicy, CapturePolicyConfig, ControlRegime, EnergySwingUpConfig,
    EnergySwingUpController, HybridController,
};
use rip_measurement_model::{EncoderScale, PendulumCalibration};
use rip_plant_observation::{
    MeasurementQuality, RawArmEncoderObservation, RawObservation, RawPendulumObservation,
};
use rip_robot_domain::{EstimatedState, TimestampUs, TorqueNm};
use rip_runtime_state::{
    ControlWatchdog, RuntimeLimits, RuntimeState, SensorTimingLimits, SensorTimingMonitor,
    WatchdogHealth,
};
use rip_state_estimator::EstimatorConfig;
use rip_state_feedback::LqrController;

const CONTROL_PERIOD_US: u64 = 1_000;
const DEFAULT_PLANT_STEP_US: u64 = 50;
const SENSOR_LATE_AFTER_US: u64 = 5_000;
const SENSOR_TIMEOUT_AFTER_US: u64 = 20_000;
const CONTROL_WATCHDOG_TIMEOUT_US: u64 = 20_000;

const PENDULUM_UPRIGHT_ADC: u16 = 2_928;
const PENDULUM_RADIANS_PER_COUNT: f32 = 2.0 * PI / 4_096.0;
const PENDULUM_DIRECTION: i8 = 1;
const ARM_ENCODER_COUNTS_PER_REVOLUTION: f32 = 1_040.0;
const ARM_ENCODER_DIRECTION: i8 = 1;

const ESTIMATOR_MAX_GAP_US: u64 = 20_000;
const ESTIMATOR_RATE_FILTER_ALPHA: f32 = 1.0;

const SHADOW_LQR_TORQUE_GAINS: [f32; 4] = [0.183_55, 0.015_85, 0.011_20, 0.007_66];
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Scenario {
    SwingUp,
    Balance,
}

impl Scenario {
    fn parse(value: &str) -> Option<Self> {
        match value {
            "swingup" => Some(Self::SwingUp),
            "balance" => Some(Self::Balance),
            _ => None,
        }
    }

    const fn initial_state(self) -> FurutaState {
        match self {
            Self::SwingUp => FurutaState {
                theta: PI - 0.05,
                theta_dot: 0.0,
                phi: 0.0,
                phi_dot: 0.0,
            },
            Self::Balance => FurutaState {
                theta: 5.0 * PI / 180.0,
                theta_dot: 0.0,
                phi: 0.0,
                phi_dot: 0.0,
            },
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct SimulationConfig {
    scenario: Scenario,
    duration_s: f32,
    plant_step_us: u64,
    csv_stride: u32,
    drop_every: Option<u32>,
    emit_csv: bool,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            scenario: Scenario::SwingUp,
            duration_s: 5.0,
            plant_step_us: DEFAULT_PLANT_STEP_US,
            csv_stride: 10,
            drop_every: None,
            emit_csv: true,
        }
    }
}

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

#[derive(Clone, Copy)]
struct SyntheticSensors {
    pendulum_upright_adc: i32,
    pendulum_radians_per_count: f32,
    pendulum_direction: f32,
    arm_radians_per_count: f32,
    arm_direction: f32,
}

impl SyntheticSensors {
    fn project_nominal() -> Self {
        Self {
            pendulum_upright_adc: i32::from(PENDULUM_UPRIGHT_ADC),
            pendulum_radians_per_count: PENDULUM_RADIANS_PER_COUNT,
            pendulum_direction: f32::from(PENDULUM_DIRECTION),
            arm_radians_per_count: 2.0 * PI / ARM_ENCODER_COUNTS_PER_REVOLUTION,
            arm_direction: f32::from(ARM_ENCODER_DIRECTION),
        }
    }

    fn observe(self, state: FurutaState, sample_index: u32, captured_at: TimestampUs) -> RawObservation {
        let theta = wrap_pi(state.theta);
        let pendulum_offset = (theta / (self.pendulum_radians_per_count * self.pendulum_direction))
            .round() as i32;
        let adc_raw = (self.pendulum_upright_adc + pendulum_offset).rem_euclid(4_096) as u16;

        let encoder_count =
            (state.phi / (self.arm_radians_per_count * self.arm_direction)).round() as i32;
        let quality = MeasurementQuality::AVAILABLE
            | MeasurementQuality::IO_OK
            | MeasurementQuality::TIMING_VALID;

        RawObservation {
            sample_index,
            pendulum: RawPendulumObservation {
                captured_at,
                adc_raw,
                quality,
            },
            arm_encoder: RawArmEncoderObservation {
                captured_at,
                accumulated_count: encoder_count,
                quality,
            },
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct CycleSnapshot {
    estimate: Option<EstimatedState>,
    regime: Option<ControlRegime>,
    demand_torque_nm: f32,
    applied_torque_nm: f32,
    saturated: bool,
    authorized: bool,
}

#[derive(Clone, Copy, Debug, Default)]
struct SimulationSummary {
    scheduled_ticks: u32,
    admitted_samples: u32,
    dropped_ticks: u32,
    computed_cycles: u32,
    authorized_cycles: u32,
    denied_cycles: u32,
    saturated_cycles: u32,
    regime_transitions: u32,
    first_capture_us: Option<u64>,
    first_balance_us: Option<u64>,
    max_abs_theta_rad: f32,
    max_abs_phi_rad: f32,
    max_abs_torque_nm: f32,
    final_state: FurutaState,
}

fn main() {
    let config = match parse_args(env::args().skip(1)) {
        Ok(config) => config,
        Err(message) => {
            eprintln!("{message}");
            print_help();
            process::exit(2);
        }
    };

    match run_simulation(config) {
        Ok(summary) => print_summary(summary),
        Err(message) => {
            eprintln!("simulation failed: {message}");
            process::exit(1);
        }
    }
}

fn run_simulation(config: SimulationConfig) -> Result<SimulationSummary, String> {
    validate_config(config)?;

    let mut plant = FurutaPlant::new(qnet_reference_plant(), config.scenario.initial_state())
        .map_err(|error| format!("plant configuration: {error:?}"))?;
    let sensors = SyntheticSensors::project_nominal();
    let adapter = EstimatorInputAdapter::new(
        PendulumCalibration::new(
            PENDULUM_UPRIGHT_ADC,
            PENDULUM_RADIANS_PER_COUNT,
            PENDULUM_DIRECTION,
        )
        .map_err(|error| format!("pendulum calibration: {error:?}"))?,
        EncoderScale::new(ARM_ENCODER_COUNTS_PER_REVOLUTION, ARM_ENCODER_DIRECTION)
            .map_err(|error| format!("encoder calibration: {error:?}"))?,
    );

    let controller = hybrid_controller()?;
    let actuator_model = ArmActuatorModel::new(
        ArmActuatorParameters::new(
            SHADOW_ACTUATOR_TORQUE_PER_EFFECTIVE_COMMAND_NM,
            SHADOW_ACTUATOR_COMMAND_DEADZONE,
        )
        .ok_or_else(|| "invalid actuator model parameters".to_owned())?,
    )
    .ok_or_else(|| "invalid actuator model".to_owned())?;

    let mut runtime = ControlRuntime::new(
        PendingObservationSource::new(),
        EstimatorConfig {
            max_gap_us: ESTIMATOR_MAX_GAP_US,
            rate_filter_alpha: ESTIMATOR_RATE_FILTER_ALPHA,
        },
        RuntimeLimits::observe_only(),
        controller,
        actuator_model,
    );
    runtime
        .authority_mut()
        .enter_closed_loop()
        .map_err(|error| format!("simulation authority setup: {error:?}"))?;

    let timing_limits = SensorTimingLimits::new(
        CONTROL_PERIOD_US,
        SENSOR_LATE_AFTER_US,
        SENSOR_TIMEOUT_AFTER_US,
    )
    .ok_or_else(|| "invalid timing limits".to_owned())?;
    let mut timing_monitor = SensorTimingMonitor::new(timing_limits, 0);
    let mut watchdog = ControlWatchdog::new(CONTROL_WATCHDOG_TIMEOUT_US)
        .ok_or_else(|| "invalid watchdog timeout".to_owned())?;

    let scheduled_ticks = (config.duration_s * 1_000_000.0 / CONTROL_PERIOD_US as f32).round() as u32;
    let substeps = CONTROL_PERIOD_US / config.plant_step_us;
    let plant_dt_s = config.plant_step_us as f32 * 1.0e-6;
    let mut applied_torque = TorqueNm(0.0);
    let mut sample_index = 0_u32;
    let mut last_regime = runtime.controller().regime();
    let mut summary = SimulationSummary {
        scheduled_ticks,
        final_state: plant.state(),
        ..SimulationSummary::default()
    };

    if config.emit_csv {
        println!("time_s,true_theta_rad,true_theta_dot_rad_s,true_phi_rad,true_phi_dot_rad_s,est_theta_rad,est_theta_dot_rad_s,est_phi_rad,est_phi_dot_rad_s,regime,demand_torque_nm,applied_torque_nm,saturated,authorized");
    }

    for tick in 0..scheduled_ticks {
        let timestamp_us = u64::from(tick + 1) * CONTROL_PERIOD_US;
        let drop_tick = config
            .drop_every
            .is_some_and(|period| period != 0 && (tick + 1) % period == 0);

        let snapshot = if drop_tick {
            summary.dropped_ticks = summary.dropped_ticks.saturating_add(1);
            CycleSnapshot {
                regime: Some(last_regime),
                applied_torque_nm: applied_torque.0,
                ..CycleSnapshot::default()
            }
        } else {
            let raw = sensors.observe(plant.state(), sample_index, TimestampUs(timestamp_us));
            sample_index = sample_index.wrapping_add(1);
            summary.admitted_samples = summary.admitted_samples.saturating_add(1);
            let measurement = adapter
                .measurement(raw)
                .map_err(|error| format!("sensor adapter at {timestamp_us} us: {error:?}"))?;
            let timing = timing_monitor.on_event(timestamp_us);
            let watchdog_health = watchdog.health(timestamp_us);

            let active_regime = runtime.controller().regime();
            runtime.set_runtime_state(RuntimeState::Active(active_regime));
            runtime.source_mut().submit(RuntimeObservation {
                measurement,
                sensor_valid: true,
                sample_age_us: 0,
                timing,
                watchdog: watchdog_health,
            });

            let cycle = runtime
                .step()
                .map_err(|error| format!("control runtime at {timestamp_us} us: {error:?}"))?;
            watchdog.kick(timestamp_us);

            match cycle {
                ControlCycle::Primed => {
                    applied_torque = TorqueNm(0.0);
                    CycleSnapshot {
                        regime: Some(runtime.controller().regime()),
                        ..CycleSnapshot::default()
                    }
                }
                ControlCycle::Rejected { .. } => {
                    summary.denied_cycles = summary.denied_cycles.saturating_add(1);
                    applied_torque = TorqueNm(0.0);
                    CycleSnapshot {
                        regime: Some(runtime.controller().regime()),
                        ..CycleSnapshot::default()
                    }
                }
                ControlCycle::Computed {
                    state,
                    demand,
                    bounded_command,
                    authorized,
                    ..
                } => {
                    summary.computed_cycles = summary.computed_cycles.saturating_add(1);
                    if bounded_command.saturated {
                        summary.saturated_cycles = summary.saturated_cycles.saturating_add(1);
                    }
                    let authorized = authorized.map(|proof| proof.command());
                    if let Some(command) = authorized {
                        summary.authorized_cycles = summary.authorized_cycles.saturating_add(1);
                        applied_torque = command.predicted_arm_torque;
                    } else {
                        summary.denied_cycles = summary.denied_cycles.saturating_add(1);
                        applied_torque = TorqueNm(0.0);
                    }
                    CycleSnapshot {
                        estimate: Some(state),
                        regime: Some(runtime.controller().regime()),
                        demand_torque_nm: demand.arm_torque.0,
                        applied_torque_nm: applied_torque.0,
                        saturated: bounded_command.saturated,
                        authorized: authorized.is_some(),
                    }
                }
            }
        };

        if let Some(regime) = snapshot.regime {
            if regime != last_regime {
                summary.regime_transitions = summary.regime_transitions.saturating_add(1);
                if regime == ControlRegime::Capture && summary.first_capture_us.is_none() {
                    summary.first_capture_us = Some(timestamp_us);
                }
                if regime == ControlRegime::Balance && summary.first_balance_us.is_none() {
                    summary.first_balance_us = Some(timestamp_us);
                }
                last_regime = regime;
            }
        }

        summary.max_abs_torque_nm = summary.max_abs_torque_nm.max(applied_torque.0.abs());
        integrate_control_period(&mut plant, applied_torque, substeps, plant_dt_s)?;
        let true_state = plant.state();
        summary.max_abs_theta_rad = summary.max_abs_theta_rad.max(wrap_pi(true_state.theta).abs());
        summary.max_abs_phi_rad = summary.max_abs_phi_rad.max(true_state.phi.abs());
        summary.final_state = true_state;

        if config.emit_csv && tick % config.csv_stride == 0 {
            print_csv_row(timestamp_us, true_state, snapshot);
        }
    }

    Ok(summary)
}

fn integrate_control_period(
    plant: &mut FurutaPlant,
    torque: TorqueNm,
    substeps: u64,
    dt_s: f32,
) -> Result<(), String> {
    for _ in 0..substeps {
        plant
            .step_rk4(dt_s, torque)
            .map_err(|error| format!("plant integration: {error:?}"))?;
    }
    Ok(())
}

fn qnet_reference_plant() -> FurutaParameters {
    // Abdullah et al. (2021), Table 2 and Eq. (17). Jeq is used exactly as
    // reported in the source. The arm damping term is Kt*Ke/Rm.
    let kt = 0.042;
    let ke = 0.042;
    let rm = 8.4;
    FurutaParameters {
        pendulum_mass_kg: 0.04,
        arm_length_m: 0.085,
        pendulum_com_length_m: 0.129,
        arm_inertia_kg_m2: 0.000_005_7,
        pendulum_inertia_kg_m2: 0.0001,
        gravity_m_s2: 9.81,
        arm_viscous_damping_nm_per_rad_s: kt * ke / rm,
        pendulum_viscous_damping_nm_per_rad_s: 0.0,
    }
}

fn hybrid_controller() -> Result<HybridController, String> {
    let balance = LqrController::new(SHADOW_LQR_TORQUE_GAINS)
        .map_err(|error| format!("LQR setup: {error:?}"))?;
    let swing = EnergySwingUpController::new(EnergySwingUpConfig {
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
    .map_err(|error| format!("swing-up setup: {error:?}"))?;
    let capture = CapturePolicy::new(CapturePolicyConfig {
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
    .map_err(|error| format!("capture setup: {error:?}"))?;
    Ok(HybridController::new(swing, balance, capture))
}

fn print_csv_row(timestamp_us: u64, true_state: FurutaState, snapshot: CycleSnapshot) {
    let (est_theta, est_theta_dot, est_phi, est_phi_dot) = snapshot
        .estimate
        .map(|state| (state.theta.0, state.theta_dot.0, state.phi.0, state.phi_dot.0))
        .unwrap_or((f32::NAN, f32::NAN, f32::NAN, f32::NAN));
    println!(
        "{:.6},{:.7},{:.7},{:.7},{:.7},{:.7},{:.7},{:.7},{:.7},{},{:.7},{:.7},{},{}",
        timestamp_us as f64 * 1.0e-6,
        wrap_pi(true_state.theta),
        true_state.theta_dot,
        true_state.phi,
        true_state.phi_dot,
        est_theta,
        est_theta_dot,
        est_phi,
        est_phi_dot,
        snapshot.regime.map(regime_name).unwrap_or("none"),
        snapshot.demand_torque_nm,
        snapshot.applied_torque_nm,
        u8::from(snapshot.saturated),
        u8::from(snapshot.authorized),
    );
}

fn print_summary(summary: SimulationSummary) {
    eprintln!("SIL summary");
    eprintln!("scheduled_ticks........ {}", summary.scheduled_ticks);
    eprintln!("admitted_samples....... {}", summary.admitted_samples);
    eprintln!("dropped_ticks.......... {}", summary.dropped_ticks);
    eprintln!("computed_cycles........ {}", summary.computed_cycles);
    eprintln!("authorized_cycles...... {}", summary.authorized_cycles);
    eprintln!("denied_cycles.......... {}", summary.denied_cycles);
    eprintln!("saturated_cycles....... {}", summary.saturated_cycles);
    eprintln!("regime_transitions..... {}", summary.regime_transitions);
    eprintln!("first_capture_us........ {:?}", summary.first_capture_us);
    eprintln!("first_balance_us........ {:?}", summary.first_balance_us);
    eprintln!("max_abs_theta_rad...... {:.6}", summary.max_abs_theta_rad);
    eprintln!("max_abs_phi_rad........ {:.6}", summary.max_abs_phi_rad);
    eprintln!("max_abs_torque_nm...... {:.6}", summary.max_abs_torque_nm);
    eprintln!(
        "final_state............ theta={:.6}, theta_dot={:.6}, phi={:.6}, phi_dot={:.6}",
        wrap_pi(summary.final_state.theta),
        summary.final_state.theta_dot,
        summary.final_state.phi,
        summary.final_state.phi_dot,
    );
}

fn regime_name(regime: ControlRegime) -> &'static str {
    match regime {
        ControlRegime::SwingUp => "swingup",
        ControlRegime::Capture => "capture",
        ControlRegime::Balance => "balance",
    }
}

fn validate_config(config: SimulationConfig) -> Result<(), String> {
    if !config.duration_s.is_finite() || config.duration_s <= 0.0 {
        return Err("--duration-s must be positive".to_owned());
    }
    if config.plant_step_us == 0 || CONTROL_PERIOD_US % config.plant_step_us != 0 {
        return Err("--plant-step-us must be a non-zero divisor of 1000".to_owned());
    }
    if config.csv_stride == 0 {
        return Err("--csv-stride must be non-zero".to_owned());
    }
    if config.drop_every == Some(0) {
        return Err("--drop-every must be non-zero".to_owned());
    }
    Ok(())
}

fn parse_args<I>(mut args: I) -> Result<SimulationConfig, String>
where
    I: Iterator<Item = String>,
{
    let mut config = SimulationConfig::default();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--scenario" => {
                let value = args.next().ok_or_else(|| "missing --scenario value".to_owned())?;
                config.scenario = Scenario::parse(&value)
                    .ok_or_else(|| "--scenario must be swingup or balance".to_owned())?;
            }
            "--duration-s" => {
                config.duration_s = parse_value(&mut args, "--duration-s")?;
            }
            "--plant-step-us" => {
                config.plant_step_us = parse_value(&mut args, "--plant-step-us")?;
            }
            "--csv-stride" => {
                config.csv_stride = parse_value(&mut args, "--csv-stride")?;
            }
            "--drop-every" => {
                config.drop_every = Some(parse_value(&mut args, "--drop-every")?);
            }
            "--no-csv" => config.emit_csv = false,
            "-h" | "--help" => {
                print_help();
                process::exit(0);
            }
            _ => return Err(format!("unknown argument: {arg}")),
        }
    }
    validate_config(config)?;
    Ok(config)
}

fn parse_value<T, I>(args: &mut I, name: &str) -> Result<T, String>
where
    T: std::str::FromStr,
    I: Iterator<Item = String>,
{
    let raw = args.next().ok_or_else(|| format!("missing {name} value"))?;
    raw.parse().map_err(|_| format!("invalid {name} value: {raw}"))
}

fn print_help() {
    eprintln!("rip-simulation - end-to-end rotary inverted pendulum SIL");
    eprintln!("usage: cargo run -p rip-simulation --release -- [options]");
    eprintln!("  --scenario swingup|balance   initial condition (default: swingup)");
    eprintln!("  --duration-s SECONDS         simulated duration (default: 5)");
    eprintln!("  --plant-step-us US           RK4 step; divisor of 1000 (default: 50)");
    eprintln!("  --csv-stride N               emit every N control ticks (default: 10)");
    eprintln!("  --drop-every N               omit every Nth 1 kHz observation opportunity");
    eprintln!("  --no-csv                     suppress CSV rows and print summary only");
}

fn wrap_pi(mut value: f32) -> f32 {
    let tau = 2.0 * PI;
    while value > PI {
        value -= tau;
    }
    while value < -PI {
        value += tau;
    }
    value
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn synthetic_sensors_round_trip_within_quantization() {
        let sensors = SyntheticSensors::project_nominal();
        let adapter = EstimatorInputAdapter::new(
            PendulumCalibration::new(
                PENDULUM_UPRIGHT_ADC,
                PENDULUM_RADIANS_PER_COUNT,
                PENDULUM_DIRECTION,
            )
            .unwrap(),
            EncoderScale::new(ARM_ENCODER_COUNTS_PER_REVOLUTION, ARM_ENCODER_DIRECTION).unwrap(),
        );
        let state = FurutaState {
            theta: 0.321,
            theta_dot: 0.0,
            phi: -1.234,
            phi_dot: 0.0,
        };
        let measurement = adapter
            .measurement(sensors.observe(state, 1, TimestampUs(1_000)))
            .unwrap();

        assert!((measurement.theta.0 - state.theta).abs() <= PENDULUM_RADIANS_PER_COUNT);
        let arm_quantum = 2.0 * PI / ARM_ENCODER_COUNTS_PER_REVOLUTION;
        assert!((measurement.phi.0 - state.phi).abs() <= arm_quantum);
    }

    #[test]
    fn short_balance_sil_exercises_authority_without_hardware() {
        let summary = run_simulation(SimulationConfig {
            scenario: Scenario::Balance,
            duration_s: 0.05,
            plant_step_us: 50,
            csv_stride: 10,
            drop_every: None,
            emit_csv: false,
        })
        .unwrap();

        assert_eq!(summary.scheduled_ticks, 50);
        assert_eq!(summary.dropped_ticks, 0);
        assert!(summary.computed_cycles > 0);
        assert!(summary.authorized_cycles > 0);
        assert!(summary.final_state.is_finite());
    }

    #[test]
    fn dropped_opportunities_are_not_replayed() {
        let summary = run_simulation(SimulationConfig {
            scenario: Scenario::Balance,
            duration_s: 0.02,
            plant_step_us: 50,
            csv_stride: 10,
            drop_every: Some(5),
            emit_csv: false,
        })
        .unwrap();

        assert_eq!(summary.scheduled_ticks, 20);
        assert_eq!(summary.dropped_ticks, 4);
        assert_eq!(summary.admitted_samples, 16);
        assert!(summary.computed_cycles <= summary.admitted_samples);
    }
}
