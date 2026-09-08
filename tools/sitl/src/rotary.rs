use std::cell::RefCell;
use std::error::Error;
use std::f32::consts::PI;
use std::io;
use std::rc::Rc;

use rip_actuation_interface::ActuationSink;
use rip_actuator_model::{ArmActuatorModel, ArmActuatorParameters};
use rip_control_runtime::{
    ControlCycle, ControlRuntime, RuntimeObservation, RuntimeObservationSource,
};
use rip_estimator_input_adapter::EstimatorInputAdapter;
use rip_hybrid_control::{
    CapturePolicy, CapturePolicyConfig, ControlRegime, EnergySwingUpConfig,
    EnergySwingUpController, HybridController,
};
use rip_measurement_model::{EncoderScale, PendulumCalibration};
use rip_plant_model::{FurutaParameters, FurutaPlant, FurutaState};
use rip_plant_observation::{
    MeasurementQuality, RawArmEncoderObservation, RawObservation, RawPendulumObservation,
};
use rip_robot_domain::{EstimatedState, TimestampUs, TorqueNm};
use rip_runtime_state::{
    ActuationAuthority, AuthorizedActuation, ControlWatchdog, RuntimeLimits, RuntimeState,
    SensorTimingLimits, SensorTimingMonitor,
};
use rip_state_estimator::EstimatorConfig;
use rip_state_feedback::{LqrController, QNET_REFERENCE_TORQUE_GAINS};
use rip_tb6612_actuation::{
    Tb6612BridgeMode, Tb6612ElectricalActuation, Tb6612FrameIo, Tb6612Mapper, Tb6612Output,
};
use serde_json::{json, Value};

use crate::parameters::{ReferenceAssemblyParameters, VirtualSensorParameters};
use crate::scenario::RotaryScenario;
use crate::scheduler::EventKind;
use crate::virtual_time::VirtualTime;
use crate::SitlSystem;

const ESTIMATOR_RATE_FILTER_ALPHA: f32 = 1.0;

const TARGET_ENERGY_J: f32 = 0.025;
const ENERGY_TORQUE_GAIN: f32 = 0.175;
const MAX_ABS_TORQUE_NM: f32 = 0.05;
const SWING_KICK_TORQUE_NM: f32 = 0.01;
const SWING_KICK_BELOW_RATE_RAD_S: f32 = 0.05;

const CAPTURE_ENTER_ANGLE_RAD: f32 = 20.0 * PI / 180.0;
const CAPTURE_ENTER_RATE_RAD_S: f32 = 3.0;
const BALANCE_ENTER_ANGLE_RAD: f32 = 8.0 * PI / 180.0;
const BALANCE_ENTER_RATE_RAD_S: f32 = 1.0;
const BALANCE_EXIT_ANGLE_RAD: f32 = 12.0 * PI / 180.0;
const BALANCE_EXIT_RATE_RAD_S: f32 = 2.0;
const CAPTURE_EXIT_ANGLE_RAD: f32 = 30.0 * PI / 180.0;
const CAPTURE_EXIT_RATE_RAD_S: f32 = 4.0;
const CAPTURE_SETTLE_CYCLES: u16 = 20;

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
    pendulum_adc_modulus: i32,
    arm_radians_per_count: f32,
    arm_direction: f32,
}

impl SyntheticSensors {
    fn new(parameters: &VirtualSensorParameters) -> Result<Self, Box<dyn Error>> {
        let pendulum_radians_per_count = parameters.pendulum_radians_per_count.value;
        let arm_counts_per_revolution = parameters.arm_encoder_counts_per_revolution.value;
        let pendulum_direction = f32::from(parameters.pendulum_direction.value);
        let arm_direction = f32::from(parameters.arm_encoder_direction.value);
        let pendulum_adc_modulus = i32::from(parameters.pendulum_adc_modulus.value);

        if !pendulum_radians_per_count.is_finite()
            || pendulum_radians_per_count <= 0.0
            || !arm_counts_per_revolution.is_finite()
            || arm_counts_per_revolution <= 0.0
            || !matches!(parameters.pendulum_direction.value, -1 | 1)
            || !matches!(parameters.arm_encoder_direction.value, -1 | 1)
            || pendulum_adc_modulus <= 0
        {
            return Err(boxed("invalid virtual sensor parameters"));
        }

        Ok(Self {
            pendulum_upright_adc: i32::from(parameters.pendulum_upright_adc.value),
            pendulum_radians_per_count,
            pendulum_direction,
            pendulum_adc_modulus,
            arm_radians_per_count: 2.0 * PI / arm_counts_per_revolution,
            arm_direction,
        })
    }

    fn observe(
        self,
        state: FurutaState,
        sample_index: u32,
        captured_at: TimestampUs,
    ) -> RawObservation {
        let theta = wrap_pi(state.theta);
        let pendulum_offset =
            (theta / (self.pendulum_radians_per_count * self.pendulum_direction)).round() as i32;
        let adc_raw = (self.pendulum_upright_adc + pendulum_offset)
            .rem_euclid(self.pendulum_adc_modulus) as u16;
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

#[derive(Clone, Copy, Debug)]
struct VirtualActuatorState {
    frame: Tb6612ElectricalActuation,
    applied_torque_nm: f32,
}

struct VirtualTb6612Io {
    state: Rc<RefCell<VirtualActuatorState>>,
    torque_per_duty_nm: f32,
    positive_drive_is_positive_arm_torque: bool,
}

impl Tb6612FrameIo for VirtualTb6612Io {
    type Error = io::Error;

    fn apply_frame(&mut self, frame: Tb6612ElectricalActuation) -> Result<(), Self::Error> {
        if !frame.is_valid() {
            return Err(io::Error::other("invalid TB6612 frame"));
        }

        let positive_sign = if self.positive_drive_is_positive_arm_torque {
            1.0
        } else {
            -1.0
        };
        let applied_torque_nm = match frame.mode() {
            Tb6612BridgeMode::Coast | Tb6612BridgeMode::Brake => 0.0,
            Tb6612BridgeMode::DrivePositive => {
                positive_sign * frame.duty_fraction() * self.torque_per_duty_nm
            }
            Tb6612BridgeMode::DriveNegative => {
                -positive_sign * frame.duty_fraction() * self.torque_per_duty_nm
            }
        };

        *self.state.borrow_mut() = VirtualActuatorState {
            frame,
            applied_torque_nm,
        };
        Ok(())
    }
}

#[derive(Debug, Default)]
struct RotaryMetrics {
    computed_cycles: u64,
    authorized_cycles: u64,
    denied_cycles: u64,
    saturated_cycles: u64,
    regime_transitions: u64,
    first_capture_us: Option<u64>,
    first_balance_us: Option<u64>,
    max_abs_theta_rad: f32,
    max_abs_phi_rad: f32,
    max_abs_torque_nm: f32,
}

pub struct RotarySitlSystem {
    plant: FurutaPlant,
    plant_step_us: u64,
    sensors: SyntheticSensors,
    adapter: EstimatorInputAdapter,
    runtime: ControlRuntime<PendingObservationSource, HybridController>,
    timing_monitor: SensorTimingMonitor,
    watchdog: ControlWatchdog,
    output: Tb6612Output<VirtualTb6612Io>,
    actuator_state: Rc<RefCell<VirtualActuatorState>>,
    pending_raw: Option<RawObservation>,
    pending_authorized: Option<AuthorizedActuation>,
    sample_index: u32,
    last_regime: ControlRegime,
    metrics: RotaryMetrics,
}

impl RotarySitlSystem {
    pub fn new(
        parameters: &ReferenceAssemblyParameters,
        scenario: RotaryScenario,
        sensor_period_us: u64,
        runtime_period_us: u64,
    ) -> Result<Self, Box<dyn Error>> {
        if sensor_period_us != runtime_period_us {
            return Err(boxed(
                "rotary full semantic-path SITL requires one runtime opportunity per fresh observation",
            ));
        }

        let p = &parameters.plant;
        let furuta_parameters = FurutaParameters {
            pendulum_mass_kg: p.pendulum_mass_kg.value,
            arm_length_m: p.arm_length_m.value,
            pendulum_com_length_m: p.pendulum_com_length_m.value,
            arm_inertia_kg_m2: p.arm_inertia_kg_m2.value,
            pendulum_inertia_kg_m2: p.pendulum_inertia_kg_m2.value,
            gravity_m_s2: p.gravity_m_s2.value,
            arm_viscous_damping_nm_per_rad_s: p.arm_viscous_damping_nm_per_rad_s.value,
            pendulum_viscous_damping_nm_per_rad_s: p.pendulum_viscous_damping_nm_per_rad_s.value,
        };
        let initial_state = FurutaState {
            theta: scenario.initial_theta_rad,
            theta_dot: scenario.initial_theta_dot_rad_s,
            phi: scenario.initial_phi_rad,
            phi_dot: scenario.initial_phi_dot_rad_s,
        };
        let plant = FurutaPlant::new(furuta_parameters, initial_state)
            .map_err(|error| boxed(format!("plant configuration: {error:?}")))?;
        let sensors = SyntheticSensors::new(&parameters.virtual_sensor)?;

        let m = &parameters.measurement_model;
        let adapter = EstimatorInputAdapter::new(
            PendulumCalibration::new(
                m.pendulum_upright_adc.value,
                m.pendulum_radians_per_count.value,
                m.pendulum_direction.value,
            )
            .map_err(|error| boxed(format!("pendulum calibration: {error:?}")))?,
            EncoderScale::new(
                m.arm_encoder_counts_per_revolution.value,
                m.arm_encoder_direction.value,
            )
            .map_err(|error| boxed(format!("encoder calibration: {error:?}")))?,
        );

        let controller = hybrid_controller(furuta_parameters)?;
        let last_regime = controller.regime();
        let a = &parameters.production_actuator_model;
        let actuator_model = ArmActuatorModel::new(
            ArmActuatorParameters::new(
                a.torque_per_effective_command_nm.value,
                a.command_deadzone.value,
            )
            .ok_or_else(|| boxed("invalid production actuator-model parameters"))?,
        )
        .ok_or_else(|| boxed("invalid production actuator model"))?;

        let max_gap_us = runtime_period_us
            .checked_mul(20)
            .ok_or_else(|| boxed("estimator gap configuration overflow"))?;
        let mut runtime = ControlRuntime::new(
            PendingObservationSource::new(),
            EstimatorConfig {
                max_gap_us,
                rate_filter_alpha: ESTIMATOR_RATE_FILTER_ALPHA,
            },
            RuntimeLimits::observe_only(),
            controller,
            actuator_model,
        );
        runtime
            .authority_mut()
            .enter_closed_loop()
            .map_err(|error| boxed(format!("runtime authority setup: {error:?}")))?;

        let late_after_us = sensor_period_us
            .checked_mul(5)
            .ok_or_else(|| boxed("sensor late threshold overflow"))?;
        let timeout_after_us = sensor_period_us
            .checked_mul(20)
            .ok_or_else(|| boxed("sensor timeout threshold overflow"))?;
        let timing_limits =
            SensorTimingLimits::new(sensor_period_us, late_after_us, timeout_after_us)
                .ok_or_else(|| boxed("invalid sensor timing limits"))?;
        let timing_monitor = SensorTimingMonitor::new(timing_limits, 0);
        let watchdog = ControlWatchdog::new(timeout_after_us)
            .ok_or_else(|| boxed("invalid control watchdog timeout"))?;

        let virtual_actuator = &parameters.virtual_physical_actuator;
        if !virtual_actuator.torque_per_duty_nm.value.is_finite()
            || virtual_actuator.torque_per_duty_nm.value <= 0.0
        {
            return Err(boxed("invalid virtual physical actuator parameters"));
        }
        let actuator_state = Rc::new(RefCell::new(VirtualActuatorState {
            frame: Tb6612ElectricalActuation::safe_off(),
            applied_torque_nm: 0.0,
        }));
        let io = VirtualTb6612Io {
            state: Rc::clone(&actuator_state),
            torque_per_duty_nm: virtual_actuator.torque_per_duty_nm.value,
            positive_drive_is_positive_arm_torque: virtual_actuator
                .positive_drive_is_positive_arm_torque
                .value,
        };
        let output = Tb6612Output::new(
            Tb6612Mapper::new(
                parameters
                    .firmware_actuator_mapping
                    .positive_command_is_positive_drive
                    .value,
            ),
            io,
        );

        Ok(Self {
            plant,
            plant_step_us: scenario.plant_step_us,
            sensors,
            adapter,
            runtime,
            timing_monitor,
            watchdog,
            output,
            actuator_state,
            pending_raw: None,
            pending_authorized: None,
            sample_index: 0,
            last_regime,
            metrics: RotaryMetrics::default(),
        })
    }

    fn sensor_sample(&mut self, at: VirtualTime) -> Result<Value, Box<dyn Error>> {
        let raw = self.sensors.observe(
            self.plant.state(),
            self.sample_index,
            TimestampUs(at.as_micros()),
        );
        self.sample_index = self
            .sample_index
            .checked_add(1)
            .ok_or_else(|| boxed("SITL sample index exhausted"))?;
        self.pending_raw = Some(raw);

        Ok(json!({
            "true_plant_state": state_json(self.plant.state()),
            "raw_observation": {
                "sample_index": raw.sample_index,
                "pendulum_adc_raw": raw.pendulum.adc_raw,
                "pendulum_quality_bits": raw.pendulum.quality.bits(),
                "arm_encoder_accumulated_count": raw.arm_encoder.accumulated_count,
                "arm_encoder_quality_bits": raw.arm_encoder.quality.bits()
            }
        }))
    }

    fn observation_delivery(&mut self, at: VirtualTime) -> Result<Value, Box<dyn Error>> {
        let raw = self
            .pending_raw
            .take()
            .ok_or_else(|| boxed("observation delivery without fresh RawObservation"))?;
        let measurement = self
            .adapter
            .measurement(raw)
            .map_err(|error| boxed(format!("estimator-input adapter: {error:?}")))?;
        let timing = self.timing_monitor.on_event(at.as_micros());
        let watchdog = self.watchdog.health(at.as_micros());
        self.runtime.source_mut().submit(RuntimeObservation {
            measurement,
            sensor_valid: true,
            sample_age_us: 0,
            timing,
            watchdog,
        });

        Ok(json!({
            "true_plant_state": state_json(self.plant.state()),
            "estimator_measurement": {
                "theta_rad": measurement.theta.0,
                "phi_rad": measurement.phi.0,
                "captured_at_us": measurement.captured_at.0
            },
            "timing_health": format!("{timing:?}"),
            "watchdog_health": format!("{watchdog:?}")
        }))
    }

    fn production_runtime(&mut self, at: VirtualTime) -> Result<Value, Box<dyn Error>> {
        self.pending_authorized = None;
        let active_regime = self.runtime.controller().regime();
        self.runtime
            .set_runtime_state(RuntimeState::Active(active_regime));
        let cycle = self
            .runtime
            .step()
            .map_err(|error| boxed(format!("control runtime: {error:?}")))?;
        self.watchdog.kick(at.as_micros());

        let new_regime = self.runtime.controller().regime();
        if new_regime != self.last_regime {
            self.metrics.regime_transitions = self.metrics.regime_transitions.saturating_add(1);
            if new_regime == ControlRegime::Capture && self.metrics.first_capture_us.is_none() {
                self.metrics.first_capture_us = Some(at.as_micros());
            }
            if new_regime == ControlRegime::Balance && self.metrics.first_balance_us.is_none() {
                self.metrics.first_balance_us = Some(at.as_micros());
            }
            self.last_regime = new_regime;
        }

        let payload = match cycle {
            ControlCycle::Primed => json!({
                "true_plant_state": state_json(self.plant.state()),
                "runtime_cycle": "primed",
                "control_regime": regime_name(new_regime),
                "authorized_actuation_present": false
            }),
            ControlCycle::Rejected { qualification } => {
                self.metrics.denied_cycles = self.metrics.denied_cycles.saturating_add(1);
                json!({
                    "true_plant_state": state_json(self.plant.state()),
                    "runtime_cycle": "rejected",
                    "control_regime": regime_name(new_regime),
                    "qualification_reasons_bits": qualification.reasons.bits(),
                    "authorized_actuation_present": false
                })
            }
            ControlCycle::Computed {
                state,
                demand,
                bounded_command,
                authority,
                authorized,
            } => {
                self.metrics.computed_cycles = self.metrics.computed_cycles.saturating_add(1);
                if bounded_command.saturated {
                    self.metrics.saturated_cycles = self.metrics.saturated_cycles.saturating_add(1);
                }
                if authorized.is_some() {
                    self.metrics.authorized_cycles =
                        self.metrics.authorized_cycles.saturating_add(1);
                } else {
                    self.metrics.denied_cycles = self.metrics.denied_cycles.saturating_add(1);
                }
                self.pending_authorized = authorized;
                json!({
                    "true_plant_state": state_json(self.plant.state()),
                    "runtime_cycle": "computed",
                    "control_regime": regime_name(new_regime),
                    "estimated_state": estimated_state_json(state),
                    "generalized_demand": {
                        "arm_torque_nm": demand.arm_torque.0
                    },
                    "bounded_actuator_command": {
                        "normalized_command": bounded_command.command.get(),
                        "saturated": bounded_command.saturated,
                        "predicted_arm_torque_nm": bounded_command.predicted_arm_torque.0
                    },
                    "authority": match authority.authority {
                        ActuationAuthority::Denied => "denied",
                        ActuationAuthority::ClosedLoop => "closed_loop"
                    },
                    "authority_reasons_bits": authority.reasons.bits(),
                    "authority_constrained": authority.constrained,
                    "authorized_actuation_present": authorized.is_some()
                })
            }
        };

        Ok(payload)
    }

    fn actuation_commit(&mut self) -> Result<Value, Box<dyn Error>> {
        let authorized = self.pending_authorized.take();
        if let Some(proof) = authorized {
            self.output.apply_closed_loop(proof)?;
        } else {
            self.output.safe_off()?;
        }

        let actuator = *self.actuator_state.borrow();
        self.metrics.max_abs_torque_nm = self
            .metrics
            .max_abs_torque_nm
            .max(actuator.applied_torque_nm.abs());

        Ok(json!({
            "true_plant_state": state_json(self.plant.state()),
            "authorized_actuation_present": authorized.is_some(),
            "tb6612_frame": {
                "mode": bridge_mode_name(actuator.frame.mode()),
                "duty_fraction": actuator.frame.duty_fraction()
            },
            "virtual_physical_actuator": {
                "applied_arm_torque_nm": actuator.applied_torque_nm
            }
        }))
    }
}

impl SitlSystem for RotarySitlSystem {
    fn advance_physical_time(
        &mut self,
        from: VirtualTime,
        to: VirtualTime,
    ) -> Result<(), Box<dyn Error>> {
        let delta_us = to
            .as_micros()
            .checked_sub(from.as_micros())
            .ok_or_else(|| boxed("virtual time moved backwards"))?;
        if delta_us % self.plant_step_us != 0 {
            return Err(boxed("physical time advance is not aligned to plant step"));
        }

        let substeps = delta_us / self.plant_step_us;
        let dt_s = self.plant_step_us as f32 * 1.0e-6;
        let applied_torque = TorqueNm(self.actuator_state.borrow().applied_torque_nm);
        for _ in 0..substeps {
            self.plant
                .step_rk4(dt_s, applied_torque)
                .map_err(|error| boxed(format!("plant integration: {error:?}")))?;
        }

        let state = self.plant.state();
        self.metrics.max_abs_theta_rad = self
            .metrics
            .max_abs_theta_rad
            .max(wrap_pi(state.theta).abs());
        self.metrics.max_abs_phi_rad = self.metrics.max_abs_phi_rad.max(state.phi.abs());
        Ok(())
    }

    fn on_event(
        &mut self,
        kind: EventKind,
        at: VirtualTime,
    ) -> Result<Option<Value>, Box<dyn Error>> {
        let payload = match kind {
            EventKind::ScenarioStart => json!({
                "true_plant_state": state_json(self.plant.state()),
                "tb6612_frame": {
                    "mode": "coast",
                    "duty_fraction": 0.0
                },
                "virtual_physical_actuator": {
                    "applied_arm_torque_nm": 0.0
                }
            }),
            EventKind::SensorSample => self.sensor_sample(at)?,
            EventKind::ObservationDelivery => self.observation_delivery(at)?,
            EventKind::ProductionRuntime => self.production_runtime(at)?,
            EventKind::RuntimeOpportunityMissed => json!({
                "true_plant_state": state_json(self.plant.state()),
                "runtime_cycle": "missed",
                "control_regime": regime_name(self.runtime.controller().regime())
            }),
            EventKind::ActuationCommit => self.actuation_commit()?,
        };
        Ok(Some(payload))
    }

    fn summary(&self) -> Value {
        let final_state = self.plant.state();
        let actuator = *self.actuator_state.borrow();
        json!({
            "computed_cycles": self.metrics.computed_cycles,
            "authorized_cycles": self.metrics.authorized_cycles,
            "denied_cycles": self.metrics.denied_cycles,
            "saturated_cycles": self.metrics.saturated_cycles,
            "regime_transitions": self.metrics.regime_transitions,
            "first_capture_us": self.metrics.first_capture_us,
            "first_balance_us": self.metrics.first_balance_us,
            "max_abs_theta_rad": self.metrics.max_abs_theta_rad,
            "max_abs_phi_rad": self.metrics.max_abs_phi_rad,
            "max_abs_torque_nm": self.metrics.max_abs_torque_nm,
            "final_state": state_json(final_state),
            "final_tb6612_frame": {
                "mode": bridge_mode_name(actuator.frame.mode()),
                "duty_fraction": actuator.frame.duty_fraction()
            },
            "final_applied_arm_torque_nm": actuator.applied_torque_nm
        })
    }
}

fn hybrid_controller(parameters: FurutaParameters) -> Result<HybridController, Box<dyn Error>> {
    let balance = LqrController::new(QNET_REFERENCE_TORQUE_GAINS)
        .map_err(|error| boxed(format!("LQR setup: {error:?}")))?;
    let swing = EnergySwingUpController::new(EnergySwingUpConfig {
        pendulum_mass_kg: parameters.pendulum_mass_kg,
        pendulum_com_length_m: parameters.pendulum_com_length_m,
        pendulum_inertia_kg_m2: parameters.pendulum_inertia_kg_m2,
        gravity_m_s2: parameters.gravity_m_s2,
        target_energy_j: TARGET_ENERGY_J,
        energy_gain: ENERGY_TORQUE_GAIN,
        max_abs_torque_nm: MAX_ABS_TORQUE_NM,
        kick_torque_nm: SWING_KICK_TORQUE_NM,
        kick_below_rate_rad_s: SWING_KICK_BELOW_RATE_RAD_S,
    })
    .map_err(|error| boxed(format!("swing-up setup: {error:?}")))?;
    let capture = CapturePolicy::new(CapturePolicyConfig {
        capture_enter_angle_rad: CAPTURE_ENTER_ANGLE_RAD,
        capture_enter_rate_rad_s: CAPTURE_ENTER_RATE_RAD_S,
        balance_enter_angle_rad: BALANCE_ENTER_ANGLE_RAD,
        balance_enter_rate_rad_s: BALANCE_ENTER_RATE_RAD_S,
        balance_exit_angle_rad: BALANCE_EXIT_ANGLE_RAD,
        balance_exit_rate_rad_s: BALANCE_EXIT_RATE_RAD_S,
        capture_exit_angle_rad: CAPTURE_EXIT_ANGLE_RAD,
        capture_exit_rate_rad_s: CAPTURE_EXIT_RATE_RAD_S,
        settle_cycles: CAPTURE_SETTLE_CYCLES,
    })
    .map_err(|error| boxed(format!("capture setup: {error:?}")))?;
    Ok(HybridController::new(swing, balance, capture))
}

fn state_json(state: FurutaState) -> Value {
    json!({
        "theta_rad": wrap_pi(state.theta),
        "theta_dot_rad_s": state.theta_dot,
        "phi_rad": state.phi,
        "phi_dot_rad_s": state.phi_dot
    })
}

fn estimated_state_json(state: EstimatedState) -> Value {
    json!({
        "theta_rad": state.theta.0,
        "theta_dot_rad_s": state.theta_dot.0,
        "phi_rad": state.phi.0,
        "phi_dot_rad_s": state.phi_dot.0,
        "captured_at_us": state.timestamp.0,
        "validity": format!("{:?}", state.validity)
    })
}

fn regime_name(regime: ControlRegime) -> &'static str {
    match regime {
        ControlRegime::SwingUp => "swingup",
        ControlRegime::Capture => "capture",
        ControlRegime::Balance => "balance",
    }
}

fn bridge_mode_name(mode: Tb6612BridgeMode) -> &'static str {
    match mode {
        Tb6612BridgeMode::Coast => "coast",
        Tb6612BridgeMode::DrivePositive => "drive_positive",
        Tb6612BridgeMode::DriveNegative => "drive_negative",
        Tb6612BridgeMode::Brake => "brake",
    }
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

fn boxed(message: impl Into<String>) -> Box<dyn Error> {
    Box::new(io::Error::other(message.into()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{execute_with_system, RunContext, Scenario};

    fn parameters() -> ReferenceAssemblyParameters {
        ReferenceAssemblyParameters::parse(include_str!(
            "../../../parameters/reference-assembly.json"
        ))
        .unwrap()
    }

    fn balance_scenario() -> Scenario {
        Scenario {
            id: "rotary-balance-test".to_string(),
            duration_us: 20_000,
            seed: 1,
            sensor_period_us: 1_000,
            runtime_period_us: 1_000,
            missed_runtime_at_us: Vec::new(),
            rotary: Some(RotaryScenario {
                plant_step_us: 50,
                initial_theta_rad: 5.0 * PI / 180.0,
                initial_theta_dot_rad_s: 0.0,
                initial_phi_rad: 0.0,
                initial_phi_dot_rad_s: 0.0,
            }),
        }
    }

    #[test]
    fn full_semantic_path_reaches_production_tb6612_frame() {
        let parameters = parameters();
        let scenario = balance_scenario();
        let mut system = RotarySitlSystem::new(
            &parameters,
            scenario.rotary.unwrap(),
            scenario.sensor_period_us,
            scenario.runtime_period_us,
        )
        .unwrap();
        let context = RunContext::scheduler_only("rotary-inverted-pendulum", "test")
            .with_model_configurations(
                parameters.production_model_configuration(),
                parameters.virtual_physical_truth_configuration(),
            );

        let artifacts = execute_with_system(&context, &scenario, &mut system).unwrap();
        let summary: Value = serde_json::from_str(&artifacts.summary_json).unwrap();

        assert_eq!(summary["pass"], true);
        assert!(summary["system"]["computed_cycles"].as_u64().unwrap() > 0);
        assert!(summary["system"]["authorized_cycles"].as_u64().unwrap() > 0);
        assert!(artifacts.trace_jsonl.contains("tb6612_frame"));
        assert!(artifacts
            .trace_jsonl
            .contains("authorized_actuation_present"));
    }

    #[test]
    fn synthetic_sensor_and_production_adapter_round_trip_within_quantization() {
        let parameters = parameters();
        let sensors = SyntheticSensors::new(&parameters.virtual_sensor).unwrap();
        let m = &parameters.measurement_model;
        let adapter = EstimatorInputAdapter::new(
            PendulumCalibration::new(
                m.pendulum_upright_adc.value,
                m.pendulum_radians_per_count.value,
                m.pendulum_direction.value,
            )
            .unwrap(),
            EncoderScale::new(
                m.arm_encoder_counts_per_revolution.value,
                m.arm_encoder_direction.value,
            )
            .unwrap(),
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

        assert!(
            (measurement.theta.0 - state.theta).abs()
                <= parameters
                    .measurement_model
                    .pendulum_radians_per_count
                    .value
        );
        let arm_quantum = 2.0 * PI
            / parameters
                .measurement_model
                .arm_encoder_counts_per_revolution
                .value;
        assert!((measurement.phi.0 - state.phi).abs() <= arm_quantum);
    }
}
