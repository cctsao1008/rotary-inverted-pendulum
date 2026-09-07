#![no_std]
#![forbid(unsafe_code)]

use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};

use rip_control_runtime::ControlCycle;
use rip_hybrid_control::ControlRegime;
use rip_plant_observation::RawObservation;
use rip_robot_domain::EstimatedState;
use rip_runtime_state::{SensorTimingHealth, WatchdogHealth};

pub const CYCLE_IDLE: u32 = 0;
pub const CYCLE_PRIMED: u32 = 1;
pub const CYCLE_REJECTED: u32 = 2;
pub const CYCLE_COMPUTED: u32 = 3;
pub const CYCLE_ERROR: u32 = 4;

pub static SHADOW_SAMPLE_INDEX: AtomicU32 = AtomicU32::new(0);
pub static SHADOW_TIMESTAMP_US_LOW: AtomicU32 = AtomicU32::new(0);
pub static SHADOW_PENDULUM_ADC: AtomicU32 = AtomicU32::new(0);
pub static SHADOW_ARM_ENCODER_COUNT: AtomicI32 = AtomicI32::new(0);
pub static SHADOW_THETA_MRAD: AtomicI32 = AtomicI32::new(0);
pub static SHADOW_THETA_DOT_MRAD_S: AtomicI32 = AtomicI32::new(0);
pub static SHADOW_PHI_MRAD: AtomicI32 = AtomicI32::new(0);
pub static SHADOW_PHI_DOT_MRAD_S: AtomicI32 = AtomicI32::new(0);
pub static SHADOW_CYCLE: AtomicU32 = AtomicU32::new(CYCLE_IDLE);
pub static SHADOW_CONTROL_REGIME: AtomicU32 = AtomicU32::new(0);
pub static SHADOW_DEMAND_TORQUE_UNM: AtomicI32 = AtomicI32::new(0);
pub static SHADOW_BOUNDED_COMMAND_PPM: AtomicI32 = AtomicI32::new(0);
pub static SHADOW_PREDICTED_TORQUE_UNM: AtomicI32 = AtomicI32::new(0);
pub static SHADOW_ACTUATOR_SATURATED: AtomicU32 = AtomicU32::new(0);
pub static SHADOW_QUALIFICATION_REASONS: AtomicU32 = AtomicU32::new(0);
pub static SHADOW_AUTHORITY_REASONS: AtomicU32 = AtomicU32::new(0);
pub static SHADOW_AUTHORIZED: AtomicU32 = AtomicU32::new(0);
pub static SHADOW_SENSOR_TIMING_HEALTH: AtomicU32 = AtomicU32::new(0);
pub static SHADOW_WATCHDOG_HEALTH: AtomicU32 = AtomicU32::new(0);

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RuntimeRecordSnapshot {
    pub sample_index: u32,
    pub timestamp_us_low: u32,
    pub pendulum_adc: u16,
    pub arm_encoder_count: i32,
    pub theta_mrad: i32,
    pub theta_dot_mrad_s: i32,
    pub phi_mrad: i32,
    pub phi_dot_mrad_s: i32,
    pub cycle: u32,
    pub control_regime: u32,
    pub demand_torque_unm: i32,
    pub bounded_command_ppm: i32,
    pub predicted_torque_unm: i32,
    pub actuator_saturated: bool,
    pub qualification_reasons: u32,
    pub authority_reasons: u32,
    pub authorized: bool,
    pub sensor_timing_health: u32,
    pub watchdog_health: u32,
}

pub fn publish_raw(raw: RawObservation) {
    SHADOW_SAMPLE_INDEX.store(raw.sample_index, Ordering::Relaxed);
    SHADOW_TIMESTAMP_US_LOW.store(raw.pendulum.captured_at.0 as u32, Ordering::Relaxed);
    SHADOW_PENDULUM_ADC.store(u32::from(raw.pendulum.adc_raw), Ordering::Relaxed);
    SHADOW_ARM_ENCODER_COUNT.store(raw.arm_encoder.accumulated_count, Ordering::Relaxed);
}

pub fn publish_cycle(cycle: ControlCycle) {
    match cycle {
        ControlCycle::Primed => {
            SHADOW_CYCLE.store(CYCLE_PRIMED, Ordering::Relaxed);
            SHADOW_QUALIFICATION_REASONS.store(0, Ordering::Relaxed);
            clear_computed_snapshot();
        }
        ControlCycle::Rejected { qualification } => {
            SHADOW_CYCLE.store(CYCLE_REJECTED, Ordering::Relaxed);
            SHADOW_QUALIFICATION_REASONS
                .store(u32::from(qualification.reasons.bits()), Ordering::Relaxed);
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
            SHADOW_ACTUATOR_SATURATED
                .store(u32::from(bounded_command.saturated), Ordering::Relaxed);
            SHADOW_AUTHORITY_REASONS.store(u32::from(authority.reasons.bits()), Ordering::Relaxed);
            SHADOW_AUTHORIZED.store(u32::from(authorized.is_some()), Ordering::Relaxed);
        }
    }
}

pub fn publish_cycle_error() {
    SHADOW_CYCLE.store(CYCLE_ERROR, Ordering::Relaxed);
    SHADOW_QUALIFICATION_REASONS.store(0, Ordering::Relaxed);
    clear_computed_snapshot();
}

pub fn publish_regime(regime: ControlRegime) {
    let code = match regime {
        ControlRegime::SwingUp => 0,
        ControlRegime::Capture => 1,
        ControlRegime::Balance => 2,
    };
    SHADOW_CONTROL_REGIME.store(code, Ordering::Relaxed);
}

pub fn publish_runtime_health(timing: SensorTimingHealth, watchdog: WatchdogHealth) {
    SHADOW_SENSOR_TIMING_HEALTH.store(sensor_timing_code(timing), Ordering::Relaxed);
    SHADOW_WATCHDOG_HEALTH.store(watchdog_code(watchdog), Ordering::Relaxed);
}

pub fn snapshot() -> RuntimeRecordSnapshot {
    RuntimeRecordSnapshot {
        sample_index: SHADOW_SAMPLE_INDEX.load(Ordering::Relaxed),
        timestamp_us_low: SHADOW_TIMESTAMP_US_LOW.load(Ordering::Relaxed),
        pendulum_adc: SHADOW_PENDULUM_ADC.load(Ordering::Relaxed) as u16,
        arm_encoder_count: SHADOW_ARM_ENCODER_COUNT.load(Ordering::Relaxed),
        theta_mrad: SHADOW_THETA_MRAD.load(Ordering::Relaxed),
        theta_dot_mrad_s: SHADOW_THETA_DOT_MRAD_S.load(Ordering::Relaxed),
        phi_mrad: SHADOW_PHI_MRAD.load(Ordering::Relaxed),
        phi_dot_mrad_s: SHADOW_PHI_DOT_MRAD_S.load(Ordering::Relaxed),
        cycle: SHADOW_CYCLE.load(Ordering::Relaxed),
        control_regime: SHADOW_CONTROL_REGIME.load(Ordering::Relaxed),
        demand_torque_unm: SHADOW_DEMAND_TORQUE_UNM.load(Ordering::Relaxed),
        bounded_command_ppm: SHADOW_BOUNDED_COMMAND_PPM.load(Ordering::Relaxed),
        predicted_torque_unm: SHADOW_PREDICTED_TORQUE_UNM.load(Ordering::Relaxed),
        actuator_saturated: SHADOW_ACTUATOR_SATURATED.load(Ordering::Relaxed) != 0,
        qualification_reasons: SHADOW_QUALIFICATION_REASONS.load(Ordering::Relaxed),
        authority_reasons: SHADOW_AUTHORITY_REASONS.load(Ordering::Relaxed),
        authorized: SHADOW_AUTHORIZED.load(Ordering::Relaxed) != 0,
        sensor_timing_health: SHADOW_SENSOR_TIMING_HEALTH.load(Ordering::Relaxed),
        watchdog_health: SHADOW_WATCHDOG_HEALTH.load(Ordering::Relaxed),
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scaling_saturates_at_i32_bounds() {
        assert_eq!(scale(f32::MAX, 1.0), i32::MAX);
        assert_eq!(scale(-f32::MAX, 1.0), i32::MIN);
    }
}
