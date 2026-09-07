#![no_std]
#![forbid(unsafe_code)]

use rip_runtime_observation_record::RuntimeRecordSnapshot;
use rip_timing_evidence::TimingEvidenceSnapshot;

pub const KEY_M_MASK: u32 = 1 << 0;
pub const KEY_X_MASK: u32 = 1 << 1;
pub const KEY_PLUS_MASK: u32 = 1 << 2;
pub const KEY_MINUS_MASK: u32 = 1 << 3;
pub const KEY_USER_MASK: u32 = 1 << 4;
const KEY_COUNT: usize = 5;
const KEY_DEBOUNCE_TICKS: u32 = 20;
const KEY_REPEAT_DELAY_TICKS: u32 = 500;
const KEY_REPEAT_TICKS: u32 = 150;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum HealthState {
    #[default]
    Unknown,
    Ok,
    Late,
    Fault,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum StatusPage {
    #[default]
    Status,
    Sensor,
    Safety,
    Control,
    Maintenance,
}

impl StatusPage {
    pub const fn next(self) -> Self {
        match self {
            Self::Status => Self::Sensor,
            Self::Sensor => Self::Safety,
            Self::Safety => Self::Control,
            Self::Control => Self::Maintenance,
            Self::Maintenance => Self::Status,
        }
    }

    pub const fn previous(self) -> Self {
        match self {
            Self::Status => Self::Maintenance,
            Self::Sensor => Self::Status,
            Self::Safety => Self::Sensor,
            Self::Control => Self::Safety,
            Self::Maintenance => Self::Control,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StatusView {
    pub cycle: u32,
    pub control_regime: u32,
    pub timing: HealthState,
    pub watchdog: HealthState,
    pub authorized: bool,
    pub actuator_saturated: bool,
    pub telemetry_enabled: bool,
    pub motor_sink_bound: bool,
    pub qualification_reasons: u32,
    pub authority_reasons: u32,
    pub pendulum_adc: u16,
    pub arm_encoder_count: i32,
    pub theta_mrad: i32,
    pub theta_dot_mrad_s: i32,
    pub phi_mrad: i32,
    pub phi_dot_mrad_s: i32,
    pub demand_torque_unm: i32,
    pub bounded_command_ppm: i32,
    pub predicted_torque_unm: i32,
    pub inferred_missed_ticks: u32,
    pub deadline_overruns: u32,
}

impl StatusView {
    pub fn from_records(
        runtime: RuntimeRecordSnapshot,
        timing: TimingEvidenceSnapshot,
        telemetry_enabled: bool,
        motor_sink_bound: bool,
    ) -> Self {
        Self {
            cycle: runtime.cycle,
            control_regime: runtime.control_regime,
            timing: sensor_health(runtime.sensor_timing_health),
            watchdog: watchdog_health(runtime.watchdog_health),
            authorized: runtime.authorized,
            actuator_saturated: runtime.actuator_saturated,
            telemetry_enabled,
            motor_sink_bound,
            qualification_reasons: runtime.qualification_reasons,
            authority_reasons: runtime.authority_reasons,
            pendulum_adc: runtime.pendulum_adc,
            arm_encoder_count: runtime.arm_encoder_count,
            theta_mrad: runtime.theta_mrad,
            theta_dot_mrad_s: runtime.theta_dot_mrad_s,
            phi_mrad: runtime.phi_mrad,
            phi_dot_mrad_s: runtime.phi_dot_mrad_s,
            demand_torque_unm: runtime.demand_torque_unm,
            bounded_command_ppm: runtime.bounded_command_ppm,
            predicted_torque_unm: runtime.predicted_torque_unm,
            inferred_missed_ticks: timing.inferred_missed_tick_count,
            deadline_overruns: timing.deadline_overrun_count,
        }
    }
}

const fn sensor_health(code: u32) -> HealthState {
    match code {
        1 => HealthState::Ok,
        2 => HealthState::Late,
        3 => HealthState::Fault,
        _ => HealthState::Unknown,
    }
}

const fn watchdog_health(code: u32) -> HealthState {
    match code {
        1 => HealthState::Ok,
        2 => HealthState::Fault,
        _ => HealthState::Unknown,
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct KeyEvents {
    pub pressed: u32,
    pub released: u32,
    pub repeat: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct KeyService {
    stable_mask: u32,
    candidate_mask: u32,
    candidate_since: [u32; KEY_COUNT],
    pressed_since: [u32; KEY_COUNT],
    last_repeat: [u32; KEY_COUNT],
}

impl KeyService {
    pub const fn new(initial_pressed_mask: u32, tick: u32) -> Self {
        Self {
            stable_mask: initial_pressed_mask,
            candidate_mask: initial_pressed_mask,
            candidate_since: [tick; KEY_COUNT],
            pressed_since: [tick; KEY_COUNT],
            last_repeat: [tick; KEY_COUNT],
        }
    }

    pub fn update(&mut self, raw_pressed_mask: u32, tick: u32) -> KeyEvents {
        let mut events = KeyEvents::default();
        for key in 0..KEY_COUNT {
            let bit = 1_u32 << key;
            let raw_pressed = raw_pressed_mask & bit != 0;
            let candidate_pressed = self.candidate_mask & bit != 0;
            let stable_pressed = self.stable_mask & bit != 0;

            if raw_pressed != candidate_pressed {
                if raw_pressed {
                    self.candidate_mask |= bit;
                } else {
                    self.candidate_mask &= !bit;
                }
                self.candidate_since[key] = tick;
                continue;
            }

            if candidate_pressed != stable_pressed {
                if tick.wrapping_sub(self.candidate_since[key]) < KEY_DEBOUNCE_TICKS {
                    continue;
                }
                if candidate_pressed {
                    self.stable_mask |= bit;
                    self.pressed_since[key] = tick;
                    self.last_repeat[key] = tick;
                    events.pressed |= bit;
                } else {
                    self.stable_mask &= !bit;
                    events.released |= bit;
                }
                continue;
            }

            if stable_pressed
                && tick.wrapping_sub(self.pressed_since[key]) >= KEY_REPEAT_DELAY_TICKS
                && tick.wrapping_sub(self.last_repeat[key]) >= KEY_REPEAT_TICKS
            {
                self.last_repeat[key] = tick;
                events.repeat |= bit;
            }
        }
        events
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LocalUi {
    page: StatusPage,
    contrast: u8,
}

impl Default for LocalUi {
    fn default() -> Self {
        Self::new()
    }
}

impl LocalUi {
    pub const fn new() -> Self {
        Self {
            page: StatusPage::Status,
            contrast: 0x7f,
        }
    }

    pub const fn page(&self) -> StatusPage {
        self.page
    }

    pub const fn contrast(&self) -> u8 {
        self.contrast
    }

    pub fn next_page(&mut self) {
        self.page = self.page.next();
    }

    pub fn previous_page(&mut self) {
        self.page = self.page.previous();
    }

    pub fn increase_contrast(&mut self) -> u8 {
        self.contrast = self.contrast.saturating_add(16);
        self.contrast
    }

    pub fn decrease_contrast(&mut self) -> u8 {
        self.contrast = self.contrast.saturating_sub(16);
        self.contrast
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn page_cycle_is_closed() {
        let page = StatusPage::Status.next().next().next().next().next();
        assert_eq!(page, StatusPage::Status);
    }

    #[test]
    fn key_service_requires_stable_debounce_before_press() {
        let mut keys = KeyService::new(0, 0);
        assert_eq!(keys.update(KEY_M_MASK, 1).pressed, 0);
        assert_eq!(keys.update(KEY_M_MASK, 20).pressed, 0);
        assert_eq!(keys.update(KEY_M_MASK, 21).pressed, KEY_M_MASK);
    }
}
