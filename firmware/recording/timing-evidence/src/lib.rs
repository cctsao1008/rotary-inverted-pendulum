#![no_std]
#![forbid(unsafe_code)]

use core::sync::atomic::{AtomicU32, Ordering};

use rip_runtime_state::SensorTimingHealth;

pub static TIMING_ADMISSION_COUNT: AtomicU32 = AtomicU32::new(0);
pub static TIMING_ELAPSED_MS: AtomicU32 = AtomicU32::new(0);
pub static TIMING_LAST_PERIOD_US: AtomicU32 = AtomicU32::new(0);
pub static TIMING_MIN_PERIOD_US: AtomicU32 = AtomicU32::new(u32::MAX);
pub static TIMING_MAX_PERIOD_US: AtomicU32 = AtomicU32::new(0);
pub static TIMING_MAX_PERIOD_JITTER_US: AtomicU32 = AtomicU32::new(0);
pub static TIMING_MAX_TICK_PHASE_US: AtomicU32 = AtomicU32::new(0);
pub static TIMING_LAST_EXECUTION_US: AtomicU32 = AtomicU32::new(0);
pub static TIMING_MAX_EXECUTION_US: AtomicU32 = AtomicU32::new(0);
pub static TIMING_MAX_EXECUTION_CYCLES: AtomicU32 = AtomicU32::new(0);
pub static TIMING_DEADLINE_OVERRUN_COUNT: AtomicU32 = AtomicU32::new(0);
pub static TIMING_INFERRED_MISSED_TICK_COUNT: AtomicU32 = AtomicU32::new(0);
pub static TIMING_SUPERVISOR_LATE_COUNT: AtomicU32 = AtomicU32::new(0);
pub static TIMING_SUPERVISOR_TIMEOUT_COUNT: AtomicU32 = AtomicU32::new(0);
pub static TIMING_ADC_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static TIMING_RUNTIME_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct TimingEvidenceSnapshot {
    pub admission_count: u32,
    pub elapsed_ms: u32,
    pub last_period_us: u32,
    pub min_period_us: u32,
    pub max_period_us: u32,
    pub max_period_jitter_us: u32,
    pub max_tick_phase_us: u32,
    pub last_execution_us: u32,
    pub max_execution_us: u32,
    pub max_execution_cycles: u32,
    pub deadline_overrun_count: u32,
    pub inferred_missed_tick_count: u32,
    pub supervisor_late_count: u32,
    pub supervisor_timeout_count: u32,
    pub adc_error_count: u32,
    pub runtime_error_count: u32,
}

pub struct RuntimeTimingCharacterizer {
    expected_period_us: u32,
    last_admitted_at_us: Option<u64>,
}

impl RuntimeTimingCharacterizer {
    pub const fn new(expected_period_us: u32) -> Self {
        Self {
            expected_period_us,
            last_admitted_at_us: None,
        }
    }

    pub fn observe_admission(&mut self, admitted_at_us: u64, tick_phase_us: u32) {
        TIMING_ADMISSION_COUNT.fetch_add(1, Ordering::Relaxed);
        TIMING_ELAPSED_MS.store(
            (admitted_at_us / 1_000).min(u64::from(u32::MAX)) as u32,
            Ordering::Relaxed,
        );
        record_max(&TIMING_MAX_TICK_PHASE_US, tick_phase_us);

        if let Some(previous) = self.last_admitted_at_us {
            let period_us = admitted_at_us.saturating_sub(previous);
            let period_u32 = period_us.min(u64::from(u32::MAX)) as u32;
            TIMING_LAST_PERIOD_US.store(period_u32, Ordering::Relaxed);
            record_min(&TIMING_MIN_PERIOD_US, period_u32);
            record_max(&TIMING_MAX_PERIOD_US, period_u32);

            let expected = u64::from(self.expected_period_us);
            let observed_periods = ((period_us + expected / 2) / expected).max(1);
            let nominal_period_us = observed_periods.saturating_mul(expected);
            let jitter_us = period_us.abs_diff(nominal_period_us);
            record_max(
                &TIMING_MAX_PERIOD_JITTER_US,
                jitter_us.min(u64::from(u32::MAX)) as u32,
            );

            let inferred_missed = observed_periods.saturating_sub(1);
            if inferred_missed != 0 {
                TIMING_INFERRED_MISSED_TICK_COUNT.fetch_add(
                    inferred_missed.min(u64::from(u32::MAX)) as u32,
                    Ordering::Relaxed,
                );
            }
        }

        self.last_admitted_at_us = Some(admitted_at_us);
    }

    pub fn observe_supervisor_health(&self, health: SensorTimingHealth) {
        match health {
            SensorTimingHealth::Late => {
                TIMING_SUPERVISOR_LATE_COUNT.fetch_add(1, Ordering::Relaxed);
            }
            SensorTimingHealth::Timeout => {
                TIMING_SUPERVISOR_TIMEOUT_COUNT.fetch_add(1, Ordering::Relaxed);
            }
            SensorTimingHealth::Startup | SensorTimingHealth::Healthy => {}
        }
    }

    pub fn finish_cycle(&self, elapsed_cycles: u32, ticks_per_us: u32) {
        let execution_us = div_ceil_u32(elapsed_cycles, ticks_per_us);
        TIMING_LAST_EXECUTION_US.store(execution_us, Ordering::Relaxed);
        record_max(&TIMING_MAX_EXECUTION_US, execution_us);
        record_max(&TIMING_MAX_EXECUTION_CYCLES, elapsed_cycles);

        if execution_us > self.expected_period_us {
            TIMING_DEADLINE_OVERRUN_COUNT.fetch_add(1, Ordering::Relaxed);
        }
    }

    pub fn record_adc_error(&self) {
        TIMING_ADC_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
    }

    pub fn record_runtime_error(&self) {
        TIMING_RUNTIME_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
    }
}

pub fn snapshot() -> TimingEvidenceSnapshot {
    TimingEvidenceSnapshot {
        admission_count: TIMING_ADMISSION_COUNT.load(Ordering::Relaxed),
        elapsed_ms: TIMING_ELAPSED_MS.load(Ordering::Relaxed),
        last_period_us: TIMING_LAST_PERIOD_US.load(Ordering::Relaxed),
        min_period_us: TIMING_MIN_PERIOD_US.load(Ordering::Relaxed),
        max_period_us: TIMING_MAX_PERIOD_US.load(Ordering::Relaxed),
        max_period_jitter_us: TIMING_MAX_PERIOD_JITTER_US.load(Ordering::Relaxed),
        max_tick_phase_us: TIMING_MAX_TICK_PHASE_US.load(Ordering::Relaxed),
        last_execution_us: TIMING_LAST_EXECUTION_US.load(Ordering::Relaxed),
        max_execution_us: TIMING_MAX_EXECUTION_US.load(Ordering::Relaxed),
        max_execution_cycles: TIMING_MAX_EXECUTION_CYCLES.load(Ordering::Relaxed),
        deadline_overrun_count: TIMING_DEADLINE_OVERRUN_COUNT.load(Ordering::Relaxed),
        inferred_missed_tick_count: TIMING_INFERRED_MISSED_TICK_COUNT.load(Ordering::Relaxed),
        supervisor_late_count: TIMING_SUPERVISOR_LATE_COUNT.load(Ordering::Relaxed),
        supervisor_timeout_count: TIMING_SUPERVISOR_TIMEOUT_COUNT.load(Ordering::Relaxed),
        adc_error_count: TIMING_ADC_ERROR_COUNT.load(Ordering::Relaxed),
        runtime_error_count: TIMING_RUNTIME_ERROR_COUNT.load(Ordering::Relaxed),
    }
}

fn div_ceil_u32(value: u32, divisor: u32) -> u32 {
    assert!(divisor != 0);
    let numerator = u64::from(value) + u64::from(divisor) - 1;
    (numerator / u64::from(divisor)) as u32
}

fn record_max(target: &AtomicU32, value: u32) {
    if value > target.load(Ordering::Relaxed) {
        target.store(value, Ordering::Relaxed);
    }
}

fn record_min(target: &AtomicU32, value: u32) {
    if value < target.load(Ordering::Relaxed) {
        target.store(value, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn division_rounds_execution_time_up() {
        assert_eq!(div_ceil_u32(73, 72), 2);
        assert_eq!(div_ceil_u32(72, 72), 1);
    }
}
