#![no_std]
#![forbid(unsafe_code)]

use core::ops::{BitOr, BitOrAssign};
use rip_robot_domain::TimestampUs;

/// Quality evidence attached to one raw plant observation.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MeasurementQuality(u8);

impl MeasurementQuality {
    pub const NONE: Self = Self(0);
    pub const AVAILABLE: Self = Self(1 << 0);
    pub const IO_OK: Self = Self(1 << 1);
    pub const TIMING_VALID: Self = Self(1 << 2);
    pub const STALE: Self = Self(1 << 3);

    pub const fn bits(self) -> u8 {
        self.0
    }

    pub const fn contains(self, flag: Self) -> bool {
        self.0 & flag.0 == flag.0
    }
}

impl BitOr for MeasurementQuality {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

impl BitOrAssign for MeasurementQuality {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

/// Raw conductive-plastic pendulum-sensor observation.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RawPendulumObservation {
    pub captured_at: TimestampUs,
    pub adc_raw: u16,
    pub quality: MeasurementQuality,
}

/// Raw accumulated rotary-arm encoder observation.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RawArmEncoderObservation {
    pub captured_at: TimestampUs,
    pub accumulated_count: i32,
    pub quality: MeasurementQuality,
}

/// One raw observation batch populated by the Firmware acquisition path.
///
/// The type preserves hardware evidence without upgrading it into angles,
/// angular rates, estimator state, or runtime authority.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RawObservation {
    pub sample_index: u32,
    pub pendulum: RawPendulumObservation,
    pub arm_encoder: RawArmEncoderObservation,
}
