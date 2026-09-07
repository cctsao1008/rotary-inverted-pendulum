#![no_std]
#![forbid(unsafe_code)]

use rip_plant_observation::{MeasurementQuality, RawPendulumObservation};
use rip_robot_domain::TimestampUs;

/// Firmware acquisition boundary for the pendulum potentiometer ADC evidence.
///
/// Calibration and angle conversion remain Plant measurement semantics; this
/// type only packages the raw ADC sample with capture time and acquisition
/// quality.
#[derive(Debug, Clone, Copy, Default)]
pub struct PendulumAdcSensor;

impl PendulumAdcSensor {
    pub const fn observation(
        adc_raw: u16,
        captured_at: TimestampUs,
        quality: MeasurementQuality,
    ) -> RawPendulumObservation {
        RawPendulumObservation {
            captured_at,
            adc_raw,
            quality,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packages_raw_adc_without_applying_calibration() {
        let quality = MeasurementQuality::AVAILABLE | MeasurementQuality::IO_OK;
        let observation = PendulumAdcSensor::observation(2_928, TimestampUs(42), quality);
        assert_eq!(observation.adc_raw, 2_928);
        assert_eq!(observation.captured_at, TimestampUs(42));
        assert_eq!(observation.quality, quality);
    }
}
