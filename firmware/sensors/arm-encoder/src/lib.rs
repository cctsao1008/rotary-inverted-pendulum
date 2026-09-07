#![no_std]
#![forbid(unsafe_code)]

use rip_plant_observation::{MeasurementQuality, RawArmEncoderObservation};
use rip_robot_domain::TimestampUs;

/// Extends the wrapping STM32 timer counter into the Plant observation's
/// accumulated encoder-count semantic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EncoderCounterAccumulator {
    previous_counter: u16,
    accumulated_count: i32,
}

impl EncoderCounterAccumulator {
    pub const fn new(initial_counter: u16) -> Self {
        Self {
            previous_counter: initial_counter,
            accumulated_count: 0,
        }
    }

    pub fn update(&mut self, counter: u16) -> i32 {
        let delta = counter.wrapping_sub(self.previous_counter) as i16 as i32;
        self.previous_counter = counter;
        self.accumulated_count = self.accumulated_count.saturating_add(delta);
        self.accumulated_count
    }

    pub const fn accumulated_count(self) -> i32 {
        self.accumulated_count
    }

    pub fn observation(
        &mut self,
        counter: u16,
        captured_at: TimestampUs,
        quality: MeasurementQuality,
    ) -> RawArmEncoderObservation {
        RawArmEncoderObservation {
            captured_at,
            accumulated_count: self.update(counter),
            quality,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extends_forward_counter_wrap() {
        let mut accumulator = EncoderCounterAccumulator::new(65_530);
        assert_eq!(accumulator.update(4), 10);
    }

    #[test]
    fn extends_reverse_counter_wrap() {
        let mut accumulator = EncoderCounterAccumulator::new(4);
        assert_eq!(accumulator.update(65_530), -10);
    }
}
