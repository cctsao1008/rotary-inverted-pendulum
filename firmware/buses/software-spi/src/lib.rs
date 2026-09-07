#![no_std]
#![forbid(unsafe_code)]

/// Minimal infallible digital-output contract used by the write-only software
/// SPI transport. Concrete MCU GPIO errors are impossible on the STM32F103
/// push-pull outputs used by the reference board.
pub trait OutputLine {
    fn set_low(&mut self);
    fn set_high(&mut self);
}

/// Write-only mode-0 software SPI used by the reference OLED wiring.
///
/// This bus owns only clock/data signaling. Display reset and D/C semantics stay
/// with the OLED device adapter.
pub struct SoftwareSpi<C, D> {
    clock: C,
    data: D,
    half_period_padding: u8,
}

impl<C, D> SoftwareSpi<C, D>
where
    C: OutputLine,
    D: OutputLine,
{
    pub fn new(mut clock: C, mut data: D, half_period_padding: u8) -> Self {
        clock.set_low();
        data.set_low();
        Self {
            clock,
            data,
            half_period_padding,
        }
    }

    pub fn write_byte(&mut self, mut value: u8) {
        for _ in 0..8 {
            self.clock.set_low();
            if value & 0x80 != 0 {
                self.data.set_high();
            } else {
                self.data.set_low();
            }
            self.pad();
            self.clock.set_high();
            self.pad();
            value <<= 1;
        }
        self.clock.set_low();
    }

    pub fn write(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.write_byte(byte);
        }
    }

    pub fn into_inner(self) -> (C, D) {
        (self.clock, self.data)
    }

    #[inline(always)]
    fn pad(&self) {
        for _ in 0..self.half_period_padding {
            core::hint::spin_loop();
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::{cell::RefCell, rc::Rc, vec::Vec};

    #[derive(Clone)]
    struct TraceLine {
        id: u8,
        trace: Rc<RefCell<Vec<(u8, bool)>>>,
    }

    impl OutputLine for TraceLine {
        fn set_low(&mut self) {
            self.trace.borrow_mut().push((self.id, false));
        }

        fn set_high(&mut self) {
            self.trace.borrow_mut().push((self.id, true));
        }
    }

    #[test]
    fn writes_msb_first_and_returns_clock_low() {
        let trace = Rc::new(RefCell::new(Vec::new()));
        let clock = TraceLine {
            id: 0,
            trace: trace.clone(),
        };
        let data = TraceLine {
            id: 1,
            trace: trace.clone(),
        };
        let mut spi = SoftwareSpi::new(clock, data, 0);
        trace.borrow_mut().clear();
        spi.write_byte(0b1010_0000);

        let events = trace.borrow();
        let data_levels: Vec<bool> = events
            .iter()
            .filter_map(|(id, level)| (*id == 1).then_some(*level))
            .collect();
        assert_eq!(data_levels, [true, false, true, false, false, false, false, false]);
        assert_eq!(events.last(), Some(&(0, false)));
    }
}
