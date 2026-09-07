#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct VirtualTime(pub u64);

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct VirtualDuration(pub u64);

impl VirtualTime {
    pub const ZERO: Self = Self(0);

    pub const fn as_micros(self) -> u64 {
        self.0
    }

    pub fn checked_add(self, duration: VirtualDuration) -> Option<Self> {
        self.0.checked_add(duration.0).map(Self)
    }
}

impl VirtualDuration {
    pub const fn from_micros(value: u64) -> Self {
        Self(value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checked_add_preserves_microsecond_semantics() {
        assert_eq!(
            VirtualTime(5_000).checked_add(VirtualDuration::from_micros(2_500)),
            Some(VirtualTime(7_500))
        );
        assert_eq!(VirtualTime(u64::MAX).checked_add(VirtualDuration(1)), None);
    }
}
