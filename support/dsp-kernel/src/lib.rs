#![no_std]
#![forbid(unsafe_code)]

/// Canonical real-time floating-point dot product.
///
/// Production ARM builds use CMSIS-DSP. Non-ARM builds provide a deterministic
/// semantic implementation for host tests and SITL without changing Control
/// semantics.
#[inline(always)]
pub fn dot_f32(lhs: &[f32], rhs: &[f32]) -> f32 {
    assert_eq!(lhs.len(), rhs.len());
    backend::dot_f32(lhs, rhs)
}

#[cfg(target_arch = "arm")]
mod backend {
    #[inline(always)]
    pub fn dot_f32(lhs: &[f32], rhs: &[f32]) -> f32 {
        cmsis_dsp::basic::dot_product_f32(lhs, rhs)
    }
}

#[cfg(not(target_arch = "arm"))]
mod backend {
    /// Host-only semantic implementation used for deterministic verification.
    #[inline(always)]
    pub fn dot_f32(lhs: &[f32], rhs: &[f32]) -> f32 {
        lhs.iter()
            .zip(rhs.iter())
            .map(|(left, right)| left * right)
            .sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dot_product_matches_linear_algebra_definition() {
        let lhs = [1.0, -2.0, 3.0, 0.5];
        let rhs = [4.0, 5.0, -1.0, 2.0];
        assert!((dot_f32(&lhs, &rhs) + 8.0).abs() < 1.0e-6);
    }
}
