use rip_robot_domain::{AngleRad, TimestampUs};
use rip_state_estimator::{BasicEstimator, Estimate, EstimatorConfig, EstimatorMeasurement};

fn config(alpha: f32) -> EstimatorConfig {
    EstimatorConfig {
        max_gap_us: 20_000,
        rate_filter_alpha: alpha,
    }
}

fn measurement(theta: f32, phi: f32, timestamp_us: u64) -> EstimatorMeasurement {
    EstimatorMeasurement {
        theta: AngleRad(theta),
        phi: AngleRad(phi),
        captured_at: TimestampUs(timestamp_us),
    }
}

fn ready_theta_dot(
    estimator: &mut BasicEstimator,
    alpha: f32,
    theta: f32,
    timestamp_us: u64,
) -> f32 {
    match estimator
        .step(config(alpha), measurement(theta, 0.0, timestamp_us))
        .unwrap()
    {
        Estimate::Ready(state) => state.theta_dot.0,
        Estimate::Primed => panic!("expected a ready estimate"),
    }
}

#[test]
fn isolated_position_outlier_produces_a_rate_spike_and_rebound() {
    let mut estimator = BasicEstimator::new();
    estimator
        .step(config(1.0), measurement(0.0, 0.0, 1_000))
        .unwrap();

    let nominal = ready_theta_dot(&mut estimator, 1.0, 0.01, 11_000);
    let spike = ready_theta_dot(&mut estimator, 1.0, 1.0, 21_000);
    let rebound = ready_theta_dot(&mut estimator, 1.0, 0.02, 31_000);

    assert!((nominal - 1.0).abs() < 1.0e-5);
    assert!((spike - 99.0).abs() < 1.0e-3);
    assert!((rebound + 98.0).abs() < 1.0e-3);
}

#[test]
fn low_pass_filter_attenuates_but_does_not_reject_an_outlier() {
    let mut estimator = BasicEstimator::new();
    estimator
        .step(config(0.1), measurement(0.0, 0.0, 1_000))
        .unwrap();

    let nominal = ready_theta_dot(&mut estimator, 0.1, 0.01, 11_000);
    let spike = ready_theta_dot(&mut estimator, 0.1, 1.0, 21_000);
    let rebound = ready_theta_dot(&mut estimator, 0.1, 0.02, 31_000);

    assert!((nominal - 0.1).abs() < 1.0e-5);
    assert!((spike - 9.99).abs() < 1.0e-3);
    assert!((rebound + 0.809).abs() < 1.0e-3);
}

#[test]
fn gap_reprime_uses_the_gap_sample_as_the_next_derivative_baseline() {
    let mut estimator = BasicEstimator::new();
    estimator
        .step(config(1.0), measurement(0.0, 0.0, 1_000))
        .unwrap();
    let _ = ready_theta_dot(&mut estimator, 1.0, 0.01, 11_000);

    // The first sample after a 39 ms gap is accepted only as a new baseline.
    // If that sample is itself an outlier, the following fresh sample can still
    // produce a large derivative because the outlier is now the baseline.
    assert_eq!(
        estimator
            .step(config(1.0), measurement(1.0, 0.0, 50_000))
            .unwrap(),
        Estimate::Primed
    );

    let recovery = ready_theta_dot(&mut estimator, 1.0, 0.02, 60_000);
    assert!((recovery + 98.0).abs() < 1.0e-3);
}
