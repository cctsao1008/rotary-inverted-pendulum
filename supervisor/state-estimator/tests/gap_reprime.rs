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

#[test]
fn timing_gap_causes_exactly_one_reprime_cycle_then_recovers() {
    let mut estimator = BasicEstimator::new();

    assert_eq!(
        estimator
            .step(config(1.0), measurement(0.0, 0.0, 1_000))
            .unwrap(),
        Estimate::Primed
    );

    match estimator
        .step(config(1.0), measurement(0.01, 0.02, 11_000))
        .unwrap()
    {
        Estimate::Ready(_) => {}
        Estimate::Primed => panic!("fresh second sample should be ready"),
    }

    // 39 ms from the previous accepted measurement exceeds max_gap_us.
    // The gap sample becomes the new derivative baseline and intentionally
    // returns Primed instead of manufacturing a rate across stale time.
    assert_eq!(
        estimator
            .step(config(1.0), measurement(0.20, 0.40, 50_000))
            .unwrap(),
        Estimate::Primed
    );

    match estimator
        .step(config(1.0), measurement(0.21, 0.42, 60_000))
        .unwrap()
    {
        Estimate::Ready(state) => {
            assert!((state.theta_dot.0 - 1.0).abs() < 1.0e-5);
            assert!((state.phi_dot.0 - 2.0).abs() < 1.0e-5);
        }
        Estimate::Primed => panic!("first fresh sample after the new baseline should recover"),
    }
}

#[test]
fn filter_alpha_changes_spike_amplitude_but_does_not_reject_measurement_steps() {
    let mut unfiltered = BasicEstimator::new();
    let mut filtered = BasicEstimator::new();

    unfiltered
        .step(config(1.0), measurement(0.0, 0.0, 1_000))
        .unwrap();
    filtered
        .step(config(0.1), measurement(0.0, 0.0, 1_000))
        .unwrap();

    let unfiltered_rate = match unfiltered
        .step(config(1.0), measurement(1.0, 0.0, 11_000))
        .unwrap()
    {
        Estimate::Ready(state) => state.theta_dot.0,
        Estimate::Primed => panic!("second sample should be ready"),
    };
    let filtered_rate = match filtered
        .step(config(0.1), measurement(1.0, 0.0, 11_000))
        .unwrap()
    {
        Estimate::Ready(state) => state.theta_dot.0,
        Estimate::Primed => panic!("second sample should be ready"),
    };

    assert!((unfiltered_rate - 100.0).abs() < 1.0e-3);
    assert!((filtered_rate - 10.0).abs() < 1.0e-3);
    assert!(filtered_rate.abs() < unfiltered_rate.abs());
}
