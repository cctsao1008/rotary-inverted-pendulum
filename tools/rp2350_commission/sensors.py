from __future__ import annotations

import statistics
import time

from device import Rp2350Device
from recording import RunRecorder


_FREE_SWING_MIN_PEAK_TO_PEAK = 64
_FREE_SWING_MIN_SIGNAL_TO_STEP_RATIO = 8.0
_FREE_SWING_MIN_HYSTERESIS_COUNTS = 8.0
_FREE_SWING_HYSTERESIS_STEP_MULTIPLIER = 3.0
_FREE_SWING_MIN_SAMPLES_PER_PERIOD = 8.0
_FREE_SWING_MAX_RELATIVE_PERIOD_MAD = 0.20


def _metadata(device: Rp2350Device, test: str) -> dict[str, object]:
    return {
        "test": test,
        "hid_vid": "0xCAFE",
        "hid_pid": "0x4010",
        "cdc_port": device.cdc.port if device.cdc_available else None,
        "firmware_version": device.version(),
    }


def _estimate_free_swing_period(
    timestamps_us: list[int], adc_values: list[int]
) -> dict[str, object]:
    """Estimate a passive period without turning floating/noisy ADC data into a result.

    These are diagnostic-quality gates, not plant calibration. A result is only published when
    the ADC excursion is clearly larger than sample-to-sample motion, crossings traverse a
    Schmitt-style hysteresis band, the period is resolved by enough telemetry samples, and at
    least two candidate periods agree.
    """
    if len(timestamps_us) != len(adc_values):
        raise ValueError("timestamp and ADC sample counts differ")
    if len(adc_values) < 3:
        raise ValueError("at least three samples are required")

    dt_s = [
        (later - earlier) * 1.0e-6
        for earlier, later in zip(timestamps_us, timestamps_us[1:])
        if later > earlier
    ]
    if not dt_s:
        raise ValueError("timestamps do not advance")

    center = statistics.fmean(adc_values)
    adc_min = min(adc_values)
    adc_max = max(adc_values)
    peak_to_peak = adc_max - adc_min
    step_sizes = [abs(later - earlier) for earlier, later in zip(adc_values, adc_values[1:])]
    median_step = statistics.median(step_sizes) if step_sizes else 0.0
    signal_to_step_ratio = (
        float("inf") if median_step == 0.0 and peak_to_peak > 0 else
        (peak_to_peak / median_step if median_step > 0.0 else 0.0)
    )

    sample_interval_s = statistics.median(dt_s)
    min_resolved_period_s = sample_interval_s * _FREE_SWING_MIN_SAMPLES_PER_PERIOD
    hysteresis_counts = max(
        _FREE_SWING_MIN_HYSTERESIS_COUNTS,
        median_step * _FREE_SWING_HYSTERESIS_STEP_MULTIPLIER,
    )
    lower = center - hysteresis_counts
    upper = center + hysteresis_counts

    crossings_us: list[int] = []
    armed = adc_values[0] <= lower
    for timestamp_us, value in zip(timestamps_us[1:], adc_values[1:]):
        if value <= lower:
            armed = True
        elif armed and value >= upper:
            crossings_us.append(timestamp_us)
            armed = False

    candidate_periods_s = [
        (later - earlier) * 1.0e-6
        for earlier, later in zip(crossings_us, crossings_us[1:])
        if later > earlier
    ]
    resolved_periods_s = [
        period for period in candidate_periods_s if period >= min_resolved_period_s
    ]

    valid = False
    reason = "ok"
    period_s: float | None = None
    frequency_hz: float | None = None
    relative_period_mad: float | None = None

    if peak_to_peak < _FREE_SWING_MIN_PEAK_TO_PEAK:
        reason = "insufficient_adc_excursion"
    elif signal_to_step_ratio < _FREE_SWING_MIN_SIGNAL_TO_STEP_RATIO:
        reason = "noise_dominated"
    elif len(resolved_periods_s) < 2:
        reason = "insufficient_resolved_cycles"
    else:
        median_period = statistics.median(resolved_periods_s)
        period_mad = statistics.median(abs(period - median_period) for period in resolved_periods_s)
        relative_period_mad = period_mad / median_period if median_period > 0.0 else float("inf")
        if relative_period_mad > _FREE_SWING_MAX_RELATIVE_PERIOD_MAD:
            reason = "inconsistent_periods"
        else:
            valid = True
            period_s = median_period
            frequency_hz = 1.0 / period_s

    return {
        "valid": valid,
        "reason": reason,
        "adc_center": center,
        "adc_min": adc_min,
        "adc_max": adc_max,
        "adc_peak_to_peak": peak_to_peak,
        "adc_median_step": median_step,
        "signal_to_step_ratio": signal_to_step_ratio,
        "crossing_hysteresis_counts": hysteresis_counts,
        "sample_interval_s": sample_interval_s,
        "min_resolved_period_s": min_resolved_period_s,
        "rising_crossings": len(crossings_us),
        "candidate_periods": len(candidate_periods_s),
        "resolved_periods": len(resolved_periods_s),
        "relative_period_mad": relative_period_mad,
        "estimated_period_s": period_s,
        "estimated_frequency_hz": frequency_hz,
    }


def monitor(
    device: Rp2350Device,
    duration_s: float = 10.0,
    *,
    motor_command: float | None = None,
) -> dict[str, object]:
    device.start_telemetry()
    count = 0
    last = None
    deadline = time.monotonic() + duration_s
    next_refresh = 0.0
    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if motor_command is not None and now >= next_refresh:
                device.set_motor_command(motor_command, lease_ms=250)
                next_refresh = now + 0.10
            sample = device.read_sample(100)
            if sample is None:
                continue
            last = sample
            count += 1
            print(
                f"t={sample.timestamp_us / 1e6:9.3f}s "
                f"adc={sample.pendulum_adc_raw:4d} "
                f"A/B={sample.encoder_a}/{sample.encoder_b} "
                f"enc={sample.arm_encoder_count:8d} "
                f"theta={sample.theta:+.5f} theta_dot={sample.theta_dot:+.5f} "
                f"phi={sample.phi:+.5f} phi_dot={sample.phi_dot:+.5f} "
                f"cmd={sample.normalized_command:+.3f} exec={sample.execution_time_us}us"
            )
    finally:
        if motor_command is not None:
            device.safe_off()
    return {
        "test": "monitor",
        "samples": count,
        "motor_command": motor_command,
        "last": last.as_dict() if last else None,
    }


def adc(device: Rp2350Device, duration_s: float = 5.0) -> dict[str, object]:
    device.start_telemetry()
    with RunRecorder("adc") as recorder:
        recorder.write_metadata(_metadata(device, "adc"))
        values: list[int] = []
        for sample in device.samples(duration_s):
            values.append(sample.pendulum_adc_raw)
            recorder.append_sample(sample)
            recorder.append_cdc(device.drain_cdc())
        if not values:
            raise RuntimeError("no HID telemetry received")
        summary = {
            "test": "adc",
            "samples": len(values),
            "min": min(values),
            "max": max(values),
            "mean": statistics.fmean(values),
            "stdev": statistics.pstdev(values),
            "peak_to_peak": max(values) - min(values),
            "artifact_dir": str(recorder.directory),
        }
        recorder.write_summary(summary)
        return summary


def encoder(
    device: Rp2350Device,
    duration_s: float = 5.0,
    *,
    motor_command: float | None = None,
) -> dict[str, object]:
    """Capture Encoder2 A/B/count, optionally while directly driving the arm."""
    device.start_telemetry()
    with RunRecorder("encoder") as recorder:
        metadata = _metadata(device, "encoder")
        metadata["motor_command"] = motor_command
        recorder.write_metadata(metadata)

        samples = []
        deadline = time.monotonic() + duration_s
        next_refresh = 0.0
        try:
            while time.monotonic() < deadline:
                now = time.monotonic()
                if motor_command is not None and now >= next_refresh:
                    device.set_motor_command(motor_command, lease_ms=250)
                    next_refresh = now + 0.10
                sample = device.read_sample(100)
                if sample is not None:
                    samples.append(sample)
                    recorder.append_sample(
                        sample,
                        requested_command=motor_command if motor_command is not None else 0.0,
                    )
                recorder.append_cdc(device.drain_cdc())
        finally:
            if motor_command is not None:
                device.safe_off()

        if not samples:
            raise RuntimeError("no HID telemetry received")
        states = sorted({(sample.encoder_a, sample.encoder_b) for sample in samples})
        count_delta = samples[-1].arm_encoder_count - samples[0].arm_encoder_count
        summary = {
            "test": "encoder",
            "samples": len(samples),
            "motor_command": motor_command,
            "observed_ab_states": [f"{a}{b}" for a, b in states],
            "start_count": samples[0].arm_encoder_count,
            "end_count": samples[-1].arm_encoder_count,
            "count_delta": count_delta,
            "mean_phi_dot_rad_s": statistics.fmean(s.phi_dot for s in samples),
            "artifact_dir": str(recorder.directory),
        }
        recorder.write_summary(summary)
        return summary


def free_swing(device: Rp2350Device, duration_s: float = 10.0) -> dict[str, object]:
    """Record passive pendulum motion and report a period only when the trace is credible."""
    if duration_s <= 0.0:
        raise ValueError("duration must be > 0")

    device.safe_off()
    device.start_telemetry()
    with RunRecorder("free-swing") as recorder:
        recorder.write_metadata(
            {
                **_metadata(device, "free-swing"),
                "duration_s": duration_s,
                "motor_command": 0.0,
            }
        )
        samples = []
        for sample in device.samples(duration_s):
            samples.append(sample)
            recorder.append_sample(sample, requested_command=0.0)
            recorder.append_cdc(device.drain_cdc())

        if len(samples) < 3:
            raise RuntimeError("insufficient telemetry during free-swing test")

        estimate = _estimate_free_swing_period(
            [sample.timestamp_us for sample in samples],
            [sample.pendulum_adc_raw for sample in samples],
        )
        summary = {
            "test": "free-swing",
            "samples": len(samples),
            "duration_s": duration_s,
            "estimate_valid": estimate.pop("valid"),
            "estimate_reason": estimate.pop("reason"),
            **estimate,
            "artifact_dir": str(recorder.directory),
        }
        recorder.write_summary(summary)
        return summary
