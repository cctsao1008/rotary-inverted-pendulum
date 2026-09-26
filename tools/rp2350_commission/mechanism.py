from __future__ import annotations

import math
import random
import statistics
import time

from device import Rp2350Device
from recording import RunRecorder


def _capture_samples(
    device: Rp2350Device,
    recorder: RunRecorder,
    duration_s: float,
    *,
    tag: str,
) -> list:
    if duration_s <= 0.0:
        raise ValueError("duration must be > 0")
    samples = []
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        sample = device.read_sample(100)
        if sample is not None:
            samples.append(sample)
            recorder.append_sample(sample, test_phase=tag)
        recorder.append_cdc(device.drain_cdc())
    return samples


def capture_pendulum_pose(
    device: Rp2350Device,
    label: str,
    *,
    duration_s: float = 2.0,
) -> dict[str, object]:
    """Capture a manually held pendulum reference pose with the motor safely off."""
    device.safe_off()
    device.start_telemetry()
    with RunRecorder(f"pendulum-{label}") as recorder:
        recorder.write_metadata({"test": "pendulum-pose", "label": label, "duration_s": duration_s})
        samples = _capture_samples(device, recorder, duration_s, tag=label)
        if not samples:
            raise RuntimeError("no HID telemetry received")
        adc = [sample.pendulum_adc_raw for sample in samples]
        counts = [sample.arm_encoder_count for sample in samples]
        summary = {
            "test": "pendulum-pose",
            "label": label,
            "samples": len(samples),
            "adc_mean": statistics.fmean(adc),
            "adc_median": statistics.median(adc),
            "adc_stdev": statistics.pstdev(adc),
            "adc_min": min(adc),
            "adc_max": max(adc),
            "adc_peak_to_peak": max(adc) - min(adc),
            "arm_encoder_count_mean": statistics.fmean(counts),
            "artifact_dir": str(recorder.directory),
        }
        recorder.write_summary(summary)
        return summary


def capture_pendulum_sweep(
    device: Rp2350Device,
    *,
    duration_s: float = 8.0,
) -> dict[str, object]:
    """Record a slow manual full-angle sweep to expose ADC span and discontinuities."""
    device.safe_off()
    device.start_telemetry()
    with RunRecorder("pendulum-sweep") as recorder:
        recorder.write_metadata({"test": "pendulum-sweep", "duration_s": duration_s})
        samples = _capture_samples(device, recorder, duration_s, tag="manual-sweep")
        if not samples:
            raise RuntimeError("no HID telemetry received")
        adc = [sample.pendulum_adc_raw for sample in samples]
        steps = [abs(b - a) for a, b in zip(adc, adc[1:])]
        summary = {
            "test": "pendulum-sweep",
            "samples": len(samples),
            "adc_min": min(adc),
            "adc_max": max(adc),
            "adc_peak_to_peak": max(adc) - min(adc),
            "adc_fraction_of_12bit_span": (max(adc) - min(adc)) / 4095.0,
            "median_abs_step": statistics.median(steps) if steps else 0.0,
            "max_abs_step": max(steps) if steps else 0,
            "artifact_dir": str(recorder.directory),
        }
        recorder.write_summary(summary)
        return summary


def _bounded_position_segment(
    device: Rp2350Device,
    recorder: RunRecorder,
    *,
    origin_phi: float,
    target_fn,
    duration_s: float,
    tag: str,
    kp: float,
    kd: float,
    max_command: float,
    max_excursion_rad: float,
) -> list:
    samples = []
    deadline = time.monotonic() + duration_s
    start = time.monotonic()
    next_refresh = 0.0
    last_command = 0.0

    while time.monotonic() < deadline:
        now = time.monotonic()
        elapsed = now - start
        sample = device.read_sample(50)
        if sample is None:
            continue
        if abs(sample.phi - origin_phi) > max_excursion_rad:
            device.safe_off()
            raise RuntimeError(
                f"arm excursion exceeded {max_excursion_rad:.3f} rad during {tag}"
            )

        target = float(target_fn(elapsed))
        error = target - sample.phi
        requested = kp * error - kd * sample.phi_dot
        requested = max(-max_command, min(max_command, requested))
        if now >= next_refresh:
            last_command = device.set_motor_command(requested, lease_ms=250)
            next_refresh = now + 0.01

        samples.append(sample)
        recorder.append_sample(
            sample,
            test_phase=tag,
            elapsed_s=elapsed,
            position_target_rad=target,
            requested_command=requested,
            acknowledged_command=last_command,
        )
        recorder.append_cdc(device.drain_cdc())

    return samples


def bounded_mechanism_excitation(
    device: Rp2350Device,
    *,
    position_amplitude_rad: float = 0.15,
    max_command: float = 0.30,
    kp: float = 2.0,
    kd: float = 0.06,
    chirp_f0_hz: float = 0.2,
    chirp_f1_hz: float = 5.0,
    chirp_duration_s: float = 20.0,
    prbs_interval_s: float = 0.30,
    prbs_duration_s: float = 20.0,
    seed: int = 1,
) -> dict[str, object]:
    """Excite the installed mechanism while keeping arm position bounded around its start pose.

    The motor command remains a host-side maintenance command. The reference trajectory is
    position bounded, and an independent excursion guard safe-offs if the arm departs too far.
    """
    if position_amplitude_rad <= 0.0 or position_amplitude_rad > 0.25:
        raise ValueError("position_amplitude_rad must be in (0, 0.25]")
    if max_command <= 0.0 or max_command > 0.40:
        raise ValueError("max_command must be in (0, 0.40]")
    if chirp_f0_hz <= 0.0 or chirp_f1_hz < chirp_f0_hz:
        raise ValueError("invalid chirp frequencies")

    device.safe_off()
    device.start_telemetry()
    first = device.read_sample(500)
    if first is None:
        raise RuntimeError("no telemetry before mechanism excitation")
    origin = first.phi
    max_excursion = max(0.45, position_amplitude_rad * 2.5)

    with RunRecorder("mechanism-excitation") as recorder:
        recorder.write_metadata(
            {
                "test": "mechanism-excitation",
                "origin_phi_rad": origin,
                "position_amplitude_rad": position_amplitude_rad,
                "max_command": max_command,
                "kp": kp,
                "kd": kd,
                "max_excursion_rad": max_excursion,
                "chirp_f0_hz": chirp_f0_hz,
                "chirp_f1_hz": chirp_f1_hz,
                "chirp_duration_s": chirp_duration_s,
                "prbs_interval_s": prbs_interval_s,
                "prbs_duration_s": prbs_duration_s,
                "seed": seed,
            }
        )

        rng = random.Random(seed)
        prbs_values: dict[int, float] = {}
        ratio = chirp_f1_hz / chirp_f0_hz

        def chirp_target(t: float) -> float:
            if abs(ratio - 1.0) < 1.0e-12:
                phase = 2.0 * math.pi * chirp_f0_hz * t
            else:
                k = math.log(ratio) / chirp_duration_s
                phase = 2.0 * math.pi * chirp_f0_hz * (math.exp(k * t) - 1.0) / k
            return origin + position_amplitude_rad * math.sin(phase)

        def prbs_target(t: float) -> float:
            index = int(t / prbs_interval_s)
            if index not in prbs_values:
                prbs_values[index] = 1.0 if rng.getrandbits(1) else -1.0
            return origin + position_amplitude_rad * prbs_values[index]

        def constant(target: float):
            return lambda _t: target

        phase_summaries: list[dict[str, object]] = []
        try:
            phases = [
                ("hold-origin-pre", constant(origin), 1.0),
                ("step-positive", constant(origin + position_amplitude_rad), 1.5),
                ("step-negative", constant(origin - position_amplitude_rad), 2.0),
                ("step-return", constant(origin), 1.5),
                ("chirp", chirp_target, chirp_duration_s),
                ("hold-origin-mid", constant(origin), 1.0),
                ("prbs", prbs_target, prbs_duration_s),
                ("hold-origin-post", constant(origin), 1.0),
            ]
            for tag, target_fn, duration in phases:
                samples = _bounded_position_segment(
                    device,
                    recorder,
                    origin_phi=origin,
                    target_fn=target_fn,
                    duration_s=duration,
                    tag=tag,
                    kp=kp,
                    kd=kd,
                    max_command=max_command,
                    max_excursion_rad=max_excursion,
                )
                phase_summaries.append({"phase": tag, "samples": len(samples)})
        finally:
            device.safe_off()

        summary = {
            "test": "mechanism-excitation",
            "origin_phi_rad": origin,
            "position_amplitude_rad": position_amplitude_rad,
            "max_command": max_command,
            "max_excursion_rad": max_excursion,
            "phases": phase_summaries,
            "artifact_dir": str(recorder.directory),
        }
        recorder.write_summary(summary)
        return summary
