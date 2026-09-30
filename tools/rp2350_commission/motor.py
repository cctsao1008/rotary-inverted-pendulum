from __future__ import annotations

import statistics
import time

from device import Rp2350Device
from recording import RunRecorder


def _run_command_segment(
    device: Rp2350Device,
    recorder: RunRecorder,
    command: float,
    duration_s: float,
    *,
    tag: str,
    refresh_s: float = 0.10,
) -> list:
    samples = []
    deadline = time.monotonic() + duration_s
    next_refresh = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_refresh:
            device.set_motor_command(command, lease_ms=250)
            next_refresh = now + refresh_s
        sample = device.read_sample(50)
        if sample is not None:
            samples.append(sample)
            recorder.append_sample(sample, test_phase=tag, requested_command=command)
        recorder.append_cdc(device.drain_cdc())
    return samples


def motor_command(
    device: Rp2350Device,
    *,
    command: float,
    duration_s: float = 1.0,
) -> dict[str, object]:
    if duration_s <= 0.0:
        raise ValueError("duration must be > 0")
    device.start_telemetry()
    with RunRecorder("motor") as recorder:
        recorder.write_metadata({"test": "motor", "command": command, "duration_s": duration_s})
        try:
            samples = _run_command_segment(device, recorder, command, duration_s, tag="motor")
        finally:
            device.safe_off()

        summary: dict[str, object] = {
            "test": "motor",
            "command": command,
            "duration_s": duration_s,
            "sample_count": len(samples),
            "artifact_dir": str(recorder.directory),
        }
        if samples:
            summary.update(
                {
                    "encoder_count_start": samples[0].arm_encoder_count,
                    "encoder_count_end": samples[-1].arm_encoder_count,
                    "encoder_count_delta": samples[-1].arm_encoder_count
                    - samples[0].arm_encoder_count,
                    "final_phi_rad": samples[-1].phi,
                    "final_phi_dot_rad_s": samples[-1].phi_dot,
                }
            )
        recorder.write_summary(summary)
        return summary


def motor_direction(
    device: Rp2350Device,
    *,
    command: float = 0.30,
    hold_s: float = 1.0,
) -> dict[str, object]:
    device.start_telemetry()
    with RunRecorder("motor-direction") as recorder:
        recorder.write_metadata({"test": "motor-direction", "command": command, "hold_s": hold_s})
        try:
            _run_command_segment(device, recorder, 0.0, 0.25, tag="zero-pre")
            positive = _run_command_segment(device, recorder, abs(command), hold_s, tag="positive")
            _run_command_segment(device, recorder, 0.0, 0.35, tag="zero-mid")
            negative = _run_command_segment(device, recorder, -abs(command), hold_s, tag="negative")
            _run_command_segment(device, recorder, 0.0, 0.25, tag="zero-post")
        finally:
            device.safe_off()

        if len(positive) < 2 or len(negative) < 2:
            raise RuntimeError("insufficient telemetry during direction test")
        pos_delta = positive[-1].arm_encoder_count - positive[0].arm_encoder_count
        neg_delta = negative[-1].arm_encoder_count - negative[0].arm_encoder_count
        summary = {
            "test": "motor-direction",
            "positive_count_delta": pos_delta,
            "negative_count_delta": neg_delta,
            "positive_command_increases_count": pos_delta > 0,
            "negative_command_decreases_count": neg_delta < 0,
            "artifact_dir": str(recorder.directory),
        }
        recorder.write_summary(summary)
        return summary


def speed_sweep(
    device: Rp2350Device,
    *,
    commands: list[float] | None = None,
    hold_s: float = 1.5,
) -> dict[str, object]:
    commands = commands or [
        0.10,
        0.15,
        0.18,
        0.20,
        0.22,
        0.25,
        0.30,
        0.40,
        0.50,
        -0.10,
        -0.15,
        -0.18,
        -0.20,
        -0.22,
        -0.25,
        -0.30,
        -0.40,
        -0.50,
    ]
    device.start_telemetry()
    results: list[dict[str, float]] = []
    with RunRecorder("speed-sweep") as recorder:
        recorder.write_metadata({"test": "speed-sweep", "commands": commands, "hold_s": hold_s})
        try:
            _run_command_segment(device, recorder, 0.0, 0.25, tag="zero-pre")
            for command in commands:
                samples = _run_command_segment(
                    device,
                    recorder,
                    command,
                    hold_s,
                    tag=f"command-{command:+.3f}",
                )
                steady = samples[len(samples) // 2 :] if samples else []
                if steady:
                    results.append(
                        {
                            "command": command,
                            "mean_phi_dot_rad_s": statistics.fmean(s.phi_dot for s in steady),
                            "stdev_phi_dot_rad_s": statistics.pstdev(s.phi_dot for s in steady),
                        }
                    )
                _run_command_segment(device, recorder, 0.0, 0.25, tag="zero-between")
        finally:
            device.safe_off()

        summary = {
            "test": "speed-sweep",
            "points": results,
            "artifact_dir": str(recorder.directory),
        }
        recorder.write_summary(summary)
        return summary


def position_step(
    device: Rp2350Device,
    *,
    delta_rad: float = 0.25,
    kp: float = 0.8,
    kd: float = 0.08,
    max_command: float = 0.25,
    settle_s: float = 2.0,
) -> dict[str, object]:
    device.start_telemetry()
    first = device.read_sample(500)
    if first is None:
        raise RuntimeError("no telemetry before position test")
    origin = first.phi
    targets = [origin + delta_rad, origin, origin - delta_rad, origin]

    with RunRecorder("position-step") as recorder:
        recorder.write_metadata(
            {
                "test": "position-step",
                "delta_rad": delta_rad,
                "kp": kp,
                "kd": kd,
                "max_command": max_command,
                "origin_rad": origin,
            }
        )
        target_results = []
        try:
            for index, target in enumerate(targets):
                deadline = time.monotonic() + settle_s
                last = None
                while time.monotonic() < deadline:
                    sample = device.read_sample(50)
                    if sample is None:
                        continue
                    error = target - sample.phi
                    command = max(
                        -max_command,
                        min(max_command, kp * error - kd * sample.phi_dot),
                    )
                    device.set_motor_command(command, lease_ms=250)
                    recorder.append_sample(
                        sample,
                        test_phase=f"target-{index}",
                        position_target_rad=target,
                        requested_command=command,
                    )
                    last = sample
                target_results.append(
                    {
                        "target_rad": target,
                        "final_phi_rad": last.phi if last else None,
                        "final_error_rad": target - last.phi if last else None,
                    }
                )
        finally:
            device.safe_off()

        summary = {
            "test": "position-step",
            "targets": target_results,
            "artifact_dir": str(recorder.directory),
        }
        recorder.write_summary(summary)
        return summary


def breakaway(
    device: Rp2350Device,
    *,
    step: float = 0.01,
    max_command: float = 0.30,
    hold_s: float = 0.50,
    min_counts: int = 4,
) -> dict[str, object]:
    """Ramp command and report first detectable motion in each direction.

    This is intentionally a detection threshold, not a claim of a repeatable
    static-friction constant. Commissioning showed strong start hysteresis and
    rotor/gear-position dependence below the continuous-running region.
    """
    if step <= 0.0 or max_command <= 0.0 or hold_s <= 0.0:
        raise ValueError("step, max_command, and hold_s must be > 0")
    if min_counts <= 0:
        raise ValueError("min_counts must be > 0")

    device.start_telemetry()
    points: list[dict[str, object]] = []

    with RunRecorder("breakaway") as recorder:
        recorder.write_metadata(
            {
                "test": "breakaway",
                "step": step,
                "max_command": max_command,
                "hold_s": hold_s,
                "min_counts": min_counts,
            }
        )

        def sweep(sign: float) -> dict[str, object] | None:
            level = step
            while level <= max_command + 1.0e-9:
                command = sign * level
                samples = _run_command_segment(
                    device,
                    recorder,
                    command,
                    hold_s,
                    tag=f"breakaway-{command:+.3f}",
                )
                if len(samples) >= 2:
                    count_delta = samples[-1].arm_encoder_count - samples[0].arm_encoder_count
                    mean_speed = statistics.fmean(s.phi_dot for s in samples)
                    point = {
                        "command": command,
                        "count_delta": count_delta,
                        "mean_phi_dot_rad_s": mean_speed,
                    }
                    points.append(point)
                    if abs(count_delta) >= min_counts:
                        return point
                level += step
            return None

        try:
            _run_command_segment(device, recorder, 0.0, 0.50, tag="zero-pre")
            positive = sweep(+1.0)
            _run_command_segment(device, recorder, 0.0, 0.75, tag="zero-mid")
            negative = sweep(-1.0)
            _run_command_segment(device, recorder, 0.0, 0.50, tag="zero-post")
        finally:
            device.safe_off()

        summary = {
            "test": "breakaway",
            "positive": positive,
            "negative": negative,
            "points": points,
            "artifact_dir": str(recorder.directory),
        }
        recorder.write_summary(summary)
        return summary


def coast_down(
    device: Rp2350Device,
    *,
    command: float = 0.40,
    runup_s: float = 2.0,
    coast_s: float = 5.0,
) -> dict[str, object]:
    """Drive the arm, then command zero and record the mechanical coast-down trace."""
    if command == 0.0:
        raise ValueError("command must be non-zero")
    if runup_s <= 0.0 or coast_s <= 0.0:
        raise ValueError("runup_s and coast_s must be > 0")

    device.start_telemetry()
    with RunRecorder("coast-down") as recorder:
        recorder.write_metadata(
            {
                "test": "coast-down",
                "command": command,
                "runup_s": runup_s,
                "coast_s": coast_s,
            }
        )
        try:
            _run_command_segment(device, recorder, 0.0, 0.25, tag="zero-pre")
            runup = _run_command_segment(device, recorder, command, runup_s, tag="runup")
            coast = _run_command_segment(device, recorder, 0.0, coast_s, tag="coast")
        finally:
            device.safe_off()

        if not runup or not coast:
            raise RuntimeError("insufficient telemetry during coast-down test")

        runup_tail = runup[len(runup) // 2 :]
        summary = {
            "test": "coast-down",
            "command": command,
            "runup_mean_phi_dot_rad_s": statistics.fmean(s.phi_dot for s in runup_tail),
            "coast_start_phi_dot_rad_s": coast[0].phi_dot,
            "coast_end_phi_dot_rad_s": coast[-1].phi_dot,
            "coast_peak_abs_phi_dot_rad_s": max(abs(s.phi_dot) for s in coast),
            "coast_duration_s": coast_s,
            "artifact_dir": str(recorder.directory),
        }
        recorder.write_summary(summary)
        return summary
