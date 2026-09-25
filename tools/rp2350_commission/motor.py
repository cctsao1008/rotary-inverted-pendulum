from __future__ import annotations

import statistics
import time

from device import Rp2350Device
from recording import RunRecorder


def confirm_active_test(name: str, max_command: float, assume_yes: bool) -> None:
    if assume_yes:
        return
    answer = input(
        f"{name}: this test requests motor output up to {max_command:.2f}. "
        "Keep the mechanism clear and the pendulum free. Continue? [y/N] "
    ).strip().lower()
    if answer not in {"y", "yes"}:
        raise RuntimeError("test cancelled")


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


def motor_direction(
    device: Rp2350Device,
    *,
    command: float = 0.10,
    hold_s: float = 1.0,
    assume_yes: bool = False,
) -> dict[str, object]:
    confirm_active_test("motor-direction", abs(command), assume_yes)
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
    assume_yes: bool = False,
) -> dict[str, object]:
    commands = commands or [0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.50,
                            -0.05, -0.10, -0.15, -0.20, -0.30, -0.40, -0.50]
    max_command = max(abs(x) for x in commands)
    confirm_active_test("speed-sweep", max_command, assume_yes)
    device.start_telemetry()
    results: list[dict[str, float]] = []
    with RunRecorder("speed-sweep") as recorder:
        recorder.write_metadata({"test": "speed-sweep", "commands": commands, "hold_s": hold_s})
        try:
            _run_command_segment(device, recorder, 0.0, 0.25, tag="zero-pre")
            for command in commands:
                samples = _run_command_segment(
                    device, recorder, command, hold_s, tag=f"command-{command:+.3f}"
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
    assume_yes: bool = False,
) -> dict[str, object]:
    confirm_active_test("position-step", max_command, assume_yes)
    device.start_telemetry()
    first = device.read_sample(500)
    if first is None:
        raise RuntimeError("no telemetry before position test")
    origin = first.phi
    targets = [origin + delta_rad, origin, origin - delta_rad, origin]

    with RunRecorder("position-step") as recorder:
        recorder.write_metadata(
            {"test": "position-step", "delta_rad": delta_rad, "kp": kp, "kd": kd,
             "max_command": max_command, "origin_rad": origin}
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
                    command = max(-max_command, min(max_command, kp * error - kd * sample.phi_dot))
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
