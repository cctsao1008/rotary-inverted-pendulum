from __future__ import annotations

import math
import random
import time

from device import Rp2350Device
from motor import confirm_active_test
from recording import RunRecorder


def _stream_command(
    device: Rp2350Device,
    recorder: RunRecorder,
    command_fn,
    duration_s: float,
    *,
    tag: str,
    update_hz: float = 50.0,
) -> None:
    period = 1.0 / update_hz
    start = time.monotonic()
    next_update = start
    last_command = 0.0
    while True:
        now = time.monotonic()
        elapsed = now - start
        if elapsed >= duration_s:
            break
        if now >= next_update:
            last_command = float(command_fn(elapsed))
            device.set_motor_command(last_command, lease_ms=250)
            next_update += period
        sample = device.read_sample(20)
        if sample is not None:
            recorder.append_sample(
                sample,
                test_phase=tag,
                elapsed_s=elapsed,
                requested_command=last_command,
            )
        recorder.append_cdc(device.drain_cdc())


def step_response(
    device: Rp2350Device,
    *,
    amplitude: float = 0.10,
    hold_s: float = 1.5,
    assume_yes: bool = False,
) -> dict[str, object]:
    confirm_active_test("step-response", abs(amplitude), assume_yes)
    device.start_telemetry()
    with RunRecorder("step-response") as recorder:
        recorder.write_metadata({"test": "step-response", "amplitude": amplitude, "hold_s": hold_s})
        try:
            _stream_command(device, recorder, lambda _: 0.0, 0.5, tag="zero-pre")
            _stream_command(device, recorder, lambda _: amplitude, hold_s, tag="positive-step")
            _stream_command(device, recorder, lambda _: 0.0, 0.7, tag="zero-mid")
            _stream_command(device, recorder, lambda _: -amplitude, hold_s, tag="negative-step")
            _stream_command(device, recorder, lambda _: 0.0, 0.5, tag="zero-post")
        finally:
            device.safe_off()
        summary = {"test": "step-response", "amplitude": amplitude, "artifact_dir": str(recorder.directory)}
        recorder.write_summary(summary)
        return summary


def chirp(
    device: Rp2350Device,
    *,
    amplitude: float = 0.10,
    f0_hz: float = 0.2,
    f1_hz: float = 8.0,
    duration_s: float = 20.0,
    assume_yes: bool = False,
) -> dict[str, object]:
    confirm_active_test("chirp", abs(amplitude), assume_yes)
    device.start_telemetry()
    ratio = f1_hz / f0_hz

    def command_at(t: float) -> float:
        if abs(ratio - 1.0) < 1.0e-9:
            phase = 2.0 * math.pi * f0_hz * t
        else:
            k = math.log(ratio) / duration_s
            phase = 2.0 * math.pi * f0_hz * (math.exp(k * t) - 1.0) / k
        return amplitude * math.sin(phase)

    with RunRecorder("chirp") as recorder:
        recorder.write_metadata({"test": "chirp", "amplitude": amplitude, "f0_hz": f0_hz, "f1_hz": f1_hz, "duration_s": duration_s})
        try:
            _stream_command(device, recorder, lambda _: 0.0, 0.5, tag="zero-pre")
            _stream_command(device, recorder, command_at, duration_s, tag="chirp")
            _stream_command(device, recorder, lambda _: 0.0, 0.5, tag="zero-post")
        finally:
            device.safe_off()
        summary = {"test": "chirp", "artifact_dir": str(recorder.directory)}
        recorder.write_summary(summary)
        return summary


def prbs(
    device: Rp2350Device,
    *,
    amplitude: float = 0.10,
    interval_s: float = 0.20,
    duration_s: float = 20.0,
    seed: int = 1,
    assume_yes: bool = False,
) -> dict[str, object]:
    confirm_active_test("prbs", abs(amplitude), assume_yes)
    device.start_telemetry()
    rng = random.Random(seed)
    values: dict[int, float] = {}

    def command_at(t: float) -> float:
        index = int(t / interval_s)
        if index not in values:
            values[index] = amplitude if rng.getrandbits(1) else -amplitude
        return values[index]

    with RunRecorder("prbs") as recorder:
        recorder.write_metadata({"test": "prbs", "amplitude": amplitude, "interval_s": interval_s, "duration_s": duration_s, "seed": seed})
        try:
            _stream_command(device, recorder, lambda _: 0.0, 0.5, tag="zero-pre")
            _stream_command(device, recorder, command_at, duration_s, tag="prbs")
            _stream_command(device, recorder, lambda _: 0.0, 0.5, tag="zero-post")
        finally:
            device.safe_off()
        summary = {"test": "prbs", "artifact_dir": str(recorder.directory)}
        recorder.write_summary(summary)
        return summary
