from __future__ import annotations

import statistics
import time

from device import Rp2350Device
from recording import RunRecorder


def _metadata(device: Rp2350Device, test: str) -> dict[str, object]:
    return {
        "test": test,
        "hid_vid": "0xCAFE",
        "hid_pid": "0x4010",
        "cdc_port": device.cdc.port if device.cdc_available else None,
        "firmware_version": device.version(),
    }


def monitor(device: Rp2350Device, duration_s: float = 10.0) -> dict[str, object]:
    device.start_telemetry()
    count = 0
    last = None
    for sample in device.samples(duration_s):
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
    return {"test": "monitor", "samples": count, "last": last.as_dict() if last else None}


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
    """Capture Encoder1 A/B/count, optionally while directly driving the arm."""
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
