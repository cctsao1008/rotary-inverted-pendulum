#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import sys
import time

_TOOL_DIR = Path(__file__).resolve().parent / "rp2350_commission"
sys.path.insert(0, str(_TOOL_DIR))

from device import Rp2350Device  # noqa: E402
from recording import RunRecorder  # noqa: E402


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run one-shot encoder-only RP2350 arm velocity/position servo commissioning"
    )
    parser.add_argument("--cdc-port", help="override auto-detected CDC COM/tty port")
    parser.add_argument("--hid-path", help="override auto-detected hidapi path")
    parser.add_argument("--speed-max-command", type=float, default=0.50)
    parser.add_argument("--position-max-command", type=float, default=0.35)
    return parser


def _capture(device: Rp2350Device, recorder: RunRecorder, duration_s: float, **extra) -> list:
    samples = []
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        sample = device.read_sample(100)
        if sample is not None:
            samples.append(sample)
            recorder.append_sample(sample, **extra)
        recorder.append_cdc(device.drain_cdc())
    return samples


def _encoder_slope(samples: list) -> float | None:
    if len(samples) < 2:
        return None
    first = samples[0]
    last = samples[-1]
    dt = (last.timestamp_us - first.timestamp_us) * 1.0e-6
    if dt <= 0.0:
        return None
    return (last.arm_encoder_count - first.arm_encoder_count) * (2.0 * math.pi / 1040.0) / dt


def _steady(samples: list) -> list:
    if not samples:
        return []
    start = max(0, int(len(samples) * 0.60))
    return samples[start:]


def _speed_summary(target: float, samples: list) -> dict[str, object]:
    steady = _steady(samples)
    slope = _encoder_slope(steady)
    mean_estimator = statistics.fmean(s.phi_dot for s in steady) if steady else None
    mean_command = statistics.fmean(s.normalized_command for s in steady) if steady else None
    result: dict[str, object] = {
        "target_rad_s": target,
        "samples": len(samples),
        "steady_encoder_slope_rad_s": slope,
        "steady_firmware_phi_dot_rad_s": mean_estimator,
        "steady_mean_command": mean_command,
    }
    if slope is not None:
        result["steady_error_rad_s"] = slope - target
        result["steady_abs_error_rad_s"] = abs(slope - target)
    return result


def _position_summary(target: float, samples: list) -> dict[str, object]:
    steady = _steady(samples)
    final = samples[-1] if samples else None
    errors = [target - s.phi for s in steady]
    return {
        "target_rad": target,
        "samples": len(samples),
        "final_phi_rad": final.phi if final is not None else None,
        "final_error_rad": target - final.phi if final is not None else None,
        "steady_error_rms_rad": (
            math.sqrt(statistics.fmean(error * error for error in errors)) if errors else None
        ),
        "peak_abs_command": max((abs(s.normalized_command) for s in samples), default=0.0),
        "final_phi_dot_rad_s": final.phi_dot if final is not None else None,
    }


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if not 0.0 < args.speed_max_command <= 0.60:
        raise SystemExit("--speed-max-command must be in (0, 0.60]")
    if not 0.0 < args.position_max_command <= 0.60:
        raise SystemExit("--position-max-command must be in (0, 0.60]")

    plan = {
        "speed_targets_rad_s": [4.0, 8.0, 12.0, 16.0, 0.0, -4.0, -8.0, -12.0, -16.0, 0.0, 12.0, -12.0, 0.0],
        "speed_stage_duration_s": 1.4,
        "position_offsets_rad": [0.20, -0.20, 0.40, -0.40, 0.0],
        "position_stage_duration_s": 1.8,
        "speed_max_command": args.speed_max_command,
        "position_max_command": args.position_max_command,
    }

    result: dict[str, object] = {"test": "arm-servo-suite", "plan": plan}

    print(
        "Encoder-only arm-servo commissioning\n"
        "- pendulum ADC is not used by this test\n"
        "- clear the rotary-arm sweep envelope\n"
        "- speed control and bounded position control run in the RP2350 1 kHz loop\n"
        "- SAFE_OFF is requested on exit"
    )

    all_samples = []
    with Rp2350Device(hid_path=args.hid_path, cdc_port=args.cdc_port) as device:
        device.safe_off()
        device.start_telemetry()
        with RunRecorder("arm-servo-suite") as recorder:
            recorder.write_metadata(result)
            pre = device.status()
            result["pre_status"] = pre
            initial_illegal = int(pre.get("encoder_illegal_transitions", 0))

            samples = _capture(device, recorder, 0.4, phase="prime", control="off", target=0.0)
            all_samples.extend(samples)

            speed_results = []
            for index, target in enumerate(plan["speed_targets_rad_s"]):
                print(f"[speed {target:+.1f} rad/s]", flush=True)
                lease_ms = int(plan["speed_stage_duration_s"] * 1000.0) + 400
                device.set_arm_velocity(
                    float(target),
                    max_command=args.speed_max_command,
                    lease_ms=lease_ms,
                )
                samples = _capture(
                    device,
                    recorder,
                    float(plan["speed_stage_duration_s"]),
                    phase=f"speed-{index:02d}",
                    control="velocity",
                    target=float(target),
                )
                all_samples.extend(samples)
                speed_results.append(_speed_summary(float(target), samples))

            device.safe_off()
            settle = _capture(device, recorder, 0.8, phase="position-settle", control="off", target=0.0)
            all_samples.extend(settle)
            if not settle:
                raise RuntimeError("no telemetry before position sequence")
            origin = settle[-1].phi

            position_results = []
            for index, offset in enumerate(plan["position_offsets_rad"]):
                target = origin + float(offset)
                print(f"[position {offset:+.2f} rad from origin]", flush=True)
                lease_ms = int(plan["position_stage_duration_s"] * 1000.0) + 400
                device.set_arm_position(
                    target,
                    max_command=args.position_max_command,
                    lease_ms=lease_ms,
                )
                samples = _capture(
                    device,
                    recorder,
                    float(plan["position_stage_duration_s"]),
                    phase=f"position-{index:02d}",
                    control="position",
                    target=target,
                    origin=origin,
                    offset=float(offset),
                )
                all_samples.extend(samples)
                position_results.append(_position_summary(target, samples))

            device.safe_off()
            samples = _capture(device, recorder, 0.5, phase="post-safe-off", control="off", target=0.0)
            all_samples.extend(samples)
            post = device.status()
            result["post_status"] = post
            result["speed"] = speed_results
            result["position"] = position_results
            result["position_origin_rad"] = origin
            result["encoder_illegal_transition_delta"] = (
                int(post.get("encoder_illegal_transitions", 0)) - initial_illegal
            )
            result["runtime_timing"] = {
                "samples": len(all_samples),
                "missed_opportunities_max": max((s.missed_opportunities for s in all_samples), default=0),
                "deadline_overruns_max": max((s.deadline_overruns for s in all_samples), default=0),
                "execution_time_us_max": max((s.execution_time_us for s in all_samples), default=0),
                "execution_time_us_mean": (
                    statistics.fmean(s.execution_time_us for s in all_samples) if all_samples else None
                ),
            }
            result["artifact_dir"] = str(recorder.directory)
            recorder.write_summary(result)

    print(json.dumps(result, indent=2, default=str))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("interrupted; device context requests SAFE_OFF while closing", file=sys.stderr)
        raise SystemExit(130)
