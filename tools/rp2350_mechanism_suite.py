#!/usr/bin/env python3
from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import time

_REPO_ROOT = Path(__file__).resolve().parent.parent
_TOOL_DIR = Path(__file__).resolve().parent / "rp2350_commission"
sys.path.insert(0, str(_TOOL_DIR))

from device import Rp2350Device  # noqa: E402
import mechanism  # noqa: E402
from mechanism_analysis import analyze_mechanism_suite  # noqa: E402
import sensors  # noqa: E402


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run the integrated installed-mechanism commissioning sequence in one invocation"
        )
    )
    parser.add_argument("--cdc-port", help="override auto-detected CDC COM/tty port")
    parser.add_argument("--hid-path", help="override auto-detected hidapi path")
    parser.add_argument("--pose-duration", type=float, default=2.0)
    parser.add_argument("--sweep-duration", type=float, default=8.0)
    parser.add_argument("--free-swing-duration", type=float, default=12.0)
    parser.add_argument("--position-amplitude", type=float, default=0.15)
    parser.add_argument("--max-command", type=float, default=0.30)
    parser.add_argument("--skip-excitation", action="store_true")
    return parser


def _prompt(text: str) -> None:
    input(f"\n{text}\nPress Enter to continue... ")


def _countdown(seconds: int, final: str) -> None:
    for value in range(seconds, 0, -1):
        print(f"  {value}...", flush=True)
        time.sleep(1.0)
    print(final, flush=True)


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    suite_dir = (
        _REPO_ROOT
        / "artifacts"
        / "commissioning"
        / f"{datetime.now().strftime('%Y%m%d-%H%M%S')}-mechanism-suite"
    )
    suite_dir.mkdir(parents=True, exist_ok=True)

    results: dict[str, object] = {
        "test": "mechanism-suite",
        "plan": {
            "pose_duration_s": args.pose_duration,
            "sweep_duration_s": args.sweep_duration,
            "free_swing_duration_s": args.free_swing_duration,
            "position_amplitude_rad": args.position_amplitude,
            "max_command": args.max_command,
            "bounded_chirp_hz": [0.2, 5.0],
            "bounded_chirp_duration_s": 20.0,
            "bounded_prbs_interval_s": 0.30,
            "bounded_prbs_duration_s": 20.0,
        },
        "sections": {},
        "artifact_dir": str(suite_dir),
    }

    print(
        "Installed-mechanism commissioning\n"
        "- clamp the base securely\n"
        "- connect the 12 V motor supply and pendulum angle sensor\n"
        "- fit the pendulum protection sleeve / clear the swing envelope\n"
        "- this run never requests closed-loop balance or swing-up\n"
        "- motor excitation is position-bounded around the starting arm pose"
    )
    _prompt("Prepare the mechanism with the pendulum hanging freely downward and motionless.")

    with Rp2350Device(hid_path=args.hid_path, cdc_port=args.cdc_port) as device:
        sections: dict[str, object] = results["sections"]  # type: ignore[assignment]

        def run(name: str, fn) -> object:
            print(f"[{name}]", file=sys.stderr, flush=True)
            try:
                value = fn()
                sections[name] = value
                return value
            except Exception as exc:
                try:
                    device.safe_off()
                except Exception:
                    pass
                value = {"error": str(exc)}
                sections[name] = value
                return value

        run("pre_status", lambda: {"version": device.version(), "status": device.status()})

        # The natural hanging pose needs no operator support, so capture it first.
        run(
            "pendulum_down",
            lambda: mechanism.capture_pendulum_pose(
                device, "down", duration_s=args.pose_duration
            ),
        )

        _prompt("Hold the pendulum accurately upright. Keep it still during the short capture.")
        run(
            "pendulum_upright",
            lambda: mechanism.capture_pendulum_pose(
                device, "upright", duration_s=args.pose_duration
            ),
        )

        _prompt(
            "Return the pendulum downward. During the next capture, slowly rotate the pendulum "
            "through one complete 360-degree revolution so the potentiometer span is observed."
        )
        _countdown(3, "GO: perform one slow full pendulum revolution now.")
        run(
            "pendulum_sweep",
            lambda: mechanism.capture_pendulum_sweep(
                device, duration_s=args.sweep_duration
            ),
        )

        calibration_preview = analyze_mechanism_suite(sections)
        calibration = calibration_preview.get("pendulum_calibration", {})
        calibration_ok = bool(calibration_preview.get("valid", False))
        print(
            "Calibration preview: "
            f"valid={calibration_ok}, "
            f"down={calibration.get('down_adc_mean')}, "
            f"upright={calibration.get('upright_adc_mean')}, "
            f"sweep_pp={calibration.get('sweep_adc_peak_to_peak')}",
            flush=True,
        )
        if not calibration_ok:
            device.safe_off()
            sections["free_swing"] = {
                "skipped": True,
                "reason": "pendulum calibration did not pass basic span checks",
            }
            sections["bounded_excitation"] = {
                "skipped": True,
                "reason": "pendulum calibration did not pass basic span checks",
            }
        else:
            _prompt(
                "Displace the hanging pendulum about 20-30 degrees and HOLD it there. "
                "The script will count down; release exactly at GO and do not touch the mechanism."
            )
            _countdown(3, "GO: release the pendulum.")
            run(
                "free_swing",
                lambda: sensors.free_swing(device, args.free_swing_duration),
            )

            if args.skip_excitation:
                sections["bounded_excitation"] = {
                    "skipped": True,
                    "reason": "--skip-excitation requested",
                }
            else:
                _prompt(
                    "Let the pendulum settle hanging downward. Make sure the arm and pendulum "
                    "swing envelopes are clear. The remaining bounded step/chirp/PRBS phases "
                    "will run automatically without further prompts."
                )
                run(
                    "bounded_excitation",
                    lambda: mechanism.bounded_mechanism_excitation(
                        device,
                        position_amplitude_rad=args.position_amplitude,
                        max_command=args.max_command,
                    ),
                )

        try:
            device.safe_off()
        except Exception:
            pass
        run("post_status", lambda: {"version": device.version(), "status": device.status()})

    print("[analysis]", file=sys.stderr, flush=True)
    try:
        results["analysis"] = analyze_mechanism_suite(
            results["sections"]  # type: ignore[arg-type]
        )
    except Exception as exc:
        results["analysis"] = {"error": str(exc)}

    output = json.dumps(results, indent=2, default=str)
    (suite_dir / "summary.json").write_text(output + "\n", encoding="utf-8")
    print(output)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("interrupted; device context requests safe-off while closing", file=sys.stderr)
        raise SystemExit(130)
