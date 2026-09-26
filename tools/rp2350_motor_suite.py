#!/usr/bin/env python3
from __future__ import annotations

import argparse
from datetime import datetime
import json
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
_TOOL_DIR = Path(__file__).resolve().parent / "rp2350_commission"
sys.path.insert(0, str(_TOOL_DIR))

from device import Rp2350Device  # noqa: E402
import motor  # noqa: E402
import sysid  # noqa: E402


_SPEED_COMMANDS = [
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


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the full RP2350 motor/encoder characterization suite in one invocation"
    )
    parser.add_argument("--cdc-port", help="override auto-detected CDC COM/tty port")
    parser.add_argument("--hid-path", help="override auto-detected hidapi path")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    suite_dir = (
        _REPO_ROOT
        / "artifacts"
        / "commissioning"
        / f"{datetime.now().strftime('%Y%m%d-%H%M%S')}-motor-suite"
    )
    suite_dir.mkdir(parents=True, exist_ok=True)

    results: dict[str, object] = {
        "test": "motor-suite",
        "plan": {
            "direction_command": 0.30,
            "breakaway_step": 0.01,
            "breakaway_max_command": 0.30,
            "speed_commands": _SPEED_COMMANDS,
            "coast_commands": [0.40, -0.40],
            "step_amplitude": 0.30,
            "chirp": {"amplitude": 0.30, "f0_hz": 0.2, "f1_hz": 8.0, "duration_s": 20.0},
            "prbs": {"amplitude": 0.30, "interval_s": 0.20, "duration_s": 20.0, "seed": 1},
        },
        "sections": {},
        "artifact_dir": str(suite_dir),
    }

    with Rp2350Device(hid_path=args.hid_path, cdc_port=args.cdc_port) as device:
        sections: dict[str, object] = results["sections"]  # type: ignore[assignment]

        def run(name: str, fn) -> None:
            print(f"[{name}]", file=sys.stderr, flush=True)
            try:
                sections[name] = fn()
            except Exception as exc:
                try:
                    device.safe_off()
                except Exception:
                    pass
                sections[name] = {"error": str(exc)}

        run("pre_status", lambda: {"version": device.version(), "status": device.status()})
        run(
            "motor_direction",
            lambda: motor.motor_direction(device, command=0.30, hold_s=1.0),
        )
        run(
            "breakaway",
            lambda: motor.breakaway(
                device,
                step=0.01,
                max_command=0.30,
                hold_s=0.50,
                min_counts=4,
            ),
        )
        run(
            "speed_sweep",
            lambda: motor.speed_sweep(device, commands=_SPEED_COMMANDS, hold_s=1.5),
        )
        run(
            "coast_down_positive",
            lambda: motor.coast_down(device, command=0.40, runup_s=2.0, coast_s=5.0),
        )
        run(
            "coast_down_negative",
            lambda: motor.coast_down(device, command=-0.40, runup_s=2.0, coast_s=5.0),
        )
        run(
            "step_response",
            lambda: sysid.step_response(device, amplitude=0.30, hold_s=1.5),
        )
        run(
            "chirp",
            lambda: sysid.chirp(
                device,
                amplitude=0.30,
                f0_hz=0.2,
                f1_hz=8.0,
                duration_s=20.0,
            ),
        )
        run(
            "prbs",
            lambda: sysid.prbs(
                device,
                amplitude=0.30,
                interval_s=0.20,
                duration_s=20.0,
                seed=1,
            ),
        )
        run("post_status", lambda: {"version": device.version(), "status": device.status()})

        try:
            device.safe_off()
        except Exception:
            pass

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
