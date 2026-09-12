#!/usr/bin/env python3
"""Check the nominal local-balance scenario as a controller baseline.

This gate intentionally reuses the production semantic-path SITL artifacts and
Control-owned policy constants rather than inventing a second set of acceptance
numbers. It proves only that the reference-backed reduced nominal plant is
locally stabilized through the current estimator/controller/actuator/authority
semantics. It is not Forest D1 specimen validation and grants no physical motor
authority.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import tomllib
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
ROTARY_SOURCE = ROOT / "tools" / "sitl" / "src" / "rotary.rs"


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def source_float_degrees(name: str, source: str) -> float:
    match = re.search(
        rf"const\s+{re.escape(name)}:\s*f32\s*=\s*([0-9.]+)\s*\*\s*PI\s*/\s*180\.0\s*;",
        source,
    )
    if match is None:
        raise ValueError(f"cannot locate degree-defined {name} in rotary.rs")
    return math.radians(float(match.group(1)))


def source_float(name: str, source: str) -> float:
    match = re.search(
        rf"const\s+{re.escape(name)}:\s*f32\s*=\s*([0-9.]+)\s*;",
        source,
    )
    if match is None:
        raise ValueError(f"cannot locate {name} in rotary.rs")
    return float(match.group(1))


def source_u16(name: str, source: str) -> int:
    match = re.search(
        rf"const\s+{re.escape(name)}:\s*u16\s*=\s*([0-9]+)\s*;",
        source,
    )
    if match is None:
        raise ValueError(f"cannot locate {name} in rotary.rs")
    return int(match.group(1))


def load_trace(path: Path) -> list[dict[str, Any]]:
    records = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        try:
            records.append(json.loads(line))
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid JSONL at line {line_number}: {exc}") from exc
    return records


def state_within(state: dict[str, Any], *, angle_rad: float, rate_rad_s: float) -> bool:
    theta = float(state["theta_rad"])
    theta_dot = float(state["theta_dot_rad_s"])
    return abs(theta) <= angle_rad and abs(theta_dot) <= rate_rad_s


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--trace", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    scenario = tomllib.loads(args.scenario.read_text(encoding="utf-8"))
    summary = load_json(args.summary)
    trace = load_trace(args.trace)
    source = ROTARY_SOURCE.read_text(encoding="utf-8")

    balance_enter_angle = source_float_degrees("BALANCE_ENTER_ANGLE_RAD", source)
    balance_enter_rate = source_float("BALANCE_ENTER_RATE_RAD_S", source)
    balance_exit_angle = source_float_degrees("BALANCE_EXIT_ANGLE_RAD", source)
    balance_exit_rate = source_float("BALANCE_EXIT_RATE_RAD_S", source)
    settle_cycles = source_u16("CAPTURE_SETTLE_CYCLES", source)
    max_abs_torque = source_float("MAX_ABS_TORQUE_NM", source)

    if scenario.get("id") != "rotary-balance":
        raise ValueError("this checker is only for the rotary-balance baseline scenario")
    if summary.get("scenario") != scenario["id"]:
        raise ValueError("summary/scenario identity mismatch")

    runtime_records = [
        record
        for record in trace
        if record.get("record_kind") == "production_runtime" and isinstance(record.get("system"), dict)
    ]
    if len(runtime_records) < settle_cycles:
        raise ValueError("trace is shorter than the Control settle-cycle contract")
    settled_window = runtime_records[-settle_cycles:]

    system = summary["system"]
    final_state = system["final_state"]
    final_window_in_balance_regime = all(
        record["system"].get("control_regime") == "balance" for record in settled_window
    )
    final_window_inside_balance_entry = all(
        state_within(
            record["system"]["true_plant_state"],
            angle_rad=balance_enter_angle,
            rate_rad_s=balance_enter_rate,
        )
        for record in settled_window
    )

    checks = {
        "scheduler_semantics_pass": summary.get("pass") is True,
        "balance_regime_was_reached": system.get("first_balance_us") is not None,
        "closed_loop_actuation_was_authorized": int(system["authorized_cycles"]) > 0,
        "local_baseline_did_not_saturate": int(system["saturated_cycles"]) == 0,
        "pendulum_never_left_existing_balance_exit_angle": float(system["max_abs_theta_rad"])
        <= balance_exit_angle,
        "final_state_is_inside_existing_balance_entry_window": state_within(
            final_state,
            angle_rad=balance_enter_angle,
            rate_rad_s=balance_enter_rate,
        ),
        "final_settle_window_remains_in_balance_regime": final_window_in_balance_regime,
        "final_settle_window_remains_inside_balance_entry_window": final_window_inside_balance_entry,
        "applied_torque_respected_control_limit": float(system["max_abs_torque_nm"])
        <= max_abs_torque + 1.0e-7,
    }

    payload = {
        "schema": 1,
        "scenario": scenario["id"],
        "model_class": "source-backed reduced QNET nonlinear plant",
        "semantic_path": "synthetic sensors -> production measurement/estimator -> hybrid control -> actuator model -> authority -> TB6612 mapping -> virtual physical actuator",
        "policy_reused_from_control_source": {
            "balance_enter_angle_rad": balance_enter_angle,
            "balance_enter_rate_rad_s": balance_enter_rate,
            "balance_exit_angle_rad": balance_exit_angle,
            "balance_exit_rate_rad_s": balance_exit_rate,
            "capture_settle_cycles": settle_cycles,
            "max_abs_torque_nm": max_abs_torque,
        },
        "observed": {
            "first_balance_us": system.get("first_balance_us"),
            "computed_cycles": int(system["computed_cycles"]),
            "authorized_cycles": int(system["authorized_cycles"]),
            "saturated_cycles": int(system["saturated_cycles"]),
            "regime_transitions": int(system["regime_transitions"]),
            "max_abs_theta_rad": float(system["max_abs_theta_rad"]),
            "max_abs_phi_rad": float(system["max_abs_phi_rad"]),
            "max_abs_torque_nm": float(system["max_abs_torque_nm"]),
            "final_state": final_state,
            "settled_window_cycles": settle_cycles,
        },
        "checks": checks,
        "scope": "nominal reduced-model local-controller baseline only; not installed-specimen validation or physical authority",
        "pass": all(checks.values()),
    }
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if payload["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
