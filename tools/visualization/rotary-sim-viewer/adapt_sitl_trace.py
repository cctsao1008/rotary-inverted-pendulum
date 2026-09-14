#!/usr/bin/env python3
"""Normalize Rotary SITL JSONL evidence for the read-only simulation viewer.

This adapter does not change source evidence or compute dynamics. It only groups
existing SITL records by virtual time and projects selected fields into the
viewer schema used by `rotary-sim-viewer`.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

STATE_ORDER = ["theta", "theta_dot", "phi", "phi_dot"]


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.strip()
        if not line:
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError as error:
            raise ValueError(f"{path}:{line_number}: invalid JSONL record: {error}") from error
        if not isinstance(record, dict):
            raise ValueError(f"{path}:{line_number}: record must be an object")
        records.append(record)
    if not records:
        raise ValueError(f"{path}: no JSONL records")
    return records


def state_vector(state: dict[str, Any]) -> list[float]:
    keys = ("theta_rad", "theta_dot_rad_s", "phi_rad", "phi_dot_rad_s")
    try:
        values = [float(state[key]) for key in keys]
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"invalid true_plant_state: expected {keys}") from error
    return values


def merge_record(bucket: dict[str, Any], record: dict[str, Any]) -> None:
    system = record.get("system")
    if not isinstance(system, dict):
        return

    true_state = system.get("true_plant_state")
    if isinstance(true_state, dict):
        bucket["state"] = state_vector(true_state)

    estimated = system.get("estimated_state")
    if isinstance(estimated, dict):
        try:
            bucket["estimated_state"] = state_vector(estimated)
        except ValueError:
            pass

    regime = system.get("control_regime")
    if isinstance(regime, str):
        bucket["control_regime"] = regime

    demand = system.get("generalized_demand")
    if isinstance(demand, dict) and "arm_torque_nm" in demand:
        bucket["requested_arm_torque_nm"] = float(demand["arm_torque_nm"])

    actuator = system.get("virtual_physical_actuator")
    if isinstance(actuator, dict) and "applied_arm_torque_nm" in actuator:
        bucket["applied_arm_torque_nm"] = float(actuator["applied_arm_torque_nm"])

    authority = system.get("authority")
    if isinstance(authority, str):
        bucket["authority"] = authority

    supervisor = system.get("supervisor")
    if isinstance(supervisor, dict):
        runtime_state = supervisor.get("runtime_state")
        if isinstance(runtime_state, str):
            bucket["runtime_state"] = runtime_state


def normalize(records: list[dict[str, Any]], manifest: dict[str, Any] | None) -> dict[str, Any]:
    grouped: dict[int, dict[str, Any]] = {}
    last_regime = "—"
    last_requested = 0.0
    last_applied = 0.0
    last_authority = "unknown"
    last_runtime_state = "unknown"

    for record in records:
        if "virtual_time_us" not in record:
            continue
        t_us = int(record["virtual_time_us"])
        bucket = grouped.setdefault(t_us, {"t_us": t_us})
        merge_record(bucket, record)

    samples: list[dict[str, Any]] = []
    for t_us in sorted(grouped):
        bucket = grouped[t_us]
        if "state" not in bucket:
            continue

        last_regime = bucket.get("control_regime", last_regime)
        last_requested = bucket.get("requested_arm_torque_nm", last_requested)
        last_applied = bucket.get("applied_arm_torque_nm", last_applied)
        last_authority = bucket.get("authority", last_authority)
        last_runtime_state = bucket.get("runtime_state", last_runtime_state)

        sample = {
            "t_s": t_us * 1.0e-6,
            "state": bucket["state"],
            "arm_torque_nm": last_applied,
            "requested_arm_torque_nm": last_requested,
            "applied_arm_torque_nm": last_applied,
            "control_regime": last_regime,
            "authority": last_authority,
            "runtime_state": last_runtime_state,
        }
        if "estimated_state" in bucket:
            sample["estimated_state"] = bucket["estimated_state"]
        samples.append(sample)

    if not samples:
        raise ValueError("no SITL records contained true_plant_state")

    source: dict[str, Any] = {
        "kind": "rotary-sitl-evidence",
        "model_class": "source-backed reduced/equivalent QNET nonlinear model",
        "backend": "production Rust SITL semantic path",
        "provenance": "normalized from original SITL JSONL without modifying source evidence",
        "scope": "simulation evidence only; not Forest D1 specimen calibration and not physical actuator authority",
    }
    if manifest is not None:
        source["scenario"] = manifest.get("scenario")
        source["git_commit"] = manifest.get("git_commit")
        source["system_identifier"] = manifest.get("system_identifier")
        source["sitl_schema_version"] = manifest.get("schema_version")

    return {
        "schema": 1,
        "source": source,
        "state_order": STATE_ORDER,
        "samples": samples,
    }


def self_test() -> None:
    records = [
        {
            "virtual_time_us": 0,
            "system": {
                "true_plant_state": {
                    "theta_rad": 0.1,
                    "theta_dot_rad_s": 0.2,
                    "phi_rad": 0.3,
                    "phi_dot_rad_s": 0.4,
                },
                "control_regime": "Capture",
                "generalized_demand": {"arm_torque_nm": 0.01},
            },
        },
        {
            "virtual_time_us": 0,
            "system": {
                "true_plant_state": {
                    "theta_rad": 0.1,
                    "theta_dot_rad_s": 0.2,
                    "phi_rad": 0.3,
                    "phi_dot_rad_s": 0.4,
                },
                "virtual_physical_actuator": {"applied_arm_torque_nm": 0.008},
            },
        },
    ]
    result = normalize(records, None)
    sample = result["samples"][0]
    assert sample["state"] == [0.1, 0.2, 0.3, 0.4]
    assert sample["control_regime"] == "Capture"
    assert sample["requested_arm_torque_nm"] == 0.01
    assert sample["applied_arm_torque_nm"] == 0.008
    assert sample["arm_torque_nm"] == 0.008


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace", type=Path, help="SITL trace JSONL")
    parser.add_argument("--manifest", type=Path, help="optional SITL manifest JSON")
    parser.add_argument("--output", type=Path, help="viewer JSON output")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        self_test()
        print("adapt_sitl_trace self-test: PASS")
        return 0

    if args.trace is None or args.output is None:
        parser.error("--trace and --output are required unless --self-test is used")

    manifest = load_json(args.manifest) if args.manifest else None
    payload = normalize(load_jsonl(args.trace), manifest)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {len(payload['samples'])} viewer samples -> {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
