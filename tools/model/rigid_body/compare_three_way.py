#!/usr/bin/env python3
"""Characterize Rust/SciPy/PyBullet Furuta trace differences without pass tuning.

The SciPy path independently evaluates the project's declared nonlinear ODE,
while PyBullet solves the articulated rigid body. This tool deliberately reports
pairwise differences without inventing an acceptance threshold. A later policy
may add bounded expectations only after the structural differences are understood.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np

MODEL_DIR = Path(__file__).resolve().parents[1]
if str(MODEL_DIR) not in sys.path:
    sys.path.insert(0, str(MODEL_DIR))

from reference_furuta import load_fixture, simulate_reference  # noqa: E402

STATE_NAMES = ("theta", "theta_dot", "phi", "phi_dot")


def load_trace(path: Path, label: str) -> list[dict[str, Any]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    samples = payload.get("samples")
    if not isinstance(samples, list) or not samples:
        raise ValueError(f"{label} trace has no samples")
    return samples


def normalized_reference(samples: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [
        {
            "time_us": int(sample["time_us"]),
            "state": [float(value) for value in sample["state"]],
            "arm_torque_nm": float(sample["arm_torque_nm"]),
        }
        for sample in samples
    ]


def validate_common_grid(
    reference: list[dict[str, Any]],
    candidate: list[dict[str, Any]],
    label: str,
) -> None:
    if len(reference) != len(candidate):
        raise ValueError(f"{label} sample count {len(candidate)} != reference {len(reference)}")
    for expected, actual in zip(reference, candidate, strict=True):
        if int(actual["time_us"]) != int(expected["time_us"]):
            raise ValueError(f"{label} timestamp mismatch at {expected['time_us']} us")
        expected_torque = float(expected["arm_torque_nm"])
        actual_torque = float(actual["arm_torque_nm"])
        if not math.isfinite(actual_torque) or abs(actual_torque - expected_torque) > 1.0e-12:
            raise ValueError(f"{label} torque mismatch at {expected['time_us']} us")
        state = [float(value) for value in actual["state"]]
        if len(state) != 4 or not all(math.isfinite(value) for value in state):
            raise ValueError(f"{label} state is not four finite values at {expected['time_us']} us")


def pairwise_metrics(
    left: list[dict[str, Any]],
    right: list[dict[str, Any]],
) -> dict[str, Any]:
    left_states = np.asarray([sample["state"] for sample in left], dtype=np.float64)
    right_states = np.asarray([sample["state"] for sample in right], dtype=np.float64)
    error = left_states - right_states
    abs_error = np.abs(error)
    max_abs = np.max(abs_error, axis=0)
    rms = np.sqrt(np.mean(np.square(error), axis=0))
    final_abs = abs_error[-1]
    return {
        "sample_count": int(left_states.shape[0]),
        "overall_max_abs_error": float(np.max(max_abs)),
        "per_state": {
            name: {
                "max_abs_error": float(max_abs[index]),
                "rms_error": float(rms[index]),
                "final_abs_error": float(final_abs[index]),
            }
            for index, name in enumerate(STATE_NAMES)
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--rust-trace", type=Path, required=True)
    parser.add_argument("--pybullet-trace", type=Path, required=True)
    parser.add_argument("--summary", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    fixture = load_fixture(args.fixture)
    scipy_samples = normalized_reference(simulate_reference(fixture))
    rust_samples = load_trace(args.rust_trace, "rust")
    pybullet_samples = load_trace(args.pybullet_trace, "pybullet")

    validate_common_grid(scipy_samples, rust_samples, "rust")
    validate_common_grid(scipy_samples, pybullet_samples, "pybullet")

    summary = {
        "schema": 1,
        "state_order": list(STATE_NAMES),
        "interpretation": (
            "characterization only: Rust/SciPy agreement is numerical/equation-implementation evidence; "
            "PyBullet comparison is external rigid-body model-structure evidence; neither is specimen calibration"
        ),
        "acceptance_threshold": None,
        "pairs": {
            "rust_vs_scipy": pairwise_metrics(rust_samples, scipy_samples),
            "pybullet_vs_scipy": pairwise_metrics(pybullet_samples, scipy_samples),
            "rust_vs_pybullet": pairwise_metrics(rust_samples, pybullet_samples),
        },
    }
    rendered = json.dumps(summary, indent=2, sort_keys=True) + "\n"
    if args.summary is not None:
        args.summary.parent.mkdir(parents=True, exist_ok=True)
        args.summary.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
