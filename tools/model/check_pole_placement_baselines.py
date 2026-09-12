#!/usr/bin/env python3
"""Derive and audit nominal QNET pole-placement state-feedback baselines.

Fahmizal (2023) supplies two desired closed-loop pole sets, C1 and C2. This
project deliberately does not copy that paper's plant matrices or controller
gains because they describe a different rotary inverted pendulum. Instead, the
pole targets are applied to the project's reference-backed QNET upright
linearization and the resulting torque-domain gains are checked against the
Control-owned Rust constants.

This is nominal model/controller evidence only. It is not installed-specimen
calibration and grants no physical motor authority.
"""

from __future__ import annotations

import json
from pathlib import Path
import re
from typing import Any

import numpy as np
from scipy.signal import place_poles

from check_reference_lqr_mapping import (
    RUST_CONTROLLER,
    controllability_matrix,
    load_plant,
    upright_linearization,
)

GAIN_TOLERANCE = 2.0e-8
POLE_TOLERANCE = 2.0e-6

PROFILES: dict[str, dict[str, Any]] = {
    "c1": {
        "constant": "QNET_POLE_PLACEMENT_C1_TORQUE_GAINS",
        "desired_poles": [-1.0, -5.0, complex(-1.0, 3.0), complex(-1.0, -3.0)],
        "source_rationale": "Fahmizal 2023 C1; slower nominal response / lower feedback effort reference",
    },
    "c2": {
        "constant": "QNET_POLE_PLACEMENT_C2_TORQUE_GAINS",
        "desired_poles": [-5.0, -4.1, complex(-5.0, 3.0), complex(-5.0, -3.0)],
        "source_rationale": "Fahmizal 2023 C2; faster nominal response reference",
    },
}


def parse_rust_gains(name: str) -> np.ndarray:
    text = RUST_CONTROLLER.read_text(encoding="utf-8")
    match = re.search(
        rf"{re.escape(name)}\s*:\s*\[f32;\s*4\]\s*=\s*\[([^\]]+)\]",
        text,
        flags=re.MULTILINE,
    )
    if match is None:
        raise ValueError(f"cannot locate {name} in Control source")
    values = [
        float(token.strip().replace("_", ""))
        for token in match.group(1).split(",")
        if token.strip()
    ]
    if len(values) != 4:
        raise ValueError(f"{name} must contain four gains")
    return np.asarray(values, dtype=np.float64)


def ordered_poles(values: np.ndarray) -> list[complex]:
    return sorted((complex(value) for value in values), key=lambda value: (value.real, value.imag))


def pole_error(actual: np.ndarray, desired: list[complex]) -> float:
    actual_ordered = ordered_poles(actual)
    desired_ordered = sorted(desired, key=lambda value: (value.real, value.imag))
    return max(abs(lhs - rhs) for lhs, rhs in zip(actual_ordered, desired_ordered, strict=True))


def complex_json(value: complex) -> dict[str, float]:
    return {"real": float(value.real), "imag": float(value.imag)}


def main() -> int:
    plant = load_plant()
    a, b = upright_linearization(plant)
    rank = int(np.linalg.matrix_rank(controllability_matrix(a, b)))
    open_loop = np.linalg.eigvals(a)

    profile_results: dict[str, Any] = {}
    all_pass = rank == 4

    for profile_name, profile in PROFILES.items():
        desired = list(profile["desired_poles"])
        derived = place_poles(a, b, desired, method="YT").gain_matrix[0]
        rust = parse_rust_gains(str(profile["constant"]))
        gain_error = float(np.max(np.abs(derived - rust)))
        closed_loop = np.linalg.eigvals(a - b @ rust.reshape(1, 4))
        max_pole_error = float(pole_error(closed_loop, desired))

        positive_theta = np.asarray([0.05, 0.0, 0.0, 0.0], dtype=np.float64)
        restorative_torque = -float(rust @ positive_theta)
        checks = {
            "derived_gain_matches_control_owned_constant": gain_error <= GAIN_TOLERANCE,
            "closed_loop_poles_match_target": max_pole_error <= POLE_TOLERANCE,
            "closed_loop_is_stable": bool(np.max(np.real(closed_loop)) < 0.0),
            "positive_theta_commands_positive_arm_torque": restorative_torque > 0.0,
        }
        profile_pass = all(checks.values())
        all_pass = all_pass and profile_pass
        profile_results[profile_name] = {
            "source_rationale": profile["source_rationale"],
            "desired_poles": [complex_json(value) for value in desired],
            "derived_project_torque_gains": derived.tolist(),
            "control_owned_rust_torque_gains": rust.tolist(),
            "max_gain_error": gain_error,
            "closed_loop_eigenvalues": [complex_json(value) for value in closed_loop],
            "max_pole_error": max_pole_error,
            "causal_probe": {
                "project_theta_rad": float(positive_theta[0]),
                "commanded_arm_torque_nm": restorative_torque,
            },
            "checks": checks,
            "pass": profile_pass,
        }

    payload = {
        "schema": 1,
        "source": "Fahmizal 2023 pole-placement paper: desired pole locations only",
        "project_model": "reference-backed QNET reduced upright linearization",
        "project_state_order": ["theta", "theta_dot", "phi", "phi_dot"],
        "input": "arm torque [N*m]",
        "feedback_law": "u = -Kx",
        "controllability_rank": rank,
        "open_loop_eigenvalues": [complex_json(value) for value in open_loop],
        "profiles": profile_results,
        "scope": "nominal model/controller consistency only; not specimen calibration or physical authority",
        "pass": all_pass,
    }
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if all_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
