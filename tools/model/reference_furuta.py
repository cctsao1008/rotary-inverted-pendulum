#!/usr/bin/env python3
"""Independent float64 reference for nonlinear Furuta dynamics correlation.

The production Rust dynamics model advances the same explicit physical contract
with f32 fixed-step RK4. This reference independently evaluates the nonlinear
ODE in Python/NumPy and advances it with SciPy DOP853 using tight tolerances.
The comparison detects equation/sign/integration drift without sharing the Rust
execution engine.

The fixture is reference-backed nominal. Numerical agreement is implementation
consistency evidence only; it is not Forest D1 specimen calibration.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
from scipy.integrate import solve_ivp

STATE_NAMES = ("theta", "theta_dot", "phi", "phi_dot")


def load_fixture(path: Path) -> dict[str, Any]:
    fixture = json.loads(path.read_text(encoding="utf-8"))
    sample_period_us = int(fixture["sample_period_us"])
    integration_step_us = int(fixture["integration_step_us"])
    duration_us = int(fixture["duration_us"])
    if sample_period_us <= 0 or integration_step_us <= 0 or duration_us < 0:
        raise ValueError("time values must be positive, with nonnegative duration")
    if duration_us % sample_period_us != 0 or sample_period_us % integration_step_us != 0:
        raise ValueError("duration/sample/integration periods must form an integer grid")

    plant = {key: float(value) for key, value in fixture["plant"].items()}
    strictly_positive = (
        "pendulum_mass_kg",
        "arm_length_m",
        "pendulum_com_length_m",
        "arm_inertia_kg_m2",
        "pendulum_inertia_kg_m2",
        "gravity_m_s2",
    )
    nonnegative = (
        "arm_viscous_damping_nm_per_rad_s",
        "pendulum_viscous_damping_nm_per_rad_s",
    )
    for key in strictly_positive:
        if not np.isfinite(plant[key]) or plant[key] <= 0.0:
            raise ValueError(f"{key} must be positive and finite")
    for key in nonnegative:
        if not np.isfinite(plant[key]) or plant[key] < 0.0:
            raise ValueError(f"{key} must be nonnegative and finite")

    initial_state = np.asarray(fixture["initial_state"], dtype=np.float64)
    if initial_state.shape != (4,) or not np.all(np.isfinite(initial_state)):
        raise ValueError("initial_state must contain four finite values")

    previous_at_us = -1
    profile = fixture["input_profile"]
    if not profile or int(profile[0]["at_us"]) != 0:
        raise ValueError("input_profile must begin at 0 us")
    for event in profile:
        at_us = int(event["at_us"])
        torque = float(event["arm_torque_nm"])
        if (
            at_us < 0
            or at_us > duration_us
            or at_us % sample_period_us != 0
            or at_us <= previous_at_us
            or not np.isfinite(torque)
        ):
            raise ValueError("input_profile must be finite, ordered, and sample-aligned")
        previous_at_us = at_us
    return fixture


def derivative(plant: dict[str, float], state: np.ndarray, arm_torque_nm: float) -> np.ndarray:
    theta, theta_dot, phi, phi_dot = state
    del phi
    mass = plant["pendulum_mass_kg"]
    arm_length = plant["arm_length_m"]
    pendulum_length = plant["pendulum_com_length_m"]
    arm_inertia = plant["arm_inertia_kg_m2"]
    pendulum_inertia = plant["pendulum_inertia_kg_m2"]
    gravity = plant["gravity_m_s2"]
    arm_damping = plant["arm_viscous_damping_nm_per_rad_s"]
    pendulum_damping = plant["pendulum_viscous_damping_nm_per_rad_s"]

    a = arm_inertia + mass * arm_length**2
    b = pendulum_inertia + mass * pendulum_length**2
    c = mass * pendulum_length * arm_length
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    coupling = c * cos_theta
    determinant = a * b - coupling**2
    if determinant <= 0.0 or not np.isfinite(determinant):
        raise ValueError("Furuta mass matrix became singular")

    arm_rhs = arm_torque_nm - arm_damping * phi_dot + c * sin_theta * theta_dot**2
    pendulum_rhs = mass * gravity * pendulum_length * sin_theta - pendulum_damping * theta_dot
    phi_ddot = (b * arm_rhs - coupling * pendulum_rhs) / determinant
    theta_ddot = (a * pendulum_rhs - coupling * arm_rhs) / determinant
    return np.asarray([theta_dot, theta_ddot, phi_dot, phi_ddot], dtype=np.float64)


def simulate_reference(fixture: dict[str, Any]) -> list[dict[str, Any]]:
    plant = {key: float(value) for key, value in fixture["plant"].items()}
    sample_period_us = int(fixture["sample_period_us"])
    duration_us = int(fixture["duration_us"])
    state = np.asarray(fixture["initial_state"], dtype=np.float64).copy()
    profile = fixture["input_profile"]
    profile_index = 0
    current_torque = 0.0
    samples: list[dict[str, Any]] = []

    for at_us in range(0, duration_us + 1, sample_period_us):
        while profile_index < len(profile) and int(profile[profile_index]["at_us"]) == at_us:
            current_torque = float(profile[profile_index]["arm_torque_nm"])
            profile_index += 1

        samples.append(
            {
                "time_us": at_us,
                "state": state.tolist(),
                "arm_torque_nm": current_torque,
            }
        )
        if at_us == duration_us:
            break

        interval_s = sample_period_us * 1.0e-6
        solution = solve_ivp(
            lambda _time, values: derivative(plant, values, current_torque),
            (0.0, interval_s),
            state,
            method="DOP853",
            rtol=1.0e-12,
            atol=1.0e-13,
            t_eval=[interval_s],
        )
        if not solution.success:
            raise RuntimeError(solution.message)
        state = solution.y[:, -1]

    if profile_index != len(profile):
        raise ValueError("not all input_profile events were consumed")
    return samples


def compare_rust_trace(
    reference: list[dict[str, Any]], rust_trace_path: Path, max_abs_error: float
) -> dict[str, Any]:
    rust = json.loads(rust_trace_path.read_text(encoding="utf-8"))
    rust_samples = rust["samples"]
    if len(rust_samples) != len(reference):
        raise ValueError("Rust/reference trace lengths differ")

    per_state_max = np.zeros(4, dtype=np.float64)
    for expected, actual in zip(reference, rust_samples, strict=True):
        if int(actual["time_us"]) != int(expected["time_us"]):
            raise ValueError("Rust/reference sample timestamps differ")
        if abs(float(actual["arm_torque_nm"]) - float(expected["arm_torque_nm"])) > 1.0e-7:
            raise ValueError(f"applied torque mismatch at {expected['time_us']} us")
        expected_state = np.asarray(expected["state"], dtype=np.float64)
        actual_state = np.asarray(actual["state"], dtype=np.float64)
        per_state_max = np.maximum(per_state_max, np.abs(actual_state - expected_state))

    overall = float(np.max(per_state_max))
    return {
        "pass": overall <= max_abs_error,
        "sample_count": len(reference),
        "max_abs_error": overall,
        "max_abs_error_limit": max_abs_error,
        "per_state_max_abs_error": {
            name: float(error) for name, error in zip(STATE_NAMES, per_state_max, strict=True)
        },
        "interpretation": "implementation consistency only; not Forest D1 specimen calibration",
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--rust-trace", type=Path, required=True)
    parser.add_argument("--summary", type=Path)
    parser.add_argument("--max-abs-error", type=float, default=2.0e-4)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not np.isfinite(args.max_abs_error) or args.max_abs_error <= 0.0:
        raise ValueError("--max-abs-error must be positive and finite")
    fixture = load_fixture(args.fixture)
    reference = simulate_reference(fixture)
    summary = compare_rust_trace(reference, args.rust_trace, args.max_abs_error)
    rendered = json.dumps(summary, indent=2, sort_keys=True) + "\n"
    if args.summary is not None:
        args.summary.parent.mkdir(parents=True, exist_ok=True)
        args.summary.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
