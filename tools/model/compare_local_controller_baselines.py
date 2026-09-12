#!/usr/bin/env python3
"""Compare local state-feedback baselines on the same nominal nonlinear plant.

This is a controller-design screening comparison, not the production semantic-
path SITL. It deliberately uses the same reference-backed reduced nonlinear
Furuta equations, the same 5 degree local initial condition, the same 1 kHz
sample-and-hold control cadence, and the same nominal +/-0.05 N*m actuator
limit for all gain profiles.

The output is descriptive evidence. It does not select a winner, calibrate the
installed specimen, or grant physical actuator authority.
"""

from __future__ import annotations

import json
from pathlib import Path
import re
import tomllib
from typing import Any

import numpy as np
from scipy.integrate import solve_ivp

from reference_furuta import derivative

ROOT = Path(__file__).resolve().parents[2]
PARAMETERS = ROOT / "parameters" / "reference-assembly.json"
SCENARIO = ROOT / "tools" / "sitl" / "scenarios" / "rotary_balance.toml"
RUST_CONTROLLER = ROOT / "control" / "state-feedback" / "src" / "lib.rs"

PROFILE_CONSTANTS = {
    "qnet_lqr": "QNET_REFERENCE_TORQUE_GAINS",
    "pole_placement_c1": "QNET_POLE_PLACEMENT_C1_TORQUE_GAINS",
    "pole_placement_c2": "QNET_POLE_PLACEMENT_C2_TORQUE_GAINS",
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


def load_contract() -> tuple[dict[str, float], np.ndarray, int, int, float]:
    parameters = json.loads(PARAMETERS.read_text(encoding="utf-8"))
    scenario = tomllib.loads(SCENARIO.read_text(encoding="utf-8"))
    plant = {key: float(record["value"]) for key, record in parameters["plant"].items()}
    rotary = scenario["rotary"]
    initial = np.asarray(
        [
            rotary["initial_theta_rad"],
            rotary["initial_theta_dot_rad_s"],
            rotary["initial_phi_rad"],
            rotary["initial_phi_dot_rad_s"],
        ],
        dtype=np.float64,
    )
    period_us = int(scenario["runtime_period_us"])
    duration_us = int(scenario["duration_us"])
    max_abs_torque = float(
        parameters["production_actuator_model"]["torque_per_effective_command_nm"]["value"]
    )
    return plant, initial, period_us, duration_us, max_abs_torque


def simulate(
    plant: dict[str, float],
    initial: np.ndarray,
    gains: np.ndarray,
    period_us: int,
    duration_us: int,
    max_abs_torque: float,
) -> dict[str, Any]:
    state = initial.copy()
    dt_s = period_us * 1.0e-6
    samples = duration_us // period_us
    max_abs_state = np.abs(state)
    max_abs_torque_observed = 0.0
    saturated_samples = 0
    torque_squared_integral = 0.0
    theta_abs_integral = 0.0

    for _ in range(samples):
        requested = -float(gains @ state)
        applied = float(np.clip(requested, -max_abs_torque, max_abs_torque))
        if abs(requested) > max_abs_torque:
            saturated_samples += 1
        max_abs_torque_observed = max(max_abs_torque_observed, abs(applied))
        torque_squared_integral += applied * applied * dt_s
        theta_abs_integral += abs(float(state[0])) * dt_s

        solution = solve_ivp(
            lambda _time, values: derivative(plant, values, applied),
            (0.0, dt_s),
            state,
            method="DOP853",
            rtol=1.0e-11,
            atol=1.0e-12,
            t_eval=[dt_s],
        )
        if not solution.success:
            raise RuntimeError(solution.message)
        state = solution.y[:, -1]
        if not np.all(np.isfinite(state)):
            raise ValueError("controller simulation produced non-finite state")
        max_abs_state = np.maximum(max_abs_state, np.abs(state))

    return {
        "final_state": {
            "theta_rad": float(state[0]),
            "theta_dot_rad_s": float(state[1]),
            "phi_rad": float(state[2]),
            "phi_dot_rad_s": float(state[3]),
        },
        "max_abs_state": {
            "theta_rad": float(max_abs_state[0]),
            "theta_dot_rad_s": float(max_abs_state[1]),
            "phi_rad": float(max_abs_state[2]),
            "phi_dot_rad_s": float(max_abs_state[3]),
        },
        "max_abs_torque_nm": max_abs_torque_observed,
        "saturated_samples": saturated_samples,
        "theta_abs_integral_rad_s": theta_abs_integral,
        "torque_squared_integral_nm2_s": torque_squared_integral,
    }


def main() -> int:
    plant, initial, period_us, duration_us, max_abs_torque = load_contract()
    results: dict[str, Any] = {}
    pass_checks = True

    for profile, constant in PROFILE_CONSTANTS.items():
        gains = parse_rust_gains(constant)
        result = simulate(
            plant,
            initial,
            gains,
            period_us,
            duration_us,
            max_abs_torque,
        )
        result["gains"] = gains.tolist()
        result["checks"] = {
            "finite": all(
                np.isfinite(value)
                for value in [
                    *result["final_state"].values(),
                    *result["max_abs_state"].values(),
                    result["max_abs_torque_nm"],
                    result["theta_abs_integral_rad_s"],
                    result["torque_squared_integral_nm2_s"],
                ]
            ),
            "actuator_limit_respected": result["max_abs_torque_nm"] <= max_abs_torque + 1.0e-12,
        }
        result["pass"] = all(result["checks"].values())
        pass_checks = pass_checks and bool(result["pass"])
        results[profile] = result

    payload = {
        "schema": 1,
        "model_class": "source-backed reduced QNET nonlinear Furuta plant",
        "state_order": ["theta", "theta_dot", "phi", "phi_dot"],
        "feedback_law": "sample-and-hold u = clamp(-Kx)",
        "sample_period_us": period_us,
        "duration_us": duration_us,
        "initial_state": initial.tolist(),
        "max_abs_torque_nm": max_abs_torque,
        "profiles": results,
        "interpretation": {
            "theta_abs_integral_rad_s": "smaller means less accumulated pendulum-angle error in this nominal local test",
            "torque_squared_integral_nm2_s": "smaller means less squared control effort in this nominal local test",
            "max_abs_phi_rad": "arm excursion is descriptive and can expose pole sets that stabilize theta by moving the arm farther",
        },
        "scope": "model-level controller screening only; not production semantic-path SITL, specimen validation, or physical authority",
        "pass": pass_checks,
    }
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if pass_checks else 1


if __name__ == "__main__":
    raise SystemExit(main())
