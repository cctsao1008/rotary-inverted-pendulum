#!/usr/bin/env python3
"""Localize analytical-vs-rigid-body Furuta dynamics differences by one-step probes.

This is a diagnostic, not an acceptance test. Each probe predicts instantaneous
acceleration with the independent SciPy reference derivative and estimates the
corresponding PyBullet acceleration from one 50 us rigid-body step. Sweeps vary
one state component at a time so configuration and velocity-coupling effects are
visible before long-horizon integration can amplify them.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys
from typing import Any

import numpy as np

MODEL_DIR = Path(__file__).resolve().parents[1]
if str(MODEL_DIR) not in sys.path:
    sys.path.insert(0, str(MODEL_DIR))

from reference_furuta import derivative  # noqa: E402
from pybullet_furuta import (  # noqa: E402
    DEFAULT_CONTRACT,
    DEFAULT_FIXTURE,
    DEFAULT_PARAMETERS,
    DEFAULT_TEMPLATE,
    load_fixture,
    simulate,
)

STEP_US = 50
STEP_S = STEP_US * 1.0e-6


def one_step_fixture(base: dict[str, Any], state: list[float], torque_nm: float) -> dict[str, Any]:
    fixture = json.loads(json.dumps(base))
    fixture["sample_period_us"] = STEP_US
    fixture["integration_step_us"] = STEP_US
    fixture["duration_us"] = STEP_US
    fixture["initial_state"] = state
    fixture["input_profile"] = [{"at_us": 0, "arm_torque_nm": torque_nm}]
    return fixture


def probe(
    base: dict[str, Any],
    name: str,
    state: list[float],
    torque_nm: float = 0.0,
) -> dict[str, Any]:
    fixture = one_step_fixture(base, state, torque_nm)
    trace = simulate(
        fixture,
        DEFAULT_PARAMETERS,
        DEFAULT_CONTRACT,
        DEFAULT_TEMPLATE,
    )
    final_state = np.asarray(trace["samples"][-1]["state"], dtype=np.float64)
    initial = np.asarray(state, dtype=np.float64)
    bullet_theta_ddot = float((final_state[1] - initial[1]) / STEP_S)
    bullet_phi_ddot = float((final_state[3] - initial[3]) / STEP_S)

    analytical = derivative(
        {key: float(value) for key, value in base["plant"].items()},
        initial,
        torque_nm,
    )
    analytical_theta_ddot = float(analytical[1])
    analytical_phi_ddot = float(analytical[3])

    return {
        "name": name,
        "state": state,
        "arm_torque_nm": torque_nm,
        "analytical": {
            "theta_ddot": analytical_theta_ddot,
            "phi_ddot": analytical_phi_ddot,
        },
        "pybullet_one_step": {
            "theta_ddot": bullet_theta_ddot,
            "phi_ddot": bullet_phi_ddot,
        },
        "difference": {
            "theta_ddot": bullet_theta_ddot - analytical_theta_ddot,
            "phi_ddot": bullet_phi_ddot - analytical_phi_ddot,
        },
    }


def max_abs(records: list[dict[str, Any]], axis: str) -> float:
    return max(abs(float(record["difference"][axis])) for record in records)


def main() -> int:
    base = load_fixture(DEFAULT_FIXTURE)

    theta_sweep = [
        probe(base, f"theta={theta:+.2f}", [theta, 0.0, 0.0, 0.0])
        for theta in (-1.2, -0.8, -0.4, 0.0, 0.4, 0.8, 1.2)
    ]
    theta_dot_sweep = [
        probe(base, f"theta_dot={rate:+.1f}", [0.4, rate, 0.0, 0.0])
        for rate in (-4.0, -2.0, 0.0, 2.0, 4.0)
    ]
    phi_dot_sweep = [
        probe(base, f"phi_dot={rate:+.1f}", [0.4, 0.0, 0.0, rate])
        for rate in (-4.0, -2.0, 0.0, 2.0, 4.0)
    ]
    torque_sweep = [
        probe(base, f"tau={torque:+.4f}", [0.4, 0.0, 0.0, 0.0], torque)
        for torque in (-0.002, -0.001, 0.0, 0.001, 0.002)
    ]

    sweeps = {
        "theta_zero_velocity_zero_torque": theta_sweep,
        "theta_dot_at_theta_0p4": theta_dot_sweep,
        "phi_dot_at_theta_0p4": phi_dot_sweep,
        "torque_at_theta_0p4_zero_velocity": torque_sweep,
    }
    summary = {
        name: {
            "max_abs_theta_ddot_difference": max_abs(records, "theta_ddot"),
            "max_abs_phi_ddot_difference": max_abs(records, "phi_ddot"),
        }
        for name, records in sweeps.items()
    }
    payload = {
        "schema": 1,
        "step_us": STEP_US,
        "scope": "diagnostic localization only; no acceptance threshold and no specimen-calibration claim",
        "summary": summary,
        "sweeps": sweeps,
    }
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
