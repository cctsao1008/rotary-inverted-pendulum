#!/usr/bin/env python3
"""Localize reduced-vs-full-vs-rigid-body Furuta dynamics differences.

This is a diagnostic, not an acceptance test. Each probe evaluates instantaneous
acceleration with both analytical model classes and estimates the corresponding
PyBullet acceleration from one 50 us rigid-body step. Sweeps vary one state
component at a time so configuration and velocity-coupling effects are visible
before long-horizon integration can amplify them.
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

from reference_furuta import derivative as reduced_derivative  # noqa: E402
from full3d_furuta import (  # noqa: E402
    derivative as full3d_derivative,
    dynamics_terms as full3d_terms,
)
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


def pendulum_axial_inertia() -> float:
    contract = json.loads(DEFAULT_CONTRACT.read_text(encoding="utf-8"))
    completion = contract["simulation_fixture_completion"]
    return float(completion["inertia_floor_kg_m2"])


def acceleration_pair(values: np.ndarray) -> dict[str, float]:
    return {
        "theta_ddot": float(values[1]),
        "phi_ddot": float(values[3]),
    }


def subtract(lhs: dict[str, float], rhs: dict[str, float]) -> dict[str, float]:
    return {
        "theta_ddot": lhs["theta_ddot"] - rhs["theta_ddot"],
        "phi_ddot": lhs["phi_ddot"] - rhs["phi_ddot"],
    }


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
    pybullet = {
        "theta_ddot": float((final_state[1] - initial[1]) / STEP_S),
        "phi_ddot": float((final_state[3] - initial[3]) / STEP_S),
    }

    plant = {key: float(value) for key, value in base["plant"].items()}
    reduced = acceleration_pair(reduced_derivative(plant, initial, torque_nm))
    axial_inertia = pendulum_axial_inertia()
    full_3d = acceleration_pair(
        full3d_derivative(
            plant,
            initial,
            torque_nm,
            pendulum_axial_inertia_kg_m2=axial_inertia,
        )
    )
    full_terms = full3d_terms(
        plant,
        initial,
        torque_nm,
        pendulum_axial_inertia_kg_m2=axial_inertia,
    )

    return {
        "name": name,
        "state": state,
        "arm_torque_nm": torque_nm,
        "qnet_reduced": reduced,
        "full_3d_analytical": full_3d,
        "pybullet_one_step": pybullet,
        "differences": {
            "full_3d_minus_qnet_reduced": subtract(full_3d, reduced),
            "pybullet_minus_qnet_reduced": subtract(pybullet, reduced),
            "pybullet_minus_full_3d": subtract(pybullet, full_3d),
        },
        "full_3d_only_terms": {
            "arm_axis_inertia_phi_phi": full_terms["mass_matrix"]["phi_phi"],
            "arm_rhs_cross_velocity": full_terms["arm_rhs_terms"]["cross_velocity"],
            "pendulum_rhs_phi_rate_squared": full_terms["pendulum_rhs_terms"][
                "phi_rate_squared"
            ],
        },
    }


def max_abs(
    records: list[dict[str, Any]],
    comparison: str,
    axis: str,
) -> float:
    return max(
        abs(float(record["differences"][comparison][axis]))
        for record in records
    )


def summarize(records: list[dict[str, Any]]) -> dict[str, Any]:
    comparisons = (
        "full_3d_minus_qnet_reduced",
        "pybullet_minus_qnet_reduced",
        "pybullet_minus_full_3d",
    )
    return {
        comparison: {
            "max_abs_theta_ddot_difference": max_abs(
                records, comparison, "theta_ddot"
            ),
            "max_abs_phi_ddot_difference": max_abs(
                records, comparison, "phi_ddot"
            ),
        }
        for comparison in comparisons
    }


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
    mixed_velocity_sweep = [
        probe(
            base,
            f"theta_dot={theta_rate:+.1f},phi_dot={phi_rate:+.1f}",
            [0.4, theta_rate, 0.0, phi_rate],
        )
        for theta_rate, phi_rate in (
            (-2.0, -2.0),
            (-2.0, 2.0),
            (2.0, -2.0),
            (2.0, 2.0),
        )
    ]
    torque_sweep = [
        probe(base, f"tau={torque:+.4f}", [0.4, 0.0, 0.0, 0.0], torque)
        for torque in (-0.002, -0.001, 0.0, 0.001, 0.002)
    ]

    sweeps = {
        "theta_zero_velocity_zero_torque": theta_sweep,
        "theta_dot_at_theta_0p4": theta_dot_sweep,
        "phi_dot_at_theta_0p4": phi_dot_sweep,
        "mixed_velocity_at_theta_0p4": mixed_velocity_sweep,
        "torque_at_theta_0p4_zero_velocity": torque_sweep,
    }
    summary = {name: summarize(records) for name, records in sweeps.items()}
    payload = {
        "schema": 2,
        "step_us": STEP_US,
        "model_classes": {
            "qnet_reduced": "source-backed reduced/equivalent nonlinear model",
            "full_3d_analytical": "geometry-derived full articulated Furuta model",
            "pybullet_one_step": "external Bullet rigid-body finite-step estimate",
        },
        "pendulum_axial_inertia_kg_m2": pendulum_axial_inertia(),
        "scope": (
            "diagnostic localization only; no acceptance threshold, "
            "production-model promotion, or specimen-calibration claim"
        ),
        "summary": summary,
        "sweeps": sweeps,
    }
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
