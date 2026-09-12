#!/usr/bin/env python3
"""Check upright linearization equivalence between reduced and full-3D models.

The full-3D-only configuration/velocity terms are nonlinear and should vanish at
first order about the upright, zero-rate equilibrium. With the simulation-only
pendulum axial-inertia completion set to zero, the Jacobians must therefore
match the source-backed reduced QNET model. The declared tiny axial-inertia
floor is then reported separately as a fixture-completion perturbation.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys
from typing import Callable

import numpy as np

MODEL_DIR = Path(__file__).resolve().parents[1]
if str(MODEL_DIR) not in sys.path:
    sys.path.insert(0, str(MODEL_DIR))

from reference_furuta import derivative as reduced_derivative  # noqa: E402
from full3d_furuta import derivative as full3d_derivative  # noqa: E402
from pybullet_furuta import (  # noqa: E402
    DEFAULT_CONTRACT,
    DEFAULT_FIXTURE,
    load_fixture,
)

FINITE_DIFFERENCE_STEP = 1.0e-7
ZERO_AXIAL_MAX_ABS_TOLERANCE = 1.0e-7
FIXTURE_FLOOR_MAX_RELATIVE_TOLERANCE = 5.0e-5


def jacobian(
    derivative: Callable[[np.ndarray, float], np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    equilibrium = np.zeros(4, dtype=np.float64)
    state_jacobian = np.zeros((4, 4), dtype=np.float64)
    step = FINITE_DIFFERENCE_STEP

    for column in range(4):
        positive = equilibrium.copy()
        negative = equilibrium.copy()
        positive[column] += step
        negative[column] -= step
        state_jacobian[:, column] = (
            derivative(positive, 0.0) - derivative(negative, 0.0)
        ) / (2.0 * step)

    input_jacobian = (
        derivative(equilibrium, step) - derivative(equilibrium, -step)
    ) / (2.0 * step)
    return state_jacobian, input_jacobian


def max_abs(*arrays: np.ndarray) -> float:
    return max(float(np.max(np.abs(values))) for values in arrays)


def max_scaled_relative(
    reference_a: np.ndarray,
    reference_b: np.ndarray,
    candidate_a: np.ndarray,
    candidate_b: np.ndarray,
) -> float:
    reference = np.concatenate((reference_a.ravel(), reference_b.ravel()))
    candidate = np.concatenate((candidate_a.ravel(), candidate_b.ravel()))
    scale = np.maximum(1.0, np.abs(reference))
    return float(np.max(np.abs(candidate - reference) / scale))


def matrix_as_lists(values: np.ndarray) -> list[list[float]]:
    return [[float(value) for value in row] for row in values]


def main() -> int:
    fixture = load_fixture(DEFAULT_FIXTURE)
    plant = {key: float(value) for key, value in fixture["plant"].items()}
    contract = json.loads(DEFAULT_CONTRACT.read_text(encoding="utf-8"))
    axial_floor = float(contract["simulation_fixture_completion"]["inertia_floor_kg_m2"])

    reduced = lambda state, torque: reduced_derivative(plant, state, torque)
    full_zero_axial = lambda state, torque: full3d_derivative(
        plant,
        state,
        torque,
        pendulum_axial_inertia_kg_m2=0.0,
    )
    full_fixture = lambda state, torque: full3d_derivative(
        plant,
        state,
        torque,
        pendulum_axial_inertia_kg_m2=axial_floor,
    )

    reduced_a, reduced_b = jacobian(reduced)
    full_zero_a, full_zero_b = jacobian(full_zero_axial)
    full_fixture_a, full_fixture_b = jacobian(full_fixture)

    zero_axial_difference = max_abs(
        full_zero_a - reduced_a,
        full_zero_b - reduced_b,
    )
    fixture_relative_difference = max_scaled_relative(
        reduced_a,
        reduced_b,
        full_fixture_a,
        full_fixture_b,
    )

    zero_axial_pass = zero_axial_difference <= ZERO_AXIAL_MAX_ABS_TOLERANCE
    fixture_floor_pass = (
        fixture_relative_difference <= FIXTURE_FLOOR_MAX_RELATIVE_TOLERANCE
    )

    payload = {
        "schema": 1,
        "state_order": ["theta", "theta_dot", "phi", "phi_dot"],
        "equilibrium": [0.0, 0.0, 0.0, 0.0],
        "finite_difference_step": FINITE_DIFFERENCE_STEP,
        "reduced_qnet": {
            "A": matrix_as_lists(reduced_a),
            "B": [float(value) for value in reduced_b],
        },
        "full_3d_zero_axial_inertia": {
            "max_abs_jacobian_difference": zero_axial_difference,
            "limit": ZERO_AXIAL_MAX_ABS_TOLERANCE,
            "pass": zero_axial_pass,
        },
        "full_3d_fixture_axial_inertia": {
            "pendulum_axial_inertia_kg_m2": axial_floor,
            "max_scaled_relative_jacobian_difference": fixture_relative_difference,
            "limit": FIXTURE_FLOOR_MAX_RELATIVE_TOLERANCE,
            "pass": fixture_floor_pass,
        },
        "interpretation": (
            "The full-3D-only nonlinear terms vanish at first order about upright. "
            "Zero axial inertia recovers the reduced-QNET linearization; the tiny "
            "URDF axial-inertia floor is reported only as simulation-fixture completion."
        ),
        "scope": "nominal model-class evidence only; not Forest D1 specimen calibration or physical authority",
        "pass": zero_axial_pass and fixture_floor_pass,
    }
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if payload["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
