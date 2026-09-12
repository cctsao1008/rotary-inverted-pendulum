#!/usr/bin/env python3
"""Audit the QNET reference LQR mapping into project coordinates.

The paper and project use opposite positive directions for the pendulum angle.
This check derives the project torque-domain gain vector from the published
paper coordinates, compares it with the Control-owned Rust constant, and then
checks the resulting upright reduced-model closed loop.

The result is reference-model/controller consistency evidence only. It is not
Forest D1 specimen calibration and grants no physical actuator authority.
"""

from __future__ import annotations

import json
from pathlib import Path
import re

import numpy as np

from reference_furuta import derivative

ROOT = Path(__file__).resolve().parents[2]
FIXTURE = ROOT / "tools" / "model" / "fixtures" / "reference_nominal_correlation.json"
RUST_CONTROLLER = ROOT / "control" / "state-feedback" / "src" / "lib.rs"

# Abdullah et al. (2021), Eq. (31), with V = -K*x.
# Paper state order: [arm_angle, pendulum_angle, arm_rate, pendulum_rate].
PAPER_LQR_VOLTAGE_GAINS = np.asarray([-2.24, 36.71, -1.49, 3.17], dtype=np.float64)
KT_NM_PER_A = 0.042
RM_OHM = 8.4

# x_paper = PAPER_FROM_PROJECT @ x_project
# x_project = [theta, theta_dot, phi, phi_dot]
# paper pendulum angle/rate use the opposite positive direction.
PAPER_FROM_PROJECT = np.asarray(
    [
        [0.0, 0.0, 1.0, 0.0],  # arm_angle = phi
        [-1.0, 0.0, 0.0, 0.0],  # pendulum_angle = -theta
        [0.0, 0.0, 0.0, 1.0],  # arm_rate = phi_dot
        [0.0, -1.0, 0.0, 0.0],  # pendulum_rate = -theta_dot
    ],
    dtype=np.float64,
)

GAIN_TOLERANCE = 5.0e-7
JACOBIAN_STEP = 1.0e-7
MAX_STABLE_REAL_PART = -1.0e-6


def load_plant() -> dict[str, float]:
    fixture = json.loads(FIXTURE.read_text(encoding="utf-8"))
    return {key: float(value) for key, value in fixture["plant"].items()}


def parse_rust_reference_gains() -> np.ndarray:
    text = RUST_CONTROLLER.read_text(encoding="utf-8")
    match = re.search(
        r"QNET_REFERENCE_TORQUE_GAINS\s*:\s*\[f32;\s*4\]\s*=\s*\[([^\]]+)\]",
        text,
        flags=re.MULTILINE,
    )
    if match is None:
        raise ValueError("cannot locate QNET_REFERENCE_TORQUE_GAINS in Control source")
    values = [
        float(token.strip().replace("_", ""))
        for token in match.group(1).split(",")
        if token.strip()
    ]
    if len(values) != 4:
        raise ValueError("Control-owned QNET gain vector must contain four values")
    return np.asarray(values, dtype=np.float64)


def derive_project_torque_gains() -> tuple[np.ndarray, np.ndarray]:
    project_voltage = PAPER_LQR_VOLTAGE_GAINS @ PAPER_FROM_PROJECT
    torque_per_volt = KT_NM_PER_A / RM_OHM
    return project_voltage, project_voltage * torque_per_volt


def upright_linearization(plant: dict[str, float]) -> tuple[np.ndarray, np.ndarray]:
    equilibrium = np.zeros(4, dtype=np.float64)
    a = np.zeros((4, 4), dtype=np.float64)
    for column in range(4):
        delta = np.zeros(4, dtype=np.float64)
        delta[column] = JACOBIAN_STEP
        plus = derivative(plant, equilibrium + delta, 0.0)
        minus = derivative(plant, equilibrium - delta, 0.0)
        a[:, column] = (plus - minus) / (2.0 * JACOBIAN_STEP)

    plus_input = derivative(plant, equilibrium, JACOBIAN_STEP)
    minus_input = derivative(plant, equilibrium, -JACOBIAN_STEP)
    b = ((plus_input - minus_input) / (2.0 * JACOBIAN_STEP)).reshape(4, 1)
    return a, b


def controllability_matrix(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.hstack((b, a @ b, a @ a @ b, a @ a @ a @ b))


def main() -> int:
    project_voltage, expected_torque = derive_project_torque_gains()
    rust_torque = parse_rust_reference_gains()
    gain_error = np.abs(rust_torque - expected_torque)

    plant = load_plant()
    a, b = upright_linearization(plant)
    closed_loop = a - b @ rust_torque.reshape(1, 4)
    open_loop_eigenvalues = np.linalg.eigvals(a)
    closed_loop_eigenvalues = np.linalg.eigvals(closed_loop)
    controllability_rank = int(np.linalg.matrix_rank(controllability_matrix(a, b)))

    positive_theta = np.asarray([0.05, 0.0, 0.0, 0.0], dtype=np.float64)
    restorative_torque = -float(rust_torque @ positive_theta)
    torque_to_theta_acceleration = float(b[1, 0])

    checks = {
        "paper_to_project_gain_mapping": bool(np.max(gain_error) <= GAIN_TOLERANCE),
        "upright_reduced_pair_controllable": controllability_rank == 4,
        "positive_project_theta_commands_positive_arm_torque": restorative_torque > 0.0,
        "positive_arm_torque_accelerates_theta_negative_at_upright": torque_to_theta_acceleration < 0.0,
        "closed_loop_upright_linearization_is_stable": bool(
            np.max(np.real(closed_loop_eigenvalues)) < MAX_STABLE_REAL_PART
        ),
    }

    payload = {
        "schema": 1,
        "source": "Abdullah et al. 2021 QNET RIP LQR reference",
        "paper_state_order": [
            "arm_angle",
            "pendulum_angle",
            "arm_rate",
            "pendulum_rate",
        ],
        "project_state_order": ["theta", "theta_dot", "phi", "phi_dot"],
        "coordinate_mapping": {
            "arm_angle": "phi",
            "pendulum_angle": "-theta",
            "arm_rate": "phi_dot",
            "pendulum_rate": "-theta_dot",
        },
        "paper_voltage_gains": PAPER_LQR_VOLTAGE_GAINS.tolist(),
        "derived_project_voltage_gains": project_voltage.tolist(),
        "torque_per_volt_nm_per_v": KT_NM_PER_A / RM_OHM,
        "derived_project_torque_gains": expected_torque.tolist(),
        "control_owned_rust_torque_gains": rust_torque.tolist(),
        "max_gain_mapping_error": float(np.max(gain_error)),
        "upright_reduced_linearization": {
            "A": a.tolist(),
            "B": b[:, 0].tolist(),
            "controllability_rank": controllability_rank,
            "open_loop_eigenvalues": [
                {"real": float(value.real), "imag": float(value.imag)}
                for value in open_loop_eigenvalues
            ],
            "closed_loop_eigenvalues": [
                {"real": float(value.real), "imag": float(value.imag)}
                for value in closed_loop_eigenvalues
            ],
        },
        "causal_probe": {
            "project_theta_rad": float(positive_theta[0]),
            "commanded_arm_torque_nm": restorative_torque,
            "upright_d_theta_ddot_d_arm_torque": torque_to_theta_acceleration,
            "prediction": "+theta must command +arm torque; +arm torque drives theta acceleration negative",
        },
        "checks": checks,
        "scope": "reference-model/controller consistency only; not specimen calibration or physical authority",
        "pass": all(checks.values()),
    }
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if payload["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
