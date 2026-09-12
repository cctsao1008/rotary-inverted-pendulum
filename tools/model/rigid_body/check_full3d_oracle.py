#!/usr/bin/env python3
"""Falsify the geometry-derived full-3D Furuta oracle against PyBullet.

This gate is intentionally local and instantaneous. It asks whether independently
executed articulated rigid-body dynamics reproduce the acceleration implied by
the declared 3-D geometry under probes that isolate configuration, velocity,
gravity and damping terms. It is not a trajectory-accuracy score, a specimen
calibration, or permission to change physical motor authority.
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

from full3d_furuta import derivative, dynamics_terms  # noqa: E402
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
NUMERICAL_IDENTITY_TOLERANCE_RAD_S2 = 1.0e-8
TERM_ZERO_TOLERANCE = 1.0e-15


def axial_inertia() -> float:
    contract = json.loads(DEFAULT_CONTRACT.read_text(encoding="utf-8"))
    return float(contract["simulation_fixture_completion"]["inertia_floor_kg_m2"])


def one_step_fixture(base: dict[str, Any], state: list[float], torque_nm: float) -> dict[str, Any]:
    fixture = json.loads(json.dumps(base))
    fixture["sample_period_us"] = STEP_US
    fixture["integration_step_us"] = STEP_US
    fixture["duration_us"] = STEP_US
    fixture["initial_state"] = state
    fixture["input_profile"] = [{"at_us": 0, "arm_torque_nm": torque_nm}]
    return fixture


def analytical_plant(
    base: dict[str, Any], *, gravity: bool, damping: bool
) -> dict[str, float]:
    plant = {key: float(value) for key, value in base["plant"].items()}
    if not gravity:
        plant["gravity_m_s2"] = 0.0
    if not damping:
        plant["arm_viscous_damping_nm_per_rad_s"] = 0.0
        plant["pendulum_viscous_damping_nm_per_rad_s"] = 0.0
    return plant


def acceleration(values: np.ndarray) -> dict[str, float]:
    return {"theta_ddot": float(values[1]), "phi_ddot": float(values[3])}


def run_probe(
    base: dict[str, Any],
    *,
    name: str,
    state: list[float],
    torque_nm: float = 0.0,
    gravity: bool = True,
    damping: bool = True,
) -> dict[str, Any]:
    fixture = one_step_fixture(base, state, torque_nm)
    gravity_override = None if gravity else 0.0
    damping_overrides = None
    if not damping:
        damping_overrides = {
            "arm_viscous_damping_nm_per_rad_s": 0.0,
            "pendulum_viscous_damping_nm_per_rad_s": 0.0,
        }

    trace = simulate(
        fixture,
        DEFAULT_PARAMETERS,
        DEFAULT_CONTRACT,
        DEFAULT_TEMPLATE,
        gravity_override_m_s2=gravity_override,
        urdf_value_overrides=damping_overrides,
    )
    initial = np.asarray(state, dtype=np.float64)
    final_state = np.asarray(trace["samples"][-1]["state"], dtype=np.float64)
    pybullet = {
        "theta_ddot": float((final_state[1] - initial[1]) / STEP_S),
        "phi_ddot": float((final_state[3] - initial[3]) / STEP_S),
    }

    plant = analytical_plant(base, gravity=gravity, damping=damping)
    full = acceleration(
        derivative(
            plant,
            initial,
            torque_nm,
            pendulum_axial_inertia_kg_m2=axial_inertia(),
        )
    )
    terms = dynamics_terms(
        plant,
        initial,
        torque_nm,
        pendulum_axial_inertia_kg_m2=axial_inertia(),
    )
    difference = {
        axis: pybullet[axis] - full[axis] for axis in ("theta_ddot", "phi_ddot")
    }
    max_abs_difference = max(abs(value) for value in difference.values())

    return {
        "name": name,
        "state": state,
        "arm_torque_nm": torque_nm,
        "environment": {
            "gravity_enabled": gravity,
            "viscous_damping_enabled": damping,
        },
        "full_3d_analytical": full,
        "pybullet_one_step": pybullet,
        "pybullet_minus_full_3d": difference,
        "max_abs_acceleration_difference": max_abs_difference,
        "pass": max_abs_difference <= NUMERICAL_IDENTITY_TOLERANCE_RAD_S2,
        "term_evidence": {
            "arm_axis_inertia_phi_phi": float(terms["mass_matrix"]["phi_phi"]),
            "arm_rhs_theta_rate_squared": float(
                terms["arm_rhs_terms"]["theta_rate_squared"]
            ),
            "arm_rhs_cross_velocity": float(
                terms["arm_rhs_terms"]["cross_velocity"]
            ),
            "pendulum_rhs_gravity": float(terms["pendulum_rhs_terms"]["gravity"]),
            "pendulum_rhs_phi_rate_squared": float(
                terms["pendulum_rhs_terms"]["phi_rate_squared"]
            ),
            "arm_rhs_viscous_damping": float(
                terms["arm_rhs_terms"]["arm_viscous_damping"]
            ),
            "pendulum_rhs_viscous_damping": float(
                terms["pendulum_rhs_terms"]["pendulum_viscous_damping"]
            ),
        },
    }


def nonzero(value: float) -> bool:
    return abs(value) > TERM_ZERO_TOLERANCE


def main() -> int:
    base = load_fixture(DEFAULT_FIXTURE)
    probes = [
        run_probe(
            base,
            name="upright-positive-torque",
            state=[0.0, 0.0, 0.0, 0.0],
            torque_nm=0.001,
        ),
        run_probe(
            base,
            name="configuration-plus-0p4",
            state=[0.4, 0.0, 0.0, 0.0],
        ),
        run_probe(
            base,
            name="configuration-minus-0p4",
            state=[-0.4, 0.0, 0.0, 0.0],
        ),
        run_probe(
            base,
            name="theta-rate-only-conservative",
            state=[0.4, 4.0, 0.0, 0.0],
            gravity=False,
            damping=False,
        ),
        run_probe(
            base,
            name="phi-rate-only-conservative",
            state=[0.4, 0.0, 0.0, 4.0],
            gravity=False,
            damping=False,
        ),
        run_probe(
            base,
            name="mixed-rates-conservative",
            state=[0.4, 2.0, 0.0, -2.0],
            gravity=False,
            damping=False,
        ),
        run_probe(
            base,
            name="gravity-only",
            state=[0.4, 0.0, 0.0, 0.0],
            damping=False,
        ),
        run_probe(
            base,
            name="arm-damping-only",
            state=[0.4, 0.0, 0.0, 4.0],
            gravity=False,
            damping=True,
        ),
    ]

    by_name = {probe["name"]: probe for probe in probes}
    term_checks = {
        "theta_rate_squared_activates": nonzero(
            by_name["theta-rate-only-conservative"]["term_evidence"][
                "arm_rhs_theta_rate_squared"
            ]
        ),
        "phi_rate_squared_activates": nonzero(
            by_name["phi-rate-only-conservative"]["term_evidence"][
                "pendulum_rhs_phi_rate_squared"
            ]
        ),
        "cross_velocity_activates": nonzero(
            by_name["mixed-rates-conservative"]["term_evidence"][
                "arm_rhs_cross_velocity"
            ]
        ),
        "conservative_probe_has_zero_gravity": not nonzero(
            by_name["mixed-rates-conservative"]["term_evidence"][
                "pendulum_rhs_gravity"
            ]
        ),
        "conservative_probe_has_zero_damping": not nonzero(
            by_name["mixed-rates-conservative"]["term_evidence"][
                "arm_rhs_viscous_damping"
            ]
        ),
        "gravity_only_activates_gravity": nonzero(
            by_name["gravity-only"]["term_evidence"]["pendulum_rhs_gravity"]
        ),
        "damping_only_activates_arm_damping": nonzero(
            by_name["arm-damping-only"]["term_evidence"][
                "arm_rhs_viscous_damping"
            ]
        ),
    }

    max_difference = max(
        float(probe["max_abs_acceleration_difference"]) for probe in probes
    )
    numerical_pass = all(bool(probe["pass"]) for probe in probes)
    term_pass = all(term_checks.values())
    payload = {
        "schema": 1,
        "model": "geometry-derived full-3d Furuta analytical oracle",
        "comparison_backend": "PyBullet articulated rigid body",
        "step_us": STEP_US,
        "pendulum_axial_inertia_kg_m2": axial_inertia(),
        "numerical_identity_tolerance_rad_s2": NUMERICAL_IDENTITY_TOLERANCE_RAD_S2,
        "max_abs_acceleration_difference": max_difference,
        "term_activation_checks": term_checks,
        "probes": probes,
        "interpretation": (
            "Agreement demonstrates equation/geometry/backend structural consistency for "
            "the declared nominal rigid body. It does not calibrate the installed specimen."
        ),
        "scope": "host-side falsification only; no production-model promotion or physical authority",
        "pass": numerical_pass and term_pass,
    }
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if payload["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
