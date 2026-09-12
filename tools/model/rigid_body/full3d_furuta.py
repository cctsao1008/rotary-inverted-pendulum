#!/usr/bin/env python3
"""Geometry-derived full 3-D Furuta dynamics for model-structure validation.

This is deliberately separate from the source-backed QNET reduced model in
``tools/model/reference_furuta.py``. It is derived from the rigid-body
kinematics declared by ``furuta_contract.json`` / ``furuta.urdf.in`` and exists
to expose model-class differences, not to replace the production plant or claim
Forest D1 specimen calibration.

Project state order is ``[theta, theta_dot, phi, phi_dot]``:
- ``phi``: arm rotation about world +Z;
- ``theta``: pendulum rotation about the arm-frame -X axis, zero at upright.

With pendulum transverse inertia Jp and axial inertia Jz, the kinetic energy has
the full arm-rate inertia

    A(theta) = Jeq + m*Lr^2
             + (Jp + m*Lp^2)*sin(theta)^2
             + Jz*cos(theta)^2

rather than the constant arm-rate inertia used by the reduced QNET model.
The resulting equations therefore include the additional ``phi_dot^2`` and
``phi_dot*theta_dot`` terms that vanish at the upright linearization.
"""

from __future__ import annotations

from typing import Any

import numpy as np


def dynamics_terms(
    plant: dict[str, float],
    state: np.ndarray,
    arm_torque_nm: float,
    *,
    pendulum_axial_inertia_kg_m2: float = 0.0,
) -> dict[str, Any]:
    """Return the full-3D mass-matrix/RHS decomposition for one state."""

    theta, theta_dot, phi, phi_dot = np.asarray(state, dtype=np.float64)
    del phi

    mass = float(plant["pendulum_mass_kg"])
    arm_length = float(plant["arm_length_m"])
    pendulum_length = float(plant["pendulum_com_length_m"])
    arm_inertia = float(plant["arm_inertia_kg_m2"])
    pendulum_inertia = float(plant["pendulum_inertia_kg_m2"])
    gravity = float(plant["gravity_m_s2"])
    arm_damping = float(plant["arm_viscous_damping_nm_per_rad_s"])
    pendulum_damping = float(plant["pendulum_viscous_damping_nm_per_rad_s"])
    axial_inertia = float(pendulum_axial_inertia_kg_m2)

    if not np.isfinite(axial_inertia) or axial_inertia < 0.0:
        raise ValueError("pendulum axial inertia must be finite and nonnegative")

    base_arm_inertia = arm_inertia + mass * arm_length**2
    pendulum_transverse_about_pivot = pendulum_inertia + mass * pendulum_length**2
    coupling_scale = mass * pendulum_length * arm_length
    orientation_inertia_scale = pendulum_transverse_about_pivot - axial_inertia

    sin_theta = float(np.sin(theta))
    cos_theta = float(np.cos(theta))
    sin_cos = sin_theta * cos_theta

    arm_axis_inertia = (
        base_arm_inertia
        + pendulum_transverse_about_pivot * sin_theta**2
        + axial_inertia * cos_theta**2
    )
    coupling = coupling_scale * cos_theta
    determinant = arm_axis_inertia * pendulum_transverse_about_pivot - coupling**2
    if determinant <= 0.0 or not np.isfinite(determinant):
        raise ValueError("full-3D Furuta mass matrix became singular")

    arm_rhs_terms = {
        "input_torque": float(arm_torque_nm),
        "arm_viscous_damping": -arm_damping * float(phi_dot),
        "theta_rate_squared": coupling_scale * sin_theta * float(theta_dot) ** 2,
        "cross_velocity": (
            -2.0
            * orientation_inertia_scale
            * sin_cos
            * float(phi_dot)
            * float(theta_dot)
        ),
    }
    pendulum_rhs_terms = {
        "gravity": mass * gravity * pendulum_length * sin_theta,
        "pendulum_viscous_damping": -pendulum_damping * float(theta_dot),
        "phi_rate_squared": (
            orientation_inertia_scale * sin_cos * float(phi_dot) ** 2
        ),
    }

    arm_rhs = float(sum(arm_rhs_terms.values()))
    pendulum_rhs = float(sum(pendulum_rhs_terms.values()))

    return {
        "mass_matrix": {
            "phi_phi": arm_axis_inertia,
            "phi_theta": coupling,
            "theta_phi": coupling,
            "theta_theta": pendulum_transverse_about_pivot,
            "determinant": determinant,
        },
        "arm_rhs_terms": arm_rhs_terms,
        "pendulum_rhs_terms": pendulum_rhs_terms,
        "arm_rhs": arm_rhs,
        "pendulum_rhs": pendulum_rhs,
        "model_terms": {
            "base_arm_inertia": base_arm_inertia,
            "pendulum_transverse_about_pivot": pendulum_transverse_about_pivot,
            "pendulum_axial_inertia": axial_inertia,
            "orientation_inertia_scale": orientation_inertia_scale,
            "coupling_scale": coupling_scale,
        },
    }


def derivative(
    plant: dict[str, float],
    state: np.ndarray,
    arm_torque_nm: float,
    *,
    pendulum_axial_inertia_kg_m2: float = 0.0,
) -> np.ndarray:
    """Evaluate the geometry-derived full 3-D Furuta state derivative."""

    state_array = np.asarray(state, dtype=np.float64)
    if state_array.shape != (4,) or not np.all(np.isfinite(state_array)):
        raise ValueError("state must contain four finite values")
    if not np.isfinite(arm_torque_nm):
        raise ValueError("arm torque must be finite")

    terms = dynamics_terms(
        plant,
        state_array,
        arm_torque_nm,
        pendulum_axial_inertia_kg_m2=pendulum_axial_inertia_kg_m2,
    )
    matrix = terms["mass_matrix"]
    arm_rhs = float(terms["arm_rhs"])
    pendulum_rhs = float(terms["pendulum_rhs"])
    determinant = float(matrix["determinant"])
    arm_axis_inertia = float(matrix["phi_phi"])
    coupling = float(matrix["phi_theta"])
    pendulum_inertia = float(matrix["theta_theta"])

    phi_ddot = (pendulum_inertia * arm_rhs - coupling * pendulum_rhs) / determinant
    theta_ddot = (arm_axis_inertia * pendulum_rhs - coupling * arm_rhs) / determinant

    theta_dot = float(state_array[1])
    phi_dot = float(state_array[3])
    return np.asarray(
        [theta_dot, theta_ddot, phi_dot, phi_ddot],
        dtype=np.float64,
    )
