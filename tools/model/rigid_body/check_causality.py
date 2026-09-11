#!/usr/bin/env python3
"""Falsification-oriented local causality checks for the PyBullet Furuta contract."""

from __future__ import annotations

import json
from typing import Any

from pybullet_furuta import (
    DEFAULT_CONTRACT,
    DEFAULT_FIXTURE,
    DEFAULT_PARAMETERS,
    DEFAULT_TEMPLATE,
    load_fixture,
    simulate,
)

PROBE_DURATION_S = 0.001
LOCAL_ACCEL_REL_TOL = 0.01


def scenario(base: dict[str, Any], initial_state: list[float], torque_nm: float) -> dict[str, Any]:
    result = json.loads(json.dumps(base))
    result["sample_period_us"] = 1000
    result["integration_step_us"] = 50
    result["duration_us"] = 1000
    result["initial_state"] = initial_state
    result["input_profile"] = [{"at_us": 0, "arm_torque_nm": torque_nm}]
    return result


def require_sign(name: str, value: float, expected: str, epsilon: float = 1.0e-10) -> dict[str, Any]:
    if expected == "positive":
        passed = value > epsilon
    elif expected == "negative":
        passed = value < -epsilon
    else:
        raise ValueError(f"unsupported expected sign: {expected}")
    return {"name": name, "value": value, "expected": expected, "pass": passed}


def require_relative(name: str, observed: float, expected: float) -> dict[str, Any]:
    relative_error = abs(observed - expected) / abs(expected)
    return {
        "name": name,
        "observed": observed,
        "expected": expected,
        "relative_error": relative_error,
        "relative_error_limit": LOCAL_ACCEL_REL_TOL,
        "pass": relative_error <= LOCAL_ACCEL_REL_TOL,
    }


def analytical_upright_acceleration(plant: dict[str, Any], torque_nm: float) -> tuple[float, float]:
    """Return (theta_ddot, phi_ddot) from the upright mass matrix only.

    This is intentionally not the nonlinear project derivative. At theta=0 and
    zero rates, gravity/Coriolis terms vanish and the declared mass/inertia
    meanings imply a direct two-by-two inertial identity.
    """
    mass = float(plant["pendulum_mass_kg"])
    arm_length = float(plant["arm_length_m"])
    pendulum_length = float(plant["pendulum_com_length_m"])
    arm_inertia = float(plant["arm_inertia_kg_m2"])
    pendulum_inertia = float(plant["pendulum_inertia_kg_m2"])

    a = arm_inertia + mass * arm_length**2
    b = pendulum_inertia + mass * pendulum_length**2
    c = mass * pendulum_length * arm_length
    determinant = a * b - c**2
    phi_ddot = b * torque_nm / determinant
    theta_ddot = -c * torque_nm / determinant
    return theta_ddot, phi_ddot


def main() -> int:
    base = load_fixture(DEFAULT_FIXTURE)
    checks: list[dict[str, Any]] = []

    probe_torque = 0.001
    positive_torque = simulate(
        scenario(base, [0.0, 0.0, 0.0, 0.0], probe_torque),
        DEFAULT_PARAMETERS,
        DEFAULT_CONTRACT,
        DEFAULT_TEMPLATE,
    )
    state = positive_torque["samples"][-1]["state"]
    observed_theta_ddot = float(state[1]) / PROBE_DURATION_S
    observed_phi_ddot = float(state[3]) / PROBE_DURATION_S
    expected_theta_ddot, expected_phi_ddot = analytical_upright_acceleration(
        base["plant"], probe_torque
    )

    checks.append(require_sign("+tau at upright -> phi_dot", float(state[3]), "positive"))
    checks.append(require_sign("+tau at upright -> theta_dot", float(state[1]), "negative"))
    checks.append(
        require_relative(
            "+tau upright local phi_ddot magnitude",
            observed_phi_ddot,
            expected_phi_ddot,
        )
    )
    checks.append(
        require_relative(
            "+tau upright local theta_ddot magnitude",
            observed_theta_ddot,
            expected_theta_ddot,
        )
    )

    positive_theta = simulate(
        scenario(base, [0.02, 0.0, 0.0, 0.0], 0.0),
        DEFAULT_PARAMETERS,
        DEFAULT_CONTRACT,
        DEFAULT_TEMPLATE,
    )
    checks.append(
        require_sign(
            "+theta unforced near upright -> theta_dot",
            float(positive_theta["samples"][-1]["state"][1]),
            "positive",
        )
    )

    negative_theta = simulate(
        scenario(base, [-0.02, 0.0, 0.0, 0.0], 0.0),
        DEFAULT_PARAMETERS,
        DEFAULT_CONTRACT,
        DEFAULT_TEMPLATE,
    )
    checks.append(
        require_sign(
            "-theta unforced near upright -> theta_dot",
            float(negative_theta["samples"][-1]["state"][1]),
            "negative",
        )
    )

    passed = all(check["pass"] for check in checks)
    summary = {
        "pass": passed,
        "scope": "local rigid-body coordinate/input/inertia causality only; not specimen calibration",
        "probe_duration_s": PROBE_DURATION_S,
        "checks": checks,
    }
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
