#!/usr/bin/env python3
"""Falsification-oriented sign checks for the PyBullet Furuta model contract."""

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


def main() -> int:
    base = load_fixture(DEFAULT_FIXTURE)
    checks: list[dict[str, Any]] = []

    positive_torque = simulate(
        scenario(base, [0.0, 0.0, 0.0, 0.0], 0.001),
        DEFAULT_PARAMETERS,
        DEFAULT_CONTRACT,
        DEFAULT_TEMPLATE,
    )
    state = positive_torque["samples"][-1]["state"]
    checks.append(require_sign("+tau at upright -> phi_dot", float(state[3]), "positive"))
    checks.append(require_sign("+tau at upright -> theta_dot", float(state[1]), "negative"))

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
        "scope": "rigid-body coordinate/input causality only; not specimen calibration",
        "checks": checks,
    }
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
