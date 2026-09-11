#!/usr/bin/env python3
"""Headless PyBullet trace for the reference-backed nominal Furuta fixture.

This path deliberately delegates articulated rigid-body dynamics to PyBullet.
It does not call or reproduce the project nonlinear Furuta derivative. Results
are model-structure evidence only, not Forest D1 specimen calibration.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import tempfile
from typing import Any

from build_furuta_urdf import (
    DEFAULT_CONTRACT,
    DEFAULT_PARAMETERS,
    DEFAULT_TEMPLATE,
    render_urdf,
)

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_FIXTURE = ROOT / "tools" / "model" / "fixtures" / "reference_nominal_correlation.json"
STATE_NAMES = ("theta", "theta_dot", "phi", "phi_dot")
PLANT_KEYS = (
    "pendulum_mass_kg",
    "arm_length_m",
    "pendulum_com_length_m",
    "arm_inertia_kg_m2",
    "pendulum_inertia_kg_m2",
    "gravity_m_s2",
    "arm_viscous_damping_nm_per_rad_s",
    "pendulum_viscous_damping_nm_per_rad_s",
)


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def load_fixture(path: Path) -> dict[str, Any]:
    fixture = load_json(path)
    if int(fixture.get("schema", 0)) != 1:
        raise ValueError("unsupported model-correlation fixture schema")

    sample_period_us = int(fixture["sample_period_us"])
    integration_step_us = int(fixture["integration_step_us"])
    duration_us = int(fixture["duration_us"])
    if sample_period_us <= 0 or integration_step_us <= 0 or duration_us < 0:
        raise ValueError("time values must be positive, with nonnegative duration")
    if duration_us % sample_period_us != 0 or sample_period_us % integration_step_us != 0:
        raise ValueError("duration/sample/integration periods must form an integer grid")

    initial_state = [float(value) for value in fixture["initial_state"]]
    if len(initial_state) != 4 or not all(math.isfinite(value) for value in initial_state):
        raise ValueError("initial_state must contain four finite values")

    plant = fixture["plant"]
    for key in PLANT_KEYS:
        value = float(plant[key])
        if not math.isfinite(value):
            raise ValueError(f"{key} must be finite")
    for key in (
        "pendulum_mass_kg",
        "arm_length_m",
        "pendulum_com_length_m",
        "arm_inertia_kg_m2",
        "pendulum_inertia_kg_m2",
        "gravity_m_s2",
    ):
        if float(plant[key]) <= 0.0:
            raise ValueError(f"{key} must be positive")
    for key in (
        "arm_viscous_damping_nm_per_rad_s",
        "pendulum_viscous_damping_nm_per_rad_s",
    ):
        if float(plant[key]) < 0.0:
            raise ValueError(f"{key} must be nonnegative")

    profile = fixture["input_profile"]
    if not profile or int(profile[0]["at_us"]) != 0:
        raise ValueError("input_profile must begin at 0 us")
    previous_at_us = -1
    for event in profile:
        at_us = int(event["at_us"])
        torque = float(event["arm_torque_nm"])
        if (
            at_us < 0
            or at_us > duration_us
            or at_us % sample_period_us != 0
            or at_us <= previous_at_us
            or not math.isfinite(torque)
        ):
            raise ValueError("input_profile must be finite, ordered, and sample-aligned")
        previous_at_us = at_us
    return fixture


def assert_fixture_matches_parameters(fixture: dict[str, Any], parameters_path: Path) -> None:
    parameters = load_json(parameters_path)
    plant = parameters["plant"]
    for key in PLANT_KEYS:
        fixture_value = float(fixture["plant"][key])
        parameter_value = float(plant[key]["value"])
        if not math.isclose(fixture_value, parameter_value, rel_tol=0.0, abs_tol=1.0e-12):
            raise ValueError(
                f"fixture {key}={fixture_value} does not match canonical parameter {parameter_value}"
            )


def joint_indices(pybullet: Any, body: int) -> dict[str, int]:
    result: dict[str, int] = {}
    for index in range(pybullet.getNumJoints(body)):
        info = pybullet.getJointInfo(body, index)
        result[info[1].decode("utf-8")] = index
    required = {"arm_joint", "pendulum_joint"}
    missing = required.difference(result)
    if missing:
        raise ValueError(f"generated URDF is missing joints: {sorted(missing)}")
    return result


def project_state(pybullet: Any, body: int, joints: dict[str, int]) -> list[float]:
    pendulum = pybullet.getJointState(body, joints["pendulum_joint"])
    arm = pybullet.getJointState(body, joints["arm_joint"])
    return [
        float(pendulum[0]),
        float(pendulum[1]),
        float(arm[0]),
        float(arm[1]),
    ]


def simulate(
    fixture: dict[str, Any],
    parameters_path: Path,
    contract_path: Path,
    template_path: Path,
) -> dict[str, Any]:
    try:
        import pybullet as p
    except ImportError as exc:
        raise RuntimeError(
            "PyBullet is required; install tools/model/requirements-rigid-body.txt"
        ) from exc

    urdf_text, _ = render_urdf(parameters_path, contract_path, template_path)
    contract = load_json(contract_path)
    assert_fixture_matches_parameters(fixture, parameters_path)

    sample_period_us = int(fixture["sample_period_us"])
    integration_step_us = int(fixture["integration_step_us"])
    duration_us = int(fixture["duration_us"])
    substeps_per_sample = sample_period_us // integration_step_us
    dt_s = integration_step_us * 1.0e-6
    gravity = float(fixture["plant"]["gravity_m_s2"])

    connection = p.connect(p.DIRECT)
    if connection < 0:
        raise RuntimeError("failed to connect to PyBullet DIRECT mode")

    try:
        p.resetSimulation()
        p.setGravity(0.0, 0.0, -gravity)
        p.setPhysicsEngineParameter(
            fixedTimeStep=dt_s,
            numSubSteps=0,
            numSolverIterations=200,
            deterministicOverlappingPairs=1,
        )

        with tempfile.TemporaryDirectory(prefix="rip-pybullet-") as directory:
            urdf_path = Path(directory) / "furuta.urdf"
            urdf_path.write_text(urdf_text, encoding="utf-8")
            body = p.loadURDF(
                str(urdf_path),
                useFixedBase=True,
                flags=p.URDF_USE_INERTIA_FROM_FILE,
            )

        joints = joint_indices(p, body)
        arm_joint = joints["arm_joint"]
        pendulum_joint = joints["pendulum_joint"]

        for link_index in range(-1, p.getNumJoints(body)):
            p.changeDynamics(body, link_index, linearDamping=0.0, angularDamping=0.0)

        # Disable Bullet's default velocity motors. The arm receives only the
        # explicit fixture torque; the pendulum remains passive.
        p.setJointMotorControl2(body, arm_joint, p.VELOCITY_CONTROL, targetVelocity=0.0, force=0.0)
        p.setJointMotorControl2(
            body, pendulum_joint, p.VELOCITY_CONTROL, targetVelocity=0.0, force=0.0
        )

        theta, theta_dot, phi, phi_dot = [float(value) for value in fixture["initial_state"]]
        p.resetJointState(body, pendulum_joint, theta, targetVelocity=theta_dot)
        p.resetJointState(body, arm_joint, phi, targetVelocity=phi_dot)

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
                    "state": project_state(p, body, joints),
                    "arm_torque_nm": current_torque,
                }
            )
            if at_us == duration_us:
                break

            for _ in range(substeps_per_sample):
                p.setJointMotorControl2(
                    body,
                    arm_joint,
                    p.TORQUE_CONTROL,
                    force=current_torque,
                )
                p.stepSimulation()

        if profile_index != len(profile):
            raise ValueError("not all input_profile events were consumed")

        return {
            "schema": 1,
            "engine": "pybullet",
            "engine_api_version": int(p.getAPIVersion()),
            "state_order": list(STATE_NAMES),
            "contract": contract["model"],
            "scope": "model-structure validation only; not Forest D1 specimen calibration",
            "samples": samples,
        }
    finally:
        p.disconnect(connection)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture", type=Path, default=DEFAULT_FIXTURE)
    parser.add_argument("--parameters", type=Path, default=DEFAULT_PARAMETERS)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--template", type=Path, default=DEFAULT_TEMPLATE)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    fixture = load_fixture(args.fixture)
    trace = simulate(fixture, args.parameters, args.contract, args.template)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(trace, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
