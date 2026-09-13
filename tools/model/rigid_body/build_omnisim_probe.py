#!/usr/bin/env python3
"""Prepare a pinned OmniSim/Newton Furuta probe fixture.

This generator adapts the canonical rigid-body URDF only where OmniSim needs
explicit fixture semantics for a bounded diagnostic run. It does not create a
new plant-parameter authority and it does not claim Forest D1 specimen truth.

The generated URDF deliberately:
- forces both joint dampings to zero because legacy Physics.damping is not live
  in OmniSim/Newton and #58 compares a common zero-damping subset;
- translates the two continuous joints into very wide diagnostic revolute
  joints so OmniSim's <rest> extension can seed exact non-zero probe states;
- adds finite effort/velocity limits solely to make imported motor devices
  explicit. Those limits are fixture mechanics, not plant limits.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
DEFAULT_PIN = HERE / "omnisim_pin.json"
DEFAULT_PARAMETERS = ROOT / "parameters" / "reference-assembly.json"
DEFAULT_CONTRACT = HERE / "furuta_contract.json"
DEFAULT_TEMPLATE = HERE / "furuta.urdf.in"
DEFAULT_CONTROLLER = HERE / "omnisim" / "controllers" / "furuta_probe" / "furuta_probe.py"

DIAGNOSTIC_POSITION_LIMIT_RAD = 3.0
DIAGNOSTIC_EFFORT_LIMIT_NM = 2.0
DIAGNOSTIC_VELOCITY_LIMIT_RAD_S = 100.0


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def finite(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    return sha256_bytes(path.read_bytes())


def load_urdf_builder():
    source = HERE / "build_furuta_urdf.py"
    spec = importlib.util.spec_from_file_location("rip_build_furuta_urdf", source)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {source}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def adapt_urdf_for_omnisim(
    urdf_text: str,
    *,
    theta0: float,
    phi0: float,
) -> str:
    root = ET.fromstring(urdf_text)
    initial_positions = {
        "arm_joint": phi0,
        "pendulum_joint": theta0,
    }
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    for name, q0 in initial_positions.items():
        if abs(q0) >= DIAGNOSTIC_POSITION_LIMIT_RAD:
            raise ValueError(
                f"{name} initial position {q0} rad reaches diagnostic fixture limit "
                f"+/-{DIAGNOSTIC_POSITION_LIMIT_RAD} rad"
            )
        joint = joints.get(name)
        if joint is None:
            raise ValueError(f"missing canonical joint {name}")
        if joint.attrib.get("type") != "continuous":
            raise ValueError(f"canonical joint {name} must be continuous before translation")

        # OmniSim seeds <rest> only on finite revolute ranges. This finite range
        # exists solely for deterministic probe initialization and is deliberately
        # much wider than every #58 experiment.
        joint.attrib["type"] = "revolute"
        for child_name in ("limit", "rest"):
            for child in list(joint.findall(child_name)):
                joint.remove(child)
        ET.SubElement(
            joint,
            "limit",
            {
                "lower": format(-DIAGNOSTIC_POSITION_LIMIT_RAD, ".17g"),
                "upper": format(DIAGNOSTIC_POSITION_LIMIT_RAD, ".17g"),
                "effort": format(DIAGNOSTIC_EFFORT_LIMIT_NM, ".17g"),
                "velocity": format(DIAGNOSTIC_VELOCITY_LIMIT_RAD_S, ".17g"),
            },
        )
        rest = ET.SubElement(joint, "rest")
        rest.text = format(q0, ".17g")

    ET.indent(root, space="  ")
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode") + "\n"


def render_world(
    *,
    basic_time_step_ms: float,
    newton_substeps: int,
    tau_nm: float,
    theta0: float,
    phi0: float,
    steps: int,
) -> str:
    args = [
        f'"--tau={tau_nm:.17g}"',
        f'"--theta0={theta0:.17g}"',
        f'"--phi0={phi0:.17g}"',
        f'"--steps={steps}"',
    ]
    controller_args = "\n    ".join(args)
    return f'''#VRML_SIM R2025a utf8

# Generated host-side validation fixture for rotary-inverted-pendulum #58.
# Model-structure evidence only; not Forest D1 specimen calibration.
WorldInfo {{
  gravity 9.81
  basicTimeStep {basic_time_step_ms:.17g}
  coordinateSystem "ENU"
  newtonSolver "mujoco"
  newtonSubsteps {newton_substeps}
}}

DEF FURUTA URDFRobot {{
  url "../urdf/furuta_probe.urdf"
  name "rotary_furuta_probe"
  controller "furuta_probe"
  controllerArgs [
    {controller_args}
  ]
  supervisor TRUE
  staticBase TRUE
  physicsBackend "newton"
}}
'''


def validate_pin(pin: dict) -> None:
    if int(pin.get("schema", 0)) != 1:
        raise ValueError("unsupported OmniSim pin schema")
    if pin.get("repository") != "omnilink-tech/omnisim":
        raise ValueError("unexpected OmniSim repository")
    commit = str(pin.get("commit", ""))
    if len(commit) != 40 or any(ch not in "0123456789abcdef" for ch in commit):
        raise ValueError("OmniSim commit must be a full lowercase SHA-1")
    solver = pin.get("solver", {})
    if solver.get("world_field") != "mujoco":
        raise ValueError("#58 requires deterministic CPU mj_step solver field 'mujoco'")
    timing = pin.get("timing", {})
    basic_ms = finite(timing.get("basic_time_step_ms"), "basic_time_step_ms")
    substeps = int(timing.get("newton_substeps"))
    internal_s = finite(timing.get("nominal_internal_substep_s"), "nominal_internal_substep_s")
    if basic_ms <= 0.0 or substeps <= 0:
        raise ValueError("OmniSim timing values must be positive")
    expected = basic_ms / 1000.0 / substeps
    if not math.isclose(internal_s, expected, rel_tol=0.0, abs_tol=1e-15):
        raise ValueError(
            f"pin internal substep {internal_s} != basic_time_step/substeps {expected}"
        )
    required_env = pin.get("required_environment", {})
    if required_env.get("OMNISIM_NEWTON_TORQUE_MODE") != "1":
        raise ValueError("direct effort probe requires OMNISIM_NEWTON_TORQUE_MODE=1")


def build(args: argparse.Namespace) -> dict:
    theta0 = finite(args.theta0, "theta0")
    phi0 = finite(args.phi0, "phi0")
    tau_nm = finite(args.tau, "tau")
    if args.steps < 1:
        raise ValueError("steps must be >= 1")
    if abs(tau_nm) > 0.1:
        raise ValueError("#58 bounded probe refuses |tau| > 0.1 N*m")

    pin = load_json(args.pin)
    validate_pin(pin)
    timing = pin["timing"]
    basic_ms = float(timing["basic_time_step_ms"])
    substeps = int(timing["newton_substeps"])

    builder = load_urdf_builder()
    canonical_urdf, _ = builder.render_urdf(
        args.parameters,
        args.contract,
        args.template,
        value_overrides={
            "arm_viscous_damping_nm_per_rad_s": 0.0,
            "pendulum_viscous_damping_nm_per_rad_s": 0.0,
        },
    )
    adapted_urdf = adapt_urdf_for_omnisim(
        canonical_urdf,
        theta0=theta0,
        phi0=phi0,
    )
    world = render_world(
        basic_time_step_ms=basic_ms,
        newton_substeps=substeps,
        tau_nm=tau_nm,
        theta0=theta0,
        phi0=phi0,
        steps=args.steps,
    )

    out = args.output_dir
    urdf_path = out / "urdf" / "furuta_probe.urdf"
    world_path = out / "worlds" / "furuta_probe.omniworld"
    controller_path = out / "controllers" / "furuta_probe" / "furuta_probe.py"
    for path in (urdf_path, world_path, controller_path):
        path.parent.mkdir(parents=True, exist_ok=True)
    urdf_path.write_text(adapted_urdf, encoding="utf-8")
    world_path.write_text(world, encoding="utf-8")
    shutil.copyfile(args.controller, controller_path)

    manifest = {
        "schema": 1,
        "lane": pin["lane"],
        "scope": pin["scope"],
        "project": {
            "parameters": str(args.parameters),
            "contract": str(args.contract),
            "parameter_provenance": "reference-backed nominal; not specimen calibration",
        },
        "omnisim": {
            "repository": pin["repository"],
            "release": pin["release"],
            "commit": pin["commit"],
            "minimum_semantics_release": pin["minimum_semantics_release"],
            "newton_version": pin["newton_version"],
            "solver": pin["solver"],
            "timing": pin["timing"],
            "required_environment": pin["required_environment"],
            "forbidden_reversion_environment": pin["forbidden_reversion_environment"],
        },
        "probe": {
            "state_order": ["theta", "theta_dot", "phi", "phi_dot"],
            "initial_state": [theta0, 0.0, phi0, 0.0],
            "input": {"arm_torque_nm": tau_nm},
            "steps": args.steps,
            "prediction": {
                "positive_tau_at_upright": ["phi_ddot > 0", "theta_ddot < 0"],
                "positive_theta_unforced_near_upright": "theta_ddot > 0",
                "negative_theta_unforced_near_upright": "theta_ddot < 0",
            },
        },
        "translation": {
            "joint_damping": "forced to zero for common solver semantics",
            "continuous_to_revolute": True,
            "diagnostic_joint_limits_rad": [
                -DIAGNOSTIC_POSITION_LIMIT_RAD,
                DIAGNOSTIC_POSITION_LIMIT_RAD,
            ],
            "diagnostic_effort_limit_nm": DIAGNOSTIC_EFFORT_LIMIT_NM,
            "diagnostic_velocity_limit_rad_s": DIAGNOSTIC_VELOCITY_LIMIT_RAD_S,
            "reason": "OmniSim <rest> seeds non-zero initial position only on finite revolute ranges",
            "authority": "fixture translation only; these limits are not plant or specimen limits",
        },
        "artifacts": {
            "pin_sha256": sha256_file(args.pin),
            "urdf_sha256": sha256_file(urdf_path),
            "world_sha256": sha256_file(world_path),
            "controller_sha256": sha256_file(controller_path),
        },
    }
    manifest_path = out / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return manifest


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--theta0", type=float, default=0.0)
    parser.add_argument("--phi0", type=float, default=0.0)
    parser.add_argument("--tau", type=float, default=0.001)
    parser.add_argument("--steps", type=int, default=1)
    parser.add_argument("--pin", type=Path, default=DEFAULT_PIN)
    parser.add_argument("--parameters", type=Path, default=DEFAULT_PARAMETERS)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--template", type=Path, default=DEFAULT_TEMPLATE)
    parser.add_argument("--controller", type=Path, default=DEFAULT_CONTROLLER)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    manifest = build(args)
    print(json.dumps({"output_dir": str(args.output_dir), "probe": manifest["probe"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
