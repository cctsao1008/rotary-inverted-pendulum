#!/usr/bin/env python3
"""Build the nominal Furuta rigid-body URDF from canonical project parameters.

The generated URDF is a host-side validation asset. It intentionally does not
encode the project nonlinear equations. Simulator agreement is model-structure
evidence only and is not Forest D1 specimen calibration.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_PARAMETERS = ROOT / "parameters" / "reference-assembly.json"
DEFAULT_CONTRACT = Path(__file__).with_name("furuta_contract.json")
DEFAULT_TEMPLATE = Path(__file__).with_name("furuta.urdf.in")

PLANT_KEYS = (
    "pendulum_mass_kg",
    "arm_length_m",
    "pendulum_com_length_m",
    "arm_inertia_kg_m2",
    "pendulum_inertia_kg_m2",
    "arm_viscous_damping_nm_per_rad_s",
    "pendulum_viscous_damping_nm_per_rad_s",
)


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def finite_float(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def load_render_values(parameters_path: Path, contract_path: Path) -> dict[str, float]:
    parameters = load_json(parameters_path)
    contract = load_json(contract_path)
    if int(parameters.get("schema", 0)) != 1:
        raise ValueError("unsupported reference-assembly schema")
    if int(contract.get("schema", 0)) != 1:
        raise ValueError("unsupported rigid-body contract schema")

    plant = parameters["plant"]
    values = {
        key: finite_float(plant[key]["value"], key)
        for key in PLANT_KEYS
    }
    strictly_positive = (
        "pendulum_mass_kg",
        "arm_length_m",
        "pendulum_com_length_m",
        "arm_inertia_kg_m2",
        "pendulum_inertia_kg_m2",
    )
    nonnegative = (
        "arm_viscous_damping_nm_per_rad_s",
        "pendulum_viscous_damping_nm_per_rad_s",
    )
    for key in strictly_positive:
        if values[key] <= 0.0:
            raise ValueError(f"{key} must be positive")
    for key in nonnegative:
        if values[key] < 0.0:
            raise ValueError(f"{key} must be nonnegative")

    completion = contract["simulation_fixture_completion"]
    values["arm_carrier_mass_kg"] = finite_float(
        completion["arm_carrier_mass_kg"], "arm_carrier_mass_kg"
    )
    values["inertia_floor_kg_m2"] = finite_float(
        completion["inertia_floor_kg_m2"], "inertia_floor_kg_m2"
    )
    if values["arm_carrier_mass_kg"] <= 0.0 or values["inertia_floor_kg_m2"] <= 0.0:
        raise ValueError("simulation fixture completion values must be positive")
    return values


def validate_contract_against_urdf(contract: dict[str, Any], urdf_text: str) -> None:
    root = ET.fromstring(urdf_text)
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    for role in ("arm", "pendulum"):
        declared = contract["joints"][role]
        name = declared["name"]
        if name not in joints:
            raise ValueError(f"URDF is missing declared joint {name}")
        axis = joints[name].find("axis")
        if axis is None:
            raise ValueError(f"URDF joint {name} has no axis")
        rendered_axis = [float(item) for item in axis.attrib["xyz"].split()]
        declared_axis = [float(item) for item in declared["axis"]]
        if rendered_axis != declared_axis:
            raise ValueError(
                f"URDF axis for {name} {rendered_axis} != contract {declared_axis}"
            )

    mapping_names = [item["project_state"] for item in contract["state_mapping"]]
    if mapping_names != ["theta", "theta_dot", "phi", "phi_dot"]:
        raise ValueError("rigid-body state mapping must use canonical project state order")


def render_urdf(parameters_path: Path, contract_path: Path, template_path: Path) -> tuple[str, dict[str, float]]:
    values = load_render_values(parameters_path, contract_path)
    template = template_path.read_text(encoding="utf-8")
    rendered_values = {key: format(value, ".17g") for key, value in values.items()}
    rendered = template.format(**rendered_values)
    contract = load_json(contract_path)
    validate_contract_against_urdf(contract, rendered)
    return rendered, values


def write_manifest(
    path: Path,
    parameters_path: Path,
    contract_path: Path,
    urdf_text: str,
    values: dict[str, float],
) -> None:
    manifest = {
        "schema": 1,
        "model": "qnet-reference-furuta-rigid-body",
        "scope": "model-structure validation only; not Forest D1 specimen calibration",
        "parameters": str(parameters_path),
        "contract": str(contract_path),
        "urdf_sha256": hashlib.sha256(urdf_text.encode("utf-8")).hexdigest(),
        "rendered_values": values,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--parameters", type=Path, default=DEFAULT_PARAMETERS)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--template", type=Path, default=DEFAULT_TEMPLATE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    urdf_text, values = render_urdf(args.parameters, args.contract, args.template)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(urdf_text, encoding="utf-8")
    if args.manifest is not None:
        write_manifest(args.manifest, args.parameters, args.contract, urdf_text, values)
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
