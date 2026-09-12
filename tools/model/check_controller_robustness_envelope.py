#!/usr/bin/env python3
"""Pressure-test local controller baselines across explicit simulation mismatch.

The suite intentionally separates four kinds of causes:
- initial-state offsets,
- plant-parameter perturbations,
- additive external plant torque,
- simulated actuator-capability reduction.

Each perturbation is declared in a machine-readable fixture. The magnitudes are
engineering pressure tests, not measured Forest D1 uncertainty bounds. Runs are
executed against both the source-backed reduced QNET model and the independent
geometry-derived full-3D analytical model so a controller is not judged only by
the model class from which its local linearization originated.

CI gates harness integrity, control-torque bounding, and nominal recovery on the
reduced QNET design model from which these local gains were derived. Recovery on
the independent full-3D model and on perturbed cases is robustness evidence, not
a CI requirement. A failed independent-model or pressure-test scenario must stay
visible rather than being tuned away to make CI green.
"""

from __future__ import annotations

import json
from pathlib import Path
import re
import tomllib
from typing import Any, Callable

import numpy as np

from reference_furuta import derivative as reduced_derivative
from rigid_body.full3d_furuta import derivative as full3d_derivative

ROOT = Path(__file__).resolve().parents[2]
PARAMETERS = ROOT / "parameters" / "reference-assembly.json"
BALANCE_SCENARIO = ROOT / "tools" / "sitl" / "scenarios" / "rotary_balance.toml"
ENVELOPE = ROOT / "tools" / "model" / "fixtures" / "controller_robustness_envelope.json"
RUST_CONTROLLER = ROOT / "control" / "state-feedback" / "src" / "lib.rs"

PROFILE_CONSTANTS = {
    "qnet_lqr": "QNET_REFERENCE_TORQUE_GAINS",
    "pole_placement_c1": "QNET_POLE_PLACEMENT_C1_TORQUE_GAINS",
    "pole_placement_c2": "QNET_POLE_PLACEMENT_C2_TORQUE_GAINS",
}

MODEL_DERIVATIVES: dict[str, Callable[[dict[str, float], np.ndarray, float], np.ndarray]] = {
    "reduced_qnet": reduced_derivative,
    "full3d_geometry": full3d_derivative,
}

DESIGN_MODEL = "reduced_qnet"
STATE_INDEX = {
    "theta_rad": 0,
    "theta_dot_rad_s": 1,
    "phi_rad": 2,
    "phi_dot_rad_s": 3,
}


def parse_rust_gains(name: str) -> np.ndarray:
    source = RUST_CONTROLLER.read_text(encoding="utf-8")
    match = re.search(
        rf"{re.escape(name)}\s*:\s*\[f32;\s*4\]\s*=\s*\[([^\]]+)\]",
        source,
        flags=re.MULTILINE,
    )
    if match is None:
        raise ValueError(f"cannot locate {name} in Control source")
    values = [
        float(token.strip().replace("_", ""))
        for token in match.group(1).split(",")
        if token.strip()
    ]
    if len(values) != 4:
        raise ValueError(f"{name} must contain four gains")
    return np.asarray(values, dtype=np.float64)


def load_contract() -> tuple[dict[str, float], np.ndarray, float, dict[str, Any]]:
    registry = json.loads(PARAMETERS.read_text(encoding="utf-8"))
    scenario = tomllib.loads(BALANCE_SCENARIO.read_text(encoding="utf-8"))
    envelope = json.loads(ENVELOPE.read_text(encoding="utf-8"))

    plant = {key: float(record["value"]) for key, record in registry["plant"].items()}
    rotary = scenario["rotary"]
    initial = np.asarray(
        [
            rotary["initial_theta_rad"],
            rotary["initial_theta_dot_rad_s"],
            rotary["initial_phi_rad"],
            rotary["initial_phi_dot_rad_s"],
        ],
        dtype=np.float64,
    )
    max_abs_torque = float(
        registry["production_actuator_model"]["torque_per_effective_command_nm"]["value"]
    )
    return plant, initial, max_abs_torque, envelope


def rk4_step(
    derivative: Callable[[dict[str, float], np.ndarray, float], np.ndarray],
    plant: dict[str, float],
    state: np.ndarray,
    torque_nm: float,
    dt_s: float,
) -> np.ndarray:
    k1 = derivative(plant, state, torque_nm)
    k2 = derivative(plant, state + 0.5 * dt_s * k1, torque_nm)
    k3 = derivative(plant, state + 0.5 * dt_s * k2, torque_nm)
    k4 = derivative(plant, state + dt_s * k3, torque_nm)
    return state + (dt_s / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def scenario_contract(
    base_plant: dict[str, float],
    base_initial: np.ndarray,
    base_max_torque: float,
    scenario: dict[str, Any],
) -> tuple[dict[str, float], np.ndarray, float, dict[str, Any] | None]:
    plant = dict(base_plant)
    initial = base_initial.copy()

    for state_name, value in scenario.get("initial_state_override", {}).items():
        if state_name not in STATE_INDEX:
            raise ValueError(f"unknown state override {state_name!r}")
        initial[STATE_INDEX[state_name]] = float(value)

    for parameter, scale in scenario.get("plant_scale", {}).items():
        if parameter not in plant:
            raise ValueError(f"unknown plant parameter {parameter!r}")
        scale_value = float(scale)
        if not np.isfinite(scale_value) or scale_value <= 0.0:
            raise ValueError(f"invalid scale for {parameter!r}")
        plant[parameter] *= scale_value

    actuator_limit_scale = float(scenario.get("actuator_limit_scale", 1.0))
    if not np.isfinite(actuator_limit_scale) or actuator_limit_scale <= 0.0:
        raise ValueError("actuator_limit_scale must be finite and positive")
    max_control_torque = base_max_torque * actuator_limit_scale

    disturbance = scenario.get("external_arm_torque")
    if disturbance is not None:
        disturbance = {
            "start_us": int(disturbance["start_us"]),
            "duration_us": int(disturbance["duration_us"]),
            "torque_nm": float(disturbance["torque_nm"]),
        }
        if disturbance["start_us"] < 0 or disturbance["duration_us"] <= 0:
            raise ValueError("invalid external disturbance timing")
        if not np.isfinite(disturbance["torque_nm"]):
            raise ValueError("external disturbance torque must be finite")

    return plant, initial, max_control_torque, disturbance


def inside_recovery_window(
    state: np.ndarray,
    evaluation: dict[str, Any],
) -> bool:
    return (
        abs(float(state[0])) <= float(evaluation["final_abs_theta_rad_max"])
        and abs(float(state[1])) <= float(evaluation["final_abs_theta_dot_rad_s_max"])
    )


def simulate(
    derivative: Callable[[dict[str, float], np.ndarray, float], np.ndarray],
    base_plant: dict[str, float],
    base_initial: np.ndarray,
    base_max_torque: float,
    gains: np.ndarray,
    scenario: dict[str, Any],
    envelope: dict[str, Any],
) -> dict[str, Any]:
    plant, state, control_limit, disturbance = scenario_contract(
        base_plant,
        base_initial,
        base_max_torque,
        scenario,
    )

    sample_period_us = int(envelope["sample_period_us"])
    integration_step_us = int(envelope["integration_step_us"])
    duration_us = int(envelope["duration_us"])
    settle_hold_samples = int(envelope["settle_hold_samples"])
    evaluation = envelope["evaluation"]

    if sample_period_us <= 0 or integration_step_us <= 0 or duration_us <= 0:
        raise ValueError("simulation periods must be positive")
    if sample_period_us % integration_step_us != 0:
        raise ValueError("integration step must divide the control sample period")
    if duration_us % sample_period_us != 0:
        raise ValueError("duration must align to the control sample period")
    if settle_hold_samples <= 0:
        raise ValueError("settle_hold_samples must be positive")

    substeps = sample_period_us // integration_step_us
    samples = duration_us // sample_period_us
    dt_s = integration_step_us * 1.0e-6
    sample_dt_s = sample_period_us * 1.0e-6

    max_abs_state = np.abs(state).copy()
    max_abs_control_torque = 0.0
    max_abs_total_plant_torque = 0.0
    saturated_samples = 0
    theta_abs_integral = 0.0
    torque_squared_integral = 0.0
    recovery_history: list[bool] = []
    finite = bool(np.all(np.isfinite(state)))

    for sample_index in range(samples):
        at_us = sample_index * sample_period_us
        requested_control_torque = -float(gains @ state)
        applied_control_torque = float(
            np.clip(requested_control_torque, -control_limit, control_limit)
        )
        if abs(requested_control_torque) > control_limit:
            saturated_samples += 1

        external_torque = 0.0
        if disturbance is not None:
            end_us = disturbance["start_us"] + disturbance["duration_us"]
            if disturbance["start_us"] <= at_us < end_us:
                external_torque = float(disturbance["torque_nm"])

        total_plant_torque = applied_control_torque + external_torque
        max_abs_control_torque = max(max_abs_control_torque, abs(applied_control_torque))
        max_abs_total_plant_torque = max(max_abs_total_plant_torque, abs(total_plant_torque))
        theta_abs_integral += abs(float(state[0])) * sample_dt_s
        torque_squared_integral += applied_control_torque**2 * sample_dt_s

        for _ in range(substeps):
            state = rk4_step(derivative, plant, state, total_plant_torque, dt_s)
            if not np.all(np.isfinite(state)):
                finite = False
                break
            max_abs_state = np.maximum(max_abs_state, np.abs(state))
        if not finite:
            break
        recovery_history.append(inside_recovery_window(state, evaluation))

    enough_history = len(recovery_history) >= settle_hold_samples
    recovered = enough_history and all(recovery_history[-settle_hold_samples:])
    settling_time_us: int | None = None
    if recovered:
        last_outside = max(
            (index for index, inside in enumerate(recovery_history) if not inside),
            default=-1,
        )
        settling_time_us = (last_outside + 1) * sample_period_us

    return {
        "recovered": recovered,
        "settling_time_us": settling_time_us,
        "finite": finite,
        "control_torque_bound_respected": max_abs_control_torque <= control_limit + 1.0e-12,
        "control_torque_limit_nm": control_limit,
        "max_abs_control_torque_nm": max_abs_control_torque,
        "max_abs_total_plant_torque_nm": max_abs_total_plant_torque,
        "saturated_samples": saturated_samples,
        "theta_abs_integral_rad_s": theta_abs_integral,
        "control_torque_squared_integral_nm2_s": torque_squared_integral,
        "max_abs_state": {
            "theta_rad": float(max_abs_state[0]),
            "theta_dot_rad_s": float(max_abs_state[1]),
            "phi_rad": float(max_abs_state[2]),
            "phi_dot_rad_s": float(max_abs_state[3]),
        },
        "final_state": {
            "theta_rad": float(state[0]),
            "theta_dot_rad_s": float(state[1]),
            "phi_rad": float(state[2]),
            "phi_dot_rad_s": float(state[3]),
        },
    }


def main() -> int:
    base_plant, base_initial, base_max_torque, envelope = load_contract()
    scenarios = envelope["scenarios"]
    scenario_ids = [str(scenario["id"]) for scenario in scenarios]
    if len(scenario_ids) != len(set(scenario_ids)):
        raise ValueError("robustness scenario ids must be unique")
    if scenario_ids.count("nominal") != 1:
        raise ValueError("robustness envelope must contain exactly one nominal scenario")

    profiles = {
        profile: parse_rust_gains(constant)
        for profile, constant in PROFILE_CONSTANTS.items()
    }

    results: dict[str, Any] = {}
    harness_pass = True
    robustness_nonrecoveries = 0
    required_design_nominal_failures = 0

    for model_name, derivative in MODEL_DERIVATIVES.items():
        model_results: dict[str, Any] = {}
        for profile_name, gains in profiles.items():
            profile_results: dict[str, Any] = {}
            for scenario in scenarios:
                result = simulate(
                    derivative,
                    base_plant,
                    base_initial,
                    base_max_torque,
                    gains,
                    scenario,
                    envelope,
                )
                scenario_id = str(scenario["id"])
                result["class"] = scenario["class"]
                result["rationale"] = scenario["rationale"]
                profile_results[scenario_id] = result

                harness_pass = (
                    harness_pass
                    and bool(result["finite"])
                    and bool(result["control_torque_bound_respected"])
                )

                is_required_design_nominal = (
                    model_name == DESIGN_MODEL and scenario_id == "nominal"
                )
                if is_required_design_nominal:
                    if not result["recovered"]:
                        required_design_nominal_failures += 1
                        harness_pass = False
                elif not result["recovered"]:
                    robustness_nonrecoveries += 1

            model_results[profile_name] = {
                "gains": gains.tolist(),
                "scenarios": profile_results,
                "recovered_count": sum(
                    bool(result["recovered"]) for result in profile_results.values()
                ),
                "scenario_count": len(profile_results),
            }
        results[model_name] = model_results

    payload = {
        "schema": 1,
        "fixture": str(ENVELOPE.relative_to(ROOT)),
        "state_order": ["theta", "theta_dot", "phi", "phi_dot"],
        "feedback_law": "1 kHz sample-and-hold u = clamp(-Kx)",
        "model_classes": list(MODEL_DERIVATIVES),
        "design_model": DESIGN_MODEL,
        "nominal_reference_plant": str(PARAMETERS.relative_to(ROOT)),
        "nominal_initial_state": base_initial.tolist(),
        "nominal_control_torque_limit_nm": base_max_torque,
        "results": results,
        "required_design_nominal_failures": required_design_nominal_failures,
        "robustness_nonrecoveries": robustness_nonrecoveries,
        "ci_policy": {
            "requires_all_runs_finite": True,
            "requires_control_torque_bound_respected": True,
            "requires_reduced_qnet_nominal_recovery_for_every_controller": True,
            "full3d_nominal_nonrecovery_is_reported_not_ci_failure": True,
            "perturbed_nonrecovery_is_reported_not_ci_failure": True,
        },
        "scope": envelope["scope"],
        "pass": harness_pass,
    }
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if harness_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
