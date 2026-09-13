#!/usr/bin/env python3
"""Characterize task authority of the reduced QNET model against full 3-D dynamics.

This check is deliberately task-specific. It does not ask which model is globally
"more accurate" and it does not create Forest D1 specimen authority. Instead it
quantifies where the source-backed reduced QNET dynamics and the independent
geometry-derived full-3D analytical dynamics make sufficiently similar local
controller decisions for a declared short-horizon engineering margin.

The output keeps three claims separate:

1. local controller reasoning, where upright linearization equivalence plus a
   bounded short-horizon command-divergence envelope can justify the reduced
   model;
2. capture / swing-up prediction, where larger configuration- and velocity-
   dependent terms require full-3D validation;
3. physical authority, which remains unavailable until physical evidence exists.
"""

from __future__ import annotations

import json
from pathlib import Path
import re
from typing import Any, Callable

import numpy as np

from reference_furuta import derivative as reduced_derivative
from rigid_body.full3d_furuta import derivative as full3d_derivative

ROOT = Path(__file__).resolve().parents[2]
PARAMETERS = ROOT / "parameters" / "reference-assembly.json"
FIXTURE = ROOT / "tools" / "model" / "fixtures" / "reduced_model_validity_envelope.json"
RUST_CONTROLLER = ROOT / "control" / "state-feedback" / "src" / "lib.rs"
CONTROLLER_CONSTANT = "QNET_REFERENCE_TORQUE_GAINS"

Derivative = Callable[[dict[str, float], np.ndarray, float], np.ndarray]


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
    if len(values) != 4 or not np.all(np.isfinite(values)):
        raise ValueError(f"{name} must contain four finite gains")
    return np.asarray(values, dtype=np.float64)


def load_contract() -> tuple[dict[str, float], float, np.ndarray, dict[str, Any]]:
    registry = json.loads(PARAMETERS.read_text(encoding="utf-8"))
    fixture = json.loads(FIXTURE.read_text(encoding="utf-8"))
    plant = {
        key: float(record["value"])
        for key, record in registry["plant"].items()
    }
    actuator_limit_nm = float(
        registry["production_actuator_model"]["torque_per_effective_command_nm"]["value"]
    )
    gains = parse_rust_gains(CONTROLLER_CONSTANT)

    if not np.isfinite(actuator_limit_nm) or actuator_limit_nm <= 0.0:
        raise ValueError("actuator torque limit must be positive and finite")
    return plant, actuator_limit_nm, gains, fixture


def rk4_step(
    derivative: Derivative,
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


def controller_command(state: np.ndarray, gains: np.ndarray, limit_nm: float) -> tuple[float, bool]:
    requested = -float(gains @ state)
    applied = float(np.clip(requested, -limit_nm, limit_nm))
    return applied, abs(requested) > limit_nm


def compare_closed_loop(
    plant: dict[str, float],
    initial_state: np.ndarray,
    gains: np.ndarray,
    actuator_limit_nm: float,
    fixture: dict[str, Any],
) -> dict[str, Any]:
    integration_step_us = int(fixture["integration_step_us"])
    sample_period_us = int(fixture["control_sample_period_us"])
    horizon_us = int(fixture["comparison_horizon_us"])
    if integration_step_us <= 0 or sample_period_us <= 0 or horizon_us <= 0:
        raise ValueError("comparison time values must be positive")
    if sample_period_us % integration_step_us != 0:
        raise ValueError("integration step must divide the controller sample period")
    if horizon_us % sample_period_us != 0:
        raise ValueError("comparison horizon must align to the controller sample period")

    reduced_state = initial_state.copy()
    full3d_state = initial_state.copy()
    dt_s = integration_step_us * 1.0e-6
    substeps = sample_period_us // integration_step_us
    samples = horizon_us // sample_period_us

    max_state_divergence = np.zeros(4, dtype=np.float64)
    max_command_divergence_nm = 0.0
    reduced_saturated_samples = 0
    full3d_saturated_samples = 0
    finite = True

    for _sample in range(samples):
        reduced_command, reduced_saturated = controller_command(
            reduced_state, gains, actuator_limit_nm
        )
        full3d_command, full3d_saturated = controller_command(
            full3d_state, gains, actuator_limit_nm
        )
        reduced_saturated_samples += int(reduced_saturated)
        full3d_saturated_samples += int(full3d_saturated)
        max_command_divergence_nm = max(
            max_command_divergence_nm,
            abs(reduced_command - full3d_command),
        )

        for _substep in range(substeps):
            reduced_state = rk4_step(
                reduced_derivative,
                plant,
                reduced_state,
                reduced_command,
                dt_s,
            )
            full3d_state = rk4_step(
                full3d_derivative,
                plant,
                full3d_state,
                full3d_command,
                dt_s,
            )
            if not (
                np.all(np.isfinite(reduced_state))
                and np.all(np.isfinite(full3d_state))
            ):
                finite = False
                break
            max_state_divergence = np.maximum(
                max_state_divergence,
                np.abs(reduced_state - full3d_state),
            )
        if not finite:
            break

    return {
        "finite": finite,
        "max_controller_command_divergence_nm": max_command_divergence_nm,
        "max_state_divergence": {
            "theta_rad": float(max_state_divergence[0]),
            "theta_dot_rad_s": float(max_state_divergence[1]),
            "phi_rad": float(max_state_divergence[2]),
            "phi_dot_rad_s": float(max_state_divergence[3]),
        },
        "reduced_saturated_samples": reduced_saturated_samples,
        "full3d_saturated_samples": full3d_saturated_samples,
        "final_reduced_state": reduced_state.tolist(),
        "final_full3d_state": full3d_state.tolist(),
    }


def local_product_sweep(
    plant: dict[str, float],
    gains: np.ndarray,
    actuator_limit_nm: float,
    fixture: dict[str, Any],
) -> dict[str, Any]:
    sweep = fixture["local_product_sweep"]
    margin_fraction = float(
        fixture["decision_margin"][
            "max_controller_command_divergence_fraction_of_actuator_limit"
        ]
    )
    if not 0.0 < margin_fraction < 1.0:
        raise ValueError("command-divergence margin fraction must be in (0, 1)")
    command_margin_nm = margin_fraction * actuator_limit_nm

    candidate_results: dict[str, Any] = {}
    fully_within_margin: list[float] = []

    for abs_theta_deg_raw in sweep["abs_theta_deg_candidates"]:
        abs_theta_deg = float(abs_theta_deg_raw)
        if not np.isfinite(abs_theta_deg) or abs_theta_deg < 0.0:
            raise ValueError("theta candidates must be finite and nonnegative")

        worst_command_divergence_nm = -1.0
        worst_case: dict[str, Any] | None = None
        all_finite = True
        case_count = 0

        signs = [1] if abs_theta_deg == 0.0 else sweep["theta_signs"]
        for theta_sign in signs:
            for theta_dot in sweep["theta_dot_rad_s"]:
                for phi_dot in sweep["phi_dot_rad_s"]:
                    state = np.asarray(
                        [
                            float(theta_sign) * np.deg2rad(abs_theta_deg),
                            float(theta_dot),
                            float(sweep["phi_rad"]),
                            float(phi_dot),
                        ],
                        dtype=np.float64,
                    )
                    result = compare_closed_loop(
                        plant,
                        state,
                        gains,
                        actuator_limit_nm,
                        fixture,
                    )
                    case_count += 1
                    all_finite = all_finite and bool(result["finite"])
                    divergence = float(result["max_controller_command_divergence_nm"])
                    if divergence > worst_command_divergence_nm:
                        worst_command_divergence_nm = divergence
                        worst_case = {
                            "initial_state": state.tolist(),
                            **result,
                        }

        within_margin = all_finite and worst_command_divergence_nm <= command_margin_nm
        if within_margin:
            fully_within_margin.append(abs_theta_deg)
        candidate_results[f"{abs_theta_deg:g}"] = {
            "case_count": case_count,
            "all_finite": all_finite,
            "within_declared_command_margin": within_margin,
            "worst_controller_command_divergence_nm": worst_command_divergence_nm,
            "worst_controller_command_divergence_fraction_of_actuator_limit": (
                worst_command_divergence_nm / actuator_limit_nm
            ),
            "worst_case": worst_case,
        }

    max_within = max(fully_within_margin, default=None)
    return {
        "controller_command_divergence_limit_nm": command_margin_nm,
        "candidate_results": candidate_results,
        "max_fully_within_margin_abs_theta_deg": max_within,
    }


def structural_probe(
    plant: dict[str, float],
    probe: dict[str, Any],
) -> dict[str, Any]:
    state = np.asarray(probe["state"], dtype=np.float64)
    torque_nm = float(probe["arm_torque_nm"])
    reduced = reduced_derivative(plant, state, torque_nm)
    full3d = full3d_derivative(plant, state, torque_nm)
    delta = full3d - reduced

    def acceleration_payload(values: np.ndarray) -> dict[str, float]:
        return {
            "theta_ddot_rad_s2": float(values[1]),
            "phi_ddot_rad_s2": float(values[3]),
        }

    return {
        "task": probe["task"],
        "state": state.tolist(),
        "arm_torque_nm": torque_nm,
        "reduced_qnet": acceleration_payload(reduced),
        "full3d_geometry": acceleration_payload(full3d),
        "full_minus_reduced": acceleration_payload(delta),
    }


def main() -> int:
    plant, actuator_limit_nm, gains, fixture = load_contract()
    sweep = local_product_sweep(plant, gains, actuator_limit_nm, fixture)
    probes = {
        probe["id"]: structural_probe(plant, probe)
        for probe in fixture["structural_probes"]
    }

    acceptance = fixture["acceptance"]
    minimum_local_deg = float(acceptance["minimum_fully_within_margin_abs_theta_deg"])
    must_exceed_by_deg = float(acceptance["must_exceed_margin_by_abs_theta_deg"])
    max_within = sweep["max_fully_within_margin_abs_theta_deg"]
    candidate_results = sweep["candidate_results"]
    required_candidate = candidate_results.get(f"{must_exceed_by_deg:g}")
    if required_candidate is None:
        raise ValueError("must_exceed_margin_by_abs_theta_deg must be a declared theta candidate")

    minimum_local_pass = max_within is not None and float(max_within) >= minimum_local_deg
    larger_region_is_distinct = not bool(
        required_candidate["within_declared_command_margin"]
    )
    harness_finite = all(
        bool(result["all_finite"])
        for result in candidate_results.values()
    )
    passed = minimum_local_pass and larger_region_is_distinct and harness_finite

    payload = {
        "schema": 1,
        "fixture": str(FIXTURE.relative_to(ROOT)),
        "parameter_registry": str(PARAMETERS.relative_to(ROOT)),
        "state_order": ["theta", "theta_dot", "phi", "phi_dot"],
        "controller_profile": fixture["controller_profile"],
        "controller_gains": gains.tolist(),
        "actuator_torque_limit_nm": actuator_limit_nm,
        "model_classes": fixture["model_classes"],
        "scope": fixture["scope"],
        "local_product_sweep": sweep,
        "structural_probes": probes,
        "acceptance": {
            "minimum_local_region_pass": minimum_local_pass,
            "larger_region_is_distinct": larger_region_is_distinct,
            "all_local_sweep_runs_finite": harness_finite,
            "minimum_fully_within_margin_abs_theta_deg": minimum_local_deg,
            "must_exceed_margin_by_abs_theta_deg": must_exceed_by_deg,
        },
        "task_authority": {
            "local_controller_reasoning": {
                "reduced_qnet": "allowed within the explicitly tested near-upright engineering envelope; upright first-order equivalence is checked separately in CI",
                "tested_abs_theta_deg": max_within,
                "theta_dot_probe_range_rad_s": [
                    min(fixture["local_product_sweep"]["theta_dot_rad_s"]),
                    max(fixture["local_product_sweep"]["theta_dot_rad_s"]),
                ],
                "phi_dot_probe_range_rad_s": [
                    min(fixture["local_product_sweep"]["phi_dot_rad_s"]),
                    max(fixture["local_product_sweep"]["phi_dot_rad_s"]),
                ],
                "comparison_horizon_us": fixture["comparison_horizon_us"],
            },
            "capture_prediction": "requires full-3D cross-check because the tested model disagreement is already outside the declared command-decision margin by the current 8 degree balance-entry boundary",
            "swing_up_prediction": "full-3D analytical or independent rigid-body validation required; the reduced model is not granted large-angle/high-rate predictive authority",
            "semantic_path_sitl": "the reduced plant may remain the fast deterministic default for software/semantic-path validation; dynamic conclusions outside its declared local envelope require the richer validation model class",
            "physical_authority": "none; neither model class is Forest D1 specimen calibration or permission to enable automatic physical actuation",
        },
        "pass": passed,
    }

    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
