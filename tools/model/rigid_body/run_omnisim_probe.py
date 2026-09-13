#!/usr/bin/env python3
"""Execute one generated #58 OmniSim/Newton probe and bind its evidence.

This is intentionally a developer/manual host runner. Normal CI prepares the
fixture but does not install the heavyweight OmniSim distribution. A successful
run still proves only simulator/model semantics; it grants no specimen or motor
authority.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from typing import Any


EXPLICIT_SEMANTIC_ENV = {
    "OMNISIM_NEWTON_TORQUE_MODE": "1",
    "OMNISIM_URDF_USE_INERTIA": "1",
    "OMNISIM_NEWTON_INERTIA_COM": "1",
    "OMNISIM_NEWTON_SPAWN_AT_POSITION": "1",
}
SIGN_ZERO_TOLERANCE = 1e-15


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def finite_float(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def parse_json_from_stdout(text: str) -> dict[str, Any]:
    """Accept doctor output with optional human-readable lines before JSON."""
    stripped = text.strip()
    if not stripped:
        raise ValueError("command produced no stdout JSON")
    try:
        value = json.loads(stripped)
        if isinstance(value, dict):
            return value
    except json.JSONDecodeError:
        pass
    lines = stripped.splitlines()
    for index, line in enumerate(lines):
        if not line.lstrip().startswith("{"):
            continue
        candidate = "\n".join(lines[index:])
        try:
            value = json.loads(candidate)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            return value
    raise ValueError("could not locate JSON object in stdout")


def expected_version(release: str) -> str:
    return release[1:] if release.startswith("v") else release


def check_doctor(doctor: dict[str, Any], manifest: dict[str, Any]) -> dict[str, Any]:
    expected = manifest["omnisim"]
    version = str(doctor.get("omnisim_version", ""))
    if version != expected_version(str(expected["release"])):
        raise RuntimeError(
            f"OmniSim version {version!r} != pinned {expected['release']!r}"
        )
    expected_commit = str(expected["commit"])
    observed_commit = str(doctor.get("git_commit") or "")
    commit_status = "not-reported-by-install"
    if observed_commit and observed_commit.lower() not in {"unknown", "none", "null"}:
        if observed_commit != expected_commit:
            raise RuntimeError(
                f"OmniSim git commit {observed_commit} != pinned {expected_commit}"
            )
        commit_status = "exact-match"
    return {
        "version": version,
        "commit_status": commit_status,
        "observed_commit": observed_commit or None,
    }


def check_sidecar(sidecar: dict[str, Any]) -> None:
    if sidecar.get("backend") != "newton":
        raise RuntimeError(f"backend verdict is not Newton: {sidecar}")
    if sidecar.get("degraded") is not False:
        raise RuntimeError(f"Newton run degraded or verdict missing: {sidecar}")
    if sidecar.get("finalised") is not True:
        raise RuntimeError(f"Newton world did not finalise: {sidecar}")
    solver = str(sidecar.get("solver", ""))
    if "MuJoCo" not in solver or "cpu/mj_step" not in solver:
        raise RuntimeError(f"unexpected Newton solver verdict: {solver!r}")


def check_probe_result(result: dict[str, Any], manifest: dict[str, Any]) -> None:
    if int(result.get("schema", 0)) != 1:
        raise RuntimeError("unsupported probe-result schema")
    if result.get("state_order") != manifest["probe"]["state_order"]:
        raise RuntimeError("probe result state order disagrees with fixture manifest")
    expected_state = [float(value) for value in manifest["probe"]["initial_state"]]
    observed_state = [float(value) for value in result.get("initial_state", [])]
    if observed_state != expected_state:
        raise RuntimeError(
            f"probe initial state {observed_state} != fixture {expected_state}"
        )
    expected_tau = finite_float(manifest["probe"]["input"]["arm_torque_nm"], "fixture tau")
    observed_tau = finite_float(result.get("arm_torque_nm"), "observed tau")
    if observed_tau != expected_tau:
        raise RuntimeError(f"probe torque {observed_tau} != fixture {expected_tau}")
    expected_step_ms = finite_float(
        manifest["omnisim"]["timing"]["basic_time_step_ms"], "fixture basic time step"
    )
    observed_step_ms = finite_float(result.get("basic_time_step_ms"), "observed basic time step")
    if observed_step_ms != expected_step_ms:
        raise RuntimeError(
            f"probe basicTimeStep {observed_step_ms} != fixture {expected_step_ms}"
        )
    samples = result.get("samples")
    if not isinstance(samples, list) or len(samples) < 2:
        raise RuntimeError("probe result must contain initial and at least one stepped sample")
    acceleration = result.get("first_step_acceleration_estimate", {})
    finite_float(acceleration.get("theta_ddot_rad_s2"), "theta_ddot")
    finite_float(acceleration.get("phi_ddot_rad_s2"), "phi_ddot")


def evaluate_causality(result: dict[str, Any]) -> dict[str, Any]:
    """Apply only the coordinate/sign predictions that the probe state licenses."""
    theta0, theta_dot0, phi0, phi_dot0 = [float(value) for value in result["initial_state"]]
    tau = float(result["arm_torque_nm"])
    acceleration = result["first_step_acceleration_estimate"]
    theta_ddot = float(acceleration["theta_ddot_rad_s2"])
    phi_ddot = float(acceleration["phi_ddot_rad_s2"])

    zero_rates = abs(theta_dot0) <= SIGN_ZERO_TOLERANCE and abs(phi_dot0) <= SIGN_ZERO_TOLERANCE
    checks: list[dict[str, Any]] = []
    if zero_rates and abs(theta0) <= SIGN_ZERO_TOLERANCE and abs(phi0) <= SIGN_ZERO_TOLERANCE and abs(tau) > SIGN_ZERO_TOLERANCE:
        checks.extend(
            [
                {
                    "claim": "arm acceleration follows applied arm-torque sign",
                    "passed": phi_ddot * tau > 0.0,
                    "observed": {"phi_ddot_rad_s2": phi_ddot, "arm_torque_nm": tau},
                },
                {
                    "claim": "upright pendulum acceleration opposes applied arm-torque sign",
                    "passed": theta_ddot * tau < 0.0,
                    "observed": {"theta_ddot_rad_s2": theta_ddot, "arm_torque_nm": tau},
                },
            ]
        )
    elif zero_rates and abs(phi0) <= SIGN_ZERO_TOLERANCE and abs(tau) <= SIGN_ZERO_TOLERANCE and abs(theta0) > SIGN_ZERO_TOLERANCE:
        checks.append(
            {
                "claim": "unforced upright equilibrium is unstable in theta",
                "passed": theta_ddot * theta0 > 0.0,
                "observed": {"theta0_rad": theta0, "theta_ddot_rad_s2": theta_ddot},
            }
        )

    return {
        "applicable": bool(checks),
        "passed": bool(checks) and all(bool(item["passed"]) for item in checks),
        "checks": checks,
    }


def run(args: argparse.Namespace) -> dict[str, Any]:
    fixture = args.fixture_dir.resolve()
    manifest_path = fixture / "manifest.json"
    world_path = fixture / "worlds" / "furuta_probe.omniworld"
    if not manifest_path.is_file() or not world_path.is_file():
        raise FileNotFoundError(
            "fixture directory must contain manifest.json and worlds/furuta_probe.omniworld"
        )
    manifest = load_json(manifest_path)

    evidence_dir = args.evidence_dir.resolve()
    evidence_dir.mkdir(parents=True, exist_ok=True)
    result_path = evidence_dir / "probe-result.json"
    log_path = evidence_dir / "omnisim.log"
    sidecar_path = Path(str(log_path) + ".newton.json")
    stdout_path = evidence_dir / "runner-stdout.txt"
    stderr_path = evidence_dir / "runner-stderr.txt"

    doctor_cmd = [args.python, "-m", "omnisim", "doctor", "--json"]
    doctor_run = subprocess.run(
        doctor_cmd,
        capture_output=True,
        text=True,
        timeout=args.doctor_timeout_s,
        check=False,
    )
    if doctor_run.returncode != 0:
        raise RuntimeError(
            f"OmniSim doctor failed ({doctor_run.returncode}): {doctor_run.stderr.strip()}"
        )
    doctor = parse_json_from_stdout(doctor_run.stdout)
    doctor_check = check_doctor(doctor, manifest)

    env = dict(os.environ)
    for key, value in EXPLICIT_SEMANTIC_ENV.items():
        env[key] = value
    env["ROTARY_OMNISIM_PROBE_OUT"] = str(result_path)
    env["OMNISIM_LOG_PATH"] = str(log_path)

    run_cmd = [
        args.python,
        "-m",
        "omnisim",
        "run-headless",
        str(world_path),
        "--duration",
        format(args.duration_s, ".17g"),
    ]
    completed = subprocess.run(
        run_cmd,
        capture_output=True,
        text=True,
        timeout=args.process_timeout_s,
        env=env,
        check=False,
    )
    stdout_path.write_text(completed.stdout, encoding="utf-8")
    stderr_path.write_text(completed.stderr, encoding="utf-8")
    if completed.returncode != 0:
        raise RuntimeError(
            f"OmniSim run-headless failed ({completed.returncode}); see {stderr_path}"
        )
    if not result_path.is_file():
        raise RuntimeError(
            "probe controller produced no result file; controller stdout is not an accepted evidence path"
        )
    if not sidecar_path.is_file():
        raise RuntimeError("Newton backend verdict sidecar is missing")

    result = load_json(result_path)
    sidecar = load_json(sidecar_path)
    check_probe_result(result, manifest)
    check_sidecar(sidecar)
    causality = evaluate_causality(result)
    if causality["applicable"] and not causality["passed"]:
        raise RuntimeError(f"OmniSim probe violated project causality contract: {causality}")

    evidence = {
        "schema": 1,
        "scope": "model-structure validation only; not Forest D1 specimen calibration",
        "fixture_manifest": manifest,
        "doctor": doctor,
        "doctor_check": doctor_check,
        "semantic_environment": EXPLICIT_SEMANTIC_ENV,
        "backend_verdict": sidecar,
        "probe_result": result,
        "causality": causality,
        "execution": {
            "doctor_command": doctor_cmd,
            "run_command": run_cmd,
            "returncode": completed.returncode,
            "stdout_file": str(stdout_path),
            "stderr_file": str(stderr_path),
        },
    }
    evidence_path = evidence_dir / "evidence.json"
    evidence_path.write_text(
        json.dumps(evidence, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return evidence


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture-dir", type=Path, required=True)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--python", default=sys.executable)
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--doctor-timeout-s", type=float, default=30.0)
    parser.add_argument("--process-timeout-s", type=float, default=90.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    evidence = run(args)
    accel = evidence["probe_result"]["first_step_acceleration_estimate"]
    print(
        json.dumps(
            {
                "evidence": str((args.evidence_dir.resolve() / "evidence.json")),
                "theta_ddot_rad_s2": accel["theta_ddot_rad_s2"],
                "phi_ddot_rad_s2": accel["phi_ddot_rad_s2"],
                "causality": evidence["causality"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
