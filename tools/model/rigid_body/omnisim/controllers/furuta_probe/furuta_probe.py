#!/usr/bin/env python3
"""Bounded OmniSim/Newton instantaneous Furuta probe for issue #58.

Run only with OMNISIM_NEWTON_TORQUE_MODE=1 so motor.setTorque reaches Newton as
an effort command instead of competing with a position/velocity servo. The
fixture starts from an authored <rest> state with zero declared rates.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys

from omnisim import Supervisor


ARM_MOTOR = "arm_joint_motor"
PENDULUM_MOTOR = "pendulum_joint_motor"
RESULT_PREFIX = "ROTARY_OMNISIM_JSON="


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--tau", type=float, required=True)
    parser.add_argument("--theta0", type=float, required=True)
    parser.add_argument("--phi0", type=float, required=True)
    parser.add_argument("--steps", type=int, default=1)
    args, unknown = parser.parse_known_args()
    if unknown:
        raise ValueError(f"unexpected controller arguments: {unknown}")
    return args


def finite(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def main() -> int:
    args = parse_args()
    tau = finite(args.tau, "tau")
    theta0 = finite(args.theta0, "theta0")
    phi0 = finite(args.phi0, "phi0")
    if args.steps < 1:
        raise ValueError("steps must be >= 1")
    if os.environ.get("OMNISIM_NEWTON_TORQUE_MODE") != "1":
        raise RuntimeError("OMNISIM_NEWTON_TORQUE_MODE=1 is required for #58 effort semantics")

    robot = Supervisor()
    step_ms = int(robot.getBasicTimeStep())
    if step_ms <= 0:
        raise RuntimeError(f"invalid basicTimeStep {step_ms}")
    dt = step_ms / 1000.0

    arm_motor = robot.getDevice(ARM_MOTOR)
    pendulum_motor = robot.getDevice(PENDULUM_MOTOR)
    if arm_motor is None or pendulum_motor is None:
        raise RuntimeError(
            f"missing imported motor(s): arm={arm_motor is not None}, pendulum={pendulum_motor is not None}"
        )
    arm_sensor = arm_motor.getPositionSensor()
    pendulum_sensor = pendulum_motor.getPositionSensor()
    if arm_sensor is None or pendulum_sensor is None:
        raise RuntimeError("imported Furuta joints must expose position sensors")
    arm_sensor.enable(step_ms)
    pendulum_sensor.enable(step_ms)

    # Direct torque command on arm; pendulum remains passive.
    pendulum_motor.setTorque(0.0)
    arm_motor.setTorque(tau)

    samples: list[dict[str, float]] = [
        {
            "t_s": 0.0,
            "theta": theta0,
            "phi": phi0,
        }
    ]
    for index in range(1, args.steps + 1):
        if robot.step(step_ms) == -1:
            raise RuntimeError(f"simulation ended before requested step {index}/{args.steps}")
        theta = finite(pendulum_sensor.getValue(), "theta")
        phi = finite(arm_sensor.getValue(), "phi")
        samples.append({"t_s": index * dt, "theta": theta, "phi": phi})

    first = samples[1]
    # Probe fixtures author zero rates. A one-step position displacement therefore
    # gives a bounded finite-step acceleration estimate without differentiating a
    # noisy velocity channel: q(dt)=q0+0.5*qddot*dt^2+O(dt^3).
    theta_ddot = 2.0 * (first["theta"] - theta0) / (dt * dt)
    phi_ddot = 2.0 * (first["phi"] - phi0) / (dt * dt)
    result = {
        "schema": 1,
        "backend": "OmniSim/Newton CPU mj_step",
        "scope": "model-structure validation only; not Forest D1 specimen calibration",
        "state_order": ["theta", "theta_dot", "phi", "phi_dot"],
        "initial_state": [theta0, 0.0, phi0, 0.0],
        "arm_torque_nm": tau,
        "basic_time_step_ms": step_ms,
        "steps": args.steps,
        "first_step_acceleration_estimate": {
            "theta_ddot_rad_s2": theta_ddot,
            "phi_ddot_rad_s2": phi_ddot,
        },
        "samples": samples,
    }

    encoded = json.dumps(result, sort_keys=True, separators=(",", ":"))
    output = os.environ.get("ROTARY_OMNISIM_PROBE_OUT")
    if output:
        path = Path(output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(RESULT_PREFIX + encoded, flush=True)
    robot.simulationQuit(0)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # noqa: BLE001 - controller must make failure visible in headless logs.
        print(f"ROTARY_OMNISIM_ERROR={type(exc).__name__}:{exc}", file=sys.stderr, flush=True)
        raise
