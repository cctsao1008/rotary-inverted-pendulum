#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys

from device import Rp2350Device
import motor
import sensors
import sysid


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="RP2350 rotary-inverted-pendulum commissioning tool (CDC + HID)"
    )
    parser.add_argument("--cdc-port", help="override auto-detected CDC COM/tty port")
    parser.add_argument("--hid-path", help="override auto-detected hidapi path")

    sub = parser.add_subparsers(dest="action", required=True)

    sub.add_parser("status")
    sub.add_parser("safe-off")

    p = sub.add_parser("led", help="control the onboard D13 blue user LED")
    p.add_argument("state", choices=("on", "off"))

    p = sub.add_parser("monitor")
    p.add_argument("--duration", type=float, default=10.0)
    p.add_argument("--motor-command", type=float)

    p = sub.add_parser("adc")
    p.add_argument("--duration", type=float, default=5.0)

    p = sub.add_parser("free-swing")
    p.add_argument("--duration", type=float, default=10.0)

    p = sub.add_parser("encoder")
    p.add_argument("--duration", type=float, default=5.0)
    p.add_argument(
        "--motor-command",
        type=float,
        help="optionally rotate the arm while capturing Encoder2 A/B/count",
    )

    p = sub.add_parser("motor")
    p.add_argument("--command", type=float, required=True)
    p.add_argument("--duration", type=float, default=1.0)

    p = sub.add_parser("motor-direction")
    p.add_argument("--command", dest="motor_command", type=float, default=0.10)
    p.add_argument("--hold", type=float, default=1.0)

    p = sub.add_parser("breakaway")
    p.add_argument("--step", type=float, default=0.01)
    p.add_argument("--max-command", type=float, default=0.30)
    p.add_argument("--hold", type=float, default=0.50)
    p.add_argument("--min-counts", type=int, default=4)

    p = sub.add_parser("speed-sweep")
    p.add_argument("--hold", type=float, default=1.5)
    p.add_argument("--commands", type=float, nargs="*")

    p = sub.add_parser("coast-down")
    p.add_argument("--command", type=float, default=0.20)
    p.add_argument("--runup", type=float, default=2.0)
    p.add_argument("--coast", type=float, default=5.0)

    p = sub.add_parser("position-step")
    p.add_argument("--delta", type=float, default=0.25)
    p.add_argument("--kp", type=float, default=0.8)
    p.add_argument("--kd", type=float, default=0.08)
    p.add_argument("--max-command", type=float, default=0.25)
    p.add_argument("--settle", type=float, default=2.0)

    p = sub.add_parser("step-response")
    p.add_argument("--amplitude", type=float, default=0.10)
    p.add_argument("--hold", type=float, default=1.5)

    p = sub.add_parser("chirp")
    p.add_argument("--amplitude", type=float, default=0.10)
    p.add_argument("--f0", type=float, default=0.2)
    p.add_argument("--f1", type=float, default=8.0)
    p.add_argument("--duration", type=float, default=20.0)

    p = sub.add_parser("prbs")
    p.add_argument("--amplitude", type=float, default=0.10)
    p.add_argument("--interval", type=float, default=0.20)
    p.add_argument("--duration", type=float, default=20.0)
    p.add_argument("--seed", type=int, default=1)

    p = sub.add_parser("all")
    p.add_argument("--sensor-duration", type=float, default=5.0)
    p.add_argument("--free-swing-duration", type=float, default=10.0)
    p.add_argument("--encoder-command", type=float, default=0.10)
    p.add_argument("--position-delta", type=float, default=0.25)
    p.add_argument("--chirp-duration", type=float, default=20.0)
    p.add_argument("--prbs-duration", type=float, default=20.0)

    return parser


def _print(value) -> None:
    print(json.dumps(value, indent=2, default=str))


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    with Rp2350Device(hid_path=args.hid_path, cdc_port=args.cdc_port) as device:
        if args.action == "status":
            _print({"version": device.version(), "status": device.status()})
        elif args.action == "safe-off":
            device.safe_off()
            _print({"safe_off": True})
        elif args.action == "led":
            on = args.state == "on"
            _print({"user_led": "on" if device.set_user_led(on) else "off"})
        elif args.action == "monitor":
            _print(
                sensors.monitor(
                    device,
                    args.duration,
                    motor_command=args.motor_command,
                )
            )
        elif args.action == "adc":
            _print(sensors.adc(device, args.duration))
        elif args.action == "free-swing":
            _print(sensors.free_swing(device, args.duration))
        elif args.action == "encoder":
            _print(
                sensors.encoder(
                    device,
                    args.duration,
                    motor_command=args.motor_command,
                )
            )
        elif args.action == "motor":
            _print(
                motor.motor_command(
                    device,
                    command=args.command,
                    duration_s=args.duration,
                )
            )
        elif args.action == "motor-direction":
            _print(
                motor.motor_direction(
                    device,
                    command=args.motor_command,
                    hold_s=args.hold,
                )
            )
        elif args.action == "breakaway":
            _print(
                motor.breakaway(
                    device,
                    step=args.step,
                    max_command=args.max_command,
                    hold_s=args.hold,
                    min_counts=args.min_counts,
                )
            )
        elif args.action == "speed-sweep":
            _print(motor.speed_sweep(device, commands=args.commands, hold_s=args.hold))
        elif args.action == "coast-down":
            _print(
                motor.coast_down(
                    device,
                    command=args.command,
                    runup_s=args.runup,
                    coast_s=args.coast,
                )
            )
        elif args.action == "position-step":
            _print(
                motor.position_step(
                    device,
                    delta_rad=args.delta,
                    kp=args.kp,
                    kd=args.kd,
                    max_command=args.max_command,
                    settle_s=args.settle,
                )
            )
        elif args.action == "step-response":
            _print(sysid.step_response(device, amplitude=args.amplitude, hold_s=args.hold))
        elif args.action == "chirp":
            _print(
                sysid.chirp(
                    device,
                    amplitude=args.amplitude,
                    f0_hz=args.f0,
                    f1_hz=args.f1,
                    duration_s=args.duration,
                )
            )
        elif args.action == "prbs":
            _print(
                sysid.prbs(
                    device,
                    amplitude=args.amplitude,
                    interval_s=args.interval,
                    duration_s=args.duration,
                    seed=args.seed,
                )
            )
        elif args.action == "all":
            results = {
                "status": {"version": device.version(), "status": device.status()},
                "adc": sensors.adc(device, args.sensor_duration),
                "free_swing": sensors.free_swing(device, args.free_swing_duration),
                "encoder": sensors.encoder(
                    device,
                    args.sensor_duration,
                    motor_command=args.encoder_command,
                ),
            }
            results["breakaway"] = motor.breakaway(device)
            results["motor_direction"] = motor.motor_direction(device)
            results["speed_sweep"] = motor.speed_sweep(device)
            results["coast_down"] = motor.coast_down(device)
            results["position_step"] = motor.position_step(
                device,
                delta_rad=args.position_delta,
            )
            results["step_response"] = sysid.step_response(device)
            results["chirp"] = sysid.chirp(device, duration_s=args.chirp_duration)
            results["prbs"] = sysid.prbs(device, duration_s=args.prbs_duration)
            _print(results)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
