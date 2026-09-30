from __future__ import annotations

import csv
import math
from pathlib import Path
import statistics
from typing import Iterable


DEFAULT_ARM_CPR = 1040.0
DEFAULT_ADC_COUNTS_PER_REV = 4096.0


def _rows(path: str | Path) -> list[dict[str, str]]:
    with Path(path).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _float(row: dict[str, str], key: str) -> float:
    return float(row[key])


def _int(row: dict[str, str], key: str) -> int:
    return int(row[key])


def _linear_slope(xs: Iterable[float], ys: Iterable[float]) -> tuple[float, float, float]:
    x = list(xs)
    y = list(ys)
    if len(x) != len(y) or len(x) < 2:
        raise ValueError("linear fit requires at least two paired samples")
    x_mean = statistics.fmean(x)
    y_mean = statistics.fmean(y)
    sxx = sum((value - x_mean) ** 2 for value in x)
    if sxx <= 0.0:
        raise ValueError("linear fit requires changing x")
    slope = sum((a - x_mean) * (b - y_mean) for a, b in zip(x, y)) / sxx
    intercept = y_mean - slope * x_mean
    residual = sum((b - (slope * a + intercept)) ** 2 for a, b in zip(x, y))
    total = sum((b - y_mean) ** 2 for b in y)
    r2 = 1.0 if total <= 0.0 else 1.0 - residual / total
    return slope, intercept, r2


def _phase_rows(rows: list[dict[str, str]], phase: str) -> list[dict[str, str]]:
    return [row for row in rows if row.get("test_phase") == phase]


def _count_rate_rad_s(rows: list[dict[str, str]], arm_cpr: float) -> float | None:
    if len(rows) < 2:
        return None
    t0 = _float(rows[0], "timestamp_us") * 1.0e-6
    times = [_float(row, "timestamp_us") * 1.0e-6 - t0 for row in rows]
    counts = [_float(row, "arm_encoder_count") for row in rows]
    slope_counts_s, _, _ = _linear_slope(times, counts)
    return slope_counts_s * (2.0 * math.pi / arm_cpr)


def analyze_speed_sweep(samples_csv: str | Path, *, arm_cpr: float = DEFAULT_ARM_CPR) -> dict[str, object]:
    rows = _rows(samples_csv)
    groups: dict[float, list[dict[str, str]]] = {}
    for row in rows:
        phase = row.get("test_phase", "")
        if not phase.startswith("command-"):
            continue
        command = float(row["requested_command"])
        groups.setdefault(command, []).append(row)

    points: list[dict[str, float]] = []
    for command in sorted(groups, key=lambda value: (value < 0.0, abs(value))):
        samples = groups[command]
        steady = samples[len(samples) // 2 :]
        rate = _count_rate_rad_s(steady, arm_cpr)
        if rate is not None:
            points.append({"command": command, "count_slope_phi_dot_rad_s": rate})

    def fit(sign: int) -> dict[str, float] | None:
        selected = [
            point
            for point in points
            if abs(point["command"]) >= 0.18
            and ((point["command"] > 0.0) if sign > 0 else (point["command"] < 0.0))
        ]
        if len(selected) < 3:
            return None
        x = [abs(point["command"]) for point in selected]
        y = [abs(point["count_slope_phi_dot_rad_s"]) for point in selected]
        gain, intercept, r2 = _linear_slope(x, y)
        axis_intercept = -intercept / gain if gain > 0.0 else math.nan
        return {
            "gain_rad_s_per_command": gain,
            "intercept_rad_s": intercept,
            "command_axis_intercept": axis_intercept,
            "r2": r2,
        }

    positive = fit(+1)
    negative = fit(-1)
    kinetic_deadzone = None
    if positive and negative:
        kinetic_deadzone = statistics.fmean(
            [positive["command_axis_intercept"], negative["command_axis_intercept"]]
        )

    return {
        "points": points,
        "positive_running_fit": positive,
        "negative_running_fit": negative,
        "symmetric_kinetic_deadzone": kinetic_deadzone,
        "arm_cpr_assumed": arm_cpr,
    }


def analyze_coast_down(samples_csv: str | Path, *, arm_cpr: float = DEFAULT_ARM_CPR) -> dict[str, object]:
    rows = _phase_rows(_rows(samples_csv), "coast")
    if len(rows) < 2:
        return {"valid": False, "reason": "insufficient_coast_samples"}

    start_us = _int(rows[0], "timestamp_us")
    start_count = _int(rows[0], "arm_encoder_count")
    previous_count = start_count
    last_motion_us = start_us
    last_motion_count = start_count
    for row in rows[1:]:
        count = _int(row, "arm_encoder_count")
        if count != previous_count:
            last_motion_us = _int(row, "timestamp_us")
            last_motion_count = count
        previous_count = count

    travel_counts = last_motion_count - start_count
    return {
        "valid": True,
        "stop_time_s": (last_motion_us - start_us) * 1.0e-6,
        "coast_travel_counts": travel_counts,
        "coast_travel_rad": travel_counts * (2.0 * math.pi / arm_cpr),
        "arm_cpr_assumed": arm_cpr,
    }


def _windowed_count_rates(
    rows: list[dict[str, str]],
    *,
    arm_cpr: float,
    half_window: int = 2,
) -> list[tuple[float, float]]:
    if len(rows) < 2 * half_window + 1:
        return []
    t0 = _float(rows[0], "timestamp_us") * 1.0e-6
    scale = 2.0 * math.pi / arm_cpr
    result: list[tuple[float, float]] = []
    for index in range(half_window, len(rows) - half_window):
        window = rows[index - half_window : index + half_window + 1]
        times = [_float(row, "timestamp_us") * 1.0e-6 - t0 for row in window]
        counts = [_float(row, "arm_encoder_count") for row in window]
        slope_counts_s, _, _ = _linear_slope(times, counts)
        center_t = _float(rows[index], "timestamp_us") * 1.0e-6 - t0
        result.append((center_t, slope_counts_s * scale))
    return result


def analyze_step_response(samples_csv: str | Path, *, arm_cpr: float = DEFAULT_ARM_CPR) -> dict[str, object]:
    rows = _rows(samples_csv)
    result: dict[str, object] = {"arm_cpr_assumed": arm_cpr}
    for phase in ("positive-step", "negative-step"):
        samples = _phase_rows(rows, phase)
        rates = _windowed_count_rates(samples, arm_cpr=arm_cpr)
        if len(rates) < 10:
            result[phase] = {"valid": False}
            continue
        tail = [rate for _, rate in rates[int(len(rates) * 0.7) :]]
        steady = statistics.median(tail)
        sign = 1.0 if steady >= 0.0 else -1.0
        magnitude = abs(steady)

        def crossing(fraction: float) -> float | None:
            threshold = fraction * magnitude
            for t, rate in rates:
                if sign * rate >= threshold:
                    return t
            return None

        result[phase] = {
            "valid": True,
            "steady_phi_dot_rad_s": steady,
            "t63_s": crossing(0.63),
            "t90_s": crossing(0.90),
        }
    return result


def analyze_runtime_timing(sample_csvs: Iterable[str | Path]) -> dict[str, object]:
    execution: list[int] = []
    max_missed = 0
    max_overruns = 0
    samples = 0
    for path in sample_csvs:
        for row in _rows(path):
            samples += 1
            execution.append(_int(row, "execution_time_us"))
            max_missed = max(max_missed, _int(row, "missed_opportunities"))
            max_overruns = max(max_overruns, _int(row, "deadline_overruns"))
    if not execution:
        return {"samples": 0}
    ordered = sorted(execution)
    p99_index = min(len(ordered) - 1, math.ceil(0.99 * len(ordered)) - 1)
    return {
        "samples": samples,
        "execution_time_median_us": statistics.median(execution),
        "execution_time_p99_us": ordered[p99_index],
        "execution_time_max_us": max(execution),
        "max_missed_opportunities": max_missed,
        "max_deadline_overruns": max_overruns,
    }


def analyze_motor_suite(
    sections: dict[str, object],
    *,
    arm_cpr: float = DEFAULT_ARM_CPR,
) -> dict[str, object]:
    def artifact(name: str) -> Path | None:
        value = sections.get(name)
        if not isinstance(value, dict) or "artifact_dir" not in value:
            return None
        return Path(str(value["artifact_dir"]))

    output: dict[str, object] = {
        "arm_cpr_assumed": arm_cpr,
        "absolute_rate_scale_provisional": True,
        "encoder_one_count_per_1ms_rad_s": 2.0 * math.pi / arm_cpr / 0.001,
        "pendulum_one_count_per_1ms_rad_s": 2.0 * math.pi / DEFAULT_ADC_COUNTS_PER_REV / 0.001,
    }

    speed_dir = artifact("speed_sweep")
    if speed_dir and (speed_dir / "samples.csv").exists():
        output["speed_sweep"] = analyze_speed_sweep(speed_dir / "samples.csv", arm_cpr=arm_cpr)

    for name in ("coast_down_positive", "coast_down_negative"):
        directory = artifact(name)
        if directory and (directory / "samples.csv").exists():
            output[name] = analyze_coast_down(directory / "samples.csv", arm_cpr=arm_cpr)

    step_dir = artifact("step_response")
    if step_dir and (step_dir / "samples.csv").exists():
        output["step_response"] = analyze_step_response(step_dir / "samples.csv", arm_cpr=arm_cpr)

    sample_paths: list[Path] = []
    for value in sections.values():
        if isinstance(value, dict) and "artifact_dir" in value:
            candidate = Path(str(value["artifact_dir"])) / "samples.csv"
            if candidate.exists():
                sample_paths.append(candidate)
    output["runtime_timing"] = analyze_runtime_timing(sample_paths)
    return output
