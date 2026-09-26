from __future__ import annotations

import csv
import math
from pathlib import Path
import statistics


SENSOR_ELECTRICAL_ANGLE_DEG = 345.0
ADC_FULL_SCALE_COUNTS = 4095.0


def _rows(path: str | Path) -> list[dict[str, str]]:
    with Path(path).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _artifact(section: object) -> Path | None:
    if not isinstance(section, dict) or "artifact_dir" not in section:
        return None
    return Path(str(section["artifact_dir"]))


def analyze_pendulum_calibration(
    down: dict[str, object],
    upright: dict[str, object],
    sweep: dict[str, object],
) -> dict[str, object]:
    down_mean = float(down["adc_mean"])
    upright_mean = float(upright["adc_mean"])
    delta = upright_mean - down_mean
    abs_delta = abs(delta)
    expected_half_turn_counts = ADC_FULL_SCALE_COUNTS * 180.0 / SENSOR_ELECTRICAL_ANGLE_DEG

    counts_per_rad = abs_delta / math.pi if abs_delta > 0.0 else None
    rad_per_count = math.pi / abs_delta if abs_delta > 0.0 else None
    equivalent_counts_per_rev = 2.0 * abs_delta

    result: dict[str, object] = {
        "down_adc_mean": down_mean,
        "upright_adc_mean": upright_mean,
        "upright_minus_down_counts": delta,
        "half_turn_counts": abs_delta,
        "counts_per_rad_local": counts_per_rad,
        "radians_per_count_local": rad_per_count,
        "equivalent_counts_per_360deg": equivalent_counts_per_rev,
        "datasheet_electrical_angle_deg": SENSOR_ELECTRICAL_ANGLE_DEG,
        "datasheet_expected_half_turn_counts_if_full_adc_span": expected_half_turn_counts,
        "half_turn_ratio_to_datasheet_ideal": (
            abs_delta / expected_half_turn_counts if expected_half_turn_counts > 0.0 else None
        ),
        "down_adc_stdev_counts": float(down["adc_stdev"]),
        "upright_adc_stdev_counts": float(upright["adc_stdev"]),
        "sweep_adc_min": int(sweep["adc_min"]),
        "sweep_adc_max": int(sweep["adc_max"]),
        "sweep_adc_peak_to_peak": int(sweep["adc_peak_to_peak"]),
        "sweep_fraction_of_12bit_span": float(sweep["adc_fraction_of_12bit_span"]),
        "pose_separation_valid": abs_delta >= 1000.0,
        "sweep_span_valid": int(sweep["adc_peak_to_peak"]) >= 2500,
    }
    if rad_per_count is not None:
        result["down_noise_rad_rms"] = float(down["adc_stdev"]) * rad_per_count
        result["upright_noise_rad_rms"] = float(upright["adc_stdev"]) * rad_per_count
    return result


def analyze_free_swing(
    section: dict[str, object],
    *,
    down_adc_center: float,
    rad_per_count: float | None,
) -> dict[str, object]:
    result: dict[str, object] = {
        "estimate_valid": bool(section.get("estimate_valid", False)),
        "estimate_reason": section.get("estimate_reason"),
        "estimated_period_s": section.get("estimated_period_s"),
        "estimated_frequency_hz": section.get("estimated_frequency_hz"),
    }
    directory = _artifact(section)
    if directory is None or not (directory / "samples.csv").exists():
        return result

    rows = _rows(directory / "samples.csv")
    if len(rows) < 5:
        return result
    times = [float(row["timestamp_us"]) * 1.0e-6 for row in rows]
    signal = [float(row["pendulum_adc_raw"]) - down_adc_center for row in rows]
    result["peak_abs_excursion_counts"] = max(abs(value) for value in signal)
    if rad_per_count is not None:
        result["peak_abs_excursion_rad"] = result["peak_abs_excursion_counts"] * rad_per_count

    period = section.get("estimated_period_s")
    if not isinstance(period, (int, float)) or period <= 0.0:
        return result

    # Detect same-sign local extrema separated by at least half a period.  This is
    # deliberately a diagnostic log-decrement estimate, not a controller model.
    min_spacing = 0.55 * float(period)
    positive: list[tuple[float, float]] = []
    negative: list[tuple[float, float]] = []
    for i in range(1, len(signal) - 1):
        if signal[i] > signal[i - 1] and signal[i] >= signal[i + 1] and signal[i] > 0.0:
            if not positive or times[i] - positive[-1][0] >= min_spacing:
                positive.append((times[i], signal[i]))
            elif signal[i] > positive[-1][1]:
                positive[-1] = (times[i], signal[i])
        if signal[i] < signal[i - 1] and signal[i] <= signal[i + 1] and signal[i] < 0.0:
            magnitude = -signal[i]
            if not negative or times[i] - negative[-1][0] >= min_spacing:
                negative.append((times[i], magnitude))
            elif magnitude > negative[-1][1]:
                negative[-1] = (times[i], magnitude)

    decrements: list[float] = []
    for peaks in (positive, negative):
        for (_, a), (_, b) in zip(peaks, peaks[1:]):
            if a > 0.0 and b > 0.0 and a > b:
                decrements.append(math.log(a / b))
    if decrements:
        delta = statistics.median(decrements)
        zeta = delta / math.sqrt((2.0 * math.pi) ** 2 + delta * delta)
        result["log_decrement_median"] = delta
        result["damping_ratio_estimate"] = zeta
        result["damping_pairs"] = len(decrements)
    return result


def analyze_bounded_excitation(
    section: dict[str, object],
    *,
    down_adc_center: float,
    rad_per_count: float | None,
) -> dict[str, object]:
    directory = _artifact(section)
    if directory is None or not (directory / "samples.csv").exists():
        return {"valid": False, "reason": "samples_missing"}
    rows = _rows(directory / "samples.csv")
    if not rows:
        return {"valid": False, "reason": "samples_empty"}

    groups: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        groups.setdefault(row.get("test_phase", "unknown"), []).append(row)

    phase_results: dict[str, object] = {}
    origin = float(section["origin_phi_rad"])
    global_max_excursion = 0.0
    global_max_command = 0.0
    for phase, samples in groups.items():
        errors = [float(row["position_target_rad"]) - float(row["phi"]) for row in samples]
        excursions = [abs(float(row["phi"]) - origin) for row in samples]
        commands = [abs(float(row.get("requested_command", "0") or 0.0)) for row in samples]
        adc_deflection_counts = [float(row["pendulum_adc_raw"]) - down_adc_center for row in samples]
        global_max_excursion = max(global_max_excursion, max(excursions, default=0.0))
        global_max_command = max(global_max_command, max(commands, default=0.0))
        phase_result: dict[str, object] = {
            "samples": len(samples),
            "arm_tracking_error_rms_rad": math.sqrt(statistics.fmean(error * error for error in errors)),
            "arm_peak_excursion_rad": max(excursions, default=0.0),
            "command_peak_abs": max(commands, default=0.0),
            "pendulum_adc_peak_to_peak": (
                max(adc_deflection_counts) - min(adc_deflection_counts)
                if adc_deflection_counts
                else 0.0
            ),
        }
        if rad_per_count is not None and adc_deflection_counts:
            deflection_rad = [value * rad_per_count for value in adc_deflection_counts]
            phase_result["pendulum_deflection_rms_rad_about_down"] = math.sqrt(
                statistics.fmean(value * value for value in deflection_rad)
            )
            phase_result["pendulum_peak_abs_deflection_rad_about_down"] = max(
                abs(value) for value in deflection_rad
            )
        phase_results[phase] = phase_result

    return {
        "valid": True,
        "origin_phi_rad": origin,
        "global_arm_peak_excursion_rad": global_max_excursion,
        "global_command_peak_abs": global_max_command,
        "phases": phase_results,
    }


def analyze_mechanism_suite(sections: dict[str, object]) -> dict[str, object]:
    down = sections.get("pendulum_down")
    upright = sections.get("pendulum_upright")
    sweep = sections.get("pendulum_sweep")
    output: dict[str, object] = {}
    if not isinstance(down, dict) or not isinstance(upright, dict) or not isinstance(sweep, dict):
        return {"valid": False, "reason": "calibration_sections_missing"}

    calibration = analyze_pendulum_calibration(down, upright, sweep)
    output["valid"] = bool(calibration["pose_separation_valid"] and calibration["sweep_span_valid"])
    output["pendulum_calibration"] = calibration
    rad_per_count = calibration.get("radians_per_count_local")
    rad_per_count_value = float(rad_per_count) if isinstance(rad_per_count, (int, float)) else None
    down_center = float(calibration["down_adc_mean"])

    free_swing = sections.get("free_swing")
    if isinstance(free_swing, dict):
        output["free_swing"] = analyze_free_swing(
            free_swing,
            down_adc_center=down_center,
            rad_per_count=rad_per_count_value,
        )

    excitation = sections.get("bounded_excitation")
    if isinstance(excitation, dict):
        output["bounded_excitation"] = analyze_bounded_excitation(
            excitation,
            down_adc_center=down_center,
            rad_per_count=rad_per_count_value,
        )
    return output
