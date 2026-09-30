from __future__ import annotations

import math
import unittest

from mechanism_analysis import analyze_pendulum_calibration


class MechanismAnalysisTest(unittest.TestCase):
    def test_half_turn_calibration(self) -> None:
        down = {
            "adc_mean": 800.0,
            "adc_stdev": 1.5,
        }
        upright = {
            "adc_mean": 2936.0,
            "adc_stdev": 1.0,
        }
        sweep = {
            "adc_min": 20,
            "adc_max": 4080,
            "adc_peak_to_peak": 4060,
            "adc_fraction_of_12bit_span": 4060.0 / 4095.0,
        }
        result = analyze_pendulum_calibration(down, upright, sweep)
        self.assertTrue(result["pose_separation_valid"])
        self.assertTrue(result["sweep_span_valid"])
        self.assertAlmostEqual(result["half_turn_counts"], 2136.0)
        self.assertAlmostEqual(result["radians_per_count_local"], math.pi / 2136.0)
        self.assertAlmostEqual(result["equivalent_counts_per_360deg"], 4272.0)
        self.assertGreater(result["half_turn_ratio_to_datasheet_ideal"], 0.99)
        self.assertLess(result["half_turn_ratio_to_datasheet_ideal"], 1.01)

    def test_bad_span_is_rejected(self) -> None:
        down = {"adc_mean": 1000.0, "adc_stdev": 1.0}
        upright = {"adc_mean": 1200.0, "adc_stdev": 1.0}
        sweep = {
            "adc_min": 900,
            "adc_max": 1300,
            "adc_peak_to_peak": 400,
            "adc_fraction_of_12bit_span": 400.0 / 4095.0,
        }
        result = analyze_pendulum_calibration(down, upright, sweep)
        self.assertFalse(result["pose_separation_valid"])
        self.assertFalse(result["sweep_span_valid"])


if __name__ == "__main__":
    unittest.main()
