from __future__ import annotations

import math
import unittest

from sensors import _estimate_free_swing_period


class FreeSwingEstimatorTests(unittest.TestCase):
    def test_accepts_clean_resolved_passive_period(self) -> None:
        sample_hz = 100.0
        period_s = 0.8
        samples = int(6.0 * sample_hz)
        timestamps_us = [int(index * 1.0e6 / sample_hz) for index in range(samples)]
        adc_values = [
            int(round(2048 + 700 * math.sin(2.0 * math.pi * (index / sample_hz) / period_s)))
            for index in range(samples)
        ]

        estimate = _estimate_free_swing_period(timestamps_us, adc_values)

        self.assertTrue(estimate["valid"])
        self.assertEqual(estimate["reason"], "ok")
        self.assertAlmostEqual(float(estimate["estimated_period_s"]), period_s, delta=0.03)
        self.assertAlmostEqual(float(estimate["estimated_frequency_hz"]), 1.0 / period_s, delta=0.05)
        self.assertGreaterEqual(int(estimate["resolved_periods"]), 2)

    def test_rejects_small_floating_adc_excursion(self) -> None:
        timestamps_us = [index * 10_000 for index in range(500)]
        pattern = (-24, 7, -11, 24, -3, 15, -18, 5)
        adc_values = [850 + pattern[index % len(pattern)] for index in range(len(timestamps_us))]

        estimate = _estimate_free_swing_period(timestamps_us, adc_values)

        self.assertFalse(estimate["valid"])
        self.assertEqual(estimate["reason"], "insufficient_adc_excursion")
        self.assertEqual(estimate["adc_peak_to_peak"], 48)
        self.assertIsNone(estimate["estimated_period_s"])
        self.assertIsNone(estimate["estimated_frequency_hz"])

    def test_rejects_frequency_not_resolved_by_telemetry(self) -> None:
        sample_hz = 100.0
        period_s = 0.025
        samples = int(3.0 * sample_hz)
        timestamps_us = [int(index * 1.0e6 / sample_hz) for index in range(samples)]
        adc_values = [
            int(round(2048 + 500 * math.sin(2.0 * math.pi * (index / sample_hz) / period_s)))
            for index in range(samples)
        ]

        estimate = _estimate_free_swing_period(timestamps_us, adc_values)

        self.assertFalse(estimate["valid"])
        self.assertIn(estimate["reason"], {"noise_dominated", "insufficient_resolved_cycles"})
        self.assertIsNone(estimate["estimated_period_s"])
        self.assertIsNone(estimate["estimated_frequency_hz"])

    def test_rejects_non_advancing_timestamps(self) -> None:
        with self.assertRaises(ValueError):
            _estimate_free_swing_period([100, 100, 100], [0, 100, 0])


if __name__ == "__main__":
    unittest.main()
