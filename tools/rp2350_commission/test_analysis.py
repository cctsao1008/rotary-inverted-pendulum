from __future__ import annotations

import csv
from pathlib import Path
import tempfile
import unittest

from analysis import _linear_slope, analyze_coast_down, analyze_runtime_timing


class AnalysisTest(unittest.TestCase):
    def test_linear_slope_recovers_exact_line(self) -> None:
        slope, intercept, r2 = _linear_slope([0.0, 1.0, 2.0], [1.0, 3.0, 5.0])
        self.assertAlmostEqual(slope, 2.0)
        self.assertAlmostEqual(intercept, 1.0)
        self.assertAlmostEqual(r2, 1.0)

    def test_coast_analysis_uses_last_encoder_motion(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "samples.csv"
            rows = [
                {"timestamp_us": "1000000", "arm_encoder_count": "10", "test_phase": "coast"},
                {"timestamp_us": "1010000", "arm_encoder_count": "11", "test_phase": "coast"},
                {"timestamp_us": "1020000", "arm_encoder_count": "12", "test_phase": "coast"},
                {"timestamp_us": "1030000", "arm_encoder_count": "12", "test_phase": "coast"},
            ]
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)

            result = analyze_coast_down(path, arm_cpr=1000.0)
            self.assertTrue(result["valid"])
            self.assertAlmostEqual(result["stop_time_s"], 0.02)
            self.assertEqual(result["coast_travel_counts"], 2)

    def test_runtime_timing_reports_tail_and_fault_counters(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "samples.csv"
            rows = []
            for index in range(100):
                rows.append(
                    {
                        "execution_time_us": str(index + 1),
                        "missed_opportunities": "2" if index == 50 else "0",
                        "deadline_overruns": "1" if index == 70 else "0",
                    }
                )
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)

            result = analyze_runtime_timing([path])
            self.assertEqual(result["samples"], 100)
            self.assertEqual(result["execution_time_p99_us"], 99)
            self.assertEqual(result["execution_time_max_us"], 100)
            self.assertEqual(result["max_missed_opportunities"], 2)
            self.assertEqual(result["max_deadline_overruns"], 1)


if __name__ == "__main__":
    unittest.main()
