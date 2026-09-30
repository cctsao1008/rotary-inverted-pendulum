from __future__ import annotations

import csv
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
from typing import Iterable

from protocol import TelemetrySample


def repository_root() -> Path:
    return Path(__file__).resolve().parents[2]


class RunRecorder:
    def __init__(self, test_name: str) -> None:
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.directory = repository_root() / "artifacts" / "commissioning" / f"{stamp}-{test_name}"
        self.directory.mkdir(parents=True, exist_ok=True)
        self._samples_path = self.directory / "samples.csv"
        self._cdc_path = self.directory / "cdc.log"
        self._writer = None
        self._csv_file = None

    def write_metadata(self, metadata: dict[str, object]) -> None:
        (self.directory / "metadata.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

    def append_sample(self, sample: TelemetrySample, **extra: object) -> None:
        row = sample.as_dict()
        row.update(extra)
        if self._writer is None:
            self._csv_file = self._samples_path.open("w", newline="", encoding="utf-8")
            self._writer = csv.DictWriter(self._csv_file, fieldnames=list(row))
            self._writer.writeheader()
        self._writer.writerow(row)
        self._csv_file.flush()

    def append_cdc(self, lines: Iterable[str]) -> None:
        lines = list(lines)
        if not lines:
            return
        with self._cdc_path.open("a", encoding="utf-8") as stream:
            for line in lines:
                stream.write(line.rstrip("\r\n") + "\n")

    def write_summary(self, summary: dict[str, object]) -> None:
        (self.directory / "summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        lines = [f"# {summary.get('test', 'Commissioning result')}", ""]
        for key, value in summary.items():
            if key == "test":
                continue
            lines.append(f"- **{key}**: {value}")
        (self.directory / "result.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    def close(self) -> None:
        if self._csv_file is not None:
            self._csv_file.close()
            self._csv_file = None
            self._writer = None

    def __enter__(self) -> "RunRecorder":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
