from __future__ import annotations

import time
from typing import Iterable

import hid

from protocol import (
    CommandAck,
    HID_REPORT_SIZE,
    PID,
    VID,
    TelemetrySample,
    decode_device_report,
)


class HidTransport:
    def __init__(self, *, path: str | bytes | None = None, timeout_ms: int = 250) -> None:
        self.path = path
        self.timeout_ms = timeout_ms
        self._dev: hid.device | None = None

    @staticmethod
    def enumerate() -> list[dict[str, object]]:
        return list(hid.enumerate(VID, PID))

    def open(self) -> None:
        if self._dev is not None:
            return
        dev = hid.device()
        if self.path is not None:
            raw_path = self.path.encode() if isinstance(self.path, str) else self.path
            dev.open_path(raw_path)
        else:
            candidates = self.enumerate()
            if not candidates:
                raise RuntimeError(f"RP2350 HID {VID:04X}:{PID:04X} not found")
            selected = next((x for x in candidates if x.get("usage_page") == 0xFF00), candidates[0])
            dev.open_path(selected["path"])
        dev.set_nonblocking(False)
        self._dev = dev

    def close(self) -> None:
        if self._dev is not None:
            self._dev.close()
            self._dev = None

    def __enter__(self) -> "HidTransport":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def write_report(self, payload: bytes) -> None:
        if len(payload) != HID_REPORT_SIZE:
            raise ValueError(f"HID payload must be {HID_REPORT_SIZE} bytes")
        if self._dev is None:
            raise RuntimeError("HID device is not open")
        written = self._dev.write(b"\x00" + payload)
        if written <= 0:
            raise RuntimeError("HID write failed")

    def read_raw(self, timeout_ms: int | None = None) -> bytes | None:
        if self._dev is None:
            raise RuntimeError("HID device is not open")
        timeout = self.timeout_ms if timeout_ms is None else timeout_ms
        data = self._dev.read(HID_REPORT_SIZE, timeout)
        if not data:
            return None
        raw = bytes(data)
        if len(raw) != HID_REPORT_SIZE:
            raise RuntimeError(f"short HID report: {len(raw)} bytes")
        return raw

    def read_report(self, timeout_ms: int | None = None) -> TelemetrySample | CommandAck | None:
        raw = self.read_raw(timeout_ms)
        return None if raw is None else decode_device_report(raw)

    def samples(self, duration_s: float) -> Iterable[TelemetrySample]:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            report = self.read_report(min(self.timeout_ms, 100))
            if isinstance(report, TelemetrySample):
                yield report
