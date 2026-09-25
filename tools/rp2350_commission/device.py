from __future__ import annotations

from collections import deque
from contextlib import suppress
from dataclasses import asdict
import time

from cdc_transport import CdcTransport
from hid_transport import HidTransport
from protocol import CommandAck, HidCommand, HidStatus, TelemetrySample, encode_command


class Rp2350Device:
    """One host session spanning the RP2350 HID and CDC interfaces."""

    def __init__(
        self,
        *,
        hid_path: str | bytes | None = None,
        cdc_port: str | None = None,
        require_cdc: bool = False,
    ) -> None:
        self.hid = HidTransport(path=hid_path)
        self.cdc = CdcTransport(port=cdc_port)
        self.require_cdc = require_cdc
        self.cdc_available = False
        self._sequence = 1
        self._telemetry_queue: deque[TelemetrySample] = deque()

    def open(self) -> None:
        self.hid.open()
        try:
            self.cdc.open()
            self.cdc_available = True
        except Exception:
            self.cdc_available = False
            if self.require_cdc:
                self.hid.close()
                raise

    def close(self) -> None:
        with suppress(Exception):
            self.safe_off()
        with suppress(Exception):
            self.stop_telemetry()
        self.cdc.close()
        self.hid.close()
        self.cdc_available = False

    def __enter__(self) -> "Rp2350Device":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _next_sequence(self) -> int:
        sequence = self._sequence
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF
        if self._sequence == 0:
            self._sequence = 1
        return sequence

    def command(
        self,
        command: HidCommand,
        *,
        value0: float = 0.0,
        value1: float = 0.0,
        duration_ms: int = 0,
        timeout_s: float = 1.0,
    ) -> CommandAck:
        sequence = self._next_sequence()
        self.hid.write_report(
            encode_command(
                command,
                sequence,
                value0=value0,
                value1=value1,
                duration_ms=duration_ms,
            )
        )
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            report = self.hid.read_report(100)
            if report is None:
                continue
            if isinstance(report, TelemetrySample):
                self._telemetry_queue.append(report)
                continue
            if isinstance(report, CommandAck) and report.sequence == sequence:
                if report.command != int(command):
                    raise RuntimeError(
                        f"HID ack command mismatch: expected {int(command):#x}, got {report.command:#x}"
                    )
                if not report.ok:
                    try:
                        status = HidStatus(report.status).name
                    except ValueError:
                        status = str(report.status)
                    raise RuntimeError(f"HID command {command.name} rejected: {status}")
                return report
        raise TimeoutError(f"no HID acknowledgement for {command.name}")

    def version(self) -> list[str]:
        return self.cdc.command("version") if self.cdc_available else []

    def status(self) -> dict[str, object]:
        ack = self.command(HidCommand.GET_STATUS)
        return {
            "hid": asdict(ack),
            "cdc": self.cdc.command("status") if self.cdc_available else [],
        }

    def start_telemetry(self) -> None:
        self.command(HidCommand.TELEMETRY_ON)

    def stop_telemetry(self) -> None:
        self.command(HidCommand.TELEMETRY_OFF)

    def read_sample(self, timeout_ms: int = 250) -> TelemetrySample | None:
        if self._telemetry_queue:
            return self._telemetry_queue.popleft()
        deadline = time.monotonic() + timeout_ms / 1000.0
        while time.monotonic() < deadline:
            report = self.hid.read_report(min(100, timeout_ms))
            if isinstance(report, TelemetrySample):
                return report
        return None

    def samples(self, duration_s: float):
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            sample = self.read_sample(100)
            if sample is not None:
                yield sample

    def drain_cdc(self) -> list[str]:
        return self.cdc.drain_lines() if self.cdc_available else []

    @staticmethod
    def sample_dict(sample: TelemetrySample) -> dict[str, object]:
        return asdict(sample)

    def safe_off(self) -> None:
        self.command(HidCommand.SAFE_OFF)

    def set_motor_command(self, value: float, *, lease_ms: int = 250) -> float:
        ack = self.command(HidCommand.SET_MOTOR_COMMAND, value0=value, duration_ms=lease_ms)
        return ack.value0
