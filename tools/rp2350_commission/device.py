from __future__ import annotations

from contextlib import suppress
from dataclasses import asdict
import time

from cdc_transport import CdcTransport
from hid_transport import HidTransport
from protocol import TelemetrySample


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
            if self.cdc_available:
                self.cdc.command("telemetry off", wait_s=0.05)
        self.cdc.close()
        self.hid.close()
        self.cdc_available = False

    def __enter__(self) -> "Rp2350Device":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def version(self) -> list[str]:
        if not self.cdc_available:
            return []
        return self.cdc.command("version")

    def status(self) -> list[str]:
        if not self.cdc_available:
            return []
        return self.cdc.command("status")

    def start_telemetry(self) -> None:
        if self.cdc_available:
            self.cdc.command("telemetry on", wait_s=0.05)
            return
        raise RuntimeError(
            "HID telemetry is disabled at boot in this firmware; CDC is required to enable it"
        )

    def stop_telemetry(self) -> None:
        if self.cdc_available:
            self.cdc.command("telemetry off", wait_s=0.05)

    def read_sample(self, timeout_ms: int = 250) -> TelemetrySample | None:
        return self.hid.read_telemetry(timeout_ms)

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

    # The unified host API already reserves the active commissioning calls.
    # Firmware must acknowledge HID OUT commands before these are enabled; this
    # avoids pretending that an ignored USB output report changed hardware state.
    def require_active_commissioning(self) -> None:
        raise RuntimeError(
            "active HID commissioning commands are not acknowledged by the current firmware image"
        )

    def safe_off(self) -> None:
        self.require_active_commissioning()

    def maintenance_enter(self) -> None:
        self.require_active_commissioning()

    def maintenance_exit(self) -> None:
        self.require_active_commissioning()

    def set_motor_command(self, value: float, *, lease_ms: int = 250) -> None:
        _ = value, lease_ms
        self.require_active_commissioning()
