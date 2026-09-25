from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import IntEnum
import struct

VID = 0xCAFE
PID = 0x4010
HID_REPORT_SIZE = 64
HID_SCHEMA = 1


class ControlRegime(IntEnum):
    SWING_UP = 0
    CAPTURE = 1
    BALANCE = 2


class RuntimeState(IntEnum):
    DISABLED = 0
    READY = 1
    ACTIVE_SWING_UP = 2
    ACTIVE_CAPTURE = 3
    ACTIVE_BALANCE = 4
    FAULT = 5


class AuthorityMode(IntEnum):
    DISARMED = 0
    CLOSED_LOOP = 1
    MAINTENANCE = 2
    FAULT = 3


class HidCommand(IntEnum):
    GET_STATUS = 0x01
    TELEMETRY_ON = 0x02
    TELEMETRY_OFF = 0x03
    MAINTENANCE_ENTER = 0x10
    MAINTENANCE_EXIT = 0x11
    SET_MOTOR_COMMAND = 0x12
    SAFE_OFF = 0x13


# Current firmware input report. The final three bytes were reserved in the
# original feature-parity image; two now carry raw Encoder1 A/B states.
_RUNTIME = struct.Struct("<BBHQIHiffffffIIIBBBBBB")
assert _RUNTIME.size == HID_REPORT_SIZE

# Reserved host->device command layout. Firmware command acknowledgement is a
# commissioning capability and is intentionally versioned independently from
# the runtime telemetry layout.
_COMMAND = struct.Struct("<BBBBIffII40s")
assert _COMMAND.size == HID_REPORT_SIZE


@dataclass(frozen=True)
class TelemetrySample:
    schema: int
    flags: int
    sequence: int
    timestamp_us: int
    sample_index: int
    pendulum_adc_raw: int
    arm_encoder_count: int
    theta: float
    theta_dot: float
    phi: float
    phi_dot: float
    arm_torque_nm: float
    normalized_command: float
    missed_opportunities: int
    deadline_overruns: int
    execution_time_us: int
    regime: int
    runtime_state: int
    authority_mode: int
    encoder_a: int
    encoder_b: int

    @property
    def state_valid(self) -> bool:
        return bool(self.flags & 0x01)

    def as_dict(self) -> dict[str, object]:
        return asdict(self)


def decode_runtime_report(data: bytes) -> TelemetrySample:
    if len(data) != HID_REPORT_SIZE:
        raise ValueError(f"expected {HID_REPORT_SIZE} HID bytes, got {len(data)}")
    values = _RUNTIME.unpack(data)
    sample = TelemetrySample(*values[:-1])
    if sample.schema != HID_SCHEMA:
        raise ValueError(f"unsupported HID telemetry schema {sample.schema}")
    return sample


def encode_command(
    command: HidCommand,
    sequence: int,
    *,
    value0: float = 0.0,
    value1: float = 0.0,
    duration_ms: int = 0,
    flags: int = 0,
) -> bytes:
    return _COMMAND.pack(
        HID_SCHEMA,
        0x01,
        int(command),
        flags & 0xFF,
        sequence & 0xFFFFFFFF,
        float(value0),
        float(value1),
        int(duration_ms) & 0xFFFFFFFF,
        0,
        b"\x00" * 40,
    )
