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
    SET_MOTOR_COMMAND = 0x12
    SAFE_OFF = 0x13
    SET_USER_LED = 0x20


class HidStatus(IntEnum):
    OK = 0
    INVALID = 1
    DENIED = 2
    RANGE = 3
    BUSY = 4


_RUNTIME = struct.Struct("<BBHQIHiffffffIIIBBBBBB")
_COMMAND = struct.Struct("<BBBBIffII40s")
_ACK = struct.Struct("<BBBBIQffI36s")
assert _RUNTIME.size == HID_REPORT_SIZE
assert _COMMAND.size == HID_REPORT_SIZE
assert _ACK.size == HID_REPORT_SIZE


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


@dataclass(frozen=True)
class CommandAck:
    version: int
    message_type: int
    command: int
    status: int
    sequence: int
    timestamp_us: int
    value0: float
    value1: float
    detail: int

    @property
    def ok(self) -> bool:
        return self.status == int(HidStatus.OK)


def decode_runtime_report(data: bytes) -> TelemetrySample:
    if len(data) != HID_REPORT_SIZE:
        raise ValueError(f"expected {HID_REPORT_SIZE} HID bytes, got {len(data)}")
    values = _RUNTIME.unpack(data)
    sample = TelemetrySample(*values[:-1])
    if sample.schema != HID_SCHEMA:
        raise ValueError(f"unsupported HID telemetry schema {sample.schema}")
    return sample


def decode_ack(data: bytes) -> CommandAck:
    if len(data) != HID_REPORT_SIZE:
        raise ValueError(f"expected {HID_REPORT_SIZE} HID bytes, got {len(data)}")
    values = _ACK.unpack(data)
    ack = CommandAck(*values[:-1])
    if ack.version != HID_SCHEMA or ack.message_type != 0x81:
        raise ValueError("not a commissioning acknowledgement report")
    return ack


def decode_device_report(data: bytes) -> TelemetrySample | CommandAck:
    if len(data) == HID_REPORT_SIZE and data[1] == 0x81:
        return decode_ack(data)
    return decode_runtime_report(data)


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
