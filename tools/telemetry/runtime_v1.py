#!/usr/bin/env python3
"""Decode Rotary Inverted Pendulum runtime telemetry protocol v1.

The firmware protocol is a fixed 64-byte little-endian record with CRC-16/
CCITT-FALSE. This host tool preserves protocol identity and raw integer fields,
adds SI-scaled convenience fields, and reports sequence gaps / framing errors.

It does not infer missing samples or replay stale packets. A sequence gap stays a
sequence gap in the exported evidence.
"""

from __future__ import annotations

import argparse
import csv
import json
import struct
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import BinaryIO, Iterable, Iterator, TextIO

MAGIC = b"RI"
PROTOCOL_VERSION = 1
PACKET_KIND_RUNTIME = 1
PACKET_LEN = 64
CRC_OFFSET = 62
UINT32_MODULUS = 1 << 32

CYCLE_NAMES = {
    0: "idle",
    1: "primed",
    2: "rejected",
    3: "computed",
    4: "error",
}
REGIME_NAMES = {
    0: "swingup",
    1: "capture",
    2: "balance",
}
TIMING_NAMES = {
    0: "startup",
    1: "healthy",
    2: "late",
    3: "timeout",
}
WATCHDOG_NAMES = {
    0: "disarmed",
    1: "healthy",
    2: "expired",
}


class TelemetryDecodeError(ValueError):
    """A complete candidate packet violates the telemetry v1 contract."""


@dataclass(frozen=True)
class RuntimePacket:
    sequence: int
    timestamp_us_low: int
    sample_index: int
    cycle: int
    control_regime: int
    sensor_timing_health: int
    watchdog_health: int
    authorized: bool
    actuator_saturated: bool
    qualification_reasons: int
    authority_reasons: int
    theta_mrad: int
    theta_dot_mrad_s: int
    phi_mrad: int
    phi_dot_mrad_s: int
    demand_torque_unm: int
    bounded_command_ppm: int
    predicted_torque_unm: int
    inferred_missed_ticks: int
    crc16: int

    def to_record(self) -> dict[str, object]:
        record: dict[str, object] = asdict(self)
        record.update(
            {
                "protocol_version": PROTOCOL_VERSION,
                "packet_kind": PACKET_KIND_RUNTIME,
                "cycle_name": CYCLE_NAMES.get(self.cycle, f"unknown_{self.cycle}"),
                "control_regime_name": REGIME_NAMES.get(
                    self.control_regime, f"unknown_{self.control_regime}"
                ),
                "sensor_timing_health_name": TIMING_NAMES.get(
                    self.sensor_timing_health,
                    f"unknown_{self.sensor_timing_health}",
                ),
                "watchdog_health_name": WATCHDOG_NAMES.get(
                    self.watchdog_health, f"unknown_{self.watchdog_health}"
                ),
                "theta_rad": self.theta_mrad / 1_000.0,
                "theta_dot_rad_s": self.theta_dot_mrad_s / 1_000.0,
                "phi_rad": self.phi_mrad / 1_000.0,
                "phi_dot_rad_s": self.phi_dot_mrad_s / 1_000.0,
                "demand_torque_nm": self.demand_torque_unm / 1_000_000.0,
                "bounded_command": self.bounded_command_ppm / 1_000_000.0,
                "predicted_torque_nm": self.predicted_torque_unm / 1_000_000.0,
            }
        )
        return record


@dataclass
class StreamStats:
    input_bytes: int = 0
    decoded_packets: int = 0
    rejected_candidates: int = 0
    skipped_bytes: int = 0
    trailing_bytes: int = 0
    sequence_gap_events: int = 0
    missing_sequences: int = 0
    timestamp_wraps: int = 0
    first_sequence: int | None = None
    last_sequence: int | None = None
    first_sample_index: int | None = None
    last_sample_index: int | None = None

    _previous_sequence: int | None = None
    _previous_timestamp_low: int | None = None

    def observe(self, packet: RuntimePacket) -> None:
        if self.first_sequence is None:
            self.first_sequence = packet.sequence
            self.first_sample_index = packet.sample_index
        else:
            assert self._previous_sequence is not None
            delta = (packet.sequence - self._previous_sequence) % UINT32_MODULUS
            if delta != 1:
                self.sequence_gap_events += 1
                if 1 < delta < UINT32_MODULUS // 2:
                    self.missing_sequences += delta - 1

        if (
            self._previous_timestamp_low is not None
            and packet.timestamp_us_low < self._previous_timestamp_low
        ):
            self.timestamp_wraps += 1

        self.decoded_packets += 1
        self.last_sequence = packet.sequence
        self.last_sample_index = packet.sample_index
        self._previous_sequence = packet.sequence
        self._previous_timestamp_low = packet.timestamp_us_low

    def public_dict(self) -> dict[str, int | None]:
        return {
            key: value
            for key, value in asdict(self).items()
            if not key.startswith("_previous_")
        }


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def decode_packet(packet: bytes) -> RuntimePacket:
    if len(packet) != PACKET_LEN:
        raise TelemetryDecodeError(
            f"runtime packet length {len(packet)} != {PACKET_LEN}"
        )
    if packet[0:2] != MAGIC:
        raise TelemetryDecodeError("bad telemetry magic")
    if packet[2] != PROTOCOL_VERSION:
        raise TelemetryDecodeError(f"unsupported telemetry version {packet[2]}")
    if packet[3] != PACKET_KIND_RUNTIME:
        raise TelemetryDecodeError(f"unsupported telemetry packet kind {packet[3]}")

    expected_crc = struct.unpack_from("<H", packet, CRC_OFFSET)[0]
    actual_crc = crc16_ccitt_false(packet[:CRC_OFFSET])
    if actual_crc != expected_crc:
        raise TelemetryDecodeError(
            f"CRC mismatch: expected 0x{expected_crc:04x}, calculated 0x{actual_crc:04x}"
        )

    return RuntimePacket(
        sequence=struct.unpack_from("<I", packet, 4)[0],
        timestamp_us_low=struct.unpack_from("<I", packet, 8)[0],
        sample_index=struct.unpack_from("<I", packet, 12)[0],
        cycle=packet[16],
        control_regime=packet[17],
        sensor_timing_health=packet[18],
        watchdog_health=packet[19],
        authorized=packet[20] != 0,
        actuator_saturated=packet[21] != 0,
        qualification_reasons=struct.unpack_from("<I", packet, 24)[0],
        authority_reasons=struct.unpack_from("<I", packet, 28)[0],
        theta_mrad=struct.unpack_from("<i", packet, 32)[0],
        theta_dot_mrad_s=struct.unpack_from("<i", packet, 36)[0],
        phi_mrad=struct.unpack_from("<i", packet, 40)[0],
        phi_dot_mrad_s=struct.unpack_from("<i", packet, 44)[0],
        demand_torque_unm=struct.unpack_from("<i", packet, 48)[0],
        bounded_command_ppm=struct.unpack_from("<i", packet, 52)[0],
        predicted_torque_unm=struct.unpack_from("<i", packet, 56)[0],
        inferred_missed_ticks=struct.unpack_from("<H", packet, 60)[0],
        crc16=expected_crc,
    )


def iter_packets(data: bytes, stats: StreamStats | None = None) -> Iterator[RuntimePacket]:
    """Decode a byte stream while preserving corruption as explicit statistics.

    Resynchronization advances one byte after an invalid complete candidate and
    searches for the next `RI` magic. Bytes that cannot form a complete final
    packet are counted as trailing evidence rather than silently discarded.
    """

    if stats is None:
        stats = StreamStats()
    stats.input_bytes = len(data)

    cursor = 0
    while cursor + PACKET_LEN <= len(data):
        magic_at = data.find(MAGIC, cursor)
        if magic_at < 0:
            stats.skipped_bytes += len(data) - cursor
            cursor = len(data)
            break
        if magic_at > cursor:
            stats.skipped_bytes += magic_at - cursor
            cursor = magic_at
        if cursor + PACKET_LEN > len(data):
            break

        candidate = data[cursor : cursor + PACKET_LEN]
        try:
            packet = decode_packet(candidate)
        except TelemetryDecodeError:
            stats.rejected_candidates += 1
            stats.skipped_bytes += 1
            cursor += 1
            continue

        stats.observe(packet)
        yield packet
        cursor += PACKET_LEN

    stats.trailing_bytes = len(data) - cursor


def decode_stream(data: bytes) -> tuple[list[RuntimePacket], StreamStats]:
    stats = StreamStats()
    packets = list(iter_packets(data, stats))
    return packets, stats


def write_jsonl(records: Iterable[dict[str, object]], output: TextIO) -> None:
    for record in records:
        output.write(json.dumps(record, sort_keys=True, separators=(",", ":")))
        output.write("\n")


def write_csv(records: list[dict[str, object]], output: TextIO) -> None:
    if not records:
        return
    writer = csv.DictWriter(output, fieldnames=list(records[0].keys()))
    writer.writeheader()
    writer.writerows(records)


def read_all_binary(path: str) -> bytes:
    if path == "-":
        source: BinaryIO = sys.stdin.buffer
        return source.read()
    return Path(path).read_bytes()


def open_text_output(path: str) -> tuple[TextIO, bool]:
    if path == "-":
        return sys.stdout, False
    return Path(path).open("w", encoding="utf-8", newline=""), True


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="raw telemetry byte stream, or - for stdin")
    parser.add_argument(
        "--format", choices=("jsonl", "csv"), default="jsonl", help="decoded output format"
    )
    parser.add_argument("--output", default="-", help="decoded output path, or - for stdout")
    parser.add_argument(
        "--summary",
        help="optional JSON summary path; use - only when decoded output is not stdout",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="return non-zero if framing/CRC/protocol rejection or skipped/trailing bytes are seen",
    )
    args = parser.parse_args(argv)

    if args.output == "-" and args.summary == "-":
        parser.error("decoded output and summary cannot both use stdout")

    packets, stats = decode_stream(read_all_binary(args.input))
    records = [packet.to_record() for packet in packets]

    output, should_close = open_text_output(args.output)
    try:
        if args.format == "jsonl":
            write_jsonl(records, output)
        else:
            write_csv(records, output)
    finally:
        if should_close:
            output.close()

    if args.summary:
        summary_text = json.dumps(stats.public_dict(), sort_keys=True, indent=2) + "\n"
        if args.summary == "-":
            sys.stdout.write(summary_text)
        else:
            Path(args.summary).write_text(summary_text, encoding="utf-8")

    if args.strict and (
        stats.rejected_candidates or stats.skipped_bytes or stats.trailing_bytes
    ):
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
