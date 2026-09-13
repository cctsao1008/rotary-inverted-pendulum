import io
import struct
import unittest

import runtime_v1 as telemetry


def encode_packet(
    sequence: int,
    *,
    timestamp_us_low: int = 11_000,
    sample_index: int = 11,
    cycle: int = 3,
    regime: int = 2,
    timing: int = 1,
    watchdog: int = 1,
    authorized: bool = False,
    saturated: bool = False,
    qualification_reasons: int = 0x12,
    authority_reasons: int = 0x34,
    theta_mrad: int = -45,
    theta_dot_mrad_s: int = 12,
    phi_mrad: int = 123,
    phi_dot_mrad_s: int = -8,
    demand_torque_unm: int = 7_000,
    bounded_command_ppm: int = 91_000,
    predicted_torque_unm: int = 6_500,
    missed_ticks: int = 2,
) -> bytes:
    packet = bytearray(telemetry.PACKET_LEN)
    packet[0:2] = telemetry.MAGIC
    packet[2] = telemetry.PROTOCOL_VERSION
    packet[3] = telemetry.PACKET_KIND_RUNTIME
    struct.pack_into("<I", packet, 4, sequence)
    struct.pack_into("<I", packet, 8, timestamp_us_low)
    struct.pack_into("<I", packet, 12, sample_index)
    packet[16] = cycle
    packet[17] = regime
    packet[18] = timing
    packet[19] = watchdog
    packet[20] = int(authorized)
    packet[21] = int(saturated)
    struct.pack_into("<I", packet, 24, qualification_reasons)
    struct.pack_into("<I", packet, 28, authority_reasons)
    struct.pack_into("<i", packet, 32, theta_mrad)
    struct.pack_into("<i", packet, 36, theta_dot_mrad_s)
    struct.pack_into("<i", packet, 40, phi_mrad)
    struct.pack_into("<i", packet, 44, phi_dot_mrad_s)
    struct.pack_into("<i", packet, 48, demand_torque_unm)
    struct.pack_into("<i", packet, 52, bounded_command_ppm)
    struct.pack_into("<i", packet, 56, predicted_torque_unm)
    struct.pack_into("<H", packet, 60, missed_ticks)
    crc = telemetry.crc16_ccitt_false(packet[: telemetry.CRC_OFFSET])
    struct.pack_into("<H", packet, telemetry.CRC_OFFSET, crc)
    return bytes(packet)


class RuntimeTelemetryV1Tests(unittest.TestCase):
    def test_decodes_firmware_layout_and_si_scaling(self) -> None:
        packet = telemetry.decode_packet(encode_packet(7, authorized=True, saturated=True))
        self.assertEqual(packet.sequence, 7)
        self.assertEqual(packet.sample_index, 11)
        self.assertEqual(packet.theta_mrad, -45)
        self.assertEqual(packet.authority_reasons, 0x34)
        self.assertTrue(packet.authorized)
        self.assertTrue(packet.actuator_saturated)

        record = packet.to_record()
        self.assertEqual(record["cycle_name"], "computed")
        self.assertEqual(record["control_regime_name"], "balance")
        self.assertAlmostEqual(record["theta_rad"], -0.045)
        self.assertAlmostEqual(record["demand_torque_nm"], 0.007)
        self.assertAlmostEqual(record["bounded_command"], 0.091)

    def test_crc_corruption_is_rejected(self) -> None:
        packet = bytearray(encode_packet(1))
        packet[40] ^= 0x80
        with self.assertRaisesRegex(telemetry.TelemetryDecodeError, "CRC mismatch"):
            telemetry.decode_packet(bytes(packet))

    def test_stream_resynchronizes_and_preserves_sequence_gap(self) -> None:
        corrupt = bytearray(encode_packet(2))
        corrupt[32] ^= 0x01
        stream = b"noise" + encode_packet(1) + bytes(corrupt) + b"x" + encode_packet(4)

        packets, stats = telemetry.decode_stream(stream)
        self.assertEqual([packet.sequence for packet in packets], [1, 4])
        self.assertEqual(stats.decoded_packets, 2)
        self.assertGreaterEqual(stats.rejected_candidates, 1)
        self.assertGreater(stats.skipped_bytes, 0)
        self.assertEqual(stats.sequence_gap_events, 1)
        self.assertEqual(stats.missing_sequences, 2)

    def test_sequence_wrap_is_not_reported_as_gap(self) -> None:
        stream = encode_packet(0xFFFFFFFF) + encode_packet(0)
        _, stats = telemetry.decode_stream(stream)
        self.assertEqual(stats.sequence_gap_events, 0)
        self.assertEqual(stats.missing_sequences, 0)

    def test_timestamp_low_wrap_is_explicit(self) -> None:
        stream = encode_packet(1, timestamp_us_low=0xFFFFFF00) + encode_packet(
            2, timestamp_us_low=200
        )
        _, stats = telemetry.decode_stream(stream)
        self.assertEqual(stats.timestamp_wraps, 1)

    def test_trailing_partial_packet_is_reported(self) -> None:
        stream = encode_packet(1) + encode_packet(2)[:17]
        packets, stats = telemetry.decode_stream(stream)
        self.assertEqual(len(packets), 1)
        self.assertEqual(stats.trailing_bytes, 17)

    def test_jsonl_writer_emits_one_record_per_packet(self) -> None:
        packets, _ = telemetry.decode_stream(encode_packet(5) + encode_packet(6))
        output = io.StringIO()
        telemetry.write_jsonl((packet.to_record() for packet in packets), output)
        lines = output.getvalue().splitlines()
        self.assertEqual(len(lines), 2)
        self.assertIn('"sequence":5', lines[0])
        self.assertIn('"sequence":6', lines[1])

    def test_unknown_enum_value_is_preserved_not_reinterpreted(self) -> None:
        packet = telemetry.decode_packet(encode_packet(1, cycle=99))
        record = packet.to_record()
        self.assertEqual(record["cycle"], 99)
        self.assertEqual(record["cycle_name"], "unknown_99")


if __name__ == "__main__":
    unittest.main()
