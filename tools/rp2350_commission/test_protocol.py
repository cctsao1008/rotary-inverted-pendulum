from __future__ import annotations

import struct
import unittest

from protocol import (
    HID_REPORT_SIZE,
    HID_SCHEMA,
    CommandAck,
    HidCommand,
    HidStatus,
    TelemetrySample,
    decode_ack,
    decode_device_report,
    decode_runtime_report,
    encode_command,
)


class ProtocolTests(unittest.TestCase):
    def test_encode_motor_command_layout(self) -> None:
        report = encode_command(
            HidCommand.SET_MOTOR_COMMAND,
            0x12345678,
            value0=-0.25,
            value1=1.5,
            duration_ms=250,
            flags=0xA5,
        )
        self.assertEqual(len(report), HID_REPORT_SIZE)
        version, message_type, command, flags, sequence, value0, value1, duration_ms, reserved0, reserved = struct.unpack(
            "<BBBBIffII40s", report
        )
        self.assertEqual(version, HID_SCHEMA)
        self.assertEqual(message_type, 0x01)
        self.assertEqual(command, int(HidCommand.SET_MOTOR_COMMAND))
        self.assertEqual(flags, 0xA5)
        self.assertEqual(sequence, 0x12345678)
        self.assertAlmostEqual(value0, -0.25)
        self.assertAlmostEqual(value1, 1.5)
        self.assertEqual(duration_ms, 250)
        self.assertEqual(reserved0, 0)
        self.assertEqual(reserved, b"\x00" * 40)

    def test_decode_ack(self) -> None:
        raw = struct.pack(
            "<BBBBIQffI36s",
            HID_SCHEMA,
            0x81,
            int(HidCommand.GET_STATUS),
            int(HidStatus.OK),
            77,
            123456789,
            0.25,
            -1.0,
            0x00010001,
            b"\x00" * 36,
        )
        ack = decode_ack(raw)
        self.assertIsInstance(ack, CommandAck)
        self.assertTrue(ack.ok)
        self.assertEqual(ack.sequence, 77)
        self.assertEqual(ack.timestamp_us, 123456789)
        self.assertAlmostEqual(ack.value0, 0.25)
        self.assertAlmostEqual(ack.value1, -1.0)
        self.assertEqual(ack.detail, 0x00010001)
        self.assertIsInstance(decode_device_report(raw), CommandAck)

    def test_decode_runtime_report(self) -> None:
        raw = struct.pack(
            "<BBHQIHiffffffIIIBBBBBB",
            HID_SCHEMA,
            0x01,
            123,
            987654321,
            456,
            2048,
            -123,
            0.1,
            -0.2,
            0.3,
            -0.4,
            0.005,
            -0.25,
            7,
            8,
            19,
            2,
            4,
            1,
            1,
            0,
            0,
        )
        sample = decode_runtime_report(raw)
        self.assertIsInstance(sample, TelemetrySample)
        self.assertTrue(sample.state_valid)
        self.assertEqual(sample.sequence, 123)
        self.assertEqual(sample.timestamp_us, 987654321)
        self.assertEqual(sample.sample_index, 456)
        self.assertEqual(sample.pendulum_adc_raw, 2048)
        self.assertEqual(sample.arm_encoder_count, -123)
        self.assertAlmostEqual(sample.normalized_command, -0.25)
        self.assertEqual(sample.missed_opportunities, 7)
        self.assertEqual(sample.deadline_overruns, 8)
        self.assertEqual(sample.execution_time_us, 19)
        self.assertEqual(sample.encoder_a, 1)
        self.assertEqual(sample.encoder_b, 0)
        self.assertIsInstance(decode_device_report(raw), TelemetrySample)

    def test_rejects_invalid_report_shapes_and_headers(self) -> None:
        with self.assertRaises(ValueError):
            decode_runtime_report(b"\x00" * (HID_REPORT_SIZE - 1))
        with self.assertRaises(ValueError):
            decode_ack(b"\x00" * (HID_REPORT_SIZE + 1))

        bad_runtime = bytearray(HID_REPORT_SIZE)
        bad_runtime[0] = HID_SCHEMA + 1
        with self.assertRaises(ValueError):
            decode_runtime_report(bytes(bad_runtime))

        bad_ack = bytearray(HID_REPORT_SIZE)
        bad_ack[0] = HID_SCHEMA
        bad_ack[1] = 0x80
        with self.assertRaises(ValueError):
            decode_ack(bytes(bad_ack))


if __name__ == "__main__":
    unittest.main()
