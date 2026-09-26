from __future__ import annotations

from collections import deque
import unittest

from device import Rp2350Device
from protocol import CommandAck, HidCommand, HidStatus, NeopixelColor, TelemetrySample


class FakeHid:
    def __init__(self, reports) -> None:
        self.reports = deque(reports)
        self.writes: list[bytes] = []

    def write_report(self, report: bytes) -> None:
        self.writes.append(report)

    def read_report(self, timeout_ms: int):
        del timeout_ms
        return self.reports.popleft() if self.reports else None


class DeviceCommandTests(unittest.TestCase):
    @staticmethod
    def _ack(
        command: HidCommand,
        *,
        sequence: int = 1,
        status: HidStatus = HidStatus.OK,
        value0: float = 0.0,
    ) -> CommandAck:
        return CommandAck(
            version=1,
            message_type=0x81,
            command=int(command),
            status=int(status),
            sequence=sequence,
            timestamp_us=1234,
            value0=value0,
            value1=0.0,
            detail=0,
        )

    @staticmethod
    def _telemetry() -> TelemetrySample:
        return TelemetrySample(
            schema=1,
            flags=1,
            sequence=7,
            timestamp_us=1000,
            sample_index=10,
            pendulum_adc_raw=2048,
            arm_encoder_count=3,
            theta=0.0,
            theta_dot=0.0,
            phi=0.0,
            phi_dot=0.0,
            arm_torque_nm=0.0,
            normalized_command=0.0,
            missed_opportunities=0,
            deadline_overruns=0,
            execution_time_us=20,
            regime=0,
            runtime_state=1,
            authority_mode=0,
            encoder_a=0,
            encoder_b=1,
        )

    def test_command_queues_interleaved_telemetry_until_matching_ack(self) -> None:
        device = Rp2350Device()
        telemetry = self._telemetry()
        fake_hid = FakeHid([telemetry, self._ack(HidCommand.GET_STATUS)])
        device.hid = fake_hid

        ack = device.command(HidCommand.GET_STATUS)

        self.assertTrue(ack.ok)
        self.assertEqual(len(fake_hid.writes), 1)
        self.assertIs(device.read_sample(1), telemetry)

    def test_command_rejects_ack_for_different_command(self) -> None:
        device = Rp2350Device()
        device.hid = FakeHid([self._ack(HidCommand.SAFE_OFF)])

        with self.assertRaisesRegex(RuntimeError, "ack command mismatch"):
            device.command(HidCommand.GET_STATUS)

    def test_command_surfaces_firmware_rejection_status(self) -> None:
        device = Rp2350Device()
        device.hid = FakeHid(
            [self._ack(HidCommand.SET_MOTOR_COMMAND, status=HidStatus.RANGE)]
        )

        with self.assertRaisesRegex(RuntimeError, "RANGE"):
            device.command(HidCommand.SET_MOTOR_COMMAND)

    def test_user_led_returns_acknowledged_state(self) -> None:
        device = Rp2350Device()
        device.hid = FakeHid(
            [self._ack(HidCommand.SET_USER_LED, value0=1.0)]
        )

        self.assertTrue(device.set_user_led(True))

    def test_neopixel_returns_acknowledged_color(self) -> None:
        device = Rp2350Device()
        device.hid = FakeHid(
            [self._ack(HidCommand.SET_NEOPIXEL, value0=float(int(NeopixelColor.GREEN)))]
        )

        self.assertEqual(device.set_neopixel(NeopixelColor.GREEN), NeopixelColor.GREEN)

    def test_bootloader_handoff_marks_session_non_closable(self) -> None:
        device = Rp2350Device()
        device.hid = FakeHid([self._ack(HidCommand.ENTER_USB_BOOTLOADER)])

        device.enter_usb_bootloader()

        self.assertTrue(device._bootloader_requested)


if __name__ == "__main__":
    unittest.main()
