import unittest

import runtime_v1 as telemetry

# This byte vector is the exact firmware v1 layout for the snapshot used by
# firmware/communications/telemetry/src/lib.rs::packet_has_identity_sequence_and_crc:
# sequence=7, sample_index=11, timestamp=11000 us, cycle=3, regime=2,
# timing/watchdog=1/1, reasons=0x12/0x34, theta=-45 mrad,
# theta_dot=12 mrad/s, phi=123 mrad, phi_dot=-8 mrad/s,
# demand=7000 uN*m, command=91000 ppm, predicted=6500 uN*m, missed=2.
FIRMWARE_V1_GOLDEN = bytes.fromhex(
    "5249010107000000f82a00000b0000000302010100000000"
    "1200000034000000d3ffffff0c0000007b000000f8ffffff"
    "581b00007863010064190000020079d4"
)


class FirmwareGoldenVectorTests(unittest.TestCase):
    def test_host_decoder_matches_firmware_v1_golden_vector(self) -> None:
        packet = telemetry.decode_packet(FIRMWARE_V1_GOLDEN)
        self.assertEqual(packet.sequence, 7)
        self.assertEqual(packet.timestamp_us_low, 11_000)
        self.assertEqual(packet.sample_index, 11)
        self.assertEqual(packet.cycle, 3)
        self.assertEqual(packet.control_regime, 2)
        self.assertEqual(packet.sensor_timing_health, 1)
        self.assertEqual(packet.watchdog_health, 1)
        self.assertFalse(packet.authorized)
        self.assertFalse(packet.actuator_saturated)
        self.assertEqual(packet.qualification_reasons, 0x12)
        self.assertEqual(packet.authority_reasons, 0x34)
        self.assertEqual(packet.theta_mrad, -45)
        self.assertEqual(packet.theta_dot_mrad_s, 12)
        self.assertEqual(packet.phi_mrad, 123)
        self.assertEqual(packet.phi_dot_mrad_s, -8)
        self.assertEqual(packet.demand_torque_unm, 7_000)
        self.assertEqual(packet.bounded_command_ppm, 91_000)
        self.assertEqual(packet.predicted_torque_unm, 6_500)
        self.assertEqual(packet.inferred_missed_ticks, 2)
        self.assertEqual(packet.crc16, 0xD479)


if __name__ == "__main__":
    unittest.main()
