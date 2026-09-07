#![no_std]
#![forbid(unsafe_code)]

use rip_runtime_observation_record::RuntimeRecordSnapshot;
use rip_timing_evidence::TimingEvidenceSnapshot;

pub const TELEMETRY_PROTOCOL_VERSION: u8 = 1;
pub const TELEMETRY_PACKET_KIND_RUNTIME: u8 = 1;
pub const TELEMETRY_PACKET_LEN: usize = 64;

const MAGIC: [u8; 2] = *b"RI";

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TelemetrySnapshot {
    pub timestamp_us_low: u32,
    pub sample_index: u32,
    pub cycle: u8,
    pub control_regime: u8,
    pub sensor_timing_health: u8,
    pub watchdog_health: u8,
    pub authorized: bool,
    pub actuator_saturated: bool,
    pub qualification_reasons: u32,
    pub authority_reasons: u32,
    pub theta_mrad: i32,
    pub theta_dot_mrad_s: i32,
    pub phi_mrad: i32,
    pub phi_dot_mrad_s: i32,
    pub demand_torque_unm: i32,
    pub bounded_command_ppm: i32,
    pub predicted_torque_unm: i32,
    pub inferred_missed_ticks: u16,
}

impl TelemetrySnapshot {
    pub fn from_records(
        runtime: RuntimeRecordSnapshot,
        timing: TimingEvidenceSnapshot,
    ) -> Self {
        Self {
            timestamp_us_low: runtime.timestamp_us_low,
            sample_index: runtime.sample_index,
            cycle: runtime.cycle.min(u32::from(u8::MAX)) as u8,
            control_regime: runtime.control_regime.min(u32::from(u8::MAX)) as u8,
            sensor_timing_health: runtime.sensor_timing_health.min(u32::from(u8::MAX)) as u8,
            watchdog_health: runtime.watchdog_health.min(u32::from(u8::MAX)) as u8,
            authorized: runtime.authorized,
            actuator_saturated: runtime.actuator_saturated,
            qualification_reasons: runtime.qualification_reasons,
            authority_reasons: runtime.authority_reasons,
            theta_mrad: runtime.theta_mrad,
            theta_dot_mrad_s: runtime.theta_dot_mrad_s,
            phi_mrad: runtime.phi_mrad,
            phi_dot_mrad_s: runtime.phi_dot_mrad_s,
            demand_torque_unm: runtime.demand_torque_unm,
            bounded_command_ppm: runtime.bounded_command_ppm,
            predicted_torque_unm: runtime.predicted_torque_unm,
            inferred_missed_ticks: timing
                .inferred_missed_tick_count
                .min(u32::from(u16::MAX)) as u16,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TelemetryTxOutcome {
    Sent,
    Busy,
}

pub trait TelemetryTransport {
    type Error;

    fn try_send(&mut self, packet: &[u8]) -> Result<TelemetryTxOutcome, Self::Error>;
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TelemetryPublisherStats {
    pub opportunities: u32,
    pub sent: u32,
    pub dropped_busy: u32,
}

pub struct TelemetryPublisher<T> {
    transport: T,
    next_sequence: u32,
    stats: TelemetryPublisherStats,
}

impl<T> TelemetryPublisher<T>
where
    T: TelemetryTransport,
{
    pub const fn new(transport: T) -> Self {
        Self {
            transport,
            next_sequence: 0,
            stats: TelemetryPublisherStats {
                opportunities: 0,
                sent: 0,
                dropped_busy: 0,
            },
        }
    }

    /// Publish only the freshest snapshot for this opportunity.
    ///
    /// The publisher owns no replay queue. If the transport is still busy with
    /// an older packet, this opportunity is dropped and the next call encodes
    /// the next fresh snapshot.
    pub fn publish_latest(
        &mut self,
        snapshot: TelemetrySnapshot,
    ) -> Result<TelemetryTxOutcome, T::Error> {
        let sequence = self.next_sequence;
        self.next_sequence = self.next_sequence.wrapping_add(1);
        self.stats.opportunities = self.stats.opportunities.wrapping_add(1);
        let packet = encode_runtime_packet(sequence, snapshot);
        let outcome = self.transport.try_send(&packet)?;
        match outcome {
            TelemetryTxOutcome::Sent => self.stats.sent = self.stats.sent.wrapping_add(1),
            TelemetryTxOutcome::Busy => {
                self.stats.dropped_busy = self.stats.dropped_busy.wrapping_add(1)
            }
        }
        Ok(outcome)
    }

    pub const fn stats(&self) -> TelemetryPublisherStats {
        self.stats
    }

    pub fn transport(&self) -> &T {
        &self.transport
    }

    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    pub fn into_transport(self) -> T {
        self.transport
    }
}

pub fn encode_runtime_packet(
    sequence: u32,
    snapshot: TelemetrySnapshot,
) -> [u8; TELEMETRY_PACKET_LEN] {
    let mut out = [0_u8; TELEMETRY_PACKET_LEN];
    out[0..2].copy_from_slice(&MAGIC);
    out[2] = TELEMETRY_PROTOCOL_VERSION;
    out[3] = TELEMETRY_PACKET_KIND_RUNTIME;
    put_u32(&mut out, 4, sequence);
    put_u32(&mut out, 8, snapshot.timestamp_us_low);
    put_u32(&mut out, 12, snapshot.sample_index);
    out[16] = snapshot.cycle;
    out[17] = snapshot.control_regime;
    out[18] = snapshot.sensor_timing_health;
    out[19] = snapshot.watchdog_health;
    out[20] = u8::from(snapshot.authorized);
    out[21] = u8::from(snapshot.actuator_saturated);
    put_u32(&mut out, 24, snapshot.qualification_reasons);
    put_u32(&mut out, 28, snapshot.authority_reasons);
    put_i32(&mut out, 32, snapshot.theta_mrad);
    put_i32(&mut out, 36, snapshot.theta_dot_mrad_s);
    put_i32(&mut out, 40, snapshot.phi_mrad);
    put_i32(&mut out, 44, snapshot.phi_dot_mrad_s);
    put_i32(&mut out, 48, snapshot.demand_torque_unm);
    put_i32(&mut out, 52, snapshot.bounded_command_ppm);
    put_i32(&mut out, 56, snapshot.predicted_torque_unm);
    put_u16(&mut out, 60, snapshot.inferred_missed_ticks);
    let crc = crc16_ccitt_false(&out[..62]);
    put_u16(&mut out, 62, crc);
    out
}

pub fn packet_crc_is_valid(packet: &[u8; TELEMETRY_PACKET_LEN]) -> bool {
    let expected = u16::from_le_bytes([packet[62], packet[63]]);
    crc16_ccitt_false(&packet[..62]) == expected
}

fn put_u16(out: &mut [u8], offset: usize, value: u16) {
    out[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
}

fn put_u32(out: &mut [u8], offset: usize, value: u32) {
    out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn put_i32(out: &mut [u8], offset: usize, value: i32) {
    out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn crc16_ccitt_false(bytes: &[u8]) -> u16 {
    let mut crc = 0xffff_u16;
    for &byte in bytes {
        crc ^= u16::from(byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::vec::Vec;

    #[derive(Default)]
    struct MockTransport {
        busy_once: bool,
        packets: Vec<[u8; TELEMETRY_PACKET_LEN]>,
    }

    impl TelemetryTransport for MockTransport {
        type Error = ();

        fn try_send(&mut self, packet: &[u8]) -> Result<TelemetryTxOutcome, Self::Error> {
            if self.busy_once {
                self.busy_once = false;
                return Ok(TelemetryTxOutcome::Busy);
            }
            let mut copy = [0_u8; TELEMETRY_PACKET_LEN];
            copy.copy_from_slice(packet);
            self.packets.push(copy);
            Ok(TelemetryTxOutcome::Sent)
        }
    }

    fn snapshot(sample_index: u32) -> TelemetrySnapshot {
        TelemetrySnapshot {
            sample_index,
            timestamp_us_low: sample_index * 1_000,
            cycle: 3,
            control_regime: 2,
            sensor_timing_health: 1,
            watchdog_health: 1,
            authorized: false,
            actuator_saturated: false,
            qualification_reasons: 0x12,
            authority_reasons: 0x34,
            theta_mrad: -45,
            theta_dot_mrad_s: 12,
            phi_mrad: 123,
            phi_dot_mrad_s: -8,
            demand_torque_unm: 7_000,
            bounded_command_ppm: 91_000,
            predicted_torque_unm: 6_500,
            inferred_missed_ticks: 2,
        }
    }

    #[test]
    fn packet_has_identity_sequence_and_crc() {
        let packet = encode_runtime_packet(7, snapshot(11));
        assert_eq!(&packet[0..2], b"RI");
        assert_eq!(packet[2], TELEMETRY_PROTOCOL_VERSION);
        assert_eq!(packet[3], TELEMETRY_PACKET_KIND_RUNTIME);
        assert_eq!(u32::from_le_bytes(packet[4..8].try_into().unwrap()), 7);
        assert_eq!(u32::from_le_bytes(packet[12..16].try_into().unwrap()), 11);
        assert!(packet_crc_is_valid(&packet));
    }

    #[test]
    fn busy_transport_drops_current_snapshot_without_replay() {
        let transport = MockTransport {
            busy_once: true,
            packets: Vec::new(),
        };
        let mut publisher = TelemetryPublisher::new(transport);
        assert_eq!(
            publisher.publish_latest(snapshot(1)).unwrap(),
            TelemetryTxOutcome::Busy
        );
        assert_eq!(
            publisher.publish_latest(snapshot(2)).unwrap(),
            TelemetryTxOutcome::Sent
        );
        assert_eq!(publisher.stats().dropped_busy, 1);
        let transport = publisher.into_transport();
        assert_eq!(transport.packets.len(), 1);
        assert_eq!(u32::from_le_bytes(transport.packets[0][12..16].try_into().unwrap()), 2);
    }
}
