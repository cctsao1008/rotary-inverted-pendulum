use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::error::Error;
use std::fmt::{Display, Formatter};

use crate::virtual_time::VirtualTime;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum SemanticPhase {
    PhysicalOrFaultEvent,
    SensorSample,
    ObservationDelivery,
    ProductionRuntime,
    ActuationCommit,
}

impl SemanticPhase {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::PhysicalOrFaultEvent => "physical_or_fault_event",
            Self::SensorSample => "sensor_sample",
            Self::ObservationDelivery => "observation_delivery",
            Self::ProductionRuntime => "production_runtime",
            Self::ActuationCommit => "actuation_commit",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EventKind {
    ScenarioStart,
    SensorSample,
    ObservationDelivery,
    ProductionRuntime,
    RuntimeOpportunityMissed,
    ActuationCommit,
}

impl EventKind {
    pub const fn phase(self) -> SemanticPhase {
        match self {
            Self::ScenarioStart => SemanticPhase::PhysicalOrFaultEvent,
            Self::SensorSample => SemanticPhase::SensorSample,
            Self::ObservationDelivery => SemanticPhase::ObservationDelivery,
            Self::ProductionRuntime | Self::RuntimeOpportunityMissed => {
                SemanticPhase::ProductionRuntime
            }
            Self::ActuationCommit => SemanticPhase::ActuationCommit,
        }
    }

    pub const fn record_kind(self) -> &'static str {
        match self {
            Self::ScenarioStart => "scenario_start",
            Self::SensorSample => "sensor_sample",
            Self::ObservationDelivery => "observation_delivery",
            Self::ProductionRuntime => "production_runtime",
            Self::RuntimeOpportunityMissed => "runtime_opportunity_missed",
            Self::ActuationCommit => "actuation_commit",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScheduledEvent {
    pub at: VirtualTime,
    pub phase: SemanticPhase,
    pub insertion_sequence: u64,
    pub kind: EventKind,
}

impl Ord for ScheduledEvent {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .at
            .cmp(&self.at)
            .then_with(|| other.phase.cmp(&self.phase))
            .then_with(|| other.insertion_sequence.cmp(&self.insertion_sequence))
    }
}

impl PartialOrd for ScheduledEvent {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum ScheduleError {
    EventInPast { now: VirtualTime, at: VirtualTime },
    SequenceExhausted,
}

impl Display for ScheduleError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EventInPast { now, at } => write!(
                formatter,
                "cannot schedule event at {} us while virtual time is {} us",
                at.as_micros(),
                now.as_micros()
            ),
            Self::SequenceExhausted => write!(formatter, "scheduler insertion sequence exhausted"),
        }
    }
}

impl Error for ScheduleError {}

#[derive(Debug, PartialEq, Eq)]
pub struct TimeSlice {
    pub at: VirtualTime,
    pub events: Vec<ScheduledEvent>,
}

#[derive(Debug, Default)]
pub struct Scheduler {
    now: VirtualTime,
    next_sequence: u64,
    queue: BinaryHeap<ScheduledEvent>,
}

impl Scheduler {
    pub fn new() -> Self {
        Self::default()
    }

    pub const fn now(&self) -> VirtualTime {
        self.now
    }

    pub fn schedule(&mut self, at: VirtualTime, kind: EventKind) -> Result<u64, ScheduleError> {
        if at < self.now {
            return Err(ScheduleError::EventInPast { now: self.now, at });
        }

        let insertion_sequence = self.next_sequence;
        self.next_sequence = self
            .next_sequence
            .checked_add(1)
            .ok_or(ScheduleError::SequenceExhausted)?;

        self.queue.push(ScheduledEvent {
            at,
            phase: kind.phase(),
            insertion_sequence,
            kind,
        });
        Ok(insertion_sequence)
    }

    /// Advances virtual time exactly once to the next timestamp and returns all
    /// events at that timestamp in deterministic semantic order.
    pub fn next_slice(&mut self) -> Option<TimeSlice> {
        let first = self.queue.pop()?;
        let at = first.at;
        self.now = at;

        let mut events = vec![first];
        while self.queue.peek().is_some_and(|event| event.at == at) {
            events.push(self.queue.pop().expect("peeked event must exist"));
        }
        events.sort_by_key(|event| (event.phase, event.insertion_sequence));

        Some(TimeSlice { at, events })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn events_at_same_time_follow_semantic_phase_not_insertion_order() {
        let mut scheduler = Scheduler::new();
        let at = VirtualTime(5_000);

        scheduler.schedule(at, EventKind::ActuationCommit).unwrap();
        scheduler
            .schedule(at, EventKind::ProductionRuntime)
            .unwrap();
        scheduler
            .schedule(at, EventKind::ObservationDelivery)
            .unwrap();
        scheduler.schedule(at, EventKind::SensorSample).unwrap();
        scheduler.schedule(at, EventKind::ScenarioStart).unwrap();

        let slice = scheduler.next_slice().unwrap();
        let phases: Vec<_> = slice.events.iter().map(|event| event.phase).collect();
        assert_eq!(
            phases,
            vec![
                SemanticPhase::PhysicalOrFaultEvent,
                SemanticPhase::SensorSample,
                SemanticPhase::ObservationDelivery,
                SemanticPhase::ProductionRuntime,
                SemanticPhase::ActuationCommit,
            ]
        );
        assert_eq!(scheduler.now(), at);
    }

    #[test]
    fn equal_phase_events_preserve_insertion_sequence() {
        let mut scheduler = Scheduler::new();
        let at = VirtualTime(5_000);
        let first = scheduler
            .schedule(at, EventKind::ProductionRuntime)
            .unwrap();
        let second = scheduler
            .schedule(at, EventKind::RuntimeOpportunityMissed)
            .unwrap();

        let slice = scheduler.next_slice().unwrap();
        assert_eq!(slice.events[0].insertion_sequence, first);
        assert_eq!(slice.events[1].insertion_sequence, second);
    }
}
