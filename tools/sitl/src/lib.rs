pub mod evidence;
pub mod parameters;
pub mod rotary;
pub mod scenario;
pub mod scheduler;
pub mod virtual_time;

use std::error::Error;
use std::fs;
use std::io;
use std::path::Path;

pub use evidence::RunArtifacts;
use evidence::{Manifest, Summary, TraceRecord, SCHEMA_VERSION};
pub use parameters::ReferenceAssemblyParameters;
pub use rotary::RotarySitlSystem;
pub use scenario::{RotaryScenario, Scenario};
pub use scheduler::EventKind;
use scheduler::Scheduler;
use serde_json::{json, Value};
use virtual_time::{VirtualDuration, VirtualTime};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RunContext {
    pub system_identifier: String,
    pub git_commit: String,
    pub production_model_configuration: Value,
    pub virtual_physical_truth_configuration: Value,
}

impl RunContext {
    pub fn stage1(system_identifier: impl Into<String>, git_commit: impl Into<String>) -> Self {
        Self {
            system_identifier: system_identifier.into(),
            git_commit: git_commit.into(),
            production_model_configuration: json!({"mode": "not-materialized"}),
            virtual_physical_truth_configuration: json!({"mode": "not-materialized"}),
        }
    }

    pub fn with_model_configurations(
        mut self,
        production_model_configuration: Value,
        virtual_physical_truth_configuration: Value,
    ) -> Self {
        self.production_model_configuration = production_model_configuration;
        self.virtual_physical_truth_configuration = virtual_physical_truth_configuration;
        self
    }
}

pub trait SitlSystem {
    fn advance_physical_time(
        &mut self,
        _from: VirtualTime,
        _to: VirtualTime,
    ) -> Result<(), Box<dyn Error>> {
        Ok(())
    }

    fn on_event(
        &mut self,
        _kind: EventKind,
        _at: VirtualTime,
    ) -> Result<Option<Value>, Box<dyn Error>> {
        Ok(None)
    }

    fn summary(&self) -> Value {
        Value::Null
    }
}

#[derive(Debug, Default)]
pub struct NoopSitlSystem;

impl SitlSystem for NoopSitlSystem {}

pub fn execute(context: &RunContext, scenario: &Scenario) -> Result<RunArtifacts, Box<dyn Error>> {
    let mut system = NoopSitlSystem;
    execute_with_system(context, scenario, &mut system)
}

pub fn execute_with_system<S: SitlSystem>(
    context: &RunContext,
    scenario: &Scenario,
    system: &mut S,
) -> Result<RunArtifacts, Box<dyn Error>> {
    scenario.validate()?;

    let mut scheduler = Scheduler::new();
    scheduler.schedule(VirtualTime::ZERO, EventKind::ScenarioStart)?;
    schedule_periodic(
        scenario.duration_us,
        VirtualDuration::from_micros(scenario.sensor_period_us),
        |at| {
            scheduler.schedule(VirtualTime(at), EventKind::SensorSample)?;
            scheduler.schedule(VirtualTime(at), EventKind::ObservationDelivery)?;
            Ok(())
        },
    )?;
    schedule_periodic(
        scenario.duration_us,
        VirtualDuration::from_micros(scenario.runtime_period_us),
        |at| {
            if scenario.runtime_is_missed(at) {
                scheduler.schedule(VirtualTime(at), EventKind::RuntimeOpportunityMissed)?;
            } else {
                scheduler.schedule(VirtualTime(at), EventKind::ProductionRuntime)?;
                scheduler.schedule(VirtualTime(at), EventKind::ActuationCommit)?;
            }
            Ok(())
        },
    )?;

    let mut records = Vec::new();
    let mut event_sequence = 0_u64;
    let mut scheduled_sensor_samples = 0_u64;
    let mut delivered_observations = 0_u64;
    let mut admitted_runtime_opportunities = 0_u64;
    let mut missed_runtime_opportunities = 0_u64;
    let mut actuation_commits = 0_u64;
    let mut time_slices = 0_u64;
    let mut time_advances = 0_u64;
    let mut previous_time = VirtualTime::ZERO;

    while let Some(slice) = scheduler.next_slice() {
        debug_assert_eq!(scheduler.now(), slice.at);
        if slice.at > previous_time {
            system.advance_physical_time(previous_time, slice.at)?;
            time_advances = time_advances
                .checked_add(1)
                .ok_or_else(|| io::Error::other("time-advance counter exhausted"))?;
        }
        previous_time = slice.at;
        time_slices = time_slices
            .checked_add(1)
            .ok_or_else(|| io::Error::other("time-slice counter exhausted"))?;

        for event in slice.events {
            match event.kind {
                EventKind::ScenarioStart => {}
                EventKind::SensorSample => scheduled_sensor_samples += 1,
                EventKind::ObservationDelivery => delivered_observations += 1,
                EventKind::ProductionRuntime => admitted_runtime_opportunities += 1,
                EventKind::RuntimeOpportunityMissed => missed_runtime_opportunities += 1,
                EventKind::ActuationCommit => actuation_commits += 1,
            }

            let system_record = system.on_event(event.kind, event.at)?;
            records.push(TraceRecord {
                schema_version: SCHEMA_VERSION,
                event_sequence,
                virtual_time_us: event.at.as_micros(),
                semantic_phase: event.phase.as_str(),
                record_kind: event.kind.record_kind(),
                system: system_record,
            });
            event_sequence = event_sequence
                .checked_add(1)
                .ok_or_else(|| io::Error::other("trace event sequence exhausted"))?;
        }
    }

    let expected_sensor_samples = periodic_count(scenario.duration_us, scenario.sensor_period_us);
    let expected_runtime_opportunities =
        periodic_count(scenario.duration_us, scenario.runtime_period_us);
    let scheduled_runtime_opportunities =
        admitted_runtime_opportunities + missed_runtime_opportunities;
    let pass = scheduled_sensor_samples == expected_sensor_samples
        && delivered_observations == expected_sensor_samples
        && scheduled_runtime_opportunities == expected_runtime_opportunities
        && actuation_commits == admitted_runtime_opportunities
        && time_advances == time_slices.saturating_sub(1);

    let manifest = Manifest {
        schema_version: SCHEMA_VERSION,
        system_identifier: context.system_identifier.clone(),
        git_commit: context.git_commit.clone(),
        scenario: scenario.id.clone(),
        seed: scenario.seed,
        duration_us: scenario.duration_us,
        sensor_period_us: scenario.sensor_period_us,
        runtime_period_us: scenario.runtime_period_us,
        missed_runtime_at_us: scenario.missed_runtime_at_us.clone(),
        production_model_configuration: context.production_model_configuration.clone(),
        virtual_physical_truth_configuration: context.virtual_physical_truth_configuration.clone(),
    };
    let summary = Summary {
        schema_version: SCHEMA_VERSION,
        scenario: scenario.id.clone(),
        pass,
        time_slices,
        time_advances,
        scheduled_sensor_samples,
        delivered_observations,
        scheduled_runtime_opportunities,
        admitted_runtime_opportunities,
        missed_runtime_opportunities,
        actuation_commits,
        system: system.summary(),
    };

    Ok(RunArtifacts {
        manifest_json: evidence::render_json(&manifest)?,
        trace_jsonl: evidence::render_trace(&records)?,
        summary_json: evidence::render_json(&summary)?,
    })
}

fn schedule_periodic<F>(
    duration_us: u64,
    period: VirtualDuration,
    mut schedule: F,
) -> Result<(), Box<dyn Error>>
where
    F: FnMut(u64) -> Result<(), scheduler::ScheduleError>,
{
    let end = VirtualTime(duration_us);
    let mut at = VirtualTime::ZERO;
    loop {
        schedule(at.as_micros())?;
        let Some(next) = at.checked_add(period) else {
            break;
        };
        if next > end {
            break;
        }
        at = next;
    }
    Ok(())
}

const fn periodic_count(duration_us: u64, period_us: u64) -> u64 {
    duration_us / period_us + 1
}

pub fn write_artifacts(output: &Path, artifacts: &RunArtifacts) -> io::Result<()> {
    fs::create_dir_all(output)?;
    fs::write(output.join("manifest.json"), &artifacts.manifest_json)?;
    fs::write(output.join("trace.jsonl"), &artifacts.trace_jsonl)?;
    fs::write(output.join("summary.json"), &artifacts.summary_json)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn deterministic_scenario() -> Scenario {
        Scenario {
            id: "deterministic-smoke".to_string(),
            duration_us: 100_000,
            seed: 1,
            sensor_period_us: 5_000,
            runtime_period_us: 5_000,
            missed_runtime_at_us: Vec::new(),
            rotary: None,
        }
    }

    fn context() -> RunContext {
        RunContext::stage1("rotary-inverted-pendulum", "test-commit")
    }

    #[test]
    fn same_scenario_and_context_produce_byte_identical_artifacts() {
        let scenario = deterministic_scenario();
        let context = context();
        let first = execute(&context, &scenario).unwrap();
        let second = execute(&context, &scenario).unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn manifest_preserves_production_and_physical_truth_provenance() {
        let artifacts = execute(&context(), &deterministic_scenario()).unwrap();
        let manifest: serde_json::Value = serde_json::from_str(&artifacts.manifest_json).unwrap();

        assert_eq!(manifest["system_identifier"], "rotary-inverted-pendulum");
        assert_eq!(manifest["git_commit"], "test-commit");
        assert_eq!(
            manifest["production_model_configuration"]["mode"],
            "not-materialized"
        );
        assert_eq!(
            manifest["virtual_physical_truth_configuration"]["mode"],
            "not-materialized"
        );
    }

    #[test]
    fn missed_runtime_opportunity_is_recorded_and_never_replayed() {
        let mut scenario = deterministic_scenario();
        scenario.duration_us = 20_000;
        scenario.missed_runtime_at_us = vec![10_000];

        let artifacts = execute(&context(), &scenario).unwrap();
        let records: Vec<serde_json::Value> = artifacts
            .trace_jsonl
            .lines()
            .map(|line| serde_json::from_str(line).unwrap())
            .collect();
        let at_missed_time: Vec<_> = records
            .iter()
            .filter(|record| record["virtual_time_us"] == 10_000)
            .collect();

        assert!(at_missed_time
            .iter()
            .any(|record| record["record_kind"] == "runtime_opportunity_missed"));
        assert!(!at_missed_time
            .iter()
            .any(|record| record["record_kind"] == "production_runtime"));
        assert!(!at_missed_time
            .iter()
            .any(|record| record["record_kind"] == "actuation_commit"));

        let summary: serde_json::Value = serde_json::from_str(&artifacts.summary_json).unwrap();
        assert_eq!(summary["missed_runtime_opportunities"], 1);
        assert_eq!(summary["admitted_runtime_opportunities"], 4);
        assert_eq!(summary["actuation_commits"], 4);
        assert_eq!(summary["pass"], true);
    }

    #[derive(Default)]
    struct TimeAdvanceProbe {
        intervals: Vec<(u64, u64)>,
    }

    impl SitlSystem for TimeAdvanceProbe {
        fn advance_physical_time(
            &mut self,
            from: VirtualTime,
            to: VirtualTime,
        ) -> Result<(), Box<dyn Error>> {
            self.intervals.push((from.as_micros(), to.as_micros()));
            Ok(())
        }
    }

    #[test]
    fn physical_time_advances_between_slices_not_as_a_queued_event() {
        let mut scenario = deterministic_scenario();
        scenario.duration_us = 20_000;
        let mut probe = TimeAdvanceProbe::default();

        let artifacts = execute_with_system(&context(), &scenario, &mut probe).unwrap();

        assert_eq!(
            probe.intervals,
            vec![
                (0, 5_000),
                (5_000, 10_000),
                (10_000, 15_000),
                (15_000, 20_000)
            ]
        );
        assert!(!artifacts.trace_jsonl.contains("integrate_plant_to"));

        let summary: serde_json::Value = serde_json::from_str(&artifacts.summary_json).unwrap();
        assert_eq!(summary["time_slices"], 5);
        assert_eq!(summary["time_advances"], 4);
        assert_eq!(summary["pass"], true);
    }
}
