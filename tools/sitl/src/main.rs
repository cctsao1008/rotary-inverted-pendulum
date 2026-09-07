mod evidence;
mod scenario;
mod scheduler;
mod virtual_time;

use std::env;
use std::error::Error;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

use evidence::{Manifest, RunArtifacts, Summary, TraceRecord, SCHEMA_VERSION};
use scenario::Scenario;
use scheduler::{EventKind, Scheduler};
use virtual_time::{VirtualDuration, VirtualTime};

#[derive(Debug)]
struct Cli {
    scenario: PathBuf,
    output: PathBuf,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("rip-sitl: {error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let cli =
        parse_cli().map_err(|message| io::Error::new(io::ErrorKind::InvalidInput, message))?;
    let scenario = Scenario::load(&cli.scenario)?;
    let artifacts = execute(&scenario)?;
    write_artifacts(&cli.output, &artifacts)?;

    println!("SITL deterministic skeleton");
    println!("scenario............... {}", scenario.id);
    println!("duration_us............ {}", scenario.duration_us);
    println!("output................. {}", cli.output.display());
    Ok(())
}

fn parse_cli() -> Result<Cli, String> {
    let mut args = env::args().skip(1);
    let mut scenario = None;
    let mut output = None;

    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--scenario" => {
                scenario = Some(PathBuf::from(
                    args.next()
                        .ok_or_else(|| "--scenario requires a path".to_string())?,
                ));
            }
            "--output" => {
                output = Some(PathBuf::from(
                    args.next()
                        .ok_or_else(|| "--output requires a path".to_string())?,
                ));
            }
            "--help" | "-h" => {
                println!("usage: rip-sitl --scenario <scenario.toml> --output <directory>");
                std::process::exit(0);
            }
            _ => return Err(format!("unknown argument: {argument}")),
        }
    }

    Ok(Cli {
        scenario: scenario.ok_or_else(|| "missing --scenario".to_string())?,
        output: output.ok_or_else(|| "missing --output".to_string())?,
    })
}

fn execute(scenario: &Scenario) -> Result<RunArtifacts, Box<dyn Error>> {
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

    while let Some(slice) = scheduler.next_slice() {
        debug_assert_eq!(scheduler.now(), slice.at);
        for event in slice.events {
            match event.kind {
                EventKind::ScenarioStart => {}
                EventKind::SensorSample => scheduled_sensor_samples += 1,
                EventKind::ObservationDelivery => delivered_observations += 1,
                EventKind::ProductionRuntime => admitted_runtime_opportunities += 1,
                EventKind::RuntimeOpportunityMissed => missed_runtime_opportunities += 1,
                EventKind::ActuationCommit => actuation_commits += 1,
            }

            records.push(TraceRecord {
                schema_version: SCHEMA_VERSION,
                event_sequence,
                virtual_time_us: event.at.as_micros(),
                semantic_phase: event.phase.as_str(),
                record_kind: event.kind.record_kind(),
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
        && actuation_commits == admitted_runtime_opportunities;

    let manifest = Manifest {
        schema_version: SCHEMA_VERSION,
        scenario: scenario.id.clone(),
        seed: scenario.seed,
        duration_us: scenario.duration_us,
        sensor_period_us: scenario.sensor_period_us,
        runtime_period_us: scenario.runtime_period_us,
        missed_runtime_at_us: scenario.missed_runtime_at_us.clone(),
    };
    let summary = Summary {
        schema_version: SCHEMA_VERSION,
        scenario: scenario.id.clone(),
        pass,
        scheduled_sensor_samples,
        delivered_observations,
        scheduled_runtime_opportunities,
        admitted_runtime_opportunities,
        missed_runtime_opportunities,
        actuation_commits,
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

fn write_artifacts(output: &Path, artifacts: &RunArtifacts) -> io::Result<()> {
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
        }
    }

    #[test]
    fn same_scenario_and_seed_produce_byte_identical_artifacts() {
        let scenario = deterministic_scenario();
        let first = execute(&scenario).unwrap();
        let second = execute(&scenario).unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn missed_runtime_opportunity_is_recorded_and_never_replayed() {
        let mut scenario = deterministic_scenario();
        scenario.duration_us = 20_000;
        scenario.missed_runtime_at_us = vec![10_000];

        let artifacts = execute(&scenario).unwrap();
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
}
