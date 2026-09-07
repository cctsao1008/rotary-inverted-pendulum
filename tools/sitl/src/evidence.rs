use serde::Serialize;
use serde_json::Value;

pub const SCHEMA_VERSION: u32 = 2;

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct Manifest {
    pub schema_version: u32,
    pub system_identifier: String,
    pub git_commit: String,
    pub scenario: String,
    pub seed: u64,
    pub duration_us: u64,
    pub sensor_period_us: u64,
    pub runtime_period_us: u64,
    pub missed_runtime_at_us: Vec<u64>,
    pub production_model_configuration: Value,
    pub virtual_physical_truth_configuration: Value,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct TraceRecord {
    pub schema_version: u32,
    pub event_sequence: u64,
    pub virtual_time_us: u64,
    pub semantic_phase: &'static str,
    pub record_kind: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub system: Option<Value>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct Summary {
    pub schema_version: u32,
    pub scenario: String,
    pub pass: bool,
    pub time_slices: u64,
    pub time_advances: u64,
    pub scheduled_sensor_samples: u64,
    pub delivered_observations: u64,
    pub scheduled_runtime_opportunities: u64,
    pub admitted_runtime_opportunities: u64,
    pub missed_runtime_opportunities: u64,
    pub actuation_commits: u64,
    pub system: Value,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RunArtifacts {
    pub manifest_json: String,
    pub trace_jsonl: String,
    pub summary_json: String,
}

pub fn render_json<T: Serialize>(value: &T) -> Result<String, serde_json::Error> {
    let mut output = serde_json::to_string_pretty(value)?;
    output.push('\n');
    Ok(output)
}

pub fn render_trace(records: &[TraceRecord]) -> Result<String, serde_json::Error> {
    let mut output = String::new();
    for record in records {
        output.push_str(&serde_json::to_string(record)?);
        output.push('\n');
    }
    Ok(output)
}
