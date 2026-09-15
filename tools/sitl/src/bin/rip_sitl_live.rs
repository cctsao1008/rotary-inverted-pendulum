use std::env;
use std::error::Error;
use std::io::{self, Write};
use std::path::PathBuf;

use rip_sitl::{
    rotary::BalanceControllerProfile, ReferenceAssemblyParameters, RotarySitlSystem, Scenario,
    SitlSystem,
};
use rip_sitl::scheduler::EventKind;
use rip_sitl::virtual_time::VirtualTime;
use serde_json::{json, Map, Value};

const DEFAULT_PARAMETER_PATH: &str = "parameters/reference-assembly.json";

// Diagnostic mirrors of the current hybrid-control capture envelope. These are
// emitted only to explain an already-computed control decision; they do not
// participate in controller execution or actuator authority.
const ENERGY_TORQUE_GAIN: f64 = 0.175;
const MAX_ABS_TORQUE_NM: f64 = 0.05;
const SWING_KICK_TORQUE_NM: f64 = 0.01;
const SWING_KICK_BELOW_RATE_RAD_S: f64 = 0.05;
const CAPTURE_ENTER_ANGLE_RAD: f64 = 20.0_f64.to_radians();
const LEGACY_CAPTURE_ENTER_RATE_RAD_S: f64 = 3.0;
const BALANCE_ENTER_ANGLE_RAD: f64 = 8.0_f64.to_radians();
const BALANCE_ENTER_RATE_RAD_S: f64 = 1.0;

#[derive(Debug)]
struct Cli {
    scenario: PathBuf,
    parameters: PathBuf,
    balance_controller: BalanceControllerProfile,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("rip-sitl-live: {error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let cli = parse_cli().map_err(|message| io::Error::new(io::ErrorKind::InvalidInput, message))?;
    let scenario = Scenario::load(&cli.scenario)?;
    scenario.validate()?;
    let rotary = scenario
        .rotary
        .ok_or_else(|| io::Error::other("live SITL requires a [rotary] scenario"))?;
    if scenario.sensor_period_us != scenario.runtime_period_us {
        return Err(io::Error::other(
            "live SITL requires sensor_period_us == runtime_period_us",
        )
        .into());
    }
    if !scenario.missed_runtime_at_us.is_empty() {
        return Err(io::Error::other(
            "live SITL does not replay finite missed_runtime_at_us fault schedules",
        )
        .into());
    }

    let parameters = ReferenceAssemblyParameters::load(&cli.parameters)?;
    let mut system = RotarySitlSystem::new_with_balance_profile(
        &parameters,
        rotary,
        scenario.sensor_period_us,
        scenario.runtime_period_us,
        cli.balance_controller,
    )?;

    let stdout = io::stdout();
    let mut out = stdout.lock();
    writeln!(
        out,
        "{}",
        json!({
            "type": "meta",
            "schema": 1,
            "state_order": ["theta", "theta_dot", "phi", "phi_dot"],
            "source": {
                "kind": "rotary-sitl-live",
                "model_class": "source-backed reduced/equivalent QNET nonlinear model",
                "backend": "persistent production Rust SITL semantic path",
                "scenario": scenario.id,
                "balance_controller": cli.balance_controller.as_str(),
                "scope": "simulation evidence only; persistent incremental run; diagnostic branch-torque recomputation is explanatory only; no physical actuator authority"
            }
        })
    )?;
    out.flush()?;

    let mut previous = VirtualTime::ZERO;
    let mut at = VirtualTime::ZERO;
    let period_us = scenario.runtime_period_us;
    let mut first = true;

    loop {
        if !first {
            system.advance_physical_time(previous, at)?;
        }

        let mut merged = Map::new();
        if first {
            merge_payload(&mut merged, system.on_event(EventKind::ScenarioStart, at)?);
        }
        merge_payload(&mut merged, system.on_event(EventKind::SensorSample, at)?);
        merge_payload(
            &mut merged,
            system.on_event(EventKind::ObservationDelivery, at)?,
        );
        merge_payload(&mut merged, system.on_event(EventKind::ProductionRuntime, at)?);
        merge_payload(&mut merged, system.on_event(EventKind::ActuationCommit, at)?);

        let sample = viewer_sample(at, &merged, &parameters, cli.balance_controller)?;
        writeln!(out, "{}", json!({"type": "sample", "sample": sample}))?;
        out.flush()?;

        first = false;
        previous = at;
        at = VirtualTime(
            at.as_micros()
                .checked_add(period_us)
                .ok_or_else(|| io::Error::other("live SITL virtual time exhausted"))?,
        );
    }
}

fn merge_payload(target: &mut Map<String, Value>, payload: Option<Value>) {
    let Some(Value::Object(object)) = payload else {
        return;
    };
    for (key, value) in object {
        target.insert(key, value);
    }
}

fn viewer_sample(
    at: VirtualTime,
    merged: &Map<String, Value>,
    parameters: &ReferenceAssemblyParameters,
    balance_controller: BalanceControllerProfile,
) -> Result<Value, Box<dyn Error>> {
    let state = merged
        .get("true_plant_state")
        .and_then(Value::as_object)
        .ok_or_else(|| io::Error::other("live SITL tick missing true_plant_state"))?;

    let state_vector = json!([
        number(state, "theta_rad")?,
        number(state, "theta_dot_rad_s")?,
        number(state, "phi_rad")?,
        number(state, "phi_dot_rad_s")?
    ]);

    let requested = merged
        .get("generalized_demand")
        .and_then(Value::as_object)
        .and_then(|value| value.get("arm_torque_nm"))
        .and_then(Value::as_f64);
    let applied = merged
        .get("virtual_physical_actuator")
        .and_then(Value::as_object)
        .and_then(|value| value.get("applied_arm_torque_nm"))
        .and_then(Value::as_f64)
        .unwrap_or(0.0);
    let runtime_state = merged
        .get("supervisor")
        .and_then(Value::as_object)
        .and_then(|value| value.get("runtime_state"))
        .and_then(Value::as_str);
    let regime = merged
        .get("control_regime")
        .and_then(Value::as_str)
        .unwrap_or("unknown");

    let mut sample = Map::new();
    sample.insert("t_s".into(), json!(at.as_micros() as f64 * 1.0e-6));
    sample.insert("state".into(), state_vector);
    sample.insert("arm_torque_nm".into(), json!(applied));
    sample.insert("applied_arm_torque_nm".into(), json!(applied));
    if let Some(value) = requested {
        sample.insert("requested_arm_torque_nm".into(), json!(value));
    }
    if regime != "unknown" {
        sample.insert("control_regime".into(), json!(regime));
    }
    if let Some(value) = merged.get("authority").and_then(Value::as_str) {
        sample.insert("authority".into(), json!(value));
    }
    if let Some(value) = runtime_state {
        sample.insert("runtime_state".into(), json!(value));
    }
    if let Some(estimated) = merged.get("estimated_state").and_then(Value::as_object) {
        let theta = number(estimated, "theta_rad")?;
        let theta_dot = number(estimated, "theta_dot_rad_s")?;
        let phi = number(estimated, "phi_rad")?;
        let phi_dot = number(estimated, "phi_dot_rad_s")?;
        sample.insert(
            "estimated_state".into(),
            json!([theta, theta_dot, phi, phi_dot]),
        );
        sample.insert(
            "hybrid_diagnostics".into(),
            hybrid_diagnostics(
                theta,
                theta_dot,
                phi,
                phi_dot,
                regime,
                parameters,
                balance_controller,
            ),
        );
    }

    Ok(Value::Object(sample))
}

fn hybrid_diagnostics(
    theta: f64,
    theta_dot: f64,
    phi: f64,
    phi_dot: f64,
    regime: &str,
    parameters: &ReferenceAssemblyParameters,
    balance_controller: BalanceControllerProfile,
) -> Value {
    let m = parameters.plant.pendulum_mass_kg.value as f64;
    let l = parameters.plant.pendulum_com_length_m.value as f64;
    let j = parameters.plant.pendulum_inertia_kg_m2.value as f64;
    let g = parameters.plant.gravity_m_s2.value as f64;
    let target_energy = 2.0 * m * g * l;
    let energy = m * g * l * (1.0 + theta.cos()) + 0.5 * j * theta_dot * theta_dot;
    let energy_error = target_energy - energy;
    let mut swing_torque = ENERGY_TORQUE_GAIN * energy_error * theta_dot * (-theta.cos());
    if energy_error > 0.0
        && theta_dot.abs() <= SWING_KICK_BELOW_RATE_RAD_S
        && swing_torque.abs() < SWING_KICK_TORQUE_NM
    {
        swing_torque = if theta < 0.0 {
            -SWING_KICK_TORQUE_NM
        } else {
            SWING_KICK_TORQUE_NM
        };
    }
    swing_torque = swing_torque.clamp(-MAX_ABS_TORQUE_NM, MAX_ABS_TORQUE_NM);

    let gains = balance_controller.gains();
    let u_theta = -(gains[0] as f64 * theta);
    let u_theta_dot = -(gains[1] as f64 * theta_dot);
    let u_phi = -(gains[2] as f64 * phi);
    let u_phi_dot = -(gains[3] as f64 * phi_dot);
    let balance_torque = u_theta + u_theta_dot + u_phi + u_phi_dot;

    let capture_angle_eligible = theta.abs() <= CAPTURE_ENTER_ANGLE_RAD;
    let legacy_capture_rate_eligible = theta_dot.abs() <= LEGACY_CAPTURE_ENTER_RATE_RAD_S;
    let balance_eligible = theta.abs() <= BALANCE_ENTER_ANGLE_RAD
        && theta_dot.abs() <= BALANCE_ENTER_RATE_RAD_S;
    let state_feedback_active = matches!(regime, "capture" | "balance");

    // Keep the old blend-named fields for the existing console logger, but the
    // value is now deliberately binary: Capture/Balance are 100% state feedback.
    let capture_blend_weight = if state_feedback_active { 1.0 } else { 0.0 };
    let capture_blended_torque = if state_feedback_active {
        balance_torque
    } else {
        swing_torque
    };

    json!({
        "pendulum_energy_j": energy,
        "target_energy_j": target_energy,
        "energy_error_j": energy_error,
        "swing_torque_nm": swing_torque,
        "balance_torque_nm": balance_torque,
        "state_feedback_terms_nm": {
            "theta": u_theta,
            "theta_dot": u_theta_dot,
            "phi": u_phi,
            "phi_dot": u_phi_dot,
            "sum": balance_torque
        },
        "capture_blend_weight": capture_blend_weight,
        "capture_blended_torque_nm": capture_blended_torque,
        "capture_angle_eligible": capture_angle_eligible,
        "capture_rate_eligible": legacy_capture_rate_eligible,
        "legacy_capture_rate_eligible": legacy_capture_rate_eligible,
        "capture_eligible": capture_angle_eligible,
        "balance_eligible": balance_eligible,
        "active_controller": if state_feedback_active { "state_feedback" } else { "swing_up" }
    })
}

fn number(object: &Map<String, Value>, key: &str) -> Result<f64, Box<dyn Error>> {
    object
        .get(key)
        .and_then(Value::as_f64)
        .ok_or_else(|| io::Error::other(format!("missing numeric field {key}")))
        .map_err(Into::into)
}

fn parse_cli() -> Result<Cli, String> {
    let mut args = env::args().skip(1);
    let mut scenario = None;
    let mut parameters = None;
    let mut balance_controller = None;

    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--scenario" => {
                scenario = Some(PathBuf::from(
                    args.next()
                        .ok_or_else(|| "--scenario requires a path".to_string())?,
                ));
            }
            "--parameters" => {
                parameters = Some(PathBuf::from(
                    args.next()
                        .ok_or_else(|| "--parameters requires a value".to_string())?,
                ));
            }
            "--balance-controller" => {
                let value = args
                    .next()
                    .ok_or_else(|| "--balance-controller requires a value".to_string())?;
                balance_controller = Some(BalanceControllerProfile::parse(&value)?);
            }
            "--help" | "-h" => {
                println!(
                    "usage: rip-sitl-live --scenario <scenario.toml> \\\n                     [--parameters <reference-assembly.json>] \\\n                     [--balance-controller <qnet_lqr|pole_placement_c1|pole_placement_c2>]"
                );
                std::process::exit(0);
            }
            _ => return Err(format!("unknown argument: {argument}")),
        }
    }

    Ok(Cli {
        scenario: scenario.ok_or_else(|| "missing --scenario".to_string())?,
        parameters: parameters.unwrap_or_else(|| PathBuf::from(DEFAULT_PARAMETER_PATH)),
        balance_controller: balance_controller.unwrap_or_default(),
    })
}
