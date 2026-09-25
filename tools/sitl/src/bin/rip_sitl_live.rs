use std::env;
use std::error::Error;
use std::io::{self, Write};
use std::path::PathBuf;

use rip_sitl::scheduler::EventKind;
use rip_sitl::virtual_time::VirtualTime;
use rip_sitl::{
    rotary::BalanceControllerProfile, ReferenceAssemblyParameters, RotarySitlSystem, Scenario,
    SitlSystem,
};
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
    let cli =
        parse_cli().map_err(|message| io::Error::new(io::ErrorKind::InvalidInput, message))?;
    let scenario = Scenario::load(&cli.scenario)?;
    scenario.validate()?;
    let rotary = scenario
        .rotary
        .ok_or_else(|| io::Error::other("live SITL requires a [rotary] scenario"))?;
    if scenario.sensor_period_us != scenario.runtime_period_us {
        return Err(
            io::Error::other("live SITL requires sensor_period_us == runtime_period_us").into(),
        );
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
                "scope": "simulation evidence only; persistent incremental run; Balance may use moving-reference spin-down and nearest-branch recenter; diagnostic branch-torque recomputation is explanatory only; no physical actuator authority"
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
        merge_payload(
            &mut merged,
            system.on_event(EventKind::ProductionRuntime, at)?,
        );
        merge_payload(
            &mut merged,
            system.on_event(EventKind::ActuationCommit, at)?,
        );

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
    let balance_reference_object = merged.get("balance_reference").and_then(Value::as_object);
    let balance_reference = balance_reference_object
        .map(|reference| {
            Ok::<(f64, f64), Box<dyn Error>>((
                number(reference, "phi_ref_rad")?,
                number(reference, "phi_dot_ref_rad_s")?,
            ))
        })
        .transpose()?;
    let balance_reference_phase = balance_reference_object
        .and_then(|reference| reference.get("phase"))
        .and_then(Value::as_str);
    let balance_recenter_target = balance_reference_object
        .and_then(|reference| reference.get("recenter_target_phi_rad"))
        .and_then(Value::as_f64);

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
    if let Some(reference) = merged.get("balance_reference") {
        if !reference.is_null() {
            sample.insert("balance_reference".into(), reference.clone());
        }
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
                balance_reference,
                balance_reference_phase,
                balance_recenter_target,
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
    balance_reference: Option<(f64, f64)>,
    balance_reference_phase: Option<&str>,
    balance_recenter_target: Option<f64>,
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
    let capture_torque = u_theta + u_theta_dot;

    let global_u_phi = -(gains[2] as f64 * phi);
    let global_u_phi_dot = -(gains[3] as f64 * phi_dot);
    let global_zero_balance_torque = capture_torque + global_u_phi + global_u_phi_dot;

    let (phi_ref, phi_dot_ref) = balance_reference.unwrap_or((0.0, 0.0));
    let phi_error = phi - phi_ref;
    let phi_dot_error = phi_dot - phi_dot_ref;
    let tracking_u_phi = -(gains[2] as f64 * phi_error);
    let tracking_u_phi_dot = -(gains[3] as f64 * phi_dot_error);
    let tracking_balance_torque = capture_torque + tracking_u_phi + tracking_u_phi_dot;

    let capture_angle_eligible = theta.abs() <= CAPTURE_ENTER_ANGLE_RAD;
    let legacy_capture_rate_eligible = theta_dot.abs() <= LEGACY_CAPTURE_ENTER_RATE_RAD_S;
    let balance_eligible =
        theta.abs() <= BALANCE_ENTER_ANGLE_RAD && theta_dot.abs() <= BALANCE_ENTER_RATE_RAD_S;
    let state_feedback_active = matches!(regime, "capture" | "balance");
    let capture_projection_active = regime == "capture";

    let balance_controller_label = match balance_reference_phase {
        Some("spin_down") => "balance_reference_spin_down",
        Some("recenter") => "balance_reference_recenter",
        Some("hold") => "balance_reference_hold",
        _ => "balance_reference_tracking",
    };
    let (active_u_phi, active_u_phi_dot, active_feedback_torque, active_controller) = match regime {
        "capture" => (0.0, 0.0, capture_torque, "capture_pendulum_subspace"),
        "balance" => (
            tracking_u_phi,
            tracking_u_phi_dot,
            tracking_balance_torque,
            balance_controller_label,
        ),
        _ => (
            global_u_phi,
            global_u_phi_dot,
            global_zero_balance_torque,
            "swing_up",
        ),
    };

    // Keep the old blend-named fields for the existing console logger. Their
    // value is now a selected-controller mirror: SwingUp uses EBC, Capture uses
    // the theta/theta_dot projection, and Balance uses moving-reference tracking.
    let capture_blend_weight = if state_feedback_active { 1.0 } else { 0.0 };
    let capture_blended_torque = match regime {
        "capture" => capture_torque,
        "balance" => tracking_balance_torque,
        _ => swing_torque,
    };

    json!({
        "pendulum_energy_j": energy,
        "target_energy_j": target_energy,
        "energy_error_j": energy_error,
        "swing_torque_nm": swing_torque,
        "capture_torque_nm": capture_torque,
        "balance_torque_nm": tracking_balance_torque,
        "global_zero_shadow_torque_nm": global_zero_balance_torque,
        "state_feedback_terms_nm": {
            "theta": u_theta,
            "theta_dot": u_theta_dot,
            "phi": active_u_phi,
            "phi_dot": active_u_phi_dot,
            "sum": active_feedback_torque
        },
        "balance_tracking_terms_nm": {
            "theta": u_theta,
            "theta_dot": u_theta_dot,
            "phi": tracking_u_phi,
            "phi_dot": tracking_u_phi_dot,
            "sum": tracking_balance_torque
        },
        "full_state_shadow_terms_nm": {
            "theta": u_theta,
            "theta_dot": u_theta_dot,
            "phi": global_u_phi,
            "phi_dot": global_u_phi_dot,
            "sum": global_zero_balance_torque
        },
        "balance_reference": balance_reference.map(|_| json!({
            "phi_ref_rad": phi_ref,
            "phi_dot_ref_rad_s": phi_dot_ref,
            "phi_error_rad": phi_error,
            "phi_dot_error_rad_s": phi_dot_error,
            "phase": balance_reference_phase,
            "recenter_target_phi_rad": balance_recenter_target
        })),
        "capture_projection_active": capture_projection_active,
        "capture_blend_weight": capture_blend_weight,
        "capture_blended_torque_nm": capture_blended_torque,
        "capture_angle_eligible": capture_angle_eligible,
        "capture_rate_eligible": legacy_capture_rate_eligible,
        "legacy_capture_rate_eligible": legacy_capture_rate_eligible,
        "capture_eligible": capture_angle_eligible,
        "balance_eligible": balance_eligible,
        "active_controller": active_controller
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
                parameters =
                    Some(PathBuf::from(args.next().ok_or_else(|| {
                        "--parameters requires a value".to_string()
                    })?));
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
