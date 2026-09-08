use std::{env, fs, path::PathBuf};

use rip_plant_model::{FurutaParameters, FurutaPlant, FurutaState};
use rip_robot_domain::TorqueNm;
use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize)]
struct Fixture {
    schema: u32,
    description: String,
    sample_period_us: u64,
    integration_step_us: u64,
    duration_us: u64,
    plant: FurutaParameterFixture,
    initial_state: [f32; 4],
    input_profile: Vec<TorqueEvent>,
}

#[derive(Clone, Copy, Debug, Deserialize)]
struct FurutaParameterFixture {
    pendulum_mass_kg: f32,
    arm_length_m: f32,
    pendulum_com_length_m: f32,
    arm_inertia_kg_m2: f32,
    pendulum_inertia_kg_m2: f32,
    gravity_m_s2: f32,
    arm_viscous_damping_nm_per_rad_s: f32,
    pendulum_viscous_damping_nm_per_rad_s: f32,
}

impl From<FurutaParameterFixture> for FurutaParameters {
    fn from(value: FurutaParameterFixture) -> Self {
        Self {
            pendulum_mass_kg: value.pendulum_mass_kg,
            arm_length_m: value.arm_length_m,
            pendulum_com_length_m: value.pendulum_com_length_m,
            arm_inertia_kg_m2: value.arm_inertia_kg_m2,
            pendulum_inertia_kg_m2: value.pendulum_inertia_kg_m2,
            gravity_m_s2: value.gravity_m_s2,
            arm_viscous_damping_nm_per_rad_s: value.arm_viscous_damping_nm_per_rad_s,
            pendulum_viscous_damping_nm_per_rad_s: value.pendulum_viscous_damping_nm_per_rad_s,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize)]
struct TorqueEvent {
    at_us: u64,
    arm_torque_nm: f32,
}

#[derive(Debug, Serialize)]
struct Trace {
    schema: u32,
    fixture_description: String,
    interpretation: &'static str,
    samples: Vec<TraceSample>,
}

#[derive(Debug, Serialize)]
struct TraceSample {
    time_us: u64,
    state: [f32; 4],
    arm_torque_nm: f32,
}

fn parse_args() -> Result<(PathBuf, PathBuf), String> {
    let mut fixture = None;
    let mut output = None;
    let mut args = env::args().skip(1);
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--fixture" => fixture = args.next().map(PathBuf::from),
            "--output" => output = args.next().map(PathBuf::from),
            other => return Err(format!("unknown argument: {other}")),
        }
    }
    match (fixture, output) {
        (Some(fixture), Some(output)) => Ok((fixture, output)),
        _ => Err("usage: plant_reference_trace --fixture <path> --output <path>".to_owned()),
    }
}

fn validate_fixture(fixture: &Fixture) -> Result<(), String> {
    if fixture.schema != 1 {
        return Err("unsupported fixture schema".to_owned());
    }
    if fixture.sample_period_us == 0
        || fixture.integration_step_us == 0
        || !fixture.duration_us.is_multiple_of(fixture.sample_period_us)
        || !fixture
            .sample_period_us
            .is_multiple_of(fixture.integration_step_us)
    {
        return Err("duration/sample/integration periods must form an integer grid".to_owned());
    }
    if fixture.input_profile.is_empty() || fixture.input_profile[0].at_us != 0 {
        return Err("input profile must begin at 0 us".to_owned());
    }
    let mut previous = None;
    for event in &fixture.input_profile {
        if event.at_us > fixture.duration_us
            || !event.at_us.is_multiple_of(fixture.sample_period_us)
            || !event.arm_torque_nm.is_finite()
            || previous.is_some_and(|value| event.at_us <= value)
        {
            return Err("input profile must be finite, ordered, and sample-aligned".to_owned());
        }
        previous = Some(event.at_us);
    }
    Ok(())
}

fn state_array(state: FurutaState) -> [f32; 4] {
    [state.theta, state.theta_dot, state.phi, state.phi_dot]
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (fixture_path, output_path) = parse_args().map_err(std::io::Error::other)?;
    let fixture: Fixture = serde_json::from_str(&fs::read_to_string(fixture_path)?)?;
    validate_fixture(&fixture).map_err(std::io::Error::other)?;

    let parameters: FurutaParameters = fixture.plant.into();
    let initial_state = FurutaState {
        theta: fixture.initial_state[0],
        theta_dot: fixture.initial_state[1],
        phi: fixture.initial_state[2],
        phi_dot: fixture.initial_state[3],
    };
    let mut plant = FurutaPlant::new(parameters, initial_state)
        .map_err(|error| std::io::Error::other(format!("invalid Furuta plant: {error:?}")))?;

    let mut samples = Vec::with_capacity(
        usize::try_from(fixture.duration_us / fixture.sample_period_us + 1).unwrap_or(0),
    );
    let mut profile_index = 0_usize;
    let mut arm_torque_nm = 0.0_f32;
    let step_seconds = fixture.integration_step_us as f32 * 1.0e-6;

    for at_us in (0..=fixture.duration_us).step_by(
        usize::try_from(fixture.integration_step_us)
            .map_err(|_| std::io::Error::other("integration step does not fit usize"))?,
    ) {
        while profile_index < fixture.input_profile.len()
            && fixture.input_profile[profile_index].at_us == at_us
        {
            arm_torque_nm = fixture.input_profile[profile_index].arm_torque_nm;
            profile_index += 1;
        }

        if at_us.is_multiple_of(fixture.sample_period_us) {
            samples.push(TraceSample {
                time_us: at_us,
                state: state_array(plant.state()),
                arm_torque_nm,
            });
        }
        if at_us != fixture.duration_us {
            plant
                .step_rk4(step_seconds, TorqueNm(arm_torque_nm))
                .map_err(|error| std::io::Error::other(format!("Furuta step failed: {error:?}")))?;
        }
    }

    if profile_index != fixture.input_profile.len() {
        return Err(std::io::Error::other("not all input profile events were consumed").into());
    }

    let trace = Trace {
        schema: fixture.schema,
        fixture_description: fixture.description,
        interpretation: "implementation consistency only; not Forest D1 specimen calibration",
        samples,
    };
    if let Some(parent) = output_path.parent() {
        fs::create_dir_all(parent)?;
    }
    fs::write(output_path, serde_json::to_string_pretty(&trace)? + "\n")?;
    Ok(())
}
