use std::error::Error;
use std::fmt::{Display, Formatter};
use std::fs;
use std::path::Path;

use serde::Deserialize;

#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Scenario {
    pub id: String,
    pub duration_us: u64,
    pub seed: u64,
    pub sensor_period_us: u64,
    pub runtime_period_us: u64,
    #[serde(default)]
    pub missed_runtime_at_us: Vec<u64>,
    #[serde(default)]
    pub rotary: Option<RotaryScenario>,
}

#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RotaryScenario {
    pub plant_step_us: u64,
    pub initial_theta_rad: f32,
    pub initial_theta_dot_rad_s: f32,
    pub initial_phi_rad: f32,
    pub initial_phi_dot_rad_s: f32,
}

#[derive(Debug)]
pub enum ScenarioError {
    Io(std::io::Error),
    Parse(toml::de::Error),
    EmptyId,
    ZeroSensorPeriod,
    ZeroRuntimePeriod,
    MissedRuntimeOutOfRange(u64),
    MissedRuntimeOffGrid(u64),
    MissedRuntimeNotStrictlyIncreasing,
    ZeroPlantStep,
    PlantStepNotAligned,
    RotaryInitialStateNonFinite,
    RotarySensorRuntimeCadenceMismatch,
}

impl Display for ScenarioError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(error) => write!(formatter, "failed to read scenario: {error}"),
            Self::Parse(error) => write!(formatter, "failed to parse scenario: {error}"),
            Self::EmptyId => write!(formatter, "scenario id must not be empty"),
            Self::ZeroSensorPeriod => write!(formatter, "sensor period must be greater than zero"),
            Self::ZeroRuntimePeriod => {
                write!(formatter, "runtime period must be greater than zero")
            }
            Self::MissedRuntimeOutOfRange(at) => write!(
                formatter,
                "missed runtime opportunity {at} us exceeds duration"
            ),
            Self::MissedRuntimeOffGrid(at) => write!(
                formatter,
                "missed runtime opportunity {at} us is not aligned to runtime period"
            ),
            Self::MissedRuntimeNotStrictlyIncreasing => write!(
                formatter,
                "missed runtime opportunities must be unique and strictly increasing"
            ),
            Self::ZeroPlantStep => write!(formatter, "rotary plant step must be greater than zero"),
            Self::PlantStepNotAligned => write!(
                formatter,
                "rotary plant step must divide duration, sensor period, and runtime period"
            ),
            Self::RotaryInitialStateNonFinite => {
                write!(formatter, "rotary initial state must be finite")
            }
            Self::RotarySensorRuntimeCadenceMismatch => write!(
                formatter,
                "rotary full semantic-path SITL currently requires sensor and runtime periods to match"
            ),
        }
    }
}

impl Error for ScenarioError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Io(error) => Some(error),
            Self::Parse(error) => Some(error),
            _ => None,
        }
    }
}

impl Scenario {
    pub fn load(path: &Path) -> Result<Self, ScenarioError> {
        let source = fs::read_to_string(path).map_err(ScenarioError::Io)?;
        let scenario: Self = toml::from_str(&source).map_err(ScenarioError::Parse)?;
        scenario.validate()?;
        Ok(scenario)
    }

    pub fn validate(&self) -> Result<(), ScenarioError> {
        if self.id.trim().is_empty() {
            return Err(ScenarioError::EmptyId);
        }
        if self.sensor_period_us == 0 {
            return Err(ScenarioError::ZeroSensorPeriod);
        }
        if self.runtime_period_us == 0 {
            return Err(ScenarioError::ZeroRuntimePeriod);
        }

        let mut previous = None;
        for &at in &self.missed_runtime_at_us {
            if at > self.duration_us {
                return Err(ScenarioError::MissedRuntimeOutOfRange(at));
            }
            if !at.is_multiple_of(self.runtime_period_us) {
                return Err(ScenarioError::MissedRuntimeOffGrid(at));
            }
            if previous.is_some_and(|value| value >= at) {
                return Err(ScenarioError::MissedRuntimeNotStrictlyIncreasing);
            }
            previous = Some(at);
        }

        if let Some(rotary) = self.rotary {
            if rotary.plant_step_us == 0 {
                return Err(ScenarioError::ZeroPlantStep);
            }
            if !self.duration_us.is_multiple_of(rotary.plant_step_us)
                || !self.sensor_period_us.is_multiple_of(rotary.plant_step_us)
                || !self.runtime_period_us.is_multiple_of(rotary.plant_step_us)
            {
                return Err(ScenarioError::PlantStepNotAligned);
            }
            if self.sensor_period_us != self.runtime_period_us {
                return Err(ScenarioError::RotarySensorRuntimeCadenceMismatch);
            }
            if ![
                rotary.initial_theta_rad,
                rotary.initial_theta_dot_rad_s,
                rotary.initial_phi_rad,
                rotary.initial_phi_dot_rad_s,
            ]
            .iter()
            .all(|value| value.is_finite())
            {
                return Err(ScenarioError::RotaryInitialStateNonFinite);
            }
        }

        Ok(())
    }

    pub fn runtime_is_missed(&self, at_us: u64) -> bool {
        self.missed_runtime_at_us.binary_search(&at_us).is_ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        Scenario {
            id: "test".to_string(),
            duration_us: 20_000,
            seed: 1,
            sensor_period_us: 5_000,
            runtime_period_us: 5_000,
            missed_runtime_at_us: vec![10_000],
            rotary: None,
        }
    }

    #[test]
    fn valid_scenario_accepts_aligned_missed_runtime_slot() {
        let scenario = scenario();
        assert!(scenario.validate().is_ok());
        assert!(scenario.runtime_is_missed(10_000));
        assert!(!scenario.runtime_is_missed(15_000));
    }

    #[test]
    fn missed_runtime_slots_must_be_strictly_increasing() {
        let mut scenario = scenario();
        scenario.missed_runtime_at_us = vec![10_000, 10_000];
        assert!(matches!(
            scenario.validate(),
            Err(ScenarioError::MissedRuntimeNotStrictlyIncreasing)
        ));
    }

    #[test]
    fn rotary_scenario_requires_integrator_alignment() {
        let mut scenario = scenario();
        scenario.rotary = Some(RotaryScenario {
            plant_step_us: 3_000,
            initial_theta_rad: 0.0,
            initial_theta_dot_rad_s: 0.0,
            initial_phi_rad: 0.0,
            initial_phi_dot_rad_s: 0.0,
        });
        assert!(matches!(
            scenario.validate(),
            Err(ScenarioError::PlantStepNotAligned)
        ));
    }
}
