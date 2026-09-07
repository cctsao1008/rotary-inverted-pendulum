use std::error::Error;
use std::fs;
use std::io;
use std::path::Path;

use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Parameter<T> {
    pub value: T,
    pub unit: String,
    pub source: String,
    pub applicability: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MechanicalPlantParameters {
    pub pendulum_mass_kg: Parameter<f32>,
    pub arm_length_m: Parameter<f32>,
    pub pendulum_com_length_m: Parameter<f32>,
    pub arm_inertia_kg_m2: Parameter<f32>,
    pub pendulum_inertia_kg_m2: Parameter<f32>,
    pub gravity_m_s2: Parameter<f32>,
    pub arm_viscous_damping_nm_per_rad_s: Parameter<f32>,
    pub pendulum_viscous_damping_nm_per_rad_s: Parameter<f32>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeasurementModelParameters {
    pub pendulum_upright_adc: Parameter<u16>,
    pub pendulum_radians_per_count: Parameter<f32>,
    pub pendulum_direction: Parameter<i8>,
    pub arm_encoder_counts_per_revolution: Parameter<f32>,
    pub arm_encoder_direction: Parameter<i8>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VirtualSensorParameters {
    pub pendulum_upright_adc: Parameter<u16>,
    pub pendulum_radians_per_count: Parameter<f32>,
    pub pendulum_direction: Parameter<i8>,
    pub pendulum_adc_modulus: Parameter<u16>,
    pub arm_encoder_counts_per_revolution: Parameter<f32>,
    pub arm_encoder_direction: Parameter<i8>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProductionActuatorModelParameters {
    pub torque_per_effective_command_nm: Parameter<f32>,
    pub command_deadzone: Parameter<f32>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FirmwareActuatorMappingParameters {
    pub positive_command_is_positive_drive: Parameter<bool>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VirtualPhysicalActuatorParameters {
    pub torque_per_duty_nm: Parameter<f32>,
    pub positive_drive_is_positive_arm_torque: Parameter<bool>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReferenceAssemblyParameters {
    pub schema: u32,
    pub assembly: String,
    pub plant: MechanicalPlantParameters,
    pub measurement_model: MeasurementModelParameters,
    pub virtual_sensor: VirtualSensorParameters,
    pub production_actuator_model: ProductionActuatorModelParameters,
    pub firmware_actuator_mapping: FirmwareActuatorMappingParameters,
    pub virtual_physical_actuator: VirtualPhysicalActuatorParameters,
}

impl ReferenceAssemblyParameters {
    pub fn load(path: &Path) -> Result<Self, Box<dyn Error>> {
        let source = fs::read_to_string(path)?;
        Self::parse(&source)
    }

    pub fn parse(source: &str) -> Result<Self, Box<dyn Error>> {
        let parameters: Self = serde_json::from_str(source)?;
        if parameters.schema != 1 {
            return Err(Box::new(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("unsupported parameter schema {}", parameters.schema),
            )));
        }
        if parameters.assembly.trim().is_empty() {
            return Err(Box::new(io::Error::new(
                io::ErrorKind::InvalidData,
                "parameter assembly must not be empty",
            )));
        }
        Ok(parameters)
    }

    pub fn production_model_configuration(&self) -> Value {
        json!({
            "assembly": self.assembly,
            "measurement_model": &self.measurement_model,
            "production_actuator_model": &self.production_actuator_model,
            "firmware_actuator_mapping": &self.firmware_actuator_mapping,
        })
    }

    pub fn virtual_physical_truth_configuration(&self) -> Value {
        json!({
            "assembly": self.assembly,
            "plant": &self.plant,
            "virtual_sensor": &self.virtual_sensor,
            "virtual_physical_actuator": &self.virtual_physical_actuator,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reference_assembly_registry_parses() {
        let source = include_str!("../../../parameters/reference-assembly.json");
        let parameters = ReferenceAssemblyParameters::parse(source).unwrap();
        assert_eq!(parameters.schema, 1);
        assert_eq!(parameters.assembly, "reference-assembly");
        assert_eq!(parameters.plant.pendulum_mass_kg.value, 0.04);
        assert_eq!(
            parameters
                .firmware_actuator_mapping
                .positive_command_is_positive_drive
                .value,
            true
        );
    }
}
