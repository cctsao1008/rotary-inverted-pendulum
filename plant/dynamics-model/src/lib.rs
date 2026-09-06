#![no_std]
#![forbid(unsafe_code)]

use libm::{cosf, sinf};
use rip_robot_domain::TorqueNm;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FurutaParameters {
    pub pendulum_mass_kg: f32,
    pub arm_length_m: f32,
    pub pendulum_com_length_m: f32,
    pub arm_inertia_kg_m2: f32,
    pub pendulum_inertia_kg_m2: f32,
    pub gravity_m_s2: f32,
    pub arm_viscous_damping_nm_per_rad_s: f32,
    pub pendulum_viscous_damping_nm_per_rad_s: f32,
}

impl FurutaParameters {
    pub fn is_valid(self) -> bool {
        if !positive(self.pendulum_mass_kg)
            || !positive(self.arm_length_m)
            || !positive(self.pendulum_com_length_m)
            || !positive(self.arm_inertia_kg_m2)
            || !positive(self.pendulum_inertia_kg_m2)
            || !positive(self.gravity_m_s2)
            || !nonnegative(self.arm_viscous_damping_nm_per_rad_s)
            || !nonnegative(self.pendulum_viscous_damping_nm_per_rad_s)
        {
            return false;
        }

        let (a, b, c) = mass_matrix_terms(self);
        a * b > c * c
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FurutaState {
    /// Pendulum angle, radians, with zero at upright.
    pub theta: f32,
    pub theta_dot: f32,
    /// Rotary-arm angle, radians.
    pub phi: f32,
    pub phi_dot: f32,
}

impl FurutaState {
    pub fn is_finite(self) -> bool {
        self.theta.is_finite()
            && self.theta_dot.is_finite()
            && self.phi.is_finite()
            && self.phi_dot.is_finite()
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FurutaDerivative {
    pub theta_dot: f32,
    pub theta_ddot: f32,
    pub phi_dot: f32,
    pub phi_ddot: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DynamicsError {
    InvalidParameters,
    NonFiniteState,
    NonFiniteInput,
    InvalidStep,
    SingularMassMatrix,
}

/// Nonlinear torque-driven Furuta pendulum model.
///
/// The coupled equations follow the Euler-Lagrange model used by Abdullah et al.
/// (2021): the rotary arm and pendulum are coupled through `m Lp Lr cos(theta)`.
/// The input is generalized rotary-arm torque, matching `GeneralizedDemand` and
/// deliberately excluding PWM, voltage, GPIO, and H-bridge semantics.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FurutaPlant {
    parameters: FurutaParameters,
    state: FurutaState,
}

impl FurutaPlant {
    pub fn new(parameters: FurutaParameters, state: FurutaState) -> Result<Self, DynamicsError> {
        if !parameters.is_valid() {
            return Err(DynamicsError::InvalidParameters);
        }
        if !state.is_finite() {
            return Err(DynamicsError::NonFiniteState);
        }
        Ok(Self { parameters, state })
    }

    pub const fn parameters(self) -> FurutaParameters {
        self.parameters
    }

    pub const fn state(self) -> FurutaState {
        self.state
    }

    pub fn derivative(self, arm_torque: TorqueNm) -> Result<FurutaDerivative, DynamicsError> {
        derivative_for(self.parameters, self.state, arm_torque)
    }

    pub fn step_rk4(&mut self, dt_s: f32, arm_torque: TorqueNm) -> Result<(), DynamicsError> {
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(DynamicsError::InvalidStep);
        }
        if !arm_torque.0.is_finite() {
            return Err(DynamicsError::NonFiniteInput);
        }

        let p = self.parameters;
        let s0 = self.state;
        let k1 = derivative_for(p, s0, arm_torque)?;
        let k2 = derivative_for(p, advance(s0, k1, 0.5 * dt_s), arm_torque)?;
        let k3 = derivative_for(p, advance(s0, k2, 0.5 * dt_s), arm_torque)?;
        let k4 = derivative_for(p, advance(s0, k3, dt_s), arm_torque)?;

        let sixth = dt_s / 6.0;
        self.state = FurutaState {
            theta: s0.theta
                + sixth
                    * (k1.theta_dot + 2.0 * k2.theta_dot + 2.0 * k3.theta_dot + k4.theta_dot),
            theta_dot: s0.theta_dot
                + sixth
                    * (k1.theta_ddot
                        + 2.0 * k2.theta_ddot
                        + 2.0 * k3.theta_ddot
                        + k4.theta_ddot),
            phi: s0.phi
                + sixth * (k1.phi_dot + 2.0 * k2.phi_dot + 2.0 * k3.phi_dot + k4.phi_dot),
            phi_dot: s0.phi_dot
                + sixth
                    * (k1.phi_ddot + 2.0 * k2.phi_ddot + 2.0 * k3.phi_ddot + k4.phi_ddot),
        };

        if !self.state.is_finite() {
            return Err(DynamicsError::NonFiniteState);
        }
        Ok(())
    }
}

fn derivative_for(
    parameters: FurutaParameters,
    state: FurutaState,
    arm_torque: TorqueNm,
) -> Result<FurutaDerivative, DynamicsError> {
    if !parameters.is_valid() {
        return Err(DynamicsError::InvalidParameters);
    }
    if !state.is_finite() {
        return Err(DynamicsError::NonFiniteState);
    }
    if !arm_torque.0.is_finite() {
        return Err(DynamicsError::NonFiniteInput);
    }

    let (a, b, c) = mass_matrix_terms(parameters);
    let cos_theta = cosf(state.theta);
    let sin_theta = sinf(state.theta);
    let coupling = c * cos_theta;
    let determinant = a * b - coupling * coupling;
    if !determinant.is_finite() || determinant <= f32::EPSILON {
        return Err(DynamicsError::SingularMassMatrix);
    }

    let arm_rhs = arm_torque.0 - parameters.arm_viscous_damping_nm_per_rad_s * state.phi_dot
        + c * sin_theta * state.theta_dot * state.theta_dot;
    let pendulum_rhs = parameters.pendulum_mass_kg
        * parameters.gravity_m_s2
        * parameters.pendulum_com_length_m
        * sin_theta
        - parameters.pendulum_viscous_damping_nm_per_rad_s * state.theta_dot;

    let phi_ddot = (b * arm_rhs - coupling * pendulum_rhs) / determinant;
    let theta_ddot = (a * pendulum_rhs - coupling * arm_rhs) / determinant;
    if !phi_ddot.is_finite() || !theta_ddot.is_finite() {
        return Err(DynamicsError::NonFiniteState);
    }

    Ok(FurutaDerivative {
        theta_dot: state.theta_dot,
        theta_ddot,
        phi_dot: state.phi_dot,
        phi_ddot,
    })
}

fn mass_matrix_terms(parameters: FurutaParameters) -> (f32, f32, f32) {
    let m = parameters.pendulum_mass_kg;
    let lr = parameters.arm_length_m;
    let lp = parameters.pendulum_com_length_m;
    (
        parameters.arm_inertia_kg_m2 + m * lr * lr,
        parameters.pendulum_inertia_kg_m2 + m * lp * lp,
        m * lp * lr,
    )
}

fn advance(state: FurutaState, derivative: FurutaDerivative, dt_s: f32) -> FurutaState {
    FurutaState {
        theta: state.theta + derivative.theta_dot * dt_s,
        theta_dot: state.theta_dot + derivative.theta_ddot * dt_s,
        phi: state.phi + derivative.phi_dot * dt_s,
        phi_dot: state.phi_dot + derivative.phi_ddot * dt_s,
    }
}

fn positive(value: f32) -> bool {
    value.is_finite() && value > 0.0
}

fn nonnegative(value: f32) -> bool {
    value.is_finite() && value >= 0.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::PI;

    fn qnet_reference() -> FurutaParameters {
        FurutaParameters {
            pendulum_mass_kg: 0.04,
            arm_length_m: 0.085,
            pendulum_com_length_m: 0.129,
            arm_inertia_kg_m2: 0.000_005_7,
            pendulum_inertia_kg_m2: 0.0001,
            gravity_m_s2: 9.81,
            arm_viscous_damping_nm_per_rad_s: 0.00021,
            pendulum_viscous_damping_nm_per_rad_s: 0.0,
        }
    }

    #[test]
    fn qnet_reference_mass_matrix_is_positive_definite() {
        assert!(qnet_reference().is_valid());
    }

    #[test]
    fn upright_and_hanging_down_are_zero_torque_equilibria() {
        for theta in [0.0, PI] {
            let plant = FurutaPlant::new(
                qnet_reference(),
                FurutaState {
                    theta,
                    ..FurutaState::default()
                },
            )
            .unwrap();
            let derivative = plant.derivative(TorqueNm(0.0)).unwrap();
            assert!(derivative.theta_ddot.abs() < 1.0e-4);
            assert!(derivative.phi_ddot.abs() < 1.0e-4);
        }
    }

    #[test]
    fn upright_equilibrium_is_locally_unstable() {
        let plant = FurutaPlant::new(
            qnet_reference(),
            FurutaState {
                theta: 0.01,
                ..FurutaState::default()
            },
        )
        .unwrap();
        assert!(plant.derivative(TorqueNm(0.0)).unwrap().theta_ddot > 0.0);
    }

    #[test]
    fn hanging_down_equilibrium_is_locally_restoring() {
        let plant = FurutaPlant::new(
            qnet_reference(),
            FurutaState {
                theta: PI + 0.01,
                ..FurutaState::default()
            },
        )
        .unwrap();
        assert!(plant.derivative(TorqueNm(0.0)).unwrap().theta_ddot < 0.0);
    }

    #[test]
    fn rk4_step_remains_finite_for_small_step() {
        let mut plant = FurutaPlant::new(
            qnet_reference(),
            FurutaState {
                theta: PI - 0.05,
                ..FurutaState::default()
            },
        )
        .unwrap();
        for _ in 0..1_000 {
            plant.step_rk4(50.0e-6, TorqueNm(0.01)).unwrap();
        }
        assert!(plant.state().is_finite());
    }
}
