# Controller Strategy

Controller algorithms are replaceable implementations behind the control architecture. The project is not organized around reproducing legacy PD/PID behavior or around proving that one named controller is universally best.

## Upright stabilization

Local upright stabilization should use the measured/identified plant and the common state/actuator contracts.

Current controller roles are:

- **Pole placement** — useful as a direct state-space structural check because desired closed-loop poles are explicit and easy to compare with the identified linearized plant.
- **LQR** — primary local state-feedback baseline for trading state regulation against control effort through explicit weighting.
- **LQI** — optional integral augmentation when measured steady-state behavior demonstrates a need for integral action and the additional state is justified.

The selected controller must remain behind the same estimator, safety, mode, and actuator-authority boundaries.

## Swing-up

Swing-up is a separate large-angle control regime. The current architectural direction is energy-based swing-up with bounded actuator output.

Swing-up is commissioned only after upright stabilization and its admission/authority path are independently validated.

## Capture / transition

Capture is not treated as an accidental threshold inside either swing-up or stabilization. Transition logic owns handover criteria, hysteresis, state validity, and recovery behavior between large-angle and local-control regimes.

## Comparison criteria

Controller evaluation should use measured system-level criteria rather than only simulation response:

- stabilization/capture region;
- settling behavior;
- pendulum-angle error;
- rotary-arm excursion;
- control effort;
- actuator saturation duration;
- sensitivity to initial condition;
- disturbance recovery;
- estimator dependence;
- transition behavior;
- fault and authority interaction.

## Estimation relationship

Begin with the simplest estimator that meets measured requirements. Add estimator complexity only when sensor noise, bias, derivative quality, sample timing, or state reconstruction accuracy demonstrates a need for it.

Controller and estimator choices may evolve independently as long as they preserve the shared state and actuator contracts.
