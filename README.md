# 🌀 Rotary Inverted Pendulum

Re-engineered embedded firmware and control software for a **rotary inverted pendulum control platform**, with emphasis on measurable state estimation, hybrid control, explicit actuator authority, and progressive commissioning.

> **A known-working plant is not the same thing as a known control system.**

This project is not primarily about proving that a microcontroller can balance an inverted pendulum. It rebuilds an existing working plant into a **measurable, testable, safety-gated, and progressively commissionable embedded control platform**.

The current reference hardware is a legacy STM32F103-based commercial rotary inverted pendulum system. Hardware facts, sensor calibration, plant behavior, control computation, and physical actuator authority are treated as separate engineering properties and are validated independently.

Platform-independent control modules are developed and tested on the host before they are connected to real motor output.

## ✨ Why is this interesting?

A rotary inverted pendulum is a compact but demanding control problem because it combines several engineering challenges in one physical system:

- **Underactuated nonlinear dynamics** — only the rotary arm is actuated, while the pendulum must be controlled indirectly through coupled motion.
- **Hybrid control behavior** — swing-up and upright stabilization are fundamentally different regimes and require explicit transition logic.
- **Real plant uncertainty** — friction, gearbox backlash, motor dead zone, sensing noise, delay, and actuator saturation materially affect controller performance.
- **Model-to-hardware validation** — a controller that works in simulation is not sufficient; the plant model, estimator, controller, and safety boundaries must all survive real measurements.
- **Embedded real-time constraints** — sensing, estimation, control, telemetry, local UI, watchdogs, and actuator safety must coexist within a deterministic runtime.
- **Reproducible commissioning** — the project emphasizes measured evidence, system identification, and staged validation rather than tuning until the pendulum happens to balance.

## 🧭 Engineering and commissioning pipeline

The development sequence is intentionally evidence-driven:

```text
Hardware / I/O / Timing Baseline
        ↓
Sensor Calibration & Coordinate Convention
        ↓
Motor / Actuator Characterization
        ↓
Pendulum Free-Response Characterization
        ↓
System Identification & Model Validation
        ↓
State Estimation & Controller Baselines
        ↓
Upright Stabilization
        ↓
Energy-Based Swing-Up
        ↓
Capture / Transition / End-to-End Validation
```

This is a **commissioning order**, not the final runtime mode order. Upright stabilization is proven first from a manually positioned or otherwise safely captured state so that the stabilization basin, feedback signs, actuator limits, and safety path are known before swing-up is allowed to hand control over to it.

The final hybrid runtime remains conceptually:

```text
Hanging / Low-Energy State
        ↓
Energy-Based Swing-Up
        ↓
Capture / Transition
        ↓
Upright Stabilization
```

Raw measurements, fitted parameters, model revisions, and real-versus-model validation are treated as engineering evidence rather than informal tuning notes.

Commissioning gates and their acceptance evidence are tracked in [Issue #48 — hardware-to-hybrid-control validation gates](https://github.com/cctsao1008/rotary-inverted-pendulum/issues/48).

## 🧠 Design principles

1. **Measurement before control**  
   Hardware mappings, polarity, calibration, timing, dead zones, and plant response are established independently before closed-loop control is allowed to depend on them.

2. **Control computation is not actuator authority**  
   A valid controller output does not automatically grant permission to drive the physical motor.

3. **Fail closed**  
   Missing, stale, invalid, unqualified, or faulted state must reduce authority rather than preserve the last actuator command.

4. **Progressive commissioning**  
   The new control stack proceeds from passive observation to bounded maintenance actuation, characterization, observe-only control, admission, authority, and only then physical closed-loop control.

5. **Control logic remains platform-independent where practical**  
   Estimation, safety, configuration, and controller logic are kept separate from MCU-specific I/O so deterministic behavior can be tested on the host.

6. **Unknown is an acceptable engineering state; assumption is not evidence**  
   Historical behavior, source-file existence, or a plausible inference is not promoted into a current validation claim without supporting evidence.

7. **Safety is part of the control architecture**  
   Admission, watchdog behavior, motor ownership, output qualification, and safe loss of authority are architectural concerns rather than afterthoughts around the controller.

## 🏗️ System architecture

The system separates three questions that are often conflated in small embedded-control projects:

```text
STATE ESTIMATION
"What is the plant doing?"
        |
        v
CONTROL REGIME / COMPUTATION
"What control strategy applies, and what command should it produce?"
        |
        v
AUTHORITY & SAFETY
"May that command physically reach the motor?"
```

### 👁️ State-estimation plane

```text
Physical Sensors
    -> Signal Acquisition
    -> Calibration / Wrapping
    -> Filtering / Estimation
    -> State Validity / Safety
    -> validated plant state
```

The estimator boundary supports interchangeable estimation strategies while keeping sensor acquisition, validation, and controller logic separated.

### 🎛️ Control plane

The rotary inverted pendulum is treated as a **mode-dependent / hybrid control problem**, not as one controller expected to work over the entire state space.

![Hybrid control mode transitions](docs/architecture/control-mode-transitions.png)

The control state machine separates the large-angle swing-up problem from local upright stabilization. **SWING-UP** uses energy-based control to drive the pendulum toward the upright equilibrium. **TRANSITION** manages controller handover after the state enters the stabilization region. **STABILIZATION** applies a local stabilizing controller such as PD/PID, pole placement, LQR, or integral-augmented LQI.

The local controller baselines serve different engineering roles: PD/PID provides an intuitive commissioning baseline, pole placement provides a direct model-sanity check against the identified linearized plant, and LQR provides the primary state-feedback trade-off between regulation and control effort. The practical stabilization basin is measured rather than inferred from a nominal upright simulation, and later becomes evidence for capture/transition design.

Transitions are state-dependent and reversible. Leaving the stabilization region returns control to swing-up, while a moderate loss of stabilization can return the system to the transition state. A detected fall bypasses transition and returns directly to swing-up.

Controller availability and controller commissioning are intentionally separate concepts. The presence of a controller or estimator implementation does **not** mean that path is runtime-validated or physically commissioned.

Controllers operate behind a common state, safety, and actuator interface. A control mode can select a policy and compute an actuator request without gaining direct access to the motor.

### 🔐 Authority and actuation plane

```text
Operator Intent
    -> Admission / Mode Management
    -> Continuous Run Permit
    -> Closed-loop Command Qualification
    -> Motor Authority Arbiter
    -> Watchdog / Safe-Shutdown Boundary
    -> board_motor
    -> rotary-arm motor
```

The **Control State Machine** owns controller-selection and mode-transition authority. The **Motor Authority Arbiter** owns physical actuator command authority. These responsibilities must not be conflated.

Motor authority is treated as an explicitly owned resource with `NONE / MAINTENANCE / CONTROL / FAULT` semantics.

### 🛟 Safe-state contract

Loss of valid control authority must converge toward a defined motor-safe condition rather than preserving the previous actuator command.

Safe loss of authority may result from operator disable, invalid state, stale control output, runtime fault, authority conflict, watchdog expiration, or an independent emergency-stop path. Shutdown policy is enforced at the actuator-authority boundary so swing-up, transition logic, and all stabilization controllers share the same safety semantics.

The exact coast / brake / standby behavior belongs to the actuator and hardware contract rather than the controller itself.

See [Control Architecture](docs/architecture/control_architecture.md).

## ⚙️ Physical platform

Only hardware properties that directly shape the control architecture are summarized here:

- **Reference controller:** STM32F103C8T6 at 72 MHz.
- **Plant:** single-link rotary inverted pendulum with an actuated horizontal rotary arm driven by a geared DC motor.
- **Pendulum sensing:** analog angular-position sensor.
- **Rotary-arm sensing:** quadrature encoder; the legacy hardware documentation specifies **1040 counts per output-shaft revolution**.
- **Actuation:** TB6612FNG H-bridge driving the rotary-arm DC motor.
- **Motor / gearing:** nominal 12 V DC motor with 1:20 gearbox on the reference mechanism.
- **Firmware timing baseline:** 1 kHz scheduler / acquisition / control-pipeline tick. Final controller execution rates are selected from measured plant dynamics, actuator response, estimator behavior, and timing margin rather than assumed from MCU capability.

The current reference implementation uses the original commercial hardware as a direct-fit development platform. Pin-level wiring, board-specific interfaces, electrical notes, and validation details are kept in [Forest D1 2016 Hardware Baseline](docs/hardware/forest-d1-2016-baseline.md).

## 🔌 Communication and ROS 2 integration

Communication remains outside the control core.

```text
                 Control / Estimation Core
                          |
                 native application state
                          |
              +-----------+-----------+
              |                       |
              v                       v
      Maintenance transport     ROS 2 / DDS integration
```

Transport and middleware must remain outside controller logic and must not implicitly confer actuator authority.

See [Communication and Parameter Architecture](docs/architecture/communications.md).

## 📚 Documentation

- [Control Architecture](docs/architecture/control_architecture.md)
- [Telemetry Schema](docs/architecture/telemetry_schema.md)
- [Communication and Parameter Architecture](docs/architecture/communications.md)
- [Forest D1 2016 Hardware Baseline](docs/hardware/forest-d1-2016-baseline.md)
- [Commissioning Philosophy](docs/commissioning/commissioning-philosophy.md)
- [Firmware Commissioning](docs/commissioning/firmware-commissioning.md)
- [Motor Commissioning and Characterization](docs/commissioning/motor-characterization.md)
- [Repository Layout](docs/development/repository-layout.md)
- [Build and Test](docs/development/build-and-test.md)
- [Validation and Evidence Model](docs/validation/evidence-model.md)
