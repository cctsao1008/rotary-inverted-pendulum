# 🌀 Rotary Inverted Pendulum

Re-engineered embedded firmware and control software for a **rotary inverted pendulum control platform**, with emphasis on measurable state estimation, hybrid control, explicit actuator authority, and progressive commissioning.

> **A known-working plant is not the same thing as a known control system.**

The project rebuilds an existing working plant into a measurable, testable, safety-gated, and progressively commissionable embedded control platform.

The reference hardware is a legacy STM32F103-based commercial rotary inverted pendulum system. Hardware facts, sensor calibration, plant behavior, control computation, and physical actuator authority are treated as separate engineering properties.

Platform-independent control modules are developed and tested on the host before they are connected to real motor output.

## ✨ Why is this interesting?

A rotary inverted pendulum combines several control problems in one compact physical system:

- **Underactuated nonlinear dynamics** — only the rotary arm is actuated, while the pendulum is controlled indirectly through coupled motion.
- **Hybrid control behavior** — swing-up and upright stabilization are fundamentally different regimes and require explicit transition logic.
- **Real plant uncertainty** — friction, gearbox backlash, motor dead zone, sensing noise, delay, and actuator saturation materially affect controller performance.
- **Model-to-hardware validation** — the plant model, estimator, controller, and safety boundaries must survive real measurements.
- **Embedded real-time constraints** — sensing, estimation, control, telemetry, local UI, watchdogs, and actuator safety share one deterministic runtime.
- **Reproducible commissioning** — measured evidence and staged commissioning replace informal tuning as the primary development method.

## 🧭 Engineering and commissioning pipeline

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

This is a commissioning order rather than the final runtime mode order. Upright stabilization is established first from a manually positioned or otherwise safely captured state so that the stabilization basin, feedback signs, actuator limits, and safety path are known before swing-up hands control over to it.

The hybrid runtime is:

```text
Hanging / Low-Energy State
        ↓
Energy-Based Swing-Up
        ↓
Capture / Transition
        ↓
Upright Stabilization
```

Raw measurements, fitted parameters, model revisions, and real-versus-model comparison are retained as engineering evidence.

## 🧠 Design principles

1. **Measurement before control**  
   Hardware mappings, polarity, calibration, timing, dead zones, and plant response are established independently before closed-loop control depends on them.

2. **Control computation is not actuator authority**  
   A valid controller output does not automatically grant permission to drive the physical motor.

3. **Fail closed**  
   Missing, stale, invalid, unqualified, or faulted state reduces actuator authority rather than preserving the last command.

4. **Progressive commissioning**  
   The control stack proceeds from passive observation to bounded maintenance actuation, characterization, observe-only control, admission, authority, and physical closed-loop control.

5. **Control logic remains platform-independent where practical**  
   Estimation, safety, configuration, and controller logic remain separate from MCU-specific I/O.

6. **Unknown is an acceptable engineering state; assumption is not evidence**  
   Historical behavior, source-file existence, or plausible inference is not promoted into a current engineering fact without supporting evidence.

7. **Safety is part of the control architecture**  
   Admission, watchdog behavior, motor ownership, output qualification, and safe loss of authority are architectural concerns.

## 🏗️ System architecture

The system separates three questions:

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

The rotary inverted pendulum is treated as a **mode-dependent / hybrid control problem**.

![Hybrid control mode transitions](docs/architecture/control-mode-transitions.png)

The control state machine separates the large-angle swing-up problem from local upright stabilization. **SWING-UP** uses energy-based control to drive the pendulum toward the upright equilibrium. **TRANSITION** manages controller handover after the state enters the stabilization region. **STABILIZATION** applies a local stabilizing controller such as PD/PID, pole placement, LQR, or integral-augmented LQI.

The local controller baselines serve different engineering roles: PD/PID provides an intuitive commissioning baseline, pole placement provides a direct model-sanity check against the identified linearized plant, and LQR provides the primary state-feedback trade-off between regulation and control effort.

Transitions are state-dependent and reversible. Leaving the stabilization region can return control to swing-up or transition depending on the measured state.

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

The **Control State Machine** owns controller-selection and mode-transition authority. The **Motor Authority Arbiter** owns physical actuator command authority.

Motor authority is represented with `NONE / MAINTENANCE / CONTROL / FAULT` semantics.

### 🛟 Safe-state contract

Loss of valid control authority converges toward a defined motor-safe condition rather than preserving the previous actuator command.

Safe loss of authority may result from operator disable, invalid state, stale control output, runtime fault, authority conflict, watchdog expiration, or an independent emergency-stop path. Shutdown policy is enforced at the actuator-authority boundary so swing-up, transition logic, and stabilization controllers share the same safety semantics.

The exact coast / brake / standby behavior belongs to the actuator and hardware contract rather than the controller itself.

See [Control Architecture](docs/architecture/control_architecture.md).

## ⚙️ Physical platform

- **Reference controller:** STM32F103C8T6 at 72 MHz
- **Plant:** single-link rotary inverted pendulum with an actuated horizontal rotary arm driven by a geared DC motor
- **Pendulum sensing:** analog angular-position sensor
- **Rotary-arm sensing:** quadrature encoder; legacy hardware documentation specifies **1040 counts per output-shaft revolution**
- **Actuation:** TB6612FNG H-bridge driving the rotary-arm DC motor
- **Motor / gearing:** nominal 12 V DC motor with 1:20 gearbox
- **Firmware timing baseline:** 1 kHz scheduler / acquisition / control-pipeline tick

The reference implementation uses the original commercial hardware as a direct-fit development platform. Pin-level wiring, board-specific interfaces, electrical notes, and validation details are kept in [Forest D1 2016 Hardware Baseline](docs/hardware/forest-d1-2016-baseline.md).

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

Transport and middleware remain outside controller logic and do not implicitly confer actuator authority.

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
