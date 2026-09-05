# 🌀 Rotary Inverted Pendulum

> **A ground-up re-architecture of a rotary inverted pendulum control system, from physical I/O to hybrid control.**

This project takes an existing rotary inverted pendulum plant and rebuilds the control system around explicit engineering boundaries: hardware access, sensing, actuation, estimation, control, mode management, telemetry, validation, and actuator authority.

The objective is **not** to reproduce the original vendor firmware and it is **not** organized around demonstrating a particular control algorithm. The plant is the physical starting point; the control architecture is the project.

> **A known-working plant is not the same thing as a known control system.**

The reference hardware is a legacy STM32F103-based commercial rotary inverted pendulum. Its original board, motor, encoder, angular sensor, and electrical interfaces are retained as hardware provenance and as a commissioning target, while the firmware architecture is rebuilt independently.

## 🧭 Project scope

The re-architecture separates concerns that were previously coupled in a monolithic embedded implementation:

```text
Physical Plant
    ↓
Platform / Hardware Contract
    ↓
Sensing & Actuation
    ↓
State Estimation
    ↓
Control Policy
    ↓
Mode / Transition Management
    ↓
Authority & Safety
    ↓
Physical Motor Output
```

The important architectural rule is that **computing a control command is not the same as having authority to actuate the motor**.

Controller implementations such as pole placement, LQR, LQI, or energy-based swing-up are interchangeable policies behind the control interfaces. They are not the organizing principle of the repository.

## 🏗️ Engineering structure

The project is developed along seven connected engineering concerns:

1. **Architecture** — define subsystem boundaries, state ownership, authority, timing, and contracts.
2. **Commissioning** — prove hardware, I/O, timing, safety paths, and runtime behavior progressively.
3. **Characterization** — measure sensor, actuator, friction, dead zone, delay, and free-response behavior.
4. **Modeling** — identify control-relevant plant dynamics from measured data and validate model limits.
5. **Estimation** — convert raw measurements into validated plant state with explicit freshness and validity.
6. **Control** — implement local stabilization, swing-up, capture/transition, and controller dispatch behind common interfaces.
7. **Validation** — compare expected and measured behavior and retain reproducible evidence.

These are engineering boundaries, not isolated phases. Evidence from later stages may force earlier assumptions or interfaces to be revised.

## 🧪 Progressive commissioning pipeline

```text
Hardware / I/O / Timing Baseline
        ↓
Sensor Calibration & Coordinate Convention
        ↓
Motor / Actuator Characterization
        ↓
Pendulum Free-Response Characterization
        ↓
Plant Identification & Model Validation
        ↓
State Estimation
        ↓
Upright Stabilization
        ↓
Energy-Based Swing-Up
        ↓
Capture / Transition
        ↓
End-to-End Validation
```

This is a **commissioning order**, not the runtime mode order. Upright stabilization is established first from a manually positioned or otherwise safely captured state so that feedback signs, stabilization region, actuator limits, and the authority path are understood before swing-up is allowed to hand control over to it.

The intended hybrid runtime is:

```text
Hanging / Low-Energy State
        ↓
Swing-Up
        ↓
Capture / Transition
        ↓
Upright Stabilization
```

See [Commissioning Philosophy](docs/commissioning/commissioning-philosophy.md).

## 🧠 Design principles

1. **Measurement before control**  
   Hardware mappings, polarity, calibration, timing, dead zones, and plant response are established before closed-loop control depends on them.

2. **Control computation is not actuator authority**  
   A controller may produce a valid request without being permitted to drive the motor.

3. **Fail closed**  
   Missing, stale, invalid, unqualified, or faulted state reduces actuator authority rather than preserving the last command.

4. **Progressive authority**  
   Bring-up proceeds from passive observation to bounded maintenance actuation, characterization, observe-only control, admission, explicit authority, and finally physical closed-loop operation.

5. **Platform independence where practical**  
   Estimation, control, safety, configuration, and state-machine logic remain independent of MCU-specific I/O.

6. **Unknown is an acceptable engineering state; assumption is not evidence**  
   Historical behavior, source-file existence, or plausible inference is not promoted into a current engineering fact without supporting evidence.

7. **Safety is architectural**  
   Admission, watchdog behavior, output qualification, motor ownership, and safe loss of authority are part of the control system design.

## 🧩 Architecture planes

### 👁️ State-estimation plane

```text
Physical Sensors
    -> Acquisition
    -> Calibration / Wrapping
    -> Filtering / Estimation
    -> State Validity / Freshness
    -> Validated Plant State
```

Sensor acquisition and estimator logic are intentionally separated so estimator implementations can change without rewriting board I/O.

### 🎛️ Control plane

The rotary inverted pendulum is treated as a **hybrid control system** with explicit mode and transition ownership.

![Hybrid control mode transitions](docs/architecture/control-mode-transitions.png)

The control state machine selects the active policy and owns transitions between swing-up, capture/transition, and upright stabilization. A selected controller computes an actuator request; it does not directly own the motor.

See [Controller Strategy](docs/control/controller-strategy.md) and [Control Architecture](docs/architecture/control_architecture.md).

### 🔐 Authority and actuation plane

```text
Operator Intent
    -> Admission / Mode Management
    -> Continuous Run Permit
    -> Closed-loop Command Qualification
    -> Motor Authority Arbiter
    -> Watchdog / Safe-Shutdown Boundary
    -> board_motor
    -> Rotary-Arm Motor
```

The **Control State Machine** owns controller selection and mode transitions. The **Motor Authority Arbiter** owns physical actuator command authority.

Motor authority is represented with `NONE / MAINTENANCE / CONTROL / FAULT` semantics.

Loss of valid authority converges toward a defined motor-safe condition rather than preserving the previous command. Coast, brake, and standby behavior belong to the actuator/hardware contract rather than to individual controllers.

## ⚙️ Platform boundary

The shared `platform/api/` contract separates application-visible hardware capabilities from MCU-specific implementations.

```text
platform/
├── api/            Shared board contract
├── stm32f103/      Current implementation
└── rp2350/         Planned implementation boundary
```

- **STM32F103** is the current reference implementation used to re-architect and commission the existing plant.
- **RP2350** is a separate platform target behind the same hardware contract. Its directory represents an implementation boundary and plan; it does not imply completed hardware validation.

The legacy Forest D1 / Forest S1 names are used only where needed to identify original vendor hardware or documentation provenance. They do not define the software architecture.

See [Platform Layer](platform/README.md) and [Repository Layout](docs/development/repository-layout.md).

## 🔩 Reference physical plant

Current hardware baseline:

- STM32F103C8T6 reference controller
- geared 12 V DC rotary-arm motor
- quadrature Hall encoder on the geared motor
- conductive-plastic angular-position sensor for the pendulum
- TB6612FNG H-bridge motor driver
- original commercial mechanical plant

Pin-level wiring, electrical notes, historical hardware details, and currently validated facts are kept separately in [Forest D1 2016 Hardware Baseline](docs/hardware/forest-d1-2016-baseline.md).

## 📂 Repository ownership

```text
app/                    System integration and application-level orchestration
control/                Platform-independent estimation, control, safety, and state-machine logic
drivers/                Reusable device drivers
platform/api/           Shared hardware contract
platform/stm32f103/     STM32F103 implementation
platform/rp2350/        RP2350 implementation boundary
tests/                  Host-side deterministic tests
tools/                  Validation and runtime tooling
docs/architecture/      Architecture and interface contracts
docs/commissioning/     Bring-up and progressive authority procedures
docs/modeling/          Plant characterization, identification, and model work
docs/control/           Controller strategy and control-specific engineering notes
docs/validation/        Reproducible evidence, replay, and validation records
docs/hardware/          Hardware provenance and physical baselines
```

The dependency rule is simple: **control logic must not know which MCU is underneath it.**

## 📚 Key documentation

- [Control Architecture](docs/architecture/control_architecture.md)
- [Control Contracts](docs/architecture/control_contracts.md)
- [System State Model](docs/architecture/system_state_model.md)
- [Telemetry Schema](docs/architecture/telemetry_schema.md)
- [Communication and Parameter Architecture](docs/architecture/communications.md)
- [Commissioning Philosophy](docs/commissioning/commissioning-philosophy.md)
- [Firmware Commissioning](docs/commissioning/firmware-commissioning.md)
- [Physical Commissioning Plan](docs/commissioning/physical-commissioning-plan.md)
- [Motor Commissioning and Characterization](docs/commissioning/motor-characterization.md)
- [Plant Characterization and Identification](docs/modeling/plant-identification.md)
- [Controller Strategy](docs/control/controller-strategy.md)
- [Validation Workflow](docs/validation/validation-workflow.md)
- [Record and Replay](docs/validation/record-replay.md)
- [Validation and Evidence Model](docs/validation/evidence-model.md)
- [Repository Layout](docs/development/repository-layout.md)
- [Build and Test](docs/development/build-and-test.md)
- [Forest D1 2016 Hardware Baseline](docs/hardware/forest-d1-2016-baseline.md)
