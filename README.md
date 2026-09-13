# 🌀 Rotary Inverted Pendulum

A Rust-first, `no_std` control system for a Furuta-style rotary inverted pendulum.

> **An unstable plant with a very stable chain of authority.**  
> Swing hard, capture cleanly, balance quietly — and never let evidence become motor authority by accident.

This project is a ground-up re-architecture from physical I/O to hybrid control. It shares one architectural grammar with `single-wheel-platform`: **same architecture, different plant**.

> The pendulum is allowed to be unstable. The architecture is not.

## 🧭 Architecture

```text
                  CONTROL
                     ▲
                     │
                 SUPERVISOR
                  ▲      ▲
                  │      │
                PLANT    │
                  ▲      │
                  └──┬───┘
                     │
                 FIRMWARE
```

The four canonical domains define ownership and dependency, not runtime execution order:

- **Plant** — physical state, units, dynamics, measurement physics, actuator physics, and generalized physical input/output semantics.
- **Control** — desired closed-loop behavior: energy-based swing-up, capture/transition management, and upright state feedback.
- **Supervisor** — state estimation, runtime qualification, output safety, timing health, watchdogs, and physical-output authority.
- **Firmware** — sensor acquisition, electrical/protocol actuation semantics, board wiring, buses, communications, UI, recording, MCU peripherals, and target composition.

## 🧬 Typed semantic path

The canonical production path is deliberately typed end to end:

```text
RawObservation
    ↓
EstimatorMeasurement
    ↓
EstimatedState
    ↓
GeneralizedDemand
    ↓
BoundedActuatorCommand
    ↓
AuthorizedActuation
    ↓
Firmware ActuationSink
    ↓
Tb6612ElectricalActuation
    ↓
Tb6612FrameIo
    ↓
STM32F103 PWM / GPIO physical output
```

These are intentionally different semantic objects. A sensor sample is not a state estimate; a torque request is not a motor command; a bounded command is not authorization; authorization is not electrical output.

`RawObservation` is Plant-owned observation semantics populated by Firmware. `EstimatorMeasurement` is a Supervisor-owned estimator input representation that preserves Plant measurement semantics. State estimation belongs to Supervisor.

The STM32F103 executable materializes the non-actuating computation path through the authority decision:

```text
TIM1 / 1 kHz control opportunity
    ↓
PendulumAdcSensor + Arm Encoder / DWT timestamp
    ↓
RawObservation
    ↓
EstimatorInputAdapter
    ↓
EstimatorMeasurement
    ↓
BasicEstimator
    ↓
EstimatedState
    ↓
HybridController
    ├── EnergySwingUpController
    ├── CapturePolicy + blended transition
    └── LqrController
    ↓
GeneralizedDemand
    ↓
ArmActuatorModel
    ↓
BoundedActuatorCommand
    ↓
Command Safety
    ↓
RuntimeAuthority::evaluate
    ↓
AuthorityDecision
```

## ⏱️ Runtime and timing

TIM1 provides a fixed **1 kHz control opportunity**. The hardware update flag is not treated as a backlog queue: one observed update admits at most one fresh acquisition/control cycle, and missed periods are not replayed. DWT/`MonoTimer` remains the independent monotonic timing source.

`firmware/recording/runtime-observation` publishes the canonical live runtime snapshot. `firmware/recording/timing-evidence` records admitted-cycle count and elapsed time, last/min/max admission period, maximum period jitter relative to the nearest 1 kHz slot, maximum TIM1 admission phase, last/max end-to-end execution time, maximum execution cycles, inferred coalesced/missed ticks, deadline overruns, Supervisor late/timeout counts, ADC errors, and runtime errors. Critical-path timing ends before UART and OLED background service.

The STM32 target enables the independent watchdog (`IWDG`) with a 100 ms timeout. It is fed after each admitted opportunity is serviced, so a stalled firmware loop resets the MCU independently of TIM1 and DWT.

## 📟 Telemetry and local UI

The reference board exposes USART1 on PA9/PA10 at 115200 baud. `firmware/communications/telemetry` publishes a fixed-size runtime packet with sequence, state, control, safety, and timing evidence plus CRC16. Telemetry is latest-snapshot only: a busy transport drops that publication opportunity rather than replaying stale backlog. The reference assembly defaults telemetry off and publishes at 10 Hz when enabled.

The local display is an SSD1315 128×64 module using write-only software SPI on PB5/PB4 with PB3 reset and PA15 D/C. JTAG is disabled to reclaim PA15/PB3/PB4 while SWD remains on PA13/PA14. `firmware/ui/status` defines the `STATUS`, `SENSOR`, `SAFETY`, `CONTROL`, and `MAINTENANCE` pages plus M/X/+/-/USER key semantics. `firmware/ui/oled` owns the 1024-byte framebuffer, dirty-page rendering, and bounded background flush. Physical display service is limited to 8 data bytes per background slice.

## 🌀 Rotary plant semantics

The canonical estimated state is:

```text
x = [theta, theta_dot, phi, phi_dot]
```

where:

- `theta` is the wrapped pendulum angle.
- `phi` is the continuous / unwrapped rotary-arm angle.

Control produces a physical generalized demand:

```text
EstimatedState
    ↓
Controller
    ↓
GeneralizedDemand { arm_torque }
```

The controller does **not** emit normalized PWM, direction GPIO, or H-bridge commands. Plant actuator modeling converts physical arm-torque demand into `BoundedActuatorCommand`; Firmware owns TB6612 electrical realization.

`plant/actuator-model` provides both a compact static torque-span model and a speed-aware DC-motor model with armature resistance, torque constant, back-EMF, gearbox ratio/efficiency, supply-voltage authority, command deadzone, and explicit current limiting.

## 🎢 Hybrid control

The Control domain implements three regimes:

```text
SwingUp
   ↓ capture window
Capture
   ↓ settled upright window
Balance
```

`EnergySwingUpController` implements an energy-balance law and a bounded dead-start kick. `CapturePolicy` provides separate entry/exit thresholds, settle-cycle qualification, and hysteresis. During `Capture`, swing-up and balance torque demands are blended as the pendulum approaches the upright region. `Balance` uses full-state feedback.

Operational permission remains separate from control regime:

```text
RuntimeState                 ControlRegime
-----------                  -------------
Disabled                     SwingUp
Ready                        Capture
Active(ControlRegime)        Balance
Fault(reason)
```

`Ready` may execute non-actuating live-shadow computation. Closed-loop physical authorization still requires `RuntimeState::Active(...)` plus the remaining Supervisor authority conditions.

A controller can know what it wants to do and still have **zero authority to do it**.

## 🛡️ Actuation authority

Closed-loop physical output requires semantic promotion by Supervisor:

```text
BoundedActuatorCommand
        ↓
Command Safety
        ↓
RuntimeAuthority
        ↓
AuthorizedActuation
        ↓
Firmware ActuationSink
        ↓
actuator-specific frame
        ↓
frame I/O backend
        ↓
physical output
```

`supervisor/actuator-safety` applies an explicit output-safety policy before physical authorization. Magnitude and slew constraints are separate from controller logic. Simulation safety profiles are simulation configuration; they are **not** claims of Forest D1 physical-safe limits.

`AuthorizedActuation` has no public constructor. Maintenance output uses a distinct `MaintenanceActuation` proof type and a mutually exclusive Supervisor-owned maintenance authority mode.

`firmware/actuators/tb6612` separates the proof-gated sink, electrical semantics, and target/backend boundary:

```text
AuthorizedActuation / MaintenanceActuation
        ↓
Tb6612Output               Firmware ActuationSink
        ↓
Tb6612Mapper               authority-proof -> electrical semantics
        ↓
Tb6612ElectricalActuation  actuator-specific frame
        ↓
Tb6612FrameIo              target/backend boundary
```

Drive frames cannot be publicly constructed. `Tb6612Output` exposes no public arbitrary-frame application route; closed-loop and maintenance entry points require their respective Supervisor proof types. `safe_off()` remains the only unqualified output action.

`Tb6612PwmDirIo` performs break-before-make: PWM is forced to zero before direction pins change, then the requested duty is applied.

### 🔒 Current physical-output status

The STM32F103 target remains deliberately non-actuating for automatic control:

```text
RuntimeAuthority   = disarmed
RuntimeState       = Ready
D2 PWM duty        = 0
D2 direction pins  = low / low
runtime D2 sink    = unbound
```

PB1/TIM3_CH4 is configured for 20 kHz PWM with zero duty, while PB13/PB12 are driven low. No runtime `ActuationSink` owns the D2 motor peripherals, so control computation cannot reach the physical motor.

Green simulation is evidence. It is not a motor key.

## 🧪 Software-In-The-Loop

Host SITL lives under `tools/sitl/`; it is verification tooling, not a fifth production domain.

```text
Virtual Furuta Plant
        ↓
Synthetic sensor evidence
        ↓
RawObservation
        ↓
production measurement adapter
        ↓
production estimator
        ↓
production hybrid controller
        ↓
production actuator model
        ↓
production output safety
        ↓
production RuntimeAuthority
        ↓
AuthorizedActuation
        ↓
virtual TB6612 / actuator
        └────────────────────→ Virtual Furuta Plant
```

Simulator truth is deliberately converted back into device-like sensor evidence before entering the production semantic path. The production estimator never receives hidden simulator state as a shortcut.

SITL safety limits are labeled simulation-only. A useful virtual limit does not become a physical commissioning limit merely because CI is green.

## 🔬 Validation and evidence

The host validation stack intentionally uses models with different failure modes:

```text
source-backed reduced QNET model
        ├── Rust production model
        └── SciPy independent reference

geometry-derived full 3-D Furuta model
        ├── analytical oracle
        └── PyBullet rigid-body reviewer
```

Rust and SciPy test equation/implementation consistency. The full-3D oracle and PyBullet pressure-test the reduced model's omitted physics. Agreement is evidence; disagreement is something to explain, not something to average away.

The reduced QNET model remains useful for local upright analysis and source-controller reproduction. Full 3-D dynamics are used to challenge wider-angle and higher-rate assumptions. Model complexity does not grant universal authority; each model must earn a task-specific validity region.

Simulation evidence is **not** Forest D1 specimen calibration and never grants physical actuator authority.

## 📐 Reference-backed nominal model

The STM32F103 live-shadow controller uses a QNET rotary-inverted-pendulum reference model from Abdullah et al. (2021), not Forest D1 specimen calibration.

The published voltage-domain LQR gain vector and DC-motor constants are converted to the project state order, project angle signs, and rotary-arm torque output. The paper's positive pendulum-angle direction is opposite the project's positive `theta`, while the arm direction maps directly to `phi`.

`control/state-feedback` owns the canonical converted reference vector:

```text
project state order: [theta, theta_dot, phi, phi_dot]
nominal torque-feedback gains used by u = -Kx:
[-0.18355, -0.01585, -0.01120, -0.00745]
```

The live-shadow swing-up model uses the same reference family for pendulum mass, center-of-mass length, inertia, target energy, and energy-balance gain. These values define a reference-backed computation path and are not claims of specimen calibration.

## 📏 Parameter provenance

Physical and model parameters may use **reference-backed nominal values** when a project-specific value is not part of the implemented system definition.

A nominal parameter carries:

```text
value
unit
source
applicability
```

Reference-backed nominal values are useful inputs to modeling and controller development. They do not silently become specimen facts.

```text
literature / vendor evidence
        ↓
reference-backed nominal parameters
        ↓
model / simulation
        ↓
written physical prediction
        ↓
bounded physical experiment
        ↓
specimen-specific evidence
```

The evidence ladder may increase confidence one step at a time. It may not skip directly from a paper or simulator to physical authority.

## 🧱 Source ownership

```text
plant/
├── robot-domain/                   Physical state, units, generalized demand
├── dynamics-model/                Furuta dynamics and integration semantics
├── measurement-model/             ADC/encoder measurement physics
├── plant-observation/             Raw observation semantics
└── actuator-model/                Static and speed-aware actuator models

control/
├── state-feedback/                Controller contract and canonical reference gains
└── hybrid-control/                Energy swing-up, capture policy, balance transition

supervisor/
├── state-estimator/               Estimator input contract and state estimation
├── runtime-state/                 Runtime policy, timing, watchdog, authority
├── actuator-safety/               Output magnitude/slew qualification
└── control-runtime/               Deterministic portable control composition

firmware/
├── interfaces/actuation/          Authorized physical-output contract
├── sensors/
│   ├── pendulum-adc/              Raw pendulum ADC acquisition boundary
│   └── arm-encoder/               QEI counter accumulation and raw observation
├── communications/telemetry/      Latest-snapshot runtime telemetry protocol
├── ui/
│   ├── status/                    Status pages and key semantics
│   └── oled/                      SSD1315 framebuffer and bounded flush
├── buses/software-spi/            Write-only OLED transport
├── actuators/tb6612/              TB6612 mapper, proof-gated sink, frame-I/O boundary
├── adapters/estimator-input/      Raw observation -> estimator input promotion
├── boards/forest-s1-d1/           Board pin/peripheral wiring
├── assemblies/forest-d1-reference/ Populated-device roles and local-service rates
├── recording/
│   ├── runtime-observation/       Canonical runtime snapshot
│   └── timing-evidence/           Runtime timing characterization
└── targets/stm32f103/             MCU composition, IWDG, telemetry/UI, hard-safe-off D2

support/
└── dsp-kernel/                    Cross-domain numerical implementation primitives

tools/
├── model/                         Analytical / rigid-body validation and provenance checks
├── sitl/                          Deterministic production-semantic host simulation
└── telemetry/                     Host-side runtime evidence tooling
```

## 📚 Documentation principle

> **README explains the system. Issues explain the journey. Code proves the current state.**

README and durable documentation explain the Plant / Control / Supervisor / Firmware architecture, typed interfaces, safety and authority boundaries, validation interpretation, reference usage, and evidence gates. GitHub Issues preserve bring-up, experiments, calibration work, temporary constraints, design alternatives, and validation journeys. Code, configuration, target composition, and tests remain the authoritative evidence of executable behavior.

Causal design rationale may remain in durable documentation when it explains why an architectural boundary exists; status logs, roadmaps, checklists, and unresolved work belong in Issues.

## 🗺️ Key documentation

- [Control Architecture](docs/architecture/control_architecture.md)
- [Runtime Supervisor](docs/architecture/runtime_supervisor.md)
- [Repository Layout](docs/development/repository-layout.md)
- [Build and Test](docs/development/build-and-test.md)
- [Forest D1 2016 Hardware Baseline](docs/hardware/forest-d1-2016-baseline.md)
