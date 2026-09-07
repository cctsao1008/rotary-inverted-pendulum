# Repository Layout

The repository uses the same top-level control-system grammar as `single-wheel-platform`. Domain ownership is visible in the path; plant-specific and controller-specific leaves remain project-specific.

```text
Cargo.toml
rust-toolchain.toml
.cargo/

plant/
├── robot-domain/
├── dynamics-model/
├── measurement-model/
├── plant-observation/
└── actuator-model/

control/
├── state-feedback/
└── hybrid-control/

supervisor/
├── state-estimator/
├── runtime-state/
└── control-runtime/

firmware/
├── interfaces/
│   └── actuation/
├── sensors/
│   ├── pendulum-adc/
│   └── arm-encoder/
├── communications/
│   └── telemetry/
├── ui/
│   ├── status/
│   └── oled/
├── buses/
│   └── software-spi/
├── actuators/
│   └── tb6612/
├── adapters/
│   └── estimator-input/
├── boards/
│   └── forest-s1-d1/
├── assemblies/
│   └── forest-d1-reference/
├── recording/
│   ├── runtime-observation/
│   └── timing-evidence/
└── targets/
    └── stm32f103/

support/
└── dsp-kernel/

parameters/
├── README.md
└── reference-assembly.json

tools/
└── sitl/

docs/
├── architecture/
├── hardware/
└── development/
```

The production architecture remains exactly:

```text
plant/
control/
supervisor/
firmware/
```

`support/`, `parameters/`, `docs/`, and `tools/` are repository support areas rather than additional production architecture domains.

## Firmware ownership

`firmware/sensors` owns device acquisition and raw hardware evidence. Pendulum ADC acquisition packages ADC samples without applying Plant calibration; arm-encoder acquisition extends the wrapping hardware counter into the accumulated-count observation semantic.

`firmware/buses/software-spi` owns the write-only mode-0 clock/data transport used by the local display. SSD1315 reset, D/C, framebuffer, page addressing, and bounded flush semantics remain in `firmware/ui/oled`.

`firmware/boards/forest-s1-d1` owns board-level pin/peripheral wiring and clock constants. `firmware/assemblies/forest-d1-reference` maps populated devices to system roles and defines the reference local-UI/telemetry composition rates.

`firmware/recording/runtime-observation` publishes the canonical live runtime snapshot; `firmware/recording/timing-evidence` owns runtime timing characterization and its debugger-visible evidence.

`firmware/communications/telemetry` owns the fixed runtime telemetry packet and latest-snapshot/no-replay publication semantics. The STM32 target realizes that transport on USART1 at 115200 baud and services it outside the critical control-path timing interval.

`firmware/ui/status` owns local status-page and key-interaction semantics. `firmware/ui/oled` owns the SSD1315 128×64 representation, dirty-page framebuffer, and bounded background display service. The reference target reclaims PA15/PB3/PB4 by disabling JTAG while retaining SWD.

`firmware/adapters/estimator-input` converts Plant-owned raw observations into the Supervisor estimator input representation. `firmware/actuators/tb6612` owns proof-gated electrical realization semantics. `firmware/targets/stm32f103` composes the concrete MCU peripherals with these Firmware boundaries and the portable production domains.

The STM32 target keeps D2 hard-safe-off: PB1/TIM3_CH4 is configured for 20 kHz PWM with zero duty, PB13/PB12 are low, and no runtime `ActuationSink` owns those peripherals.

## Support ownership

`support/dsp-kernel` owns cross-domain numerical implementation primitives. Production ARM builds use the target DSP backend while host builds preserve deterministic semantic behavior for tests and SITL. It owns no Plant, Control, Supervisor, or Firmware semantics.

## Dependency direction

Portable dependency flow follows semantic ownership:

```text
Plant semantics
   ▲      ▲
   │      │
Control  Supervisor
   ▲        ▲
   └────┬───┘
        │
     Firmware
```

Control consumes Plant semantics. Supervisor composes Plant and Control behavior while owning estimation and authority. Firmware depends on the portable domains and owns physical realization. Production-domain crates may depend on narrowly scoped `support/` implementation primitives, but `support/` must not depend back on production-domain semantics.

`tools/sitl` is host-side verification infrastructure rather than a fifth architecture domain. It provides deterministic virtual time, scenario execution, machine-readable evidence, the virtual Furuta plant/sensor/actuator world, and reuse of the production semantic path through the Firmware TB6612 actuator adapter.
