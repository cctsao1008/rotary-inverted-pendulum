# Repository Layout

This repository is organized around **control-system re-architecture boundaries**, not around a specific controller algorithm or MCU.

## Top-level ownership

```text
app/                    System integration and application-level orchestration
cmake/                  Cross-compilation toolchain support
control/                Platform-independent estimator, safety, mode, and control logic
drivers/                Reusable device-specific drivers independent of the MCU where practical
platform/api/           Shared application-to-platform hardware contract
platform/stm32f103/     Current STM32F103 implementation and linker configuration
platform/rp2350/        Planned RP2350 implementation boundary
tests/                  Host-side deterministic unit and contract tests
third_party/            External source dependencies
tools/                  Reproducible validation and runtime tooling
```

## Documentation ownership

```text
docs/architecture/      Architecture, contracts, system state, telemetry, and fault semantics
docs/commissioning/     Hardware/software bring-up and progressive actuator-authority procedures
docs/modeling/          Plant characterization, identification, and model-validation work
docs/control/           Controller strategy and control-specific engineering notes
docs/validation/        Validation workflow, replay, evidence model, and validation records
docs/hardware/          Hardware provenance, physical interfaces, and verified baselines
docs/development/       Repository structure, build, test, and developer workflow
docs/process/           Engineering-process records
docs/templates/         Reusable engineering record templates
```

`docs/testing/` is intentionally not a primary ownership category. Testing is a mechanism used by commissioning, modeling, control, and validation rather than a separate project domain.

## Dependency direction

The intended dependency direction is:

```text
                 control/
                    ^
                    |
                   app/
                    ^
                    |
              platform/api/
                    ^
                    |
          +---------+---------+
          |                   |
platform/stm32f103/      platform/rp2350/
```

`control/` must remain independent of MCU-specific code. `app/` may depend on the shared `platform/api/` contract, but it must not depend on STM32F103 or RP2350 implementation headers directly.

Platform directories implement the shared board contract and own MCU SDK, peripheral, linker, startup, PIO/DMA, and transport plumbing details. Board-specific pin assignments belong below the relevant platform rather than in the generic control core.

## Architectural interpretation

The repository should be read as a set of replaceable layers:

```text
Physical Plant
    ↓
Platform Contract
    ↓
Acquisition / Actuation
    ↓
Estimation
    ↓
Control Policy
    ↓
Mode / Authority / Safety
    ↓
Physical Output
```

A controller implementation is therefore one replaceable component inside the architecture. Adding LQR, LQI, pole placement, energy-based swing-up, or another method must not require controller-specific access to MCU peripherals or direct motor ownership.

The RP2350 directory is currently a planning and implementation boundary, not a claim of validated hardware support.
