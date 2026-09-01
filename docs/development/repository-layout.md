# Repository Layout

This document describes the top-level repository structure and the intended ownership boundaries.

```text
app/                    Platform-neutral firmware application and system-level adapters
cmake/                  Cross-compilation toolchain support
control/                Platform-independent estimator, safety, and control logic
drivers/                 Device-specific drivers that are independent of the MCU where practical
docs/architecture/      Source-coupled architecture and contracts
docs/commissioning/     Firmware and plant commissioning procedures
docs/development/       Repository structure and build instructions
docs/hardware/          Hardware baselines, plant interfaces, and validation notes
docs/process/           Engineering process records
docs/validation/        Evidence and capability-validation model
docs/templates/         Reusable engineering templates
platform/api/           Shared application-to-platform hardware contract
platform/stm32f103/     STM32F103 implementation and linker configuration
platform/rp2350/        RP2350 platform planning and future implementation
tests/                  Host-side unit tests
third_party/            External source dependencies
tools/                  Reproducible validation and runtime tooling
```

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
   +-------------------+
   |                   |
platform/stm32f103/  platform/rp2350/
```

`control/` must remain independent of MCU-specific code. `app/` may depend on the shared `platform/api/` contract, but it must not depend on STM32F103 or RP2350 implementation headers directly.

Platform directories implement the shared board contract and own MCU SDK, peripheral, linker, and target-build details. Board-specific pin assignments belong below the relevant MCU platform rather than in the generic control core.

The RP2350 directory is currently a planning boundary, not a claim of validated hardware support.
