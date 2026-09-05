# Repository Layout

The repository is organized around current control-system boundaries rather than controller names or MCU-specific control logic.

## Top-level ownership

```text
app/                    Application integration and system orchestration
cmake/                  Cross-compilation support
control/                Platform-independent estimator, safety, mode, and controller logic
drivers/                Reusable device drivers
platform/api/           Shared application-to-platform hardware contract
platform/stm32f103/     Current STM32F103 implementation
platform/rp2350/        Unsupported RP2350 namespace
tests/                  Host-side deterministic tests
third_party/            External source dependencies
tools/                  Runtime and validation tooling
```

## Documentation ownership

```text
docs/architecture/      Current architecture and interface contracts
docs/commissioning/     Current commissioning interfaces and firmware state
docs/control/           Current controller implementation status
docs/validation/        Current evidence and capability-state semantics
docs/hardware/          Current hardware facts and explicit unknowns
docs/development/       Current repository/build reference
```

Process history, postmortems, experiment journals, roadmaps, dated status records, and checklists are intentionally not Markdown documentation.

## Dependency direction

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

`control/` remains MCU-independent. Platform implementations own SDK, peripheral, linker, startup, and target-specific plumbing.

RP2350 is not currently a buildable or supported firmware target.
