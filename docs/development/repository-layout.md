# Repository Layout

The repository is organized around control-system boundaries rather than controller names or MCU-specific control logic.

## Top-level ownership

```text
app/                    Application integration and system orchestration
cmake/                  Cross-compilation support
control/                Platform-independent estimator, safety, mode, and controller logic
drivers/                Reusable device drivers
platform/api/           Shared application-to-platform hardware contract
platform/stm32f103/     STM32F103 implementation
platform/rp2350/        Reserved platform namespace
tests/                  Host-side deterministic tests
third_party/            External source dependencies
tools/                  Runtime and analysis tooling
```

## Documentation ownership

```text
docs/architecture/      Architecture and interface contracts
docs/commissioning/     Firmware and maintenance interfaces
docs/control/           Controller implementation
docs/hardware/          Hardware definition
docs/development/       Repository/build reference
```

Markdown is specification/reference material. Validation evidence, open questions, history, roadmaps, dated status records, and checklists are kept outside Markdown.

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
