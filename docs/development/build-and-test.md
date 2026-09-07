# Build and Test

## Toolchain

The repository uses the stable Rust toolchain. `rust-toolchain.toml` installs `rustfmt`, `clippy`, and the Cortex-M3 `thumbv7m-none-eabi` target.

## Host tests

```bash
cargo test-host
```

Equivalent command:

```bash
cargo test --workspace --exclude rip-firmware-stm32f103
```

This executes host tests for the portable Plant, Control, Supervisor, target-independent Firmware crates, and SITL.

## SITL

The deterministic scheduler-only smoke scenario is:

```bash
cargo run -p rip-sitl --release -- \
  --scenario tools/sitl/scenarios/deterministic_smoke.toml \
  --output target/sitl-deterministic
```

The full Rotary semantic path uses the reference assembly parameter registry:

```bash
cargo run -p rip-sitl --release -- \
  --scenario tools/sitl/scenarios/rotary_balance.toml \
  --parameters parameters/reference-assembly.json \
  --output target/sitl-balance

cargo run -p rip-sitl --release -- \
  --scenario tools/sitl/scenarios/rotary_swingup.toml \
  --parameters parameters/reference-assembly.json \
  --output target/sitl-swingup
```

Each run produces:

```text
manifest.json
trace.jsonl
summary.json
```

The full semantic-path runner executes virtual Furuta dynamics and sensor physics, then reuses production `RawObservation` promotion, estimator, hybrid control, Plant actuator model, Supervisor authority, Firmware TB6612 mapping, and a virtual physical actuator. Physical time advances between scheduler timestamps using the previously committed actuator input; missed runtime opportunities are not replayed.

## Clippy

```bash
cargo clippy --workspace --exclude rip-firmware-stm32f103 --all-targets -- -D warnings
```

## STM32F103 release build

```bash
cargo build-stm32f103
```

Equivalent command:

```bash
cargo build -p rip-firmware-stm32f103 --release --target thumbv7m-none-eabi
```

The STM32F103 executable lives under `firmware/targets/stm32f103/`. Its linker definition is 64 KiB FLASH / 20 KiB RAM.

The release profile uses size optimization, LTO, one codegen unit, and `panic = "abort"`.
