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

This executes host tests for the portable Plant, Control, Supervisor, and target-independent Firmware crates.

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
