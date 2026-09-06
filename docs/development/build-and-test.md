# Build and Test

## Toolchain

The repository uses the stable Rust toolchain. `rust-toolchain.toml` installs `rustfmt`, `clippy`, and the Cortex-M3 `thumbv7m-none-eabi` target.

## Host tests

```bash
cargo test-host
```

This tests the target-independent `control`, `plant`, and `supervisor` crates on the host.

Equivalent command:

```bash
cargo test --workspace --exclude rip-firmware-stm32f103
```

## STM32F103 target

```bash
cargo build-stm32f103
```

Equivalent command:

```bash
cargo build -p rip-firmware-stm32f103 --release --target thumbv7m-none-eabi
```

The STM32F103 firmware crate uses a 64 KiB FLASH / 20 KiB RAM linker memory definition and is the composition root for target-specific hardware integration.
