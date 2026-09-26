# Build and Test

## Toolchains

The portable/reference stack uses the stable Rust toolchain. `rust-toolchain.toml` installs `rustfmt`, `clippy`, and the Cortex-M3 `thumbv7m-none-eabi` target.

The RP2350A target uses Raspberry Pi Pico SDK + CMake + an Arm embedded GCC toolchain. The host commissioning tool uses Python with `hidapi` and `pyserial`.

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

## RP2350A Pico SDK build

The RP2350A executable lives under `firmware/targets/rp2350a/` and uses the local `uno_rp2350` Pico SDK board definition.

```bash
export PICO_SDK_PATH=/path/to/pico-sdk
cmake -S firmware/targets/rp2350a -B build/rp2350a -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/rp2350a
```

Expected outputs:

```text
build/rp2350a/rip_rp2350a.elf
build/rp2350a/rip_rp2350a.bin
build/rp2350a/rip_rp2350a.uf2
```

The dedicated `RP2350 Pico SDK` GitHub Actions workflow also validates the Python commissioning CLI and native C++ runtime semantics before producing the firmware artifact.

## RP2350A physical development tool

Install the two host dependencies once:

```bash
python -m pip install -r tools/rp2350_commission/requirements.txt
```

Then use the single CLI for CDC + HID development work:

```bash
python tools/rp2350_commission/rp2350_commission.py status
python tools/rp2350_commission/rp2350_commission.py monitor --duration 10
python tools/rp2350_commission/rp2350_commission.py adc --duration 5
python tools/rp2350_commission/rp2350_commission.py encoder --motor-command 0.10 --duration 3
python tools/rp2350_commission/rp2350_commission.py motor --command 0.15 --duration 2
python tools/rp2350_commission/rp2350_commission.py speed-sweep
python tools/rp2350_commission/rp2350_commission.py all
```

HID owns machine commands and 100 Hz binary telemetry; CDC supplies human-readable debug/status text. Development motor commands are direct normalized commands with explicit `SAFE_OFF` and a short stale-command timeout rather than a separate arming/authority workflow.
