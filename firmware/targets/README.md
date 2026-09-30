# Firmware targets

The repository currently has two execution targets:

| Target | Toolchain | Role |
| --- | --- | --- |
| `stm32f103/` | Embedded Rust / `thumbv7m-none-eabi` | Original Forest S1/D1 execution target and live-shadow reference |
| `rp2350a/` | Raspberry Pi Pico SDK / C++17 | UNO-form-factor RP2350A replacement target used for current hardware development |

The RP2350A target preserves the existing sensing, estimation, hybrid-control, actuator-model, timing, watchdog, and TB6612 semantics while replacing the MCU/peripheral backend.

For physical development, use the single host entry point:

```bash
python tools/rp2350_commission/rp2350_commission.py <command>
```

See [`rp2350a/README.md`](rp2350a/README.md) for pin mapping and firmware details, and [`../../docs/development/build-and-test.md`](../../docs/development/build-and-test.md) for build/test commands.
