# Build and Test

## Build modes

`RIP_TARGET` accepts:

- `stm32f103` — embedded firmware target
- `none` — host-test build without target firmware

## Host tests

Requirements: CMake 3.16+, Ninja, and a C11 host compiler.

```bash
cmake -S . -B build/host -G Ninja \
  -DRIP_TARGET=none \
  -DBUILD_HOST_TESTS=ON
cmake --build build/host
ctest --test-dir build/host --output-on-failure
```

## STM32F103 firmware

Additional requirements: Arm GNU Toolchain and GNU Make for libopencm3.

```bash
cmake -S . -B build/stm32f103 -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-arm-none-eabi.cmake \
  -DRIP_TARGET=stm32f103 \
  -DBUILD_HOST_TESTS=OFF
cmake --build build/stm32f103
```

Artifacts:

```text
build/stm32f103/rotary-inverted-pendulum.elf
build/stm32f103/rotary-inverted-pendulum.hex
build/stm32f103/rotary-inverted-pendulum.bin
build/stm32f103/rotary-inverted-pendulum.map
```
