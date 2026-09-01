# Build and Test

## Clone

Clone with the libopencm3 submodule:

```bash
git clone --recursive https://github.com/cctsao1008/rotary-inverted-pendulum.git
cd rotary-inverted-pendulum
```

For an existing clone:

```bash
git submodule update --init --recursive
```

## Target selection

Firmware target selection is explicit through `RIP_TARGET`.

Currently buildable values are:

- `stm32f103` — current validated STM32F103 firmware target
- `none` — do not build target firmware; used by host-side tests

The RP2350 platform directory is intentionally present before firmware enablement so its API and board mapping can be developed without presenting unvalidated hardware support as buildable.

## Host tests

Requirements:

- CMake 3.16 or newer
- Ninja
- A C11 host compiler

```bash
cmake -S . -B build/host -G Ninja \
  -DRIP_TARGET=none \
  -DBUILD_HOST_TESTS=ON

cmake --build build/host
ctest --test-dir build/host --output-on-failure
```

## STM32F103 firmware

Additional requirements:

- Arm GNU Toolchain (`arm-none-eabi-gcc`)
- GNU Make for building libopencm3

```bash
cmake -S . -B build/stm32f103 -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-arm-none-eabi.cmake \
  -DRIP_TARGET=stm32f103 \
  -DBUILD_HOST_TESTS=OFF

cmake --build build/stm32f103
```

Expected outputs:

```text
build/stm32f103/rotary-inverted-pendulum.elf
build/stm32f103/rotary-inverted-pendulum.hex
build/stm32f103/rotary-inverted-pendulum.bin
build/stm32f103/rotary-inverted-pendulum.map
```
