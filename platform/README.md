# Platform Layer

The platform layer separates application-visible hardware capabilities from MCU-specific implementations.

```text
platform/
├── api/            Shared `board_*` hardware contract
├── stm32f103/      STM32F103 implementation
└── rp2350/         Reserved platform namespace
```

Application and control code use `platform/api/` rather than MCU implementation headers.

The embedded implementation is provided by `platform/stm32f103/`.
