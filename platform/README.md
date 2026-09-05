# Platform Layer

The platform layer separates application-visible hardware capabilities from MCU-specific implementations.

```text
platform/
├── api/            Shared `board_*` hardware contract
├── stm32f103/      Implemented and buildable
└── rp2350/         Not implemented / not buildable
```

Application and control code use `platform/api/` rather than MCU implementation headers.

STM32F103 is the current supported embedded target. RP2350 is not currently a supported platform.
