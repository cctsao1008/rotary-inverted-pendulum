# Platform Layer

The platform layer separates application-visible hardware capabilities from MCU-specific implementations.

## Layout

```text
platform/
├── api/            Shared board contract used by the application
├── stm32f103/      Current STM32F103 implementation
└── rp2350/         Planned RP2350 implementation
```

`platform/api/` owns the `board_*` interfaces. Application code includes these headers without knowing which MCU provides the implementation.

Each MCU platform is responsible for implementing the same contract using its native peripherals and SDK. Platform-specific pin assignments, linker/startup requirements, PIO programs, DMA configuration, and USB/UART plumbing must stay below the platform boundary.

A platform is not considered supported merely because a directory or schematic mapping exists. Buildability and hardware validation are separate evidence states.
