# Forest D1 2016 Hardware Baseline

This document defines the current hardware facts for the original Forest S1 controller and Forest D1 rotary inverted-pendulum baseboard used as the reference plant.

## Controller and clocking

- MCU: STM32F103C8T6.
- MCU HSE: 8 MHz; current firmware derives a 72 MHz system clock.
- USB-UART: CH340G with a separate 12 MHz crystal.
- USB connector: Micro-USB on this revision.

The 12 MHz CH340G crystal is not the STM32 HSE.

## Current pin map

| Function | STM32 pin / peripheral |
|---|---|
| Pendulum-angle sensor | PA7 / ADC1_IN7 |
| Battery-voltage sense | PA6 / ADC1_IN6 |
| Arm encoder A | PA0 / TIM2_CH1 |
| Arm encoder B | PA1 / TIM2_CH2 |
| Motor D2 PWM | PB1 / TIM3_CH4 |
| Motor D2 direction | PB13 / PB12 |
| Motor D1 PWM | PB0 / TIM3_CH3 |
| Motor D1 direction | PB14 / PB15 |
| USART1 TX / RX | PA9 / PA10 |
| User LED | PA4, active-low |
| Forest S1 USER key | PA5, active-low |
| Forest D1 M / X / + / - keys | PA3 / PA2 / PA11 / PA12, active-low |
| OLED clock / data / reset / D-C | PB5 / PB4 / PB3 / PA15 |
| SWDIO / SWCLK | PA13 / PA14 |

The battery-sense divider is nominally 10 kΩ / 1 kΩ, giving an ideal 11:1 scale.

## Pendulum sensor

The supplied WDD35D4 conductive-plastic angular sensor documentation specifies:

- nominal resistance: 5 kΩ;
- independent linearity: 0.1%;
- effective electrical angle: 345° ± 2°;
- continuous mechanical rotation.

The current firmware samples its wiper through PA7 / ADC1_IN7.

## Encoder and motor

The firmware uses TIM2 quadrature encoder mode and currently uses **1040 counts per geared output-shaft revolution** as the encoder reference.

D2 is the default maintenance motor channel. STBY is tied high on the baseboard, so firmware safe-off relies on PWM/direction state rather than an MCU-controlled STBY line.

The maintenance motor service is bounded and independent of automatic control. The automatic control motor sink is currently unbound.

## Current unresolved physical facts

The following are not established as calibrated current facts:

- mechanically referenced pendulum upright ADC zero;
- pendulum sign and usable calibrated travel envelope;
- protection-grade battery-voltage scale and offset;
- fully validated encoder sign/phase convention for the assembled plant;
- calibrated active-control arm/pendulum safety limits.

The current compiled pendulum upright ADC default (`2928`) is provisional, not a mechanically calibrated result.

## TB6612 logic-level margin

The schematic supplies TB6612 logic VCC at 5 V while STM32 GPIO high level is 3.3 V. A conservative `VIH = 0.7 × VCC` interpretation gives 3.5 V. The exact populated-device/input-margin condition is therefore an unresolved electrical-margin item rather than a verified robust logic-level guarantee.
