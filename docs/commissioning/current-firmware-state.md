# Current Firmware State

## Automatic control

The automatic-control runtime is observe-only:

- real sensor acquisition is connected to the control pipeline;
- basic estimation is implemented;
- LQR is the accepted balance-controller selection;
- state-safety evaluation is active;
- `motor_output_enabled = false`;
- the automatic physical motor sink is unbound.

## Physical motor path

Physical actuation is currently available only through the maintenance service and Motor Authority Arbiter.

Implemented maintenance commands include:

```text
motor status
motor channel d1
motor channel d2
motor arm
motor identify
motor characterize right
motor characterize left
motor response ...
motor brake-response ...
motor test <signed_percent> <duration_ms>
motor stop
motor disarm
```

The default maintenance motor channel is D2.

`motor arm` expires after 30 seconds. Ordinary `motor test` is bounded to 1..20% magnitude and 50..10000 ms duration.

## Motor outputs

- D1: PB0 / TIM3_CH3 PWM with PB14 / PB15 direction.
- D2: PB1 / TIM3_CH4 PWM with PB13 / PB12 direction.

Completion/stop paths force PWM to zero and return the maintenance interface to a disarmed state.

## Voltage telemetry

`vbus_mV` uses the nominal 3.300 V VDDA assumption and the schematic 10 kΩ / 1 kΩ divider ratio. It is diagnostic telemetry; actual scale/offset calibration is not established as a protection-grade measurement.
