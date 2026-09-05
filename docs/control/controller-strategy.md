# Controller Status

Controller algorithms are replaceable implementations behind the common control architecture.

## Current runtime-selectable controller

### LQR

`BALANCE_CONTROLLER_LQR` is the only balance-controller selection accepted by the current runtime configuration validator. `lqr_controller_step()` delegates to the implemented balance-controller computation path.

The automatic physical motor sink remains unbound, so this does not constitute physically commissioned closed-loop balance.

## Current stubs / unsupported selections

| Capability | Current source state | Runtime state |
|---|---|---|
| LQI | `lqi_controller_step()` returns safe zero | Rejected |
| Energy swing-up | `energy_swing_up_step()` returns safe zero | Disabled / rejected |
| Capture | `capture_controller_step()` returns safe zero | Disabled / rejected |
| Pole placement | No runtime implementation | Not selectable |
| Kalman estimator | Not an accepted runtime estimator | Not selectable |

The basic state estimator is the currently accepted estimator selection.

No controller directly owns MCU motor peripherals; physical motor ownership remains behind the Motor Authority Arbiter.
