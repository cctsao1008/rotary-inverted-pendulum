# Controller Implementation

Controller algorithms are isolated behind the common control architecture.

## Balance controller

`BALANCE_CONTROLLER_LQR` is the runtime-selectable balance-controller implementation. `lqr_controller_step()` delegates to the implemented balance-controller computation path.

## State estimator

`STATE_ESTIMATOR_BASIC` is the runtime-selectable state-estimator implementation.

## Boundaries

Controllers consume control-domain state and produce control commands. They do not own MCU motor peripherals and do not write PWM or direction GPIO directly.

Physical motor ownership remains behind the Motor Authority Arbiter.
