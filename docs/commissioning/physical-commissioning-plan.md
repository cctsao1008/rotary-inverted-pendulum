# Physical Commissioning Plan

Physical closed-loop commissioning is intentionally blocked until the admission/run-permit model, fault decode, operator flow, telemetry evidence path, and actuator safety limits are validated.

## Preconditions

- exact decode of active control/state-safety faults;
- explicit system-state and arm/enable semantics;
- admission/run-permit split implemented and tested;
- no automatic restart after permit/fault drop;
- motor sink routes only through Motor Authority Arbiter;
- stale-command watchdog enabled;
- calibrated entry/sample limits;
- actuator magnitude limiter;
- actuator slew-rate limiter;
- emergency-stop path validated;
- validation logging ready.

## First physical runs

Use:

- restrained fixture;
- conservative power/current limit;
- low motor-output limit;
- short-duration tests;
- manual emergency-stop access;
- complete telemetry capture.

Increase actuator authority only after the previous level has repeatable evidence. A successful motion or balance result does not bypass the next commissioning boundary.
