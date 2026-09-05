# Record and Replay

Record/replay allows controller, estimator, safety, and state-machine changes to be evaluated against previously observed data before repeating a physical experiment.

## Record

Capture time-aligned inputs and state required to reproduce control decisions, including at minimum:

- timestamp/logical tick;
- measured arm and pendulum states;
- estimator outputs when applicable;
- control mode;
- safety/fault state;
- control output command;
- sample age and timing diagnostics.

## Replay

Host tests should be able to feed recorded input sequences through deterministic control pipeline components and compare:

- state transitions;
- gate/run-permit decisions;
- faults;
- controller output;
- estimator output;
- timing-independent logical behavior.

## Initial scope

Start with replay of recorded sensor/state data. Full hardware-in-the-loop is not required for the first implementation.
