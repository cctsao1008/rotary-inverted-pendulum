# Motor Characterization Interface

The current maintenance firmware contains bounded measurement commands for motor/encoder properties.

## `motor identify`

- applies +5% for 250 ms;
- stops for 250 ms;
- retries at +8% when fewer than three encoder counts are observed;
- reports encoder delta, inferred sign, and peak observed velocity;
- stops and disarms on completion.

## `motor characterize`

- direction: right or left;
- rising sweep: 5% to 30% in 2% steps;
- motion confirmation: two consecutive 250 ms windows;
- descending sweep: 1% steps;
- descending dwell: 1.5 s per step;
- overall timeout: 60 s.

Reported properties include breakaway duty, minimum sustainable duty, dropout duty, encoder sign, and peak windowed velocity.

The current encoder reference is **1040 quadrature counts per output-shaft revolution**. Rated-speed reference is 9516 counts/s and the current plausibility ceiling is 15000 counts/s.

## `motor response`

- duty range: 30..80%;
- drive duration: 1000..10000 ms;
- coast observation: up to 5 s;
- velocity window: 100 ms;
- stopped classification: three consecutive windows with no more than one count per window.

Reported data include drive displacement, cutoff velocity, stopping time, coast displacement, and peak velocity.

## `motor brake-response`

- drive duty range: 30..80%;
- drive duration: 1000..10000 ms;
- reverse-brake duty range: 10..20%;
- neutral guard: 1 ms;
- maximum reverse-brake interval: 300 ms;
- settling observation: 300 ms;
- initial release threshold: 600 counts/s.

The command reports drive, neutral, braking, release, settling, velocity, and `vbus_mV` fields through the current motor CSV record.

## Scripted brake sweep

The maintenance script service supports repeated `motor brake-response` sequences with zero-output waits. Motor arming remains external to the script itself.
