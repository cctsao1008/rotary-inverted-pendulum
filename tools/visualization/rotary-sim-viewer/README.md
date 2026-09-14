# Rotary Simulation Console

A lightweight local viewer for Rotary simulation and evidence.

The console is deliberately **read-only with respect to physics and authority**. It consumes normalized trace/evidence JSON produced by the existing Rust/Python/SciPy/rigid-body tooling. It does not reimplement Furuta dynamics, control laws, model-validity decisions, or physical authority logic in JavaScript.

> The UI may explain evidence. It may not upgrade evidence.

## Launch

### Replay-only mode

From the repository root:

```bash
python -m http.server 8000
```

Then open:

```text
http://localhost:8000/tools/visualization/rotary-sim-viewer/
```

### Continuous SITL mode

For continuous console animation backed by fresh `rip-sitl` runs, use the local live server instead of `http.server`:

```bash
python tools/visualization/rotary-sim-viewer/serve_live.py
```

Then open the same URL:

```text
http://127.0.0.1:8000/tools/visualization/rotary-sim-viewer/
```

Use the **Balance** or **Swing-up** selector and press **Live**. The server repeatedly launches the existing Rust `rip-sitl` scenario, normalizes the resulting authoritative SITL JSONL with `adapt_sitl_trace.py`, and streams display frames to the browser over Server-Sent Events (SSE). **Stop** closes the browser stream.

The current live transport is deliberately run-backed rather than a second interactive physics engine: each cycle is a fresh deterministic SITL run, then its evidence is streamed at the selected wall-clock speed. This provides continuous visualization without moving dynamics or controller logic into JavaScript. A future incremental SITL observer can replace the run-backed transport without changing the viewer schema.

The live server limits browser display transport to 60 fps by default. This is display projection only; the original SITL evidence remains at its native sample/event rate.

The first MVP uses Three.js from a CDN, so the browser needs network access for that library. The evidence/trace files themselves remain local.

## MVP

The console provides:

- configurator-style navigation and status layout;
- a 3-D Furuta scene with the project DOF topology;
- replay controls for a normalized JSON trace;
- continuous run-backed SITL visualization for Balance and Swing-up;
- live display of `[theta, theta_dot, phi, phi_dot]`;
- requested/applied torque, runtime-state, and authority fields where present;
- model/backend/evidence metadata;
- a persistent `SIMULATION ONLY / NO PHYSICAL AUTHORITY` boundary;
- an explicit synthetic demo trace for UI smoke testing.

The demo trace is **not dynamics evidence**. It exists only to exercise rendering/replay behavior.

## SITL adapter

`adapt_sitl_trace.py` projects existing SITL JSONL evidence into the viewer schema. It does not integrate dynamics, run a controller, alter the source trace, or create a new authority claim.

Self-test the adapter:

```bash
python tools/visualization/rotary-sim-viewer/adapt_sitl_trace.py --self-test
```

Run a real Rotary SITL scenario first, for example:

```bash
cargo run --manifest-path tools/sitl/Cargo.toml --bin rip-sitl -- \
  --scenario tools/sitl/scenarios/rotary_balance.toml \
  --output target/sitl/rotary-balance
```

Then project that evidence into the viewer schema:

```bash
python tools/visualization/rotary-sim-viewer/adapt_sitl_trace.py \
  --trace target/sitl/rotary-balance/trace.jsonl \
  --manifest target/sitl/rotary-balance/manifest.json \
  --output target/viewer/rotary-sitl.json
```

Then use **Load trace** in the console and select `target/viewer/rotary-sitl.json`.

The adapter groups records by SITL virtual time and preserves the canonical truth state. Where present it also carries forward:

- control regime;
- requested arm torque;
- applied virtual arm torque;
- runtime state;
- authority decision;
- estimated state.

The viewer's primary `arm_torque_nm` field is the applied virtual torque when available. Requested and applied values remain separate fields.

## Normalized viewer trace

```json
{
  "schema": 1,
  "source": {
    "kind": "simulation",
    "model_class": "full3d_geometry",
    "backend": "scipy",
    "provenance": "...",
    "scope": "simulation model evidence only"
  },
  "state_order": ["theta", "theta_dot", "phi", "phi_dot"],
  "samples": [
    {
      "t_s": 0.0,
      "state": [0.0, 0.0, 0.0, 0.0],
      "arm_torque_nm": 0.0,
      "control_regime": "Balance"
    }
  ]
}
```

Adapters for existing SITL/model/rigid-body outputs are generated host-side so original evidence files remain authoritative and unchanged.

## DOF convention

The viewer follows the canonical project state order:

```text
[theta, theta_dot, phi, phi_dot]
```

The authoritative geometry/sign contract is `tools/model/rigid_body/furuta_contract.json`:

- project `phi`: arm rotation about project `+z` (vertical);
- at `phi = 0`, the rotary arm points along project `+x`;
- project `theta`: pendulum rotation about the declared joint axis `[-1, 0, 0]`, so the hinge axis is horizontal and parallel/anti-parallel to the radial arm at `phi = 0`;
- positive `theta` moves the upright pendulum COM initially toward project `+y`.

The Three.js scene is Y-up, so the viewer maps project coordinates as:

```text
project +x -> viewer +X
project +y -> viewer -Z
project +z -> viewer +Y
```

Under that right-handed mapping, positive project `theta` is rendered as rotation about viewer `-X`. The hinge/axle mesh is aligned to viewer X as well, so the visible mechanism and the replay transform encode the same DOF.

The viewer must never reinterpret these axes simply to make an animation look nicer.

## Non-goals

- no browser-side Furuta integrator;
- no JavaScript controller implementation;
- no browser-side model-authority decision engine;
- no ROS2 requirement;
- no Octave requirement;
- no physical output path.
