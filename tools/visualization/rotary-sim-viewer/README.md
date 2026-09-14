# Rotary Simulation Console

A lightweight local viewer for Rotary simulation and evidence.

The console is deliberately **read-only with respect to physics and authority**. It consumes normalized trace/evidence JSON produced by the existing Rust/Python/SciPy/rigid-body tooling. It does not reimplement Furuta dynamics, control laws, model-validity decisions, or physical authority logic in JavaScript.

> The UI may explain evidence. It may not upgrade evidence.

## Launch

From the repository root:

```bash
python -m http.server 8000
```

Then open:

```text
http://localhost:8000/tools/visualization/rotary-sim-viewer/
```

The first MVP uses Three.js from a CDN, so the browser needs network access for that library. The evidence/trace files themselves remain local.

## MVP

The initial console provides:

- configurator-style navigation and status layout;
- a 3-D Furuta scene with the project DOF topology;
- replay controls for a normalized JSON trace;
- live display of `[theta, theta_dot, phi, phi_dot]`;
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

Convert a SITL run:

```bash
python tools/visualization/rotary-sim-viewer/adapt_sitl_trace.py \
  --trace target/sitl/<run>/trace.jsonl \
  --manifest target/sitl/<run>/manifest.json \
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

The viewer's primary `arm_torque_nm` field is the applied virtual torque when available. The requested and applied values remain separately present in normalized samples for later UI expansion.

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

- `phi`: base/rotary-arm rotation about the vertical axis;
- `theta`: pendulum rotation about the horizontal hinge at the arm tip.

The viewer must never reinterpret these axes simply to make an animation look nicer.

## Non-goals

- no browser-side Furuta integrator;
- no JavaScript controller implementation;
- no browser-side model-authority decision engine;
- no ROS2 requirement;
- no Octave requirement;
- no physical output path.
