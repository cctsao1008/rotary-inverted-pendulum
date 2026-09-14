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

Adapters for existing SITL/model/rigid-body outputs should be generated host-side so original evidence files remain authoritative and unchanged.

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
