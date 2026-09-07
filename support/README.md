# Support

Cross-domain implementation support used by production-domain crates lives here.

`support/` is not a fifth production architecture domain. The production architecture remains exactly Plant, Control, Supervisor, and Firmware.

- `dsp-kernel/` provides the canonical real-time numerical implementation boundary used by production code while preserving deterministic host semantics for verification.
