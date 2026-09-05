# Validation and Evidence Model

Engineering claims use explicit current-state labels.

## Evidence basis

- **DOC** — supported by schematic, datasheet, vendor documentation, or another controlled document.
- **CODE** — present in the current repository implementation.
- **MEASURED** — directly observed on the current physical specimen.
- **INFERRED** — engineering inference not yet directly confirmed.
- **UNKNOWN** — not established.
- **LEGACY_REFERENCE** — known only from the original product or vendor implementation and not automatically valid for the current implementation.

These labels may be combined when appropriate.

## Capability state

- **NOT IMPLEMENTED** — no current implementation exists.
- **STUB** — interface/topology exists but behavior is intentionally incomplete or safe-zero.
- **IMPLEMENTED** — implementation exists in current source.
- **HOST-VALIDATED** — deterministic host tests support the implementation.
- **RUNTIME-VALIDATED** — exercised on the embedded target.
- **PHYSICALLY-COMMISSIONED** — validated with the physical plant and actuator.

Source existence, runtime selection, and physical commissioning are separate claims.

```text
DOC != MEASURED
IMPLEMENTED != PHYSICALLY-COMMISSIONED
legacy product behavior != current implementation behavior
```
