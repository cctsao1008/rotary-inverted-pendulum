# Runtime Profiles

## Implemented profile

### `observe_only`

The current application profile has:

- automatic motor output disabled;
- automatic motor sink unbound;
- telemetry available;
- basic estimator configuration active;
- state-safety configuration structurally valid;
- physical angle/rate state-safety bounds intentionally non-restrictive.

## Not implemented

The repository does not currently provide authoritative `bench_safe`, `commissioning`, or `normal` active-control profiles.

No Markdown value for an unimplemented profile is an active safety limit.
