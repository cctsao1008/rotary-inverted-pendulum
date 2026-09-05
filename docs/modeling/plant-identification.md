# Plant Characterization and Identification

Controller quality depends on measured plant behavior. Identification should precede aggressive controller tuning, and fitted models must remain traceable to the hardware condition and dataset from which they were derived.

## Initial measurements

- motor dead zone;
- command-to-arm-velocity gain;
- direction asymmetry;
- dominant motor/mechanical time constant;
- friction and stiction;
- encoder scale and sign;
- command-to-measurement delay;
- effective sample timing/jitter;
- pendulum free-response characteristics where applicable.

## Bounded step-response sequence

Use mechanically restrained tests and bounded output levels. Candidate command points:

```text
+10%, +20%, +30%, -10%, -20%, -30%
```

Do not promote these exact values to hardware tests without first confirming that the configured limiter and physical fixture make them safe for the installed motor and supply.

## Model output

Each identification run should produce:

- a traceable dataset;
- hardware and power conditions;
- controller/runtime commit identity;
- fitted or empirical model parameters;
- fitting method and residual/error metric where applicable;
- stated uncertainty and known validity range;
- a concise real-versus-model comparison.

A model is a control-relevant approximation of measured plant behavior, not a replacement for the physical plant.
