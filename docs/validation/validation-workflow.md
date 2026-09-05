# Validation Workflow

Every hardware validation run should be reproducible and traceable to source, binary identity, hardware condition, runtime configuration, and measured evidence.

## Standard flow

The current preferred workflow uses direct, atomic GitHub commits followed by local validation:

```text
ChatGPT inspects latest GitHub main
  -> one atomic GitHub commit
  -> local git pull --ff-only
  -> tools/post_patch_check.sh
  -> flash validated artifact when runtime code changed
  -> Windows serial_tool scenario / UART capture
  -> runtime log
  -> validation record
  -> PASS / FAIL / INCONCLUSIVE
```

Patch / `git am` remains a fallback when direct repository writes are not appropriate, but it is no longer the primary path for this project.

## Repository write rules

- Read the latest `main` before each change.
- Keep one clear engineering objective per commit.
- Use a fast-forward ref update only; never force-push normal development changes.
- If `main` moves before the write completes, re-read the new baseline and rebuild the change from that source.
- Do not use Agent, Codex, Work, or similar agentic execution without explicit user approval.

## Local validation

After a new GitHub commit:

```bash
git pull --ff-only
./tools/post_patch_check.sh 2>&1 | tee post_patch_check.log
```

Flash and runtime capture are required when the commit changes firmware/runtime behavior. Documentation-only or host-tool-only changes may stop after build/test validation when no target behavior changed.

## Minimum evidence

Record:

- Git commit;
- firmware build timestamp;
- ELF/HEX/BIN SHA256 when available;
- hardware setup;
- power conditions;
- runtime parameters and profile;
- exact command scenario;
- `post_patch_check.log`;
- runtime log;
- acceptance criteria;
- measured result;
- conclusion and next action.

## Acceptance criteria

A validation run should not be marked PASS merely because it did not crash. Define measurable criteria where practical, for example:

- loop average/max execution time;
- stale-sample threshold;
- command delivery reliability;
- fault/recovery behavior;
- actuator magnitude and rate limits;
- capture/recovery envelope.

## Historical baseline

Validated pre-P1 baseline on 2026-08-15:

```text
commit: b6bd30d414c906cf9de87e9110499834cc749663
host tests: 24/24 PASS
STM32 build: PASS
ROM: 52936 B / 64 KB = 80.77%
RAM: 3428 B / 20 KB = 16.74%
```

Observed runtime remained observe-only with `motor_sink=unbound`.
