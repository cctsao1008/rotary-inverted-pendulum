#!/usr/bin/env python3
"""Check that every nominal parameter has explicit, internally consistent provenance."""

from __future__ import annotations

import json
import re
import sys
from collections import Counter
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
REGISTRY_PATH = ROOT / "parameters" / "reference-assembly.json"
EVIDENCE_PATH = ROOT / "parameters" / "reference-assembly-evidence.json"

PARAMETER_KEYS = {"value", "unit", "source", "applicability"}
ALLOWED_EVIDENCE_CLASSES = {
    "reference_nominal",
    "derived_reference",
    "vendor_documented",
    "standard_constant",
    "project_reference",
    "project_convention",
    "model_assumption",
    "simulation_fixture",
}
ALLOWED_SPECIMEN_STATUS = {
    "not_specimen_calibrated",
    "not_specimen_specific",
    "hardware_family_documented",
    "project_reference_only",
    "simulation_only",
}
EXTERNAL_REFERENCE_REQUIRED = {
    "reference_nominal",
    "derived_reference",
    "vendor_documented",
}
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: top level must be an object")
    return data


def collect_parameters(node: Any, prefix: str = "") -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    if not isinstance(node, dict):
        return result

    if PARAMETER_KEYS.issubset(node.keys()):
        if not prefix:
            raise ValueError("parameter leaf cannot be the document root")
        result[prefix] = node
        return result

    for key, value in node.items():
        child = f"{prefix}.{key}" if prefix else key
        result.update(collect_parameters(value, child))
    return result


def fail(message: str) -> None:
    print(f"parameter provenance error: {message}", file=sys.stderr)
    raise SystemExit(1)


def main() -> None:
    registry = load_json(REGISTRY_PATH)
    evidence = load_json(EVIDENCE_PATH)

    registry_parameters = collect_parameters(registry)
    evidence_parameters = evidence.get("parameters")
    sources = evidence.get("sources")

    if not isinstance(evidence_parameters, dict):
        fail("evidence document must contain a parameters object")
    if not isinstance(sources, dict):
        fail("evidence document must contain a sources object")

    registry_paths = set(registry_parameters)
    evidence_paths = set(evidence_parameters)
    missing = sorted(registry_paths - evidence_paths)
    extra = sorted(evidence_paths - registry_paths)
    if missing:
        fail(f"missing evidence for: {', '.join(missing)}")
    if extra:
        fail(f"evidence has no registry parameter: {', '.join(extra)}")

    for source_id, source in sources.items():
        if not isinstance(source, dict):
            fail(f"source {source_id} must be an object")
        sha256 = source.get("sha256")
        if sha256 is not None and (
            not isinstance(sha256, str) or not SHA256_RE.fullmatch(sha256)
        ):
            fail(f"source {source_id} has invalid sha256")

    class_counts: Counter[str] = Counter()
    status_counts: Counter[str] = Counter()

    for path, parameter in registry_parameters.items():
        for field in ("unit", "source", "applicability"):
            value = parameter.get(field)
            if not isinstance(value, str) or not value.strip():
                fail(f"{path}.{field} must be a non-empty string")

        record = evidence_parameters[path]
        if not isinstance(record, dict):
            fail(f"evidence record for {path} must be an object")

        evidence_class = record.get("evidence_class")
        specimen_status = record.get("specimen_status")
        refs = record.get("evidence_refs")
        rationale = record.get("rationale")
        derived_from = record.get("derived_from", [])

        if evidence_class not in ALLOWED_EVIDENCE_CLASSES:
            fail(f"{path} has unsupported evidence_class {evidence_class!r}")
        if specimen_status not in ALLOWED_SPECIMEN_STATUS:
            fail(f"{path} has unsupported specimen_status {specimen_status!r}")
        if not isinstance(refs, list) or any(not isinstance(ref, str) for ref in refs):
            fail(f"{path}.evidence_refs must be a string array")
        if evidence_class in EXTERNAL_REFERENCE_REQUIRED and not refs:
            fail(f"{path} requires at least one external evidence reference")
        for ref in refs:
            if ref not in sources:
                fail(f"{path} references undefined source {ref}")
        if not isinstance(rationale, str) or not rationale.strip():
            fail(f"{path}.rationale must be a non-empty string")
        if not isinstance(derived_from, list) or any(
            not isinstance(parent, str) for parent in derived_from
        ):
            fail(f"{path}.derived_from must be a string array")
        for parent in derived_from:
            if parent not in registry_parameters:
                fail(f"{path} derives from unknown parameter {parent}")

        class_counts[evidence_class] += 1
        status_counts[specimen_status] += 1

    print(f"parameter provenance: {len(registry_parameters)} parameters covered")
    print(
        "evidence classes: "
        + ", ".join(f"{key}={class_counts[key]}" for key in sorted(class_counts))
    )
    print(
        "specimen status: "
        + ", ".join(f"{key}={status_counts[key]}" for key in sorted(status_counts))
    )


if __name__ == "__main__":
    main()
