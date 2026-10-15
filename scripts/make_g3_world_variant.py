#!/usr/bin/env python3
"""Create a hash-recorded G3 physics-step world without changing the baseline."""

import argparse
import hashlib
import json
from pathlib import Path
import re


ALLOWED_STEPS = {0.001, 0.0005}
STEP_PATTERN = re.compile(
    r"(<max_step_size>\s*)([^<]+?)(\s*</max_step_size>)"
)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def create_variant(source, output, step):
    source = Path(source)
    output = Path(output)
    if step not in ALLOWED_STEPS:
        raise ValueError("step must be exactly 0.001 or 0.0005 seconds")
    if output.exists():
        raise ValueError(f"refusing to overwrite existing output: {output}")
    text = source.read_text()
    matches = list(STEP_PATTERN.finditer(text))
    if len(matches) != 1:
        raise ValueError("expected exactly one max_step_size element")
    baseline = float(matches[0].group(2).strip())
    if baseline != 0.001:
        raise ValueError(f"expected 0.001-second baseline, found {baseline}")
    rendered = STEP_PATTERN.sub(
        lambda match: f"{match.group(1)}{step:.4f}{match.group(3)}",
        text,
        count=1,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(rendered)
    return {
        "schema_version": 1,
        "source": str(source.resolve()),
        "source_sha256": sha256(source),
        "output": str(output.resolve()),
        "output_sha256": sha256(output),
        "baseline_step_seconds": baseline,
        "selected_step_seconds": step,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--step", type=float, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    args = parser.parse_args()
    if args.manifest.exists():
        parser.error(f"refusing to overwrite existing manifest: {args.manifest}")
    try:
        manifest = create_variant(args.source, args.output, args.step)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    args.manifest.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
