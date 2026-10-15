#!/usr/bin/env bash
# Generate a local photo-billboard world only from an explicitly verified asset.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

python3 - "$ROOT" "$@" <<'PY'
import argparse
import hashlib
import json
from pathlib import Path
import sys
from xml.sax.saxutils import escape

root = Path(sys.argv[1]).resolve()
parser = argparse.ArgumentParser(
    description="Generate a collision-safe G6 photo world from a manifested CC0 image.")
parser.add_argument("--texture", required=True, type=Path,
                    help="image file to use as the billboard texture")
parser.add_argument("--provenance", required=True, type=Path,
                    help="JSON provenance manifest with source, CC0 license and SHA-256")
parser.add_argument("--output", type=Path,
                    help="new SDF output path (defaults to a hash-named file in log/test_assets)")
args = parser.parse_args(sys.argv[2:])

texture = args.texture.expanduser().resolve(strict=True)
provenance_path = args.provenance.expanduser().resolve(strict=True)
if not texture.is_file() or not provenance_path.is_file():
    parser.error("texture and provenance must be regular files")

provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
required = ("source_page", "creator", "license", "license_url", "sha256",
            "dimensions_px")
missing = [key for key in required if not provenance.get(key)]
if missing:
    parser.error("provenance is missing required fields: " + ", ".join(missing))
if provenance["license"] != "CC0":
    parser.error("only an explicitly recorded CC0 asset is accepted")
if "creativecommons.org/publicdomain/zero/1.0" not in provenance["license_url"]:
    parser.error("license_url does not identify the CC0 1.0 dedication")
dimensions = provenance["dimensions_px"]
if (not isinstance(dimensions, list) or len(dimensions) != 2
        or any(type(value) is not int or value <= 0 for value in dimensions)):
    parser.error("dimensions_px must contain two positive integers")

digest = hashlib.sha256(texture.read_bytes()).hexdigest()
if digest != provenance["sha256"].lower():
    parser.error("texture SHA-256 does not match its provenance manifest")

template = root / "linorobot2_gazebo/worlds/fortress_g6_vision_photo.sdf.in"
template_text = template.read_text(encoding="utf-8")
markers = ("@PHOTO_TEXTURE@", "@PHOTO_WIDTH@", "@PHOTO_HEIGHT@")
if any(template_text.count(marker) != 1 for marker in markers):
    parser.error("world template must contain each photo marker exactly once")

output = args.output.expanduser().resolve() if args.output else (
    root / "log/test_assets" / f"fortress_g6_vision_photo_{digest[:12]}.sdf")
if output in (texture, provenance_path, template.resolve()):
    parser.error("output path must not replace the texture, manifest, or template")
output.parent.mkdir(parents=True, exist_ok=True)
board_height_m = 1.3
board_width_m = board_height_m * dimensions[0] / dimensions[1]
rendered = template_text
rendered = rendered.replace("@PHOTO_TEXTURE@", escape(str(texture)))
rendered = rendered.replace("@PHOTO_WIDTH@", f"{board_width_m:.6f}")
rendered = rendered.replace("@PHOTO_HEIGHT@", f"{board_height_m:.6f}")
with output.open("x", encoding="utf-8") as stream:
    stream.write(rendered)
print(output)
PY
