"""Safe, deterministic image/result fixture loading for offline Phase-5 replay."""

from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
from typing import Optional
import xml.etree.ElementTree as ET

from .contract import VisionContractError, load_response


class VisionReplayError(VisionContractError):
    """A replay manifest or local image fixture is invalid."""


@dataclass(frozen=True)
class ReplayCase:
    case_id: str
    display_name: str
    response: dict
    image_path: Optional[Path]
    image_width: Optional[int]
    image_height: Optional[int]
    shapes: tuple


def _safe_fixture_path(root, value, field):
    if not isinstance(value, str) or not value or Path(value).name != value:
        raise VisionReplayError(f"{field} must be a simple fixture filename")
    path = (root/value).resolve()
    if path.parent != root.resolve() or not path.is_file():
        raise VisionReplayError(f"{field} does not identify a local fixture file")
    return path


def _finite_attr(element, name, default=None):
    value = element.get(name)
    if value is None:
        if default is None:
            raise VisionReplayError(f"SVG {element.tag} is missing {name}")
        value = default
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise VisionReplayError(f"SVG {element.tag}.{name} must be numeric") from exc
    if not math.isfinite(number):
        raise VisionReplayError(f"SVG {element.tag}.{name} must be finite")
    return number


def load_svg_scene(path):
    """Parse only local rect/line SVG primitives; reject active/external content."""
    path = Path(path)
    try:
        source = path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as exc:
        raise VisionReplayError(f"cannot read SVG fixture: {exc}") from exc
    if "<!DOCTYPE" in source.upper() or "<!ENTITY" in source.upper():
        raise VisionReplayError("SVG DTD/entity declarations are not permitted")
    try:
        root = ET.fromstring(source)
    except ET.ParseError as exc:
        raise VisionReplayError(f"invalid SVG fixture: {exc}") from exc
    if root.tag.rsplit("}", 1)[-1] != "svg":
        raise VisionReplayError("scene fixture root must be <svg>")
    width, height = _finite_attr(root, "width"), _finite_attr(root, "height")
    if width <= 0 or height <= 0 or int(width) != width or int(height) != height:
        raise VisionReplayError("SVG dimensions must be positive integer pixels")

    shapes = []
    for element in root.iter():
        kind = element.tag.rsplit("}", 1)[-1]
        if kind == "svg":
            continue
        if kind not in ("rect", "line"):
            raise VisionReplayError(f"unsupported SVG element: {kind}")
        fill, stroke = element.get("fill", "none"), element.get("stroke", "none")
        for color in (fill, stroke):
            if color != "none" and not re.fullmatch(r"#[0-9a-fA-F]{6}", color):
                raise VisionReplayError("SVG colors must be #RRGGBB or none")
        stroke_width = _finite_attr(element, "stroke-width", 1.0)
        if stroke_width <= 0 or stroke_width > 32:
            raise VisionReplayError("SVG stroke-width is outside safe bounds")
        if kind == "rect":
            shape = {
                "kind": kind,
                "x": _finite_attr(element, "x", 0),
                "y": _finite_attr(element, "y", 0),
                "width": _finite_attr(element, "width"),
                "height": _finite_attr(element, "height"),
                "fill": fill,
                "stroke": stroke,
                "stroke_width": stroke_width,
            }
            if (shape["x"] < 0 or shape["y"] < 0 or shape["width"] < 0
                    or shape["height"] < 0 or shape["x"]+shape["width"] > width
                    or shape["y"]+shape["height"] > height):
                raise VisionReplayError("SVG rectangle falls outside the declared image")
        else:
            shape = {
                "kind": kind,
                "x1": _finite_attr(element, "x1"),
                "y1": _finite_attr(element, "y1"),
                "x2": _finite_attr(element, "x2"),
                "y2": _finite_attr(element, "y2"),
                "stroke": stroke,
                "stroke_width": stroke_width,
            }
            if not (0 <= shape["x1"] <= width and 0 <= shape["x2"] <= width
                    and 0 <= shape["y1"] <= height and 0 <= shape["y2"] <= height):
                raise VisionReplayError("SVG line falls outside the declared image")
        shapes.append(shape)
    if not shapes:
        raise VisionReplayError("SVG scene has no supported image primitives")
    return int(width), int(height), tuple(shapes)


def load_replay_case(manifest_path, case_id):
    """Load one manifest entry and require image and capture dimensions to agree."""
    manifest_path = Path(manifest_path).resolve()
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise VisionReplayError(f"cannot load replay manifest: {exc}") from exc
    if not isinstance(manifest, dict) or set(manifest) != {"schema_version", "cases"}:
        raise VisionReplayError("replay manifest must contain schema_version and cases")
    if manifest["schema_version"] != "1.0" or not isinstance(manifest["cases"], dict):
        raise VisionReplayError("unsupported replay manifest version or invalid cases")
    entry = manifest["cases"].get(case_id)
    if not isinstance(entry, dict) or set(entry) != {"display_name", "image", "response"}:
        raise VisionReplayError(f"unknown or invalid replay case {case_id!r}")
    display_name = entry["display_name"]
    if not isinstance(display_name, str) or not display_name.strip():
        raise VisionReplayError("display_name must be a non-empty string")
    response_path = _safe_fixture_path(manifest_path.parent, entry["response"], "response")
    response = load_response(response_path)
    image_path, image_width, image_height, shapes = None, None, None, ()
    if entry["image"] is not None:
        image_path = _safe_fixture_path(manifest_path.parent, entry["image"], "image")
        image_width, image_height, shapes = load_svg_scene(image_path)
        if response["status"] == "ok":
            capture = response["result"]["capture"]
            if (capture["width"], capture["height"]) != (image_width, image_height):
                raise VisionReplayError("image fixture dimensions do not match response capture metadata")
    return ReplayCase(case_id, display_name, response, image_path,
                      image_width, image_height, shapes)
