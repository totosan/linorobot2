"""Versioned, transport-neutral vision result validation and fixture mock."""

import copy
import json
import math
from pathlib import Path
import re


SCHEMA_VERSION = "1.0"
CLOCK_DOMAINS = frozenset({"ros_sim", "unix_utc"})
_RESPONSE_KEYS = {"schema_version", "status", "result", "error"}
_RESULT_KEYS = {"capture", "model", "detections"}
_CAPTURE_KEYS = {"stamp_ns", "clock_domain", "frame_id", "width", "height"}
_MODEL_KEYS = {"name", "version", "labels_version"}
_DETECTION_KEYS = {"class_id", "label", "confidence", "bbox_xyxy_px"}
_ERROR_KEYS = {"code", "message"}


class VisionContractError(ValueError):
    """A payload or clock comparison violates contract v1."""


class VisionNoResultError(VisionContractError):
    """The backend reported an error or no available result."""


class VisionClockDomainError(VisionContractError):
    """Ages cannot be compared across simulation and wall-clock domains."""


class VisionStaleResultError(VisionContractError):
    """A valid result exceeds the consumer's maximum age."""


class VisionFutureResultError(VisionContractError):
    """A result timestamp is later than the consumer's current time."""


def _keys(value, expected, optional=(), field="object"):
    if not isinstance(value, dict):
        raise VisionContractError(f"{field} must be an object")
    missing = expected-set(value)
    unknown = set(value)-expected-set(optional)
    if missing or unknown:
        raise VisionContractError(
            f"{field} keys mismatch: missing={sorted(missing)}, unknown={sorted(unknown)}")


def _integer(value, field, minimum=0):
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise VisionContractError(f"{field} must be an integer >= {minimum}")


def _text(value, field):
    if not isinstance(value, str) or not value.strip():
        raise VisionContractError(f"{field} must be a non-empty string")


def _number(value, field):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise VisionContractError(f"{field} must be a finite number")
    try:
        number = float(value)
    except (OverflowError, ValueError) as exc:
        raise VisionContractError(f"{field} must be a finite number") from exc
    if not math.isfinite(number):
        raise VisionContractError(f"{field} must be a finite number")
    return number


def validate_response(response):
    """Validate and return a detached copy of a v1 response.

    Successful responses may contain zero detections. Pixel boxes use
    ``[x_min, y_min, x_max, y_max]`` with exclusive right/bottom bounds.
    The contract deliberately carries no 3D position or actuator command.
    """
    _keys(response, _RESPONSE_KEYS, field="response")
    if not isinstance(response["schema_version"], str) or response["schema_version"] != SCHEMA_VERSION:
        raise VisionContractError(
            f"unsupported schema_version {response['schema_version']!r}; expected {SCHEMA_VERSION}")
    status = response["status"]
    if not isinstance(status, str) or status not in ("ok", "error", "unavailable"):
        raise VisionContractError("status must be 'ok', 'error' or 'unavailable'")

    if status == "ok":
        if response["error"] is not None:
            raise VisionContractError("successful response must have error=null")
        result = response["result"]
        _keys(result, _RESULT_KEYS, field="result")

        capture = result["capture"]
        _keys(capture, _CAPTURE_KEYS, field="capture")
        _integer(capture["stamp_ns"], "capture.stamp_ns")
        if (not isinstance(capture["clock_domain"], str)
            or capture["clock_domain"] not in CLOCK_DOMAINS):
            raise VisionContractError(
                f"capture.clock_domain must be one of {sorted(CLOCK_DOMAINS)}")
        _text(capture["frame_id"], "capture.frame_id")
        _integer(capture["width"], "capture.width", minimum=1)
        _integer(capture["height"], "capture.height", minimum=1)

        model = result["model"]
        _keys(model, _MODEL_KEYS, field="model")
        for field in _MODEL_KEYS:
            _text(model[field], f"model.{field}")

        detections = result["detections"]
        if not isinstance(detections, list):
            raise VisionContractError("result.detections must be an array")
        width, height = capture["width"], capture["height"]
        for index, detection in enumerate(detections):
            field = f"result.detections[{index}]"
            _keys(detection, _DETECTION_KEYS, optional={"track_id"}, field=field)
            _integer(detection["class_id"], f"{field}.class_id")
            _text(detection["label"], f"{field}.label")
            confidence = _number(detection["confidence"], f"{field}.confidence")
            if not 0.0 <= confidence <= 1.0:
                raise VisionContractError(f"{field}.confidence must be in [0,1]")
            box = detection["bbox_xyxy_px"]
            if not isinstance(box, list) or len(box) != 4:
                raise VisionContractError(f"{field}.bbox_xyxy_px must contain four numbers")
            x_min, y_min, x_max, y_max = (
                _number(value, f"{field}.bbox_xyxy_px[{position}]")
                for position, value in enumerate(box))
            if not (0 <= x_min < x_max <= width and 0 <= y_min < y_max <= height):
                raise VisionContractError(
                    f"{field}.bbox_xyxy_px must be a non-empty box within image bounds")
            if "track_id" in detection and detection["track_id"] is not None:
                _text(detection["track_id"], f"{field}.track_id")
    else:
        if response["result"] is not None:
            raise VisionContractError("failed response must have result=null")
        error = response["error"]
        _keys(error, _ERROR_KEYS, field="error")
        code = error["code"]
        if not isinstance(code, str) or not re.fullmatch(r"[a-z][a-z0-9_]{0,63}", code):
            raise VisionContractError("error.code must be a stable lowercase identifier")
        _text(error["message"], "error.message")

    return copy.deepcopy(response)


def require_fresh_result(response, now_ns, now_clock_domain, max_age_ns):
    """Return age in ns, refusing stale/future data and cross-clock comparisons."""
    response = validate_response(response)
    if response["status"] != "ok":
        raise VisionNoResultError(
            f"vision result status is {response['status']}: {response['error']['code']}")
    _integer(now_ns, "now_ns")
    _integer(max_age_ns, "max_age_ns")
    if (not isinstance(now_clock_domain, str)
            or now_clock_domain not in CLOCK_DOMAINS):
        raise VisionClockDomainError(f"unsupported current clock domain {now_clock_domain!r}")
    capture = response["result"]["capture"]
    if capture["clock_domain"] != now_clock_domain:
        raise VisionClockDomainError(
            f"cannot compare {capture['clock_domain']} capture with {now_clock_domain} now")
    age_ns = now_ns-capture["stamp_ns"]
    if age_ns < 0:
        raise VisionFutureResultError(f"vision result is {-age_ns} ns in the future")
    if age_ns > max_age_ns:
        raise VisionStaleResultError(
            f"vision result age {age_ns} ns exceeds limit {max_age_ns} ns")
    return age_ns


def load_response(path):
    """Load and validate one UTF-8 JSON response fixture."""
    try:
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise VisionContractError(f"cannot load vision response fixture: {exc}") from exc
    return validate_response(payload)


class FixtureVisionBackend:
    """Deterministic mock backend that returns one validated JSON fixture."""

    def __init__(self, fixture_path):
        self.fixture_path = Path(fixture_path)

    def get_response(self):
        return load_response(self.fixture_path)