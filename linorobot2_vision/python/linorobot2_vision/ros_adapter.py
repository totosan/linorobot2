"""ROS-message metadata adapters for the transport-neutral vision contract."""

import json
import re

from .contract import CLOCK_DOMAINS, SCHEMA_VERSION, VisionContractError, validate_response


def _positive_dimension(value, field):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise VisionContractError(f"{field} must be a positive integer")
    return value


def _stamp_ns(header, field):
    stamp = getattr(header, "stamp", None)
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if (isinstance(sec, bool) or not isinstance(sec, int) or sec < 0
            or isinstance(nanosec, bool) or not isinstance(nanosec, int)
            or not 0 <= nanosec < 1_000_000_000):
        raise VisionContractError(f"{field} must contain a valid nonnegative ROS timestamp")
    return sec*1_000_000_000+nanosec


def capture_from_camera_pair(image, camera_info, *, clock_domain,
                             pair_slop_ns=50_000_000, expected_frame_id=""):
    """Validate paired Image/CameraInfo metadata and return contract capture fields."""
    if clock_domain not in CLOCK_DOMAINS:
        raise VisionContractError(
            f"clock_domain must be one of {sorted(CLOCK_DOMAINS)}")
    if (isinstance(pair_slop_ns, bool) or not isinstance(pair_slop_ns, int)
            or pair_slop_ns < 0):
        raise VisionContractError("pair_slop_ns must be a nonnegative integer")

    image_width = _positive_dimension(image.width, "image.width")
    image_height = _positive_dimension(image.height, "image.height")
    info_width = _positive_dimension(camera_info.width, "camera_info.width")
    info_height = _positive_dimension(camera_info.height, "camera_info.height")
    if (image_width, image_height) != (info_width, info_height):
        raise VisionContractError("Image and CameraInfo dimensions do not match")

    image_frame = getattr(image.header, "frame_id", "")
    info_frame = getattr(camera_info.header, "frame_id", "")
    if not isinstance(image_frame, str) or not image_frame.strip():
        raise VisionContractError("image.header.frame_id must be non-empty")
    if image_frame != info_frame:
        raise VisionContractError("Image and CameraInfo frame IDs do not match")
    if expected_frame_id and image_frame != expected_frame_id:
        raise VisionContractError(
            f"camera frame {image_frame!r} does not match expected {expected_frame_id!r}")

    image_stamp_ns = _stamp_ns(image.header, "image.header.stamp")
    info_stamp_ns = _stamp_ns(camera_info.header, "camera_info.header.stamp")
    if abs(image_stamp_ns-info_stamp_ns) > pair_slop_ns:
        raise VisionContractError("Image and CameraInfo timestamps exceed pair slop")

    return {
        "stamp_ns": image_stamp_ns,
        "clock_domain": clock_domain,
        "frame_id": image_frame,
        "width": image_width,
        "height": image_height,
    }


def parse_label_map(serialized, *, allow_empty=False):
    """Parse a reviewed JSON map from string class IDs to canonical labels.

    Empty maps are rejected unless the caller explicitly allows them for a
    no-detection service probe; any returned unmapped label still fails closed.
    """
    if not isinstance(serialized, str):
        raise VisionContractError("label_map_json must be a JSON object string")
    try:
        raw = json.loads(serialized)
    except json.JSONDecodeError as exc:
        raise VisionContractError(f"label_map_json is invalid JSON: {exc}") from exc
    if not isinstance(raw, dict) or (not raw and not allow_empty):
        qualifier = "an object" if allow_empty else "a non-empty object"
        raise VisionContractError(f"label_map_json must be {qualifier}")

    label_map = {}
    for raw_id, label in raw.items():
        if not isinstance(raw_id, str) or not re.fullmatch(r"0|[1-9][0-9]*", raw_id):
            raise VisionContractError("label_map_json keys must be nonnegative class IDs")
        if not isinstance(label, str) or not label.strip():
            raise VisionContractError("label_map_json values must be non-empty labels")
        label_map[int(raw_id)] = label
    return label_map


def bind_fixture_capture(response, capture, class_id_to_label):
    """Bind mock detections to observed capture metadata and enforce reviewed labels."""
    response = validate_response(response)
    if response["status"] != "ok":
        return response
    if not isinstance(class_id_to_label, dict):
        raise VisionContractError("class_id_to_label must be an explicit mapping")

    result = response["result"]
    result["capture"] = dict(capture)
    for index, detection in enumerate(result["detections"]):
        expected_label = class_id_to_label.get(detection["class_id"])
        if expected_label is None:
            raise VisionContractError(
                f"no reviewed label for class ID {detection['class_id']}")
        if detection["label"] != expected_label:
            raise VisionContractError(
                f"label for class ID {detection['class_id']} does not match reviewed map")
    return validate_response(response)


def error_response(code, message):
    """Create a valid contract-v1 error response with a stable machine code."""
    return validate_response({
        "schema_version": SCHEMA_VERSION,
        "status": "error",
        "result": None,
        "error": {"code": code, "message": str(message) or code},
    })


def response_for_outcome(response, outcome):
    """Keep valid contract responses, or encode consumer failures as v1 errors."""
    response = validate_response(response)
    if outcome.state in ("ok", "empty") and response["status"] == "ok":
        return response
    if outcome.state == "unavailable" and response["status"] == "unavailable":
        return response
    if response["status"] in ("error", "unavailable"):
        return response
    return error_response(
        outcome.error_code or "vision_result_invalid",
        outcome.message or outcome.state)


def status_payload(outcome, capture=None):
    """Return a small JSON-serializable diagnostic independent of ROS types."""
    return {
        "state": outcome.state,
        "error_code": outcome.error_code,
        "message": outcome.message,
        "age_ns": outcome.age_ns,
        "detection_count": len(outcome.detections),
        "capture": capture,
    }
