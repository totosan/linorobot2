"""Adapters from explicitly identified source payloads into contract v1."""

from .contract import SCHEMA_VERSION, VisionContractError, validate_response


_CAPTURE_KEYS = {"stamp_ns", "clock_domain", "frame_id", "width", "height"}


def normalize_jetson_detection_v1(payload, *, capture, model_version,
                                 labels_version, label_to_class_id):
    """Normalize the archived Jetson ``/v1/vision/detection`` response shape.

    The archived endpoint's ``timestampUTC`` is generated at response time, not
    at image capture. It is intentionally ignored. Callers must pass the
    original image's capture timestamp/domain, optical frame and dimensions;
    class IDs must come from an explicitly reviewed label mapping.
    """
    if not isinstance(payload, dict):
        raise VisionContractError("source payload must be an object")
    if type(payload.get("success")) is not bool:
        raise VisionContractError("source success must be a boolean")

    if not payload["success"]:
        message = payload.get("error") or payload.get("message")
        if not isinstance(message, str) or not message.strip():
            message = "Source reported detection failure without a message"
        return validate_response({
            "schema_version": SCHEMA_VERSION,
            "status": "error",
            "result": None,
            "error": {"code": "source_detection_failed", "message": message},
        })

    if not isinstance(capture, dict) or set(capture) != _CAPTURE_KEYS:
        raise VisionContractError(
            f"capture context must contain exactly {sorted(_CAPTURE_KEYS)}")
    if not isinstance(label_to_class_id, dict):
        raise VisionContractError("label_to_class_id must be an explicit mapping")
    module_id = payload.get("moduleId")
    if not isinstance(module_id, str) or not module_id.strip():
        raise VisionContractError("source moduleId must be a non-empty string")
    predictions = payload.get("predictions")
    if not isinstance(predictions, list):
        raise VisionContractError("source predictions must be an array")

    detections = []
    for index, prediction in enumerate(predictions):
        field = f"source.predictions[{index}]"
        if not isinstance(prediction, dict):
            raise VisionContractError(f"{field} must be an object")
        label = prediction.get("label")
        if not isinstance(label, str) or not label.strip():
            raise VisionContractError(f"{field}.label must be a non-empty string")
        if label not in label_to_class_id:
            raise VisionContractError(f"no reviewed class ID for source label {label!r}")
        class_id = label_to_class_id[label]
        if isinstance(class_id, bool) or not isinstance(class_id, int) or class_id < 0:
            raise VisionContractError(f"class ID for source label {label!r} must be >= 0")
        try:
            box = [prediction[key] for key in ("x_min", "y_min", "x_max", "y_max")]
            confidence = prediction["confidence"]
        except KeyError as exc:
            raise VisionContractError(f"{field} is missing {exc.args[0]}") from exc
        detections.append({
            "class_id": class_id,
            "label": label,
            "confidence": confidence,
            "bbox_xyxy_px": box,
        })

    return validate_response({
        "schema_version": SCHEMA_VERSION,
        "status": "ok",
        "result": {
            "capture": dict(capture),
            "model": {
                "name": module_id,
                "version": model_version,
                "labels_version": labels_version,
            },
            "detections": detections,
        },
        "error": None,
    })

def normalize_zmq_detection_v1(payload, *, capture, model_name, model_version,
                               labels_version, label_to_class_id,
                               min_confidence=0.0):
    """Normalize the Jetson ``.71`` ZeroMQ ``detect`` reply into contract v1.

    The reply is ``{"detections": [{"box": [x1,y1,x2,y2], "label", "confidence"}]}``
    or ``{"error": str}``. It carries no capture time, class ID or model
    version; those come from the caller. Boxes are clamped to the image because
    the source emits unrounded floats. Unknown labels fail closed.
    """
    if not isinstance(payload, dict):
        raise VisionContractError("source payload must be an object")
    if "error" in payload:
        message = payload["error"]
        if not isinstance(message, str) or not message.strip():
            message = "Source reported detection failure without a message"
        return validate_response({
            "schema_version": SCHEMA_VERSION,
            "status": "error",
            "result": None,
            "error": {"code": "source_detection_failed", "message": message},
        })
    if not isinstance(capture, dict) or set(capture) != _CAPTURE_KEYS:
        raise VisionContractError(
            f"capture context must contain exactly {sorted(_CAPTURE_KEYS)}")
    if not isinstance(label_to_class_id, dict):
        raise VisionContractError("label_to_class_id must be an explicit mapping")
    raw = payload.get("detections")
    if not isinstance(raw, list):
        raise VisionContractError("source detections must be an array")

    width, height = capture["width"], capture["height"]
    detections = []
    for index, item in enumerate(raw):
        field = f"source.detections[{index}]"
        if not isinstance(item, dict):
            raise VisionContractError(f"{field} must be an object")
        label = item.get("label")
        if not isinstance(label, str) or not label.strip():
            raise VisionContractError(f"{field}.label must be a non-empty string")
        if label not in label_to_class_id:
            raise VisionContractError(f"no reviewed class ID for source label {label!r}")
        box = item.get("box")
        confidence = item.get("confidence")
        if (not isinstance(box, list) or len(box) != 4
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or v != v or v in (float("inf"), float("-inf")) for v in box)):
            raise VisionContractError(f"{field}.box must contain four finite numbers")
        if (isinstance(confidence, bool) or not isinstance(confidence, (int, float))
                or not 0.0 <= confidence <= 1.0):
            raise VisionContractError(f"{field}.confidence must be a number in [0,1]")
        if confidence < min_confidence:
            continue
        x_min, y_min, x_max, y_max = (float(v) for v in box)
        clamped = [min(max(x_min, 0.0), float(width)),
                   min(max(y_min, 0.0), float(height)),
                   min(max(x_max, 0.0), float(width)),
                   min(max(y_max, 0.0), float(height))]
        if clamped[0] >= clamped[2] or clamped[1] >= clamped[3]:
            raise VisionContractError(f"{field}.box is empty after clamping to the image")
        detections.append({
            "class_id": label_to_class_id[label],
            "label": label,
            "confidence": float(confidence),
            "bbox_xyxy_px": clamped,
        })

    return validate_response({
        "schema_version": SCHEMA_VERSION,
        "status": "ok",
        "result": {
            "capture": dict(capture),
            "model": {"name": model_name, "version": model_version,
                      "labels_version": labels_version},
            "detections": detections,
        },
        "error": None,
    })
