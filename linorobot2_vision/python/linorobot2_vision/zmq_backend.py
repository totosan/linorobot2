"""ZeroMQ client for the Jetson ``.71`` detector protocol (and its local sim twin).

Wire protocol: PUSH one encoded image to the image port, then REQ ``"detect"``
on the reply port and read one JSON reply. Sockets are created per request, so
a timeout can never leave a stale image or a stuck REQ state behind.
"""

import json
import math
from urllib.parse import urlsplit

from .adapters import normalize_zmq_detection_v1
from .contract import VisionContractError
from .service_backend import image_to_ppm

MAX_RESPONSE_BYTES = 1024*1024

COCO80_LABELS = (
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
)


def coco80_label_map():
    return {index: label for index, label in enumerate(COCO80_LABELS)}


def _tcp_endpoint(value, name):
    parsed = urlsplit(value) if isinstance(value, str) else None
    try:
        port = parsed.port if parsed else None
    except ValueError:
        port = None
    if (parsed is None or parsed.scheme != "tcp" or not parsed.hostname
            or port is None or parsed.path not in ("", "/")
            or parsed.username or parsed.password):
        raise ValueError(f"{name} must be tcp://host:port")
    return f"tcp://{parsed.hostname}:{port}"


class VisionZmqBackend:
    """Submit one frame to a ``.71``-protocol detector and normalize the reply."""

    def __init__(self, image_endpoint, reply_endpoint, *, timeout_sec,
                 min_confidence, model_name, model_version, labels_version,
                 class_id_to_label, context=None):
        self.image_endpoint = _tcp_endpoint(image_endpoint, "image endpoint")
        self.reply_endpoint = _tcp_endpoint(reply_endpoint, "reply endpoint")
        if (isinstance(timeout_sec, bool) or not isinstance(timeout_sec, (int, float))
                or not math.isfinite(float(timeout_sec))
                or not 0.1 <= timeout_sec <= 120):
            raise ValueError("zmq timeout_sec must be in [0.1, 120]")
        if (isinstance(min_confidence, bool)
                or not isinstance(min_confidence, (int, float))
                or not math.isfinite(float(min_confidence))
                or not 0 <= min_confidence <= 1):
            raise ValueError("zmq min_confidence must be in [0,1]")
        for value, field in ((model_name, "model_name"),
                             (model_version, "model_version"),
                             (labels_version, "labels_version")):
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{field} must be explicitly configured")
        if not isinstance(class_id_to_label, dict) or not class_id_to_label:
            raise ValueError("zmq class-ID label map must be a non-empty object")
        labels = {}
        for class_id, label in class_id_to_label.items():
            if (isinstance(class_id, bool) or not isinstance(class_id, int)
                    or class_id < 0 or not isinstance(label, str) or not label.strip()
                    or label in labels):
                raise ValueError("zmq class-ID label map is invalid")
            labels[label] = class_id
        self.timeout_sec = float(timeout_sec)
        self.min_confidence = float(min_confidence)
        self.model_name = model_name.strip()
        self.model_version = model_version.strip()
        self.labels_version = labels_version.strip()
        self.label_to_class_id = labels
        self._context = context

    def _zmq(self):
        try:
            import zmq
        except ImportError as exc:
            raise VisionContractError(
                "pyzmq is not installed; the zmq backend is unavailable") from exc
        if self._context is None:
            self._context = zmq.Context()
        return zmq

    def get_response(self, image, capture):
        """Perform one bounded request; never retries."""
        image_bytes = image_to_ppm(image)
        zmq = self._zmq()
        timeout_ms = int(self.timeout_sec*1000)
        push = req = None
        try:
            push = self._context.socket(zmq.PUSH)
            req = self._context.socket(zmq.REQ)
            for sock in (push, req):
                sock.setsockopt(zmq.LINGER, 0)
                sock.setsockopt(zmq.SNDTIMEO, timeout_ms)
                sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
            req.setsockopt(zmq.MAXMSGSIZE, MAX_RESPONSE_BYTES)
            push.connect(self.image_endpoint)
            req.connect(self.reply_endpoint)
            push.send(image_bytes)
            req.send_string("detect")
            reply = req.recv()
        except zmq.error.ZMQError as exc:
            raise VisionContractError(
                f"Vision detector request failed: {type(exc).__name__}: {exc}") from exc
        finally:
            for sock in (push, req):
                if sock is not None:
                    sock.close(linger=0)
        if len(reply) > MAX_RESPONSE_BYTES:
            raise VisionContractError("Vision detector reply exceeded 1 MiB")
        try:
            payload = json.loads(reply)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise VisionContractError("Vision detector reply is not valid JSON") from exc
        return normalize_zmq_detection_v1(
            payload, capture=capture, model_name=self.model_name,
            model_version=self.model_version, labels_version=self.labels_version,
            label_to_class_id=self.label_to_class_id,
            min_confidence=self.min_confidence)
