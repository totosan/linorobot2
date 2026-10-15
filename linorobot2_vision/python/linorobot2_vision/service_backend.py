"""HTTP client for the archived Jetson object-detection endpoint."""

import json
import math
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen
import uuid

from .adapters import normalize_jetson_detection_v1
from .contract import VisionContractError


MAX_UPLOAD_BYTES = 10*1024*1024
MAX_RESPONSE_BYTES = 1024*1024


def image_to_ppm(image):
    """Encode supported ROS Image payloads as binary PPM without disk I/O."""
    width, height = image.width, image.height
    channels_by_encoding = {"rgb8": 3, "bgr8": 3, "rgba8": 4,
                            "bgra8": 4, "mono8": 1}
    channels = channels_by_encoding.get(image.encoding)
    if channels is None:
        raise VisionContractError(f"unsupported camera encoding {image.encoding!r}")
    row_bytes = width*channels
    if width < 1 or height < 1 or image.step < row_bytes:
        raise VisionContractError("camera image dimensions/step are invalid")
    data = bytes(image.data)
    if len(data) != image.step*height:
        raise VisionContractError("camera image payload length does not match step*height")

    rgb = bytearray(width*height*3)
    output_offset = 0
    for row_index in range(height):
        row = data[row_index*image.step:row_index*image.step+row_bytes]
        if channels == 1:
            for value in row:
                rgb[output_offset:output_offset+3] = bytes((value, value, value))
                output_offset += 3
        elif image.encoding in ("rgb8", "rgba8"):
            for pixel_offset in range(0, row_bytes, channels):
                rgb[output_offset:output_offset+3] = row[pixel_offset:pixel_offset+3]
                output_offset += 3
        else:
            for pixel_offset in range(0, row_bytes, channels):
                rgb[output_offset:output_offset+3] = row[pixel_offset:pixel_offset+3][::-1]
                output_offset += 3

    ppm = f"P6\n{width} {height}\n255\n".encode("ascii")+rgb
    if len(ppm) > MAX_UPLOAD_BYTES:
        raise VisionContractError("encoded image exceeds the service upload limit")
    return ppm


def _multipart(image_bytes, min_confidence):
    boundary = "----linorobot2-vision-"+uuid.uuid4().hex
    body = b"".join((
        f"--{boundary}\r\n".encode("ascii"),
        b'Content-Disposition: form-data; name="image"; filename="camera.ppm"\r\n',
        b"Content-Type: image/x-portable-pixmap\r\n\r\n",
        image_bytes,
        b"\r\n",
        f"--{boundary}\r\n".encode("ascii"),
        b'Content-Disposition: form-data; name="min_confidence"\r\n\r\n',
        f"{min_confidence:.3f}".encode("ascii"),
        b"\r\n",
        f"--{boundary}--\r\n".encode("ascii"),
    ))
    return boundary, body


class VisionHTTPServiceBackend:
    """Submit one frame and normalize the service response into Contract v1."""

    def __init__(self, endpoint_url, *, timeout_sec, min_confidence,
                 model_version, labels_version, class_id_to_label):
        parsed = urlsplit(endpoint_url)
        if (parsed.scheme not in ("http", "https") or not parsed.hostname
                or parsed.username is not None or parsed.password is not None):
            raise ValueError("service endpoint must be an HTTP(S) URL without embedded credentials")
        if (isinstance(timeout_sec, bool) or not isinstance(timeout_sec, (int, float))
            or not math.isfinite(float(timeout_sec))
            or not 0.1 <= timeout_sec <= 60):
            raise ValueError("service timeout_sec must be in [0.1, 60]")
        if (isinstance(min_confidence, bool)
            or not isinstance(min_confidence, (int, float))
            or not math.isfinite(float(min_confidence))
                or not 0 <= min_confidence <= 1):
            raise ValueError("service min_confidence must be in [0,1]")
        for value, field in ((model_version, "model_version"),
                             (labels_version, "labels_version")):
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{field} must be explicitly configured")
        if not isinstance(class_id_to_label, dict):
            raise ValueError("service class-ID label map must be an object")
        labels = {}
        for class_id, label in class_id_to_label.items():
            if (isinstance(class_id, bool) or not isinstance(class_id, int)
                    or class_id < 0 or not isinstance(label, str) or not label.strip()):
                raise ValueError("service class-ID label map is invalid")
            if label in labels:
                raise ValueError("service class-ID label map labels must be unique")
            labels[label] = class_id

        self.endpoint_url = endpoint_url
        self.timeout_sec = float(timeout_sec)
        self.min_confidence = float(min_confidence)
        self.model_version = model_version.strip()
        self.labels_version = labels_version.strip()
        self.label_to_class_id = labels

    def get_response(self, image, capture):
        """Perform one bounded HTTP request for one validated camera capture."""
        image_bytes = image_to_ppm(image)
        boundary, body = _multipart(image_bytes, self.min_confidence)
        request = Request(
            self.endpoint_url, data=body, method="POST",
            headers={"Content-Type": f"multipart/form-data; boundary={boundary}",
                     "Accept": "application/json"})
        try:
            with urlopen(request, timeout=self.timeout_sec) as response:
                response_body = response.read(MAX_RESPONSE_BYTES+1)
        except HTTPError as exc:
            response_body = exc.read(MAX_RESPONSE_BYTES+1)
            try:
                payload = json.loads(response_body)
            except (UnicodeDecodeError, json.JSONDecodeError):
                payload = None
            if isinstance(payload, dict) and payload.get("success") is False:
                return normalize_jetson_detection_v1(
                    payload, capture=capture, model_version=self.model_version,
                    labels_version=self.labels_version,
                    label_to_class_id=self.label_to_class_id)
            raise VisionContractError(f"Vision service returned HTTP {exc.code}") from exc
        except (URLError, TimeoutError, OSError) as exc:
            raise VisionContractError(
                f"Vision service request failed: {type(exc).__name__}") from exc

        if len(response_body) > MAX_RESPONSE_BYTES:
            raise VisionContractError("Vision service response exceeded 1 MiB")
        try:
            payload = json.loads(response_body)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise VisionContractError("Vision service response is not valid JSON") from exc
        return normalize_jetson_detection_v1(
            payload, capture=capture, model_version=self.model_version,
            labels_version=self.labels_version,
            label_to_class_id=self.label_to_class_id)
