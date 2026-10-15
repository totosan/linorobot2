"""Capture-matched 2D detection boxes drawn on camera images.

This is an image-plane overlay only. Contract-v1 carries pixel boxes, not 3D
positions, so this module must not be used to place objects in a world frame.
"""

import math

from .contract import VisionContractError, validate_response


_PIXEL_FORMATS = {
    "rgb8": ((255, 32, 32), 3),
    "bgr8": ((32, 32, 255), 3),
    "rgba8": ((255, 32, 32, 255), 4),
    "bgra8": ((32, 32, 255, 255), 4),
}


def boxes_for_capture(response, *, stamp_ns, frame_id, width, height):
    """Return detections only when result metadata exactly matches the image.

    ``None`` means no usable successful result for this image; an empty list is
    a valid successful result with no detections.
    """
    response = validate_response(response)
    if response["status"] != "ok":
        return None

    result = response["result"]
    capture = result["capture"]
    if (capture["clock_domain"] != "ros_sim"
            or capture["stamp_ns"] != stamp_ns
            or capture["frame_id"] != frame_id
            or capture["width"] != width
            or capture["height"] != height):
        return None
    return [detection["bbox_xyxy_px"] for detection in result["detections"]]


def draw_bbox_overlay(data, *, width, height, step, encoding, boxes,
                      thickness=3):
    """Copy an RGB/BGR image and draw red pixel-space box outlines."""
    if encoding not in _PIXEL_FORMATS:
        raise ValueError(f"unsupported image encoding {encoding!r}")
    color, channels = _PIXEL_FORMATS[encoding]
    for name, value in (("width", width), ("height", height),
                        ("step", step), ("thickness", thickness)):
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ValueError(f"{name} must be a positive integer")
    row_bytes = width * channels
    if step < row_bytes or len(data) < step * height:
        raise ValueError("image data is shorter than its declared dimensions and step")

    pixels = bytearray(data)

    def set_pixel(x, y):
        offset = y * step + x * channels
        pixels[offset:offset + channels] = bytes(color)

    for box in boxes:
        if not isinstance(box, (list, tuple)) or len(box) != 4:
            raise ValueError("each box must contain x_min, y_min, x_max, y_max")
        coordinates = []
        for coordinate in box:
            if isinstance(coordinate, bool) or not isinstance(coordinate, (int, float)):
                raise ValueError("box coordinates must be finite numbers")
            if not math.isfinite(float(coordinate)):
                raise ValueError("box coordinates must be finite numbers")
            coordinates.append(float(coordinate))
        x_min, y_min, x_max, y_max = coordinates
        if x_min >= x_max or y_min >= y_max:
            raise ValueError("box must have positive width and height")

        left = max(0, min(width - 1, math.floor(x_min)))
        top = max(0, min(height - 1, math.floor(y_min)))
        right = max(0, min(width - 1, math.ceil(x_max) - 1))
        bottom = max(0, min(height - 1, math.ceil(y_max) - 1))
        if left > right or top > bottom:
            continue

        for inset in range(thickness):
            x0, x1 = left + inset, right - inset
            y0, y1 = top + inset, bottom - inset
            if x0 > x1 or y0 > y1:
                break
            for x in range(x0, x1 + 1):
                set_pixel(x, y0)
                set_pixel(x, y1)
            for y in range(y0 + 1, y1):
                set_pixel(x0, y)
                set_pixel(x1, y)

    return bytes(pixels)
