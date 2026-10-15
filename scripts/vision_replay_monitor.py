#!/usr/bin/env python3
"""Interactive, offline-only terminal visualization for synthetic vision fixtures."""

from pathlib import Path
import select
import sys
import termios
import tty

WORKSPACE = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = WORKSPACE/"linorobot2_vision"
sys.path.insert(0, str(PACKAGE_ROOT/"python"))

from linorobot2_vision import FixtureVisionBackend  # noqa: E402

FIXTURES = {
    "1": ("detection", "detection.json"),
    "2": ("empty", "empty.json"),
    "3": ("unavailable", "unavailable.json"),
}
CANVAS_WIDTH = 64
CANVAS_HEIGHT = 20


def safe_text(value):
    """Keep fixture-controlled labels from injecting terminal control codes."""
    return "".join(char if char.isprintable() else "?" for char in str(value))


def scene_rows(response, width=CANVAS_WIDTH, height=CANVAS_HEIGHT):
    """Render contract bounding boxes on a coarse, deterministic text canvas."""
    canvas = [["." for _ in range(width)] for _ in range(height)]
    if response["status"] != "ok":
        return ["".join(row) for row in canvas]

    capture = response["result"]["capture"]
    image_width, image_height = capture["width"], capture["height"]
    for detection in response["result"]["detections"]:
        x_min, y_min, x_max, y_max = detection["bbox_xyxy_px"]
        left = max(0, min(width-2, int(x_min*width/image_width)))
        right = max(left+1, min(width-1, int((x_max-1)*width/image_width)))
        top = max(0, min(height-2, int(y_min*height/image_height)))
        bottom = max(top+1, min(height-1, int((y_max-1)*height/image_height)))
        for x in range(left+1, right):
            canvas[top][x] = "-"
            canvas[bottom][x] = "-"
        for y in range(top+1, bottom):
            canvas[y][left] = "|"
            canvas[y][right] = "|"
            for x in range(left+1, right):
                canvas[y][x] = "#"
        for x, y in ((left, top), (right, top), (left, bottom), (right, bottom)):
            canvas[y][x] = "+"
    return ["".join(row) for row in canvas]


def render_dashboard(response, fixture_name):
    """Return plain terminal text so tests and GUI colors stay separate."""
    lines = [
        "LINOROBOT2 | VISION FIXTURE REPLAY | OFFLINE MOCK ONLY",
        "Synthetic scene; no camera, network, ROS topic, or model inference.",
        "="*76,
        f"Fixture: {fixture_name}    Schema: {response['schema_version']}",
        f"Status: {response['status'].upper()}",
    ]
    if response["status"] == "ok":
        result = response["result"]
        capture = result["capture"]
        model = result["model"]
        lines.extend((
            f"Capture: {capture['stamp_ns']} ns [{capture['clock_domain']}]",
            f"Frame: {safe_text(capture['frame_id'])} | {capture['width']}x{capture['height']}",
            f"Model: {safe_text(model['name'])} {safe_text(model['version'])} | labels {safe_text(model['labels_version'])}",
            f"Detections: {len(result['detections'])}",
            "",
            "Coarse image view (. background, # box interior, +/-/| box outline):",
            "+"+"-"*CANVAS_WIDTH+"+",
        ))
        lines.extend("|"+row+"|" for row in scene_rows(response))
        lines.append("+"+"-"*CANVAS_WIDTH+"+")
        for index, detection in enumerate(result["detections"], 1):
            box = ", ".join(f"{value:g}" for value in detection["bbox_xyxy_px"])
            lines.append(
                f"  {index}. {safe_text(detection['label'])} "
                f"conf={detection['confidence']:.3f} "
                f"class={detection['class_id']} box=[{box}]")
        if not result["detections"]:
            lines.append("No detections: this is a valid, successful empty result.")
    else:
        error = response["error"]
        lines.extend((
            "",
            "No image/capture is shown for this failure fixture.",
            f"Error: {safe_text(error['code'])} — {safe_text(error['message'])}",
        ))
    lines.extend(("", "Keys: [1] detection  [2] empty  [3] unavailable  [q] quit"))
    return "\n".join(lines)


def load_fixture(key):
    name, filename = FIXTURES[key]
    backend = FixtureVisionBackend(PACKAGE_ROOT/"fixtures"/filename)
    return name, backend.get_response()


def main():
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        print("This monitor needs an interactive XFCE terminal on the RDP desktop.",
              file=sys.stderr)
        return 2

    key = "1"
    name, response = load_fixture(key)
    fd = sys.stdin.fileno()
    previous = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        def redraw():
            nonlocal name, response
            status_color = {"ok": "\033[32m", "error": "\033[31m",
                            "unavailable": "\033[33m"}.get(response["status"], "")
            dashboard = render_dashboard(response, name)
            dashboard = dashboard.replace(
                f"Status: {response['status'].upper()}",
                f"Status: {status_color}{response['status'].upper()}\033[0m")
            sys.stdout.write("\033[2J\033[H\033[1;36m"+dashboard+"\033[0m\n")
            sys.stdout.flush()

        redraw()
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 0.5)
            if not ready:
                continue
            pressed = sys.stdin.read(1).lower()
            if pressed == "q":
                break
            if pressed in FIXTURES:
                name, response = load_fixture(pressed)
                redraw()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, previous)
        sys.stdout.write("\033[0m\033[2J\033[HVision fixture monitor closed.\n")
        sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
