#!/usr/bin/env python3
"""Remaining G1 camera ABA / GUI-response evidence, not the entire G1 gate.

Run on Linux in the harness's isolated Humble/Fortress environment. Only the
static test_obstacle pose is changed; its observed pose is restored in finally.
ROS and gui_x11_probe are runtime-only dependencies. --inspect never writes a
pose, requests a screenshot, or imports/subscribes to ROS.
"""

import argparse
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import struct
import subprocess
import sys
import tempfile
import time
import uuid
import zlib


WORLD = "fortress_test"
OBSTACLE = "test_obstacle"
POSE_SERVICE = f"/world/{WORLD}/set_pose"
SCREENSHOT_SERVICE = "/gui/screenshot"
POSE_TOPIC = f"/world/{WORLD}/pose/info"
ORIGINAL = {"position": (2.0, 0.0, 0.5), "orientation": (0.0, 0.0, 0.0, 1.0)}
OUTSIDE = {"position": (2.0, 3.0, 0.5), "orientation": (0.0, 0.0, 0.0, 1.0)}
TOPICS = {"clock": "/clock", "scan": "/scan", "imu": "/imu/data",
          "image": "/camera/image_raw", "info": "/camera/camera_info"}
SIZE = (640, 480)
FRESH_SECONDS = 10.0
STAGE_SECONDS = 30.0
SETTLE_NS = 200_000_000


def stamp_ns(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("invalid sensor timestamp")
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def require_increasing(previous, current):
    if previous is not None and current <= previous:
        raise ValueError(f"image timestamps not strictly increasing: {previous} -> {current}")


def normalize_rgb(msg):
    """Remove row padding, never mistake three byte values for RGB diversity."""
    if (msg.width, msg.height) != SIZE or msg.encoding != "rgb8":
        raise ValueError("camera must deliver 640x480 rgb8")
    row = msg.width * 3
    if msg.step < row or not msg.header.frame_id:
        raise ValueError("invalid camera step/frame")
    data = bytes(msg.data)
    if len(data) != msg.step * msg.height:
        raise ValueError("image payload length does not match step * height")
    return b"".join(data[y*msg.step:y*msg.step + row] for y in range(msg.height))


def rgb_statistics(rgb, deadline=None):
    if not rgb or len(rgb) % 3:
        raise ValueError("invalid packed RGB pixels")
    colors, red = set(), 0
    for i in range(0, len(rgb), 3):
        if deadline is not None and i % 12288 == 0 and time.monotonic() >= deadline:
            raise RuntimeError("RGB analysis stage deadline exceeded")
        r, g, b = rgb[i:i+3]
        if len(colors) < 256:
            colors.add((r, g, b))
        red += r > g + 20 and r > b + 20 and r > 50
    if len(colors) < 2:
        raise ValueError("uniform RGB frame is blank (including solid red)")
    return {"pixels": len(rgb)//3, "rgb_colors_capped_256": len(colors),
            "red_pixels": red, "sha256": hashlib.sha256(rgb).hexdigest()}


def compare_rgb(first, second, deadline=None):
    if not first or len(first) != len(second) or len(first) % 3:
        raise ValueError("comparison requires equally sized packed RGB images")
    changed, absolute = 0, 0
    for i in range(0, len(first), 3):
        if deadline is not None and i % 12288 == 0 and time.monotonic() >= deadline:
            raise RuntimeError("RGB comparison stage deadline exceeded")
        diffs = [abs(first[i+j] - second[i+j]) for j in range(3)]
        changed += max(diffs) >= 20
        absolute += sum(diffs)
    return {"changed_ratio": changed / (len(first)//3),
            "mean_absolute_rgb_difference": absolute / len(first)}


def camera_evidence(original, outside, returned=None, deadline=None):
    """All 3x3 comparisons must pass; hashes are diagnostic only."""
    if len(original) < 3 or len(outside) < 3 or (returned is not None and len(returned) < 3):
        raise ValueError("need at least three fresh images in every camera state")
    a = [rgb_statistics(x, deadline) for x in original]
    b = [rgb_statistics(x, deadline) for x in outside]
    changes = [compare_rgb(x, y, deadline) for x in original for y in outside]
    red_a = sum(s["red_pixels"] for s in a) / len(a)
    red_b = sum(s["red_pixels"] for s in b) / len(b)
    red_drop = (red_a - red_b) / red_a if red_a else 0.0
    result = {"comparison": "full image, row padding removed, max-channel threshold 20",
              "outward": changes, "red_original_mean": red_a,
              "red_outside_mean": red_b, "red_decrease_ratio": red_drop,
              "outward_pass": all(s["changed_ratio"] >= 0.01 and
                                  s["mean_absolute_rgb_difference"] >= 1.0 for s in changes)
                              and red_a >= a[0]["pixels"] * 0.01 and red_drop >= 0.20}
    if returned is not None:
        for image in returned:
            rgb_statistics(image, deadline)
        result["return"] = [compare_rgb(x, y, deadline) for x in original for y in returned]
        result["return_pass"] = all(s["changed_ratio"] <= 0.05 for s in result["return"])
    return result


def validate_info(msg, frame):
    k, p = tuple(msg.k), tuple(msg.p)
    if ((msg.width, msg.height) != SIZE or not frame or msg.header.frame_id != frame
            or len(k) != 9 or len(p) != 12
            or not all(math.isfinite(x) for x in (*k, *p, *msg.d))
            or k[0] <= 0 or k[4] <= 0 or not 0 <= k[2] < SIZE[0]
            or not 0 <= k[5] < SIZE[1]
            or any(abs(k[i]) > 1e-6 for i in (1, 3, 6, 7)) or abs(k[8] - 1) > 1e-6
            or abs(p[0] - k[0]) > 1e-3 or abs(p[5] - k[4]) > 1e-3
            or abs(p[2] - k[2]) > 1e-3 or abs(p[6] - k[5]) > 1e-3
            or abs(p[10] - 1) > 1e-6):
        raise ValueError("CameraInfo dimensions/frame/intrinsics do not match RGB image")
    return {"width": msg.width, "height": msg.height, "frame": frame,
            "k": k, "p": p, "distortion_model": msg.distortion_model}


def inspect_png(path, deadline=None):
    """Validate CRCs, bounded decompression, all PNG filters and visible RGB.

    Native Qt screenshots normally use noninterlaced 8-bit RGB/RGBA. Grayscale
    and indexed 8-bit PNGs are also supported; other modes fail explicitly.
    """
    if path.is_symlink() or not path.is_file() or not 0 < path.stat().st_size <= 64 * 1024 * 1024:
        raise ValueError("new screenshot PNG missing, empty or oversized")
    data = path.read_bytes()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("invalid PNG signature")
    offset, chunks, palette, header, ended = 8, [], None, None, False
    while offset < len(data):
        if offset + 12 > len(data):
            raise ValueError("truncated PNG chunk")
        length = struct.unpack_from(">I", data, offset)[0]
        kind = data[offset+4:offset+8]
        end = offset + 12 + length
        if end > len(data):
            raise ValueError("truncated PNG payload")
        payload = data[offset+8:end-4]
        crc = struct.unpack_from(">I", data, end-4)[0]
        if zlib.crc32(kind + payload) & 0xffffffff != crc:
            raise ValueError("PNG CRC mismatch")
        if header is None and kind != b"IHDR":
            raise ValueError("PNG IHDR must be first")
        if kind == b"IHDR":
            if header is not None or length != 13:
                raise ValueError("invalid PNG IHDR")
            header = struct.unpack(">IIBBBBB", payload)
        elif kind == b"PLTE":
            if not payload or len(payload) % 3 or len(payload) > 768:
                raise ValueError("invalid PNG palette")
            palette = [payload[i:i+3] for i in range(0, len(payload), 3)]
        elif kind == b"IDAT":
            chunks.append(payload)
        elif kind == b"tRNS":
            # Avoid accepting diversity hidden in fully transparent pixels.
            raise ValueError("transparent palette/grayscale PNG unsupported; use RGB/RGBA")
        elif kind == b"IEND":
            if length or end != len(data):
                raise ValueError("invalid PNG IEND/trailing data")
            ended = True
            break
        elif kind[0] & 32 == 0:
            raise ValueError(f"unsupported critical PNG chunk {kind!r}")
        offset = end
    if not header or not ended or not chunks:
        raise ValueError("incomplete PNG")
    width, height, depth, color, compression, filtering, interlace = header
    channels = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}.get(color)
    if (not channels or not width or not height or width * height > 8_000_000
            or depth != 8 or compression or filtering or interlace):
        raise ValueError("unsupported/oversized PNG IHDR (need noninterlaced 8-bit)")
    if color == 3 and palette is None:
        raise ValueError("indexed PNG lacks palette")
    stride = width * channels
    expected = height * (stride + 1)
    decoder = zlib.decompressobj()
    raw = decoder.decompress(b"".join(chunks), expected + 1)
    if len(raw) != expected or not decoder.eof or decoder.unused_data or decoder.unconsumed_tail:
        raise ValueError("invalid PNG decompressed length/stream")
    previous, rgb = bytearray(stride), bytearray()
    for y in range(height):
        if deadline is not None and time.monotonic() >= deadline:
            raise RuntimeError("PNG decoding stage deadline exceeded")
        base = y * (stride + 1)
        mode, row = raw[base], bytearray(raw[base+1:base+1+stride])
        if mode > 4:
            raise ValueError("invalid PNG row filter")
        for i in range(stride):
            left = row[i-channels] if i >= channels else 0
            up, corner = previous[i], previous[i-channels] if i >= channels else 0
            if mode == 1:
                predictor = left
            elif mode == 2:
                predictor = up
            elif mode == 3:
                predictor = (left + up)//2
            elif mode == 4:
                estimate = left + up - corner
                distances = (abs(estimate-left), abs(estimate-up), abs(estimate-corner))
                predictor = (left, up, corner)[distances.index(min(distances))]
            else:
                predictor = 0
            row[i] = (row[i] + predictor) & 255
        for i in range(0, stride, channels):
            if color == 3:
                if row[i] >= len(palette):
                    raise ValueError("PNG palette index out of bounds")
                pixel = palette[row[i]]
            elif color in (0, 4):
                pixel = bytes((row[i],)*3)
            else:
                pixel = row[i:i+3]
            alpha = row[i+channels-1] if color in (4, 6) else 255
            rgb.extend(value * alpha // 255 for value in pixel)
        previous = row
    stats = rgb_statistics(bytes(rgb), deadline)
    return {"path": str(path), "width": width, "height": height,
            "color_type": color, "bytes": len(data), **stats}


def bounded_command(command, deadline):
    left = deadline - time.monotonic()
    if left <= 0:
        raise RuntimeError("wall-clock deadline exceeded before command")
    return subprocess.run(command, capture_output=True, text=True, check=False,
                          timeout=min(3.0, left))


def require_boolean(result):
    if result.returncode != 0 or not re.fullmatch(r"\s*data\s*:\s*true\s*", result.stdout):
        raise RuntimeError(f"service did not return exact Boolean data: true: "
                           f"exit={result.returncode}, stdout={result.stdout[:1000]!r}, "
                           f"stderr={result.stderr[:1000]!r}")


def service_call(service, request_type, request, deadline):
    result = bounded_command(["ign", "service", "-s", service, "--reqtype", request_type,
                              "--reptype", "ignition.msgs.Boolean", "--timeout", "2000",
                              "--req", request], deadline)
    require_boolean(result)
    return {"service": service, "reply": result.stdout.strip(), "returncode": result.returncode}


def inspect_service(service, request_type, deadline, report):
    evidence = {"service": service, "expected_request": request_type,
                "expected_reply": "ignition.msgs.Boolean", "pass": False}
    report.setdefault("service_inspection", []).append(evidence)
    try:
        result = bounded_command(["ign", "service", "-i", "-s", service], deadline)
        evidence.update(returncode=result.returncode, stdout=result.stdout[:8000], stderr=result.stderr[:2000])
        types = re.findall(r"\bignition\.msgs\.[A-Za-z0-9_]+\b", result.stdout)
        # Fortress -i prints provider rows in Request, Response order. Reject
        # swapped types rather than merely checking that both strings occur.
        expected = [request_type, "ignition.msgs.Boolean"]
        if result.returncode or not types or len(types) % 2 or any(
            types[i:i+2] != expected for i in range(0, len(types), 2)):
            raise RuntimeError(f"actual service inspection does not validate {service} types: {types}")
        evidence["pass"] = True
    except Exception as exc:
        evidence["error"] = f"{type(exc).__name__}: {exc}"
        raise


def parse_obstacle_pose(text):
    decoder, offset, samples = json.JSONDecoder(), 0, []
    try:
        while True:
            while offset < len(text) and text[offset].isspace():
                offset += 1
            if offset == len(text):
                break
            document, offset = decoder.raw_decode(text, offset)
            entries = document["pose"]
            if not isinstance(entries, list):
                raise ValueError("pose must be a list")
            matches = [x for x in entries if isinstance(x, dict) and x.get("name") == OBSTACLE]
            if len(matches) != 1:
                raise ValueError("need exactly one exact test_obstacle name per message")
            p, q = matches[0]["position"], matches[0]["orientation"]
            result = {"position": tuple(float(p.get(k, 0.0)) for k in "xyz"),
                      "orientation": tuple(float(q.get(k, 0.0)) for k in "xyzw")}
            if not all(math.isfinite(v) for values in result.values() for v in values):
                raise ValueError("pose has nonfinite values")
            if abs(sum(v*v for v in result["orientation"]) - 1) > 0.001:
                raise ValueError("pose quaternion is not unit length")
            samples.append(result)
        if not samples:
            raise ValueError("no pose messages")
        # ign topic can emit more than one JSON message before exiting; the
        # last complete, validated sample is the freshest observed world state.
        return samples[-1]
    except (KeyError, TypeError, AttributeError, ValueError) as exc:
        raise ValueError(f"invalid observed obstacle pose JSON: {exc}") from exc


def require_pose(actual, expected):
    distance = math.dist(actual["position"], expected["position"])
    q, p = actual["orientation"], expected["orientation"]
    rotation = min(math.dist(q, p), math.dist(q, tuple(-v for v in p)))
    if distance > 0.01 or rotation > 0.01:
        raise ValueError(f"obstacle pose differs from required state: {actual}")


def read_obstacle_pose(deadline):
    result = bounded_command(["ign", "topic", "-e", "-t", POSE_TOPIC,
                              "-n", "1", "--json-output"], deadline)
    if result.returncode:
        raise RuntimeError(f"obstacle pose observation exit {result.returncode}: {result.stderr[:1000]}")
    return parse_obstacle_pose(result.stdout)


def process_snapshot():
    """Reuse the supervisor's tested /proc ancestry/starttime parser."""
    from supervised_process import process_table, read_process

    result = {}
    for pid, process in process_table().items():
        path = Path("/proc") / str(pid)
        try:
            argv = path.joinpath("cmdline").read_bytes().split(b"\0")
            argv = [s.decode(errors="replace") for s in argv if s]
            entry = {"pid": pid, "ppid": process.ppid, "starttime": process.starttime,
                     "alive": process.alive, "uid": path.stat().st_uid, "argv": argv}
            try:
                env = dict(s.split(b"=", 1) for s in path.joinpath("environ").read_bytes().split(b"\0")
                           if b"=" in s)
                # Never store/log unrelated environment variables (possibly secrets).
                entry["partition"] = env.get(b"IGN_PARTITION", b"").decode()
                entry["domain"] = env.get(b"ROS_DOMAIN_ID", b"").decode()
            except PermissionError:
                entry["partition"] = entry["domain"] = None
            current = read_process(pid)
            if current is None or current.identity != process.identity or current.ppid != process.ppid:
                # Drop raced records, never combine a recycled PID's identity
                # with its predecessor's argv/environment. Required owned roles
                # still fail closed if absent; unrelated process churn is normal.
                continue
            result[pid] = entry
        except (FileNotFoundError, ProcessLookupError):
            continue
    return result


def process_role(entry):
    argv = entry["argv"]
    # Fortress's Ruby CLI replaces argv with this exact single process title.
    # OwnershipGuard separately requires the original supervisor ancestry.
    if argv == ["ign gazebo server"]:
        return "server"
    if argv == ["ign gazebo gui"]:
        return "gazebo"
    if len(argv) == 1 and re.fullmatch(r"ign gazebo -s -r /[^\s;&|`$<>]+ -v [0-4]", argv[0]):
        return "server"
    if len(argv) == 1 and re.fullmatch(r"ign gazebo -g(?: -v [0-4])?", argv[0]):
        return "gazebo"
    names = [Path(s).name for s in argv]
    if "rviz2" in names:
        return "rviz"
    if "parameter_bridge" in names:
        return "bridge"
    gazebo = "gazebo" in names or any("ignition-gazebo" in s for s in names)
    if gazebo:
        if any(s in argv for s in ("-s", "--server-only", "server")):
            return "server"
        if any(s in argv for s in ("-g", "--gui-only", "gui")):
            return "gazebo"
        return "unknown_gazebo"
    return None


class OwnershipGuard:
    def __init__(self, supervisor_pid, partition_prefix="linobot2_g1_"):
        if not re.fullmatch(r"linobot2_[a-z][a-z0-9]*_", partition_prefix):
            raise ValueError("partition prefix must be a linobot2_<phase>_ prefix")
        self.pid = supervisor_pid
        self.partition_prefix = partition_prefix
        self.root_starttime = None
        self.server_identity = None

    def check(self):
        partition = os.environ.get("IGN_PARTITION", "")
        if os.environ.get("ROS_DOMAIN_ID") != "42":
            raise ValueError("ROS_DOMAIN_ID must explicitly be 42 before any runtime operation")
        if not partition.startswith(self.partition_prefix) or partition == self.partition_prefix:
            raise ValueError(f"IGN_PARTITION must be a unique {self.partition_prefix}... harness partition")
        table = process_snapshot()
        root = table.get(self.pid)
        if (not root or not root["alive"] or root["uid"] != os.getuid()
                or "supervised_process.py" not in [Path(s).name for s in root["argv"]]):
            raise RuntimeError("supervisor is absent, dead, not owned, or not the harness supervisor")
        if self.root_starttime is not None and root["starttime"] != self.root_starttime:
            raise RuntimeError("supervisor PID/starttime changed")
        self.root_starttime = root["starttime"]
        if root["partition"] != partition or root["domain"] != "42":
            raise RuntimeError("supervisor environment is not this isolated domain/partition")
        owned = {self.pid: root}
        while True:
            new = {pid: p for pid, p in table.items() if pid not in owned and p["ppid"] in owned}
            if not new:
                break
            owned.update(new)
        for pid, entry in owned.items():
            if entry["alive"] and (entry["uid"] != os.getuid() or entry["partition"] != partition
                                   or entry["domain"] != "42"):
                raise RuntimeError(f"owned subtree PID {pid} has mismatched/unreadable domain/partition")
        servers = []
        for pid, entry in table.items():
            role = process_role(entry)
            if not role or not entry["alive"]:
                continue
            if pid in owned:
                if role == "server":
                    servers.append((pid, entry["starttime"]))
            elif entry["partition"] == partition or (entry["partition"] is None and
                                                       entry["uid"] == os.getuid()):
                raise RuntimeError(f"external/unverifiable live {role} PID {pid} shares discovery partition")
        if len(servers) != 1:
            raise RuntimeError(f"require exactly one owned Gazebo server, found {len(servers)}")
        if self.server_identity is not None and self.server_identity != servers[0]:
            raise RuntimeError("Gazebo server PID/starttime changed")
        self.server_identity = servers[0]
        return owned


class GuiChecks:
    def __init__(self, probe, guard, report, screenshots, persist):
        self.probe, self.guard, self.report = probe, guard, report
        self.screenshots, self.persist = screenshots, persist
        self.identities = None

    def inventory(self, deadline, ping=False):
        if time.monotonic() >= deadline:
            raise RuntimeError("GUI inventory deadline exceeded")
        owned = self.guard.check()
        windows = self.probe.find_owned_windows(self.guard.pid)
        selected = [w for w in windows if w["kind"] in ("gazebo", "rviz")]
        if {w["kind"] for w in selected} != {"gazebo", "rviz"}:
            raise RuntimeError("owned Gazebo and RViz GUI windows are both required")
        identities = set()
        for window in selected:
            process = owned.get(window["pid"])
            if (not process or not process["alive"] or process["starttime"] != window["starttime"]
                    or process_role(process) != window["kind"] or window["width"] <= 0
                    or window["height"] <= 0 or not window["protocol_ping"]):
                raise RuntimeError("GUI window identity/geometry/_NET_WM_PING ownership invalid")
            identities.add((window["kind"], window["window_id"], window["pid"], window["starttime"]))
        if self.identities is not None and identities != self.identities:
            raise RuntimeError("owned GUI window/PID/starttime identities changed or disappeared")
        self.identities = identities
        evidence = {"monotonic_seconds": time.monotonic(), "windows": selected,
                    "server_identity": self.guard.server_identity, "pings": [], "pass": False}
        self.report.setdefault("gui_checks", []).append(evidence)
        self.persist()
        if ping:
            for window in selected:
                left = deadline - time.monotonic()
                if left <= 0:
                    raise RuntimeError("GUI ping stage deadline exceeded")
                response = self.probe.ping(window, timeout=min(2.0, left))
                if response is False or (isinstance(response, dict) and response.get("pass") is False):
                    raise RuntimeError("GUI helper returned failed ping evidence")
                evidence["pings"].append({"kind": window["kind"], "window_id": window["window_id"],
                                          "evidence": response})
        if time.monotonic() >= deadline:
            raise RuntimeError("GUI inventory/ping stage deadline exceeded")
        evidence["pass"] = True
        self.persist()
        return evidence

    def full(self, deadline):
        evidence = self.inventory(deadline, ping=True)
        evidence["pass"] = False  # Full check remains partial until PNG validation.
        # A read-only world-service reply, never world control or cmd_vel.
        result = bounded_command(["ign", "service", "-s", "/gazebo/worlds",
                                  "--reqtype", "ignition.msgs.Empty", "--reptype", "ignition.msgs.StringMsg_V",
                                  "--timeout", "2000", "--req", ""], deadline)
        evidence["world_reply"] = {"returncode": result.returncode, "stdout": result.stdout[:2000],
                                   "stderr": result.stderr[:1000]}
        if result.returncode or not re.fullmatch(r'\s*data:\s*"fortress_test"\s*', result.stdout):
            evidence["pass"] = False
            raise RuntimeError("isolated world service must report exactly fortress_test")
        directory = (self.screenshots / f"gui-{uuid.uuid4().hex}").absolute()
        self.screenshots.mkdir(parents=True, exist_ok=True)
        # Fortress ScreenshotService takes a directory and generates its own
        # timestamped filename. A unique empty directory excludes stale PNGs.
        directory.mkdir(exist_ok=False)
        evidence["screenshot_request_directory"] = str(directory)
        self.persist()
        evidence["screenshot_service"] = service_call(SCREENSHOT_SERVICE, "ignition.msgs.StringMsg",
                                                       "data: " + json.dumps(str(directory)), deadline)
        # Some GUI implementations acknowledge before the file is complete.
        last_error, next_inventory = "no PNG", time.monotonic() + 5
        while time.monotonic() < deadline:
            try:
                paths = list(directory.glob("*.png"))
                if len(paths) > 1:
                    raise RuntimeError("multiple PNGs in unique screenshot directory")
                if not paths:
                    raise ValueError("new screenshot PNG missing")
                path = paths[0]
                evidence["screenshot"] = inspect_png(path, deadline)
                break
            except (OSError, ValueError, zlib.error) as exc:
                last_error = str(exc)
                if time.monotonic() >= next_inventory:
                    self.inventory(deadline)
                    next_inventory = time.monotonic() + 5
                time.sleep(min(0.1, max(0, deadline-time.monotonic())))
        else:
            evidence["pass"] = False
            raise RuntimeError(f"screenshot not valid/nonblank within deadline: {last_error}")
        if time.monotonic() > deadline:
            raise RuntimeError("GUI/screenshot stage exceeded deadline")
        evidence["pass"] = True
        self.persist()


class SensorState:
    """ROS-free state/callback logic, using bounded image storage (three frames)."""
    def __init__(self):
        self.received, self.counts, self.stamps = {}, {}, {}
        self.clock_ns, self.clock_advances, self.clock_rewinds = None, 0, 0
        self.clock_progress_at = time.monotonic()
        self.last_image_stamp, self.info, self.frame, self.error = None, None, None, None
        self.cutoff = None
        self.images = deque(maxlen=3)

    def receive(self, key, msg):
        try:
            now = time.monotonic()
            current = stamp_ns(msg.clock if key == "clock" else msg.header.stamp)
            previous = self.stamps.get(key)
            if key == "clock":
                if self.clock_ns is not None and current < self.clock_ns:
                    self.clock_rewinds += 1
                    raise ValueError("simulation clock rewound")
                if self.clock_ns is not None and current > self.clock_ns:
                    self.clock_advances += 1
                    self.clock_progress_at = now
                self.clock_ns = current
            else:
                if not msg.header.frame_id:
                    raise ValueError(f"{key} has empty frame")
                if previous is not None and current < previous:
                    raise ValueError(f"{key} sensor timestamp rewound")
                if key == "image":
                    require_increasing(self.last_image_stamp, current)
                    self.last_image_stamp = current
                    if (msg.width, msg.height) != SIZE or msg.encoding != "rgb8" or msg.step < msg.width*3:
                        raise ValueError("invalid live camera metadata")
                    self.frame = msg.header.frame_id
                    if self.cutoff is not None and current > self.cutoff and len(self.images) < 3:
                        rgb = normalize_rgb(msg)
                        stats = rgb_statistics(rgb)
                        self.images.append({"stamp_ns": current, "frame": self.frame,
                                            "rgb": rgb, "stats": stats})
                if key == "info":
                    self.info = msg
            # Repeated old stamps cannot refresh the freshness deadline.
            if previous is None or current > previous:
                self.received[key] = now
            self.stamps[key] = current
            self.counts[key] = self.counts.get(key, 0) + 1
        except (ValueError, TypeError, AttributeError) as exc:
            self.error = self.error or str(exc)

    def health(self):
        if self.error:
            raise ValueError(self.error)
        now = time.monotonic()
        stale = [key for key in TOPICS if now - self.received.get(key, -math.inf) > FRESH_SECONDS]
        if stale or now - self.clock_progress_at > FRESH_SECONDS or not self.clock_advances or self.clock_rewinds:
            raise ValueError(f"sensor/clock freshness failure: stale={stale}, rewinds={self.clock_rewinds}")
        for key in ("image", "info"):
            if self.stamps[key] > self.clock_ns + 1_000_000_000 or self.clock_ns - self.stamps[key] > 10_000_000_000:
                raise ValueError(f"{key} timestamp is not current simulation time")
        validate_info(self.info, self.frame)


class RosRuntime:
    def __init__(self, rclpy, node, state, guard, gui, report, persist, deadline):
        self.ros, self.node, self.state = rclpy, node, state
        self.guard, self.gui, self.report, self.persist = guard, gui, report, persist
        self.deadline, self.next_inventory = deadline, time.monotonic() + 5.0

    def stage_deadline(self):
        return min(self.deadline, time.monotonic() + STAGE_SECONDS)

    def spin(self, deadline, healthy=True):
        if not self.ros.ok() or time.monotonic() >= deadline:
            raise RuntimeError("ROS stopped or wall-clock stage deadline exceeded")
        self.ros.spin_once(self.node, timeout_sec=min(0.1, max(0, deadline-time.monotonic())))
        if self.state.error:
            raise ValueError(self.state.error)
        if healthy:
            self.state.health()
        if time.monotonic() >= self.next_inventory:
            self.gui.inventory(min(deadline, time.monotonic()+5))
            self.next_inventory = time.monotonic() + 5.0

    def warmup(self):
        deadline = self.stage_deadline()
        while not (set(TOPICS) <= self.state.received.keys() and self.state.clock_advances):
            self.spin(deadline, healthy=False)
        self.state.health()

    def acquire(self, label, cutoff):
        deadline = self.stage_deadline()
        self.state.cutoff = None
        self.state.images.clear()
        # Drain all five depth-5 subscription queues, still processing clock and
        # health. More than 25 nonblocking spins covers the possible queued work.
        for _ in range(40):
            if time.monotonic() >= deadline:
                raise RuntimeError("queued image drain exceeded stage deadline")
            self.ros.spin_once(self.node, timeout_sec=0.0)
        self.state.health()
        self.state.cutoff = max(cutoff, self.state.clock_ns) + SETTLE_NS
        try:
            while len(self.state.images) < 3 or self.state.stamps.get("info", 0) <= self.state.cutoff:
                self.spin(deadline)
            images = list(self.state.images)
            metadata = validate_info(self.state.info, images[0]["frame"])
            if any(x["frame"] != metadata["frame"] for x in images):
                raise ValueError("camera frame changed during state acquisition")
            evidence = {"state": label, "minimum_stamp_exclusive_ns": self.state.cutoff,
                        "samples": [{k: v for k, v in x.items() if k != "rgb"} for x in images],
                        "camera_info": metadata, "info_stamp_ns": self.state.stamps["info"]}
            self.report.setdefault("camera_states", []).append(evidence)
            self.persist()
            return [x["rgb"] for x in images]
        finally:
            self.state.cutoff = None
            self.state.images.clear()

    def observe(self):
        self.guard.check()
        return read_obstacle_pose(self.stage_deadline())

    def set_pose(self, pose, restoring=False):
        # Revalidate ownership before every write, including restoration. Never
        # restore into an external/restarted world after losing the original root.
        self.guard.check()
        deadline = min(self.deadline + (10 if restoring else 0), time.monotonic() + 9)
        p, q = pose["position"], pose["orientation"]
        request = (f'name: "{OBSTACLE}" position {{ x: {p[0]:.17g} y: {p[1]:.17g} z: {p[2]:.17g} }} '
                   f'orientation {{ x: {q[0]:.17g} y: {q[1]:.17g} z: {q[2]:.17g} w: {q[3]:.17g} }}')
        evidence = {"pose": pose, "restoring": restoring, "pass": False}
        self.report.setdefault("pose_writes", []).append(evidence)
        # A broken report destination must not prevent a safety restoration.
        if not restoring:
            self.persist()
        evidence["service"] = service_call(POSE_SERVICE, "ignition.msgs.Pose", request, deadline)
        # Static obstacle is on pose/info, not assumed to appear on dynamic_pose.
        observed = read_obstacle_pose(deadline)
        evidence["observed"] = observed
        require_pose(observed, pose)
        evidence["pass"] = True
        self.persist()


def run_camera_sequence(runtime, report, persist):
    original = runtime.observe()
    report["observed_original_pose"] = original
    require_pose(original, ORIGINAL)  # Refuse an already-modified/foreign world.
    persist()
    attempted = False
    a = runtime.acquire("original", runtime.state.clock_ns)
    try:
        before = runtime.state.clock_ns
        attempted = True  # Even a timeout/false reply may have applied the write.
        runtime.set_pose(OUTSIDE)
        b = runtime.acquire("outside", before)
        report["camera_comparison"] = camera_evidence(a, b, deadline=runtime.stage_deadline())
        persist()
        if not report["camera_comparison"]["outward_pass"]:
            raise ValueError("camera outward spatial change/red-decrease thresholds failed")
    finally:
        if attempted:
            report["restore"] = {"attempted": True, "pass": False}
            try:
                runtime.set_pose(original, restoring=True)
                report["restore"]["pass"] = True
            except (Exception, KeyboardInterrupt) as exc:
                report["restore"]["error"] = f"{type(exc).__name__}: {exc}"
                raise RuntimeError(f"obstacle restoration failed: {exc}") from exc
            finally:
                persist()
    c = runtime.acquire("returned", runtime.state.clock_ns)
    report["camera_comparison"] = camera_evidence(a, b, c, deadline=runtime.stage_deadline())
    persist()
    if not report["camera_comparison"]["return_pass"]:
        raise ValueError("returned image differs from original by more than 5% changed pixels")


def run_stability(runtime, duration, report, persist):
    runtime.gui.full(runtime.stage_deadline())
    # Duration starts AFTER both ABA and the initial complete GUI/service check.
    start = time.monotonic()
    report["stability"] = {"started_monotonic_seconds": start, "elapsed_seconds": 0.0,
                           "required_seconds": duration, "pass": False}
    persist()
    end, next_full, next_summary = start + duration, start + 30, start
    while time.monotonic() < end:
        runtime.spin(runtime.deadline)
        now = time.monotonic()
        if now >= next_full:
            runtime.gui.full(runtime.stage_deadline())
            next_full += 30  # Do not reset cadence to completion time.
        if now >= next_summary:
            report["stability"].update(elapsed_seconds=now-start, samples=dict(runtime.state.counts),
                                       clock_advances=runtime.state.clock_advances)
            persist()
            next_summary = now + 5
    if duration:
        runtime.gui.full(runtime.stage_deadline())  # Endpoint must still be responsive.
    runtime.state.health()
    report["stability"]["pass"] = True
    report["stability"].update(elapsed_seconds=time.monotonic()-start,
                               samples=dict(runtime.state.counts), clock_advances=runtime.state.clock_advances)
    if time.monotonic() > runtime.deadline:
        raise RuntimeError("total work deadline exceeded")
    persist()


def run_ros(guard, gui, report, duration, persist, deadline):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan

    node, initialized = None, False
    try:
        # Keep main's SIGINT/SIGTERM exception handler so finally can restore.
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = Node("linorobot2_g1_camera_gui_probe", enable_rosout=False, start_parameter_services=False)
        state = SensorState()
        types = {"clock": Clock, "scan": LaserScan, "imu": Imu, "image": Image, "info": CameraInfo}
        for key, topic in TOPICS.items():
            node.create_subscription(types[key], topic, lambda msg, key=key: state.receive(key, msg),
                                     qos_profile_sensor_data)
        runtime = RosRuntime(rclpy, node, state, guard, gui, report, persist, deadline)
        runtime.warmup()
        run_camera_sequence(runtime, report, persist)
        run_stability(runtime, duration, report, persist)
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            if initialized and rclpy.ok():
                rclpy.shutdown()


def write_report(path, report):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, prefix=path.name+".",
                                         suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.write("\n")
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--supervisor-pid", type=int, required=True)
    parser.add_argument("--duration", type=float, default=900.0)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--screenshots", type=Path)
    parser.add_argument("--inspect", action="store_true")
    args = parser.parse_args(argv)
    if args.supervisor_pid <= 0 or not math.isfinite(args.duration) or args.duration < 0:
        parser.error("supervisor-pid must be positive; duration finite and nonnegative")
    start = time.monotonic()
    deadline = start + (30 if args.inspect else args.duration + 120)
    report = {"pass": False, "status": "RUNNING", "errors": [], "world": WORLD,
              "supervisor_pid": args.supervisor_pid, "domain_id": os.environ.get("ROS_DOMAIN_ID"),
              "partition": os.environ.get("IGN_PARTITION"), "requested_duration_seconds": args.duration,
              "scope": "INSPECT" if args.inspect else ("PILOT" if args.duration < 900 else "G1_CAMERA_GUI"),
              "criteria": {"images_per_state": 3, "image_size": SIZE, "encoding": "rgb8",
                           "settle_sim_seconds": 0.2, "freshness_wall_seconds": FRESH_SECONDS,
                           "pixel_channel_threshold": 20, "minimum_changed_ratio": 0.01,
                           "minimum_mean_absolute_rgb_difference": 1.0, "minimum_red_decrease_ratio": 0.20,
                           "maximum_return_changed_ratio": 0.05, "gui_full_interval_seconds": 30,
                           "gui_inventory_interval_seconds": 5, "stage_wall_seconds": STAGE_SECONDS,
                           "total_wall_budget_seconds": deadline-start},
              "api_caveats": ["Not full G1: clean starts, lidar geometry/QoS and visual sign-off are separate",
                              "X11 _NET_WM_PING plus native screenshot proves response, not human usability",
                              "Requires gui_x11_probe.X11Probe; inspect records actual Ignition service types",
                              "PNG decoder supports noninterlaced 8-bit screenshots only; no external packages",
                              "SIGKILL cannot restore an obstacle; harness owns simulator cleanup"]}
    persist = lambda: write_report(args.report, report)
    previous, interrupted = {}, False

    def interrupt(signum, frame):
        nonlocal interrupted
        if not interrupted:
            interrupted = True
            raise KeyboardInterrupt(f"received signal {signum}")

    try:
        persist()
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, interrupt)
        guard = OwnershipGuard(args.supervisor_pid)
        guard.check()
        from gui_x11_probe import X11Probe

        with X11Probe() as probe:
            gui = GuiChecks(probe, guard, report, args.screenshots or args.report.parent/"screenshots", persist)
            inspection_errors = []
            for service, request in ((POSE_SERVICE, "ignition.msgs.Pose"),
                                     (SCREENSHOT_SERVICE, "ignition.msgs.StringMsg")):
                try:
                    inspect_service(service, request, min(deadline, time.monotonic()+3), report)
                except Exception as exc:
                    inspection_errors.append(str(exc))
                persist()
            gui.inventory(min(deadline, time.monotonic()+30), ping=True)
            if inspection_errors:
                raise RuntimeError("; ".join(inspection_errors))
            if not args.inspect:
                # Reserve ten seconds for restoration within the total +120s cap.
                run_ros(guard, gui, report, args.duration, persist, deadline-10)
            if time.monotonic() > deadline:
                raise RuntimeError("total wall-clock deadline exceeded")
            report["pass"] = True
    except (Exception, KeyboardInterrupt) as exc:
        report["errors"].append(f"{type(exc).__name__}: {exc}")
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
        report["elapsed_seconds"] = time.monotonic()-start
        report["status"] = "PASS" if report["pass"] else "FAIL"
        try:
            persist()
        except (OSError, ValueError, TypeError) as exc:
            report["pass"] = False
            report["errors"].append(f"cannot save JSON report: {exc}")
    print(f"{'PASS' if report['pass'] else 'FAIL'} [{report['scope']}]: "
          + (str(args.report) if report["pass"] else "; ".join(report["errors"])))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())