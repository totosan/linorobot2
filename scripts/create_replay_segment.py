#!/usr/bin/env python3
"""Create and validate one contiguous camera/clock replay segment (Domain 43 input).

Writes a deterministic rosbag2 sqlite3 segment containing only /clock,
/camera/image_raw and /camera/camera_info. The bag is a synthetic, replayable
sensor/time fixture — not recorded robot data. The manifest pins hashes, message
counts and timestamp bounds so the replay gate fails closed on any drift.
"""

import argparse
import hashlib
import json
from pathlib import Path
import sys

TOPIC_TYPES = {
    "/clock": "rosgraph_msgs/msg/Clock",
    "/camera/image_raw": "sensor_msgs/msg/Image",
    "/camera/camera_info": "sensor_msgs/msg/CameraInfo",
}
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
FRAME_ID = "camera_link"
FOCAL_LENGTH = 554.3827


def create_segment(output_dir, *, start_ns=5_000_000_000, duration_ns=4_000_000_000,
                   clock_hz=20.0, frame_hz=1.0):
    """Write the replay segment and return its manifest values."""
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image

    if start_ns < 0 or duration_ns <= 0 or clock_hz <= 0 or frame_hz <= 0:
        raise ValueError("segment start must be >= 0, duration and rates positive")
    if clock_hz < frame_hz:
        raise ValueError("clock rate must not be below the camera frame rate")
    end_ns = start_ns+duration_ns

    output_dir = Path(output_dir)
    if output_dir.exists():
        raise ValueError(f"segment directory must not pre-exist: {output_dir}")
    output_dir.parent.mkdir(parents=True, exist_ok=True)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"))
    for topic, msg_type in TOPIC_TYPES.items():
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=topic, type=msg_type, serialization_format="cdr"))

    clock_period_ns = round(1_000_000_000/clock_hz)
    frame_period_ns = round(1_000_000_000/frame_hz)
    t = start_ns
    while t <= end_ns:
        clock = Clock()
        clock.clock.sec = t//1_000_000_000
        clock.clock.nanosec = t % 1_000_000_000
        writer.write("/clock", serialize_message(clock), t)
        t += clock_period_ns

    def pixel_bytes(frame_index):
        return bytes(((i+frame_index*37) % 251) for i in
                     range(IMAGE_WIDTH*IMAGE_HEIGHT*3))

    frame = 0
    t = start_ns
    while t <= end_ns:
        stamp_sec = t//1_000_000_000
        stamp_nanosec = t % 1_000_000_000
        image = Image()
        image.header.frame_id = FRAME_ID
        image.header.stamp.sec = stamp_sec
        image.header.stamp.nanosec = stamp_nanosec
        image.width = IMAGE_WIDTH
        image.height = IMAGE_HEIGHT
        image.encoding = "rgb8"
        image.is_bigendian = 0
        image.step = IMAGE_WIDTH*3
        image.data = pixel_bytes(frame)
        info = CameraInfo()
        info.header.frame_id = FRAME_ID
        info.header.stamp.sec = stamp_sec
        info.header.stamp.nanosec = stamp_nanosec
        info.width = IMAGE_WIDTH
        info.height = IMAGE_HEIGHT
        info.k = [FOCAL_LENGTH, 0.0, IMAGE_WIDTH/2.0,
                  0.0, FOCAL_LENGTH, IMAGE_HEIGHT/2.0,
                  0.0, 0.0, 1.0]
        writer.write("/camera/image_raw", serialize_message(image), t)
        writer.write("/camera/camera_info", serialize_message(info), t)
        frame += 1
        t += frame_period_ns
    del writer
    return {"start_ns": start_ns, "end_ns": end_ns, "clock_messages": int(
        duration_ns//clock_period_ns)+1, "frame_messages": frame}


def validate_segment(segment_dir, expected):
    """Reopen the bag read-only; verify topics, counts, monotonic contiguous time."""
    import rosbag2_py

    segment_dir = Path(segment_dir)
    if not (segment_dir/"metadata.yaml").is_file():
        raise ValueError("segment metadata.yaml missing")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(segment_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"))
    topics = {meta.name: meta.type for meta in reader.get_all_topics_and_types()}
    if topics != TOPIC_TYPES:
        raise ValueError(f"unexpected topic set/types: {topics}")

    counts = {topic: 0 for topic in TOPIC_TYPES}
    clock_stamps = []
    while reader.has_next():
        topic, _data, timestamp = reader.read_next()
        counts[topic] += 1
        if topic == "/clock":
            clock_stamps.append(timestamp)
    if any(b < a for a, b in zip(clock_stamps, clock_stamps[1:])):
        raise ValueError("recorded /clock is not monotonic within the segment")
    if (counts["/clock"] != expected["clock_messages"]
            or counts["/camera/image_raw"] != expected["frame_messages"]
            or counts["/camera/camera_info"] != expected["frame_messages"]):
        raise ValueError(f"message counts differ from manifest: {counts}")
    if clock_stamps[0] != expected["start_ns"] or clock_stamps[-1] != expected["end_ns"]:
        raise ValueError("clock bounds differ from manifest")
    return counts


def manifest_for(segment_dir, counts, bounds):
    files = {}
    for path in sorted(Path(segment_dir).iterdir()):
        if path.is_file():
            files[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
    return {
        "scope": "REPLAY_SEGMENT_VISION_TRANSPORT_ONLY",
        "storage_id": "sqlite3",
        "topics": TOPIC_TYPES,
        "message_counts": counts,
        "clock_start_ns": bounds["start_ns"],
        "clock_end_ns": bounds["end_ns"],
        "capture_clock_domain": "ros_sim",
        "capture_frame_id": FRAME_ID,
        "image_width": IMAGE_WIDTH,
        "image_height": IMAGE_HEIGHT,
        "image_encoding": "rgb8",
        "file_sha256": files,
        "synthetic_fixture": True,
        "hardware_topics_or_devices": "none",
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    args = parser.parse_args(argv)
    report = {"pass": False, "status": "FAIL", "errors": []}
    try:
        bounds = create_segment(args.output)
        counts = validate_segment(args.output, bounds)
        report.update(manifest_for(args.output, counts, bounds))
        report["pass"] = True
    except Exception as exc:
        report["errors"] = [f"{type(exc).__name__}: {exc}"]
    report["status"] = "PASS" if report["pass"] else "FAIL"
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(f"{report['status']} [REPLAY_SEGMENT]: {args.output}", flush=True)
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
