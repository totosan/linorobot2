#!/usr/bin/env python3
"""Split a ROS bag at /clock rewinds into safe Nav2-input-only segments.

The source bag is never modified. Each output segment contains only /clock,
/tf, /tf_static, /scan, /map and /odom. Prior transient-local map data and
static transforms are carried into later segments so each segment is usable on
its own. This is an offline ROS 2 Humble utility; it does not start ROS nodes.
"""

import argparse
from bisect import bisect_right
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

TOPIC_TYPES = {
    "/clock": "rosgraph_msgs/msg/Clock",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
    "/scan": "sensor_msgs/msg/LaserScan",
    "/map": "nav_msgs/msg/OccupancyGrid",
    "/odom": "nav_msgs/msg/Odometry",
}
REQUIRED_TOPICS = frozenset(TOPIC_TYPES)
SEGMENT_SCOPE = "NAV2_SENSOR_INPUT_SEGMENT_V1"


def partition_clock_samples(clock_samples, source_start_ns, source_end_ns):
    """Return bag-time epochs split at every backwards /clock transition."""
    if not clock_samples:
        raise ValueError("source bag contains no /clock samples")
    if source_start_ns < 0 or source_end_ns < source_start_ns:
        raise ValueError("invalid source bag timestamp bounds")

    epochs = []
    segment_start_bag_ns = source_start_ns
    first_bag_ns, first_clock_ns = clock_samples[0]
    if first_bag_ns < source_start_ns or first_bag_ns > source_end_ns:
        raise ValueError("/clock sample falls outside source bag bounds")
    segment_clock_start_ns = first_clock_ns
    segment_clock_end_ns = first_clock_ns
    segment_clock_count = 1
    previous_bag_ns, previous_clock_ns = first_bag_ns, first_clock_ns

    for bag_ns, clock_ns in clock_samples[1:]:
        if bag_ns < previous_bag_ns or bag_ns > source_end_ns:
            raise ValueError("/clock bag timestamps are not monotonic")
        if clock_ns < previous_clock_ns:
            if bag_ns <= previous_bag_ns:
                raise ValueError("ambiguous /clock rewind at a shared bag timestamp")
            epochs.append({
                "start_bag_ns": segment_start_bag_ns,
                "end_bag_ns_exclusive": bag_ns,
                "clock_start_ns": segment_clock_start_ns,
                "clock_end_ns": segment_clock_end_ns,
                "clock_messages": segment_clock_count,
            })
            segment_start_bag_ns = bag_ns
            segment_clock_start_ns = clock_ns
            segment_clock_end_ns = clock_ns
            segment_clock_count = 1
        else:
            segment_clock_end_ns = clock_ns
            segment_clock_count += 1
        previous_bag_ns, previous_clock_ns = bag_ns, clock_ns

    epochs.append({
        "start_bag_ns": segment_start_bag_ns,
        "end_bag_ns_exclusive": source_end_ns+1,
        "clock_start_ns": segment_clock_start_ns,
        "clock_end_ns": segment_clock_end_ns,
        "clock_messages": segment_clock_count,
    })
    return epochs


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _file_hashes(root):
    return {
        path.name: _sha256(path)
        for path in sorted(Path(root).iterdir())
        if path.is_file()
    }


def _clock_ns(message):
    return message.clock.sec*1_000_000_000+message.clock.nanosec


def _open_reader(rosbag2_py, source):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(source), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"))
    return reader


def split_bag(source, output_root):
    """Create read-only-derived, independently playable Nav2 sensor segments."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosgraph_msgs.msg import Clock

    source = Path(source).resolve()
    output_root = Path(output_root).resolve()
    if not (source/"metadata.yaml").is_file():
        raise ValueError("source must be an existing rosbag2 directory")
    if source == output_root or source in output_root.parents:
        raise ValueError("output directory must not be inside the source bag")
    if output_root.exists():
        raise ValueError(f"output directory must not pre-exist: {output_root}")
    output_root.parent.mkdir(parents=True, exist_ok=True)

    reader = _open_reader(rosbag2_py, source)
    source_topics = {topic.name: topic for topic in reader.get_all_topics_and_types()}
    missing = sorted(REQUIRED_TOPICS-source_topics.keys())
    wrong_types = {
        name: source_topics[name].type
        for name, expected in TOPIC_TYPES.items()
        if name in source_topics and source_topics[name].type != expected
    }
    if missing:
        raise ValueError(f"source bag is missing required topics: {missing}")
    if wrong_types:
        raise ValueError(f"source bag has unexpected topic types: {wrong_types}")

    clock_samples = []
    source_start_ns = None
    source_end_ns = None
    map_snapshots = []
    static_transforms = []
    bag_timestamp_counts = Counter()
    clock_timestamp_counts = Counter()
    while reader.has_next():
        topic, serialized, timestamp = reader.read_next()
        timestamp = int(timestamp)
        bag_timestamp_counts[timestamp] += 1
        source_start_ns = timestamp if source_start_ns is None else min(source_start_ns, timestamp)
        source_end_ns = timestamp if source_end_ns is None else max(source_end_ns, timestamp)
        if topic == "/clock":
            clock_timestamp_counts[timestamp] += 1
            clock = deserialize_message(serialized, Clock)
            clock_samples.append((timestamp, _clock_ns(clock)))
        elif topic == "/map":
            map_snapshots.append((timestamp, serialized))
        elif topic == "/tf_static":
            static_transforms.append((timestamp, serialized))
    if source_start_ns is None or source_end_ns is None:
        raise ValueError("source bag is empty")
    epochs = partition_clock_samples(clock_samples, source_start_ns, source_end_ns)
    for epoch in epochs[1:]:
        boundary = epoch["start_bag_ns"]
        if bag_timestamp_counts[boundary] != clock_timestamp_counts[boundary]:
            raise ValueError(
                f"ambiguous /clock rewind at shared bag timestamp {boundary}")
    output_root.mkdir()

    starts = [epoch["start_bag_ns"] for epoch in epochs[1:]]
    writers = []
    segment_dirs = []
    counts = [{topic: 0 for topic in TOPIC_TYPES} for _ in epochs]
    for index, epoch in enumerate(epochs):
        segment_dir = output_root/f"segment-{index:03d}"
        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(uri=str(segment_dir), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"))
        for name in TOPIC_TYPES:
            source_metadata = source_topics[name]
            writer.create_topic(rosbag2_py.TopicMetadata(
                name=name,
                type=source_metadata.type,
                serialization_format=source_metadata.serialization_format,
                offered_qos_profiles=source_metadata.offered_qos_profiles))
        writers.append(writer)
        segment_dirs.append(segment_dir)

        if index:
            start_ns = epoch["start_bag_ns"]
            carried_static = [
                (timestamp, serialized) for timestamp, serialized in static_transforms
                if timestamp < start_ns
            ]
            for _timestamp, serialized in carried_static:
                writer.write("/tf_static", serialized, start_ns)
            counts[index]["/tf_static"] += len(carried_static)
            previous_maps = [item for item in map_snapshots if item[0] < start_ns]
            if previous_maps:
                writer.write("/map", previous_maps[-1][1], start_ns)
                counts[index]["/map"] += 1

    clock_bounds = [{"first": None, "last": None} for _ in epochs]
    reader = _open_reader(rosbag2_py, source)
    while reader.has_next():
        topic, serialized, timestamp = reader.read_next()
        if topic not in REQUIRED_TOPICS:
            continue
        index = bisect_right(starts, int(timestamp))
        writers[index].write(topic, serialized, timestamp)
        counts[index][topic] += 1
        if topic == "/clock":
            clock = deserialize_message(serialized, Clock)
            value = _clock_ns(clock)
            bounds = clock_bounds[index]
            bounds["first"] = value if bounds["first"] is None else bounds["first"]
            bounds["last"] = value

    writer = None
    del writers

    source_hashes = _file_hashes(source)
    segment_reports = []
    for index, (segment_dir, epoch) in enumerate(zip(segment_dirs, epochs)):
        actual_clock = clock_bounds[index]
        if (counts[index]["/clock"] != epoch["clock_messages"]
                or actual_clock["first"] != epoch["clock_start_ns"]
                or actual_clock["last"] != epoch["clock_end_ns"]):
            raise ValueError(f"segment {index} clock bounds/count do not match its epoch")
        if counts[index]["/map"] == 0:
            raise ValueError(f"segment {index} has no map message after state carry-forward")
        missing_inputs = [
            topic for topic in ("/tf", "/tf_static", "/scan", "/odom")
            if counts[index][topic] == 0
        ]
        if missing_inputs:
            raise ValueError(f"segment {index} lacks required Nav2 inputs: {missing_inputs}")
        manifest = {
            "status": "PASS",
            "scope": SEGMENT_SCOPE,
            "storage_id": "sqlite3",
            "source_bag": str(source),
            "source_file_sha256": source_hashes,
            "source_topics": sorted(source_topics),
            "bag_time_start_ns": epoch["start_bag_ns"],
            "bag_time_end_ns_exclusive": epoch["end_bag_ns_exclusive"],
            "clock_start_ns": epoch["clock_start_ns"],
            "clock_end_ns": epoch["clock_end_ns"],
            "capture_clock_domain": "ros_sim",
            "topics": TOPIC_TYPES,
            "message_counts": counts[index],
            "file_sha256": _file_hashes(segment_dir),
            "carried_forward": {
                "map": bool(index and any(
                    timestamp < epoch["start_bag_ns"] for timestamp, _ in map_snapshots)),
                "tf_static_message_count": sum(
                    timestamp < epoch["start_bag_ns"]
                    for timestamp, _ in static_transforms),
            },
            "hardware_topics_or_devices": "none",
        }
        manifest_path = Path(f"{segment_dir}.manifest.json")
        manifest_path.write_text(
            json.dumps(manifest, indent=2, allow_nan=False)+"\n", encoding="utf-8")
        segment_reports.append({
            "path": str(segment_dir),
            "manifest": str(manifest_path),
            "clock_start_ns": epoch["clock_start_ns"],
            "clock_end_ns": epoch["clock_end_ns"],
            "message_counts": counts[index],
        })

    index_report = {
        "status": "PASS",
        "scope": SEGMENT_SCOPE,
        "source_bag": str(source),
        "source_file_sha256": source_hashes,
        "segment_count": len(segment_reports),
        "segments": segment_reports,
        "hardware_topics_or_devices": "none",
    }
    (output_root/"index.json").write_text(
        json.dumps(index_report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return index_report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True, help="source rosbag2 directory")
    parser.add_argument("--output", type=Path, required=True,
                        help="new output directory; it must not already exist")
    args = parser.parse_args(argv)
    try:
        report = split_bag(args.bag, args.output)
    except Exception as exc:
        print(f"FAIL [NAV2_BAG_SEGMENTATION]: {type(exc).__name__}: {exc}",
              file=sys.stderr)
        return 1
    print(f"PASS [NAV2_BAG_SEGMENTATION]: {report['segment_count']} segments "
          f"written to {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
