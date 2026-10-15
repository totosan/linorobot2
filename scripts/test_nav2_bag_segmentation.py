"""ROS-free tests for bag-time partitioning and the fixed Nav2 input allowlist."""

import importlib.util
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "segment_nav2_bag", ROOT/"scripts/segment_nav2_bag.py")
segmenter = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(segmenter)


class Nav2BagSegmentationTests(unittest.TestCase):
    def _run_fake_split(self, directory, extra_messages=()):
        topics = [
            SimpleNamespace(
                name=name,
                type=type_name,
                serialization_format="cdr",
                offered_qos_profiles="[]",
            )
            for name, type_name in segmenter.TOPIC_TYPES.items()
        ]
        topics.append(SimpleNamespace(
            name="/cmd_vel",
            type="geometry_msgs/msg/Twist",
            serialization_format="cdr",
            offered_qos_profiles="[]",
        ))
        messages = [
            ("/tf_static", "static-a", 100),
            ("/tf", "tf-0", 101),
            ("/scan", "scan-0", 102),
            ("/odom", "odom-0", 103),
            ("/map", "map-0", 104),
            ("/tf_static", "static-b", 105),
            ("/clock", "clock:0", 110),
            ("/cmd_vel", "command", 115),
            ("/clock", "clock:10", 120),
            ("/tf", "tf-1", 161),
            ("/scan", "scan-1", 162),
            ("/odom", "odom-1", 163),
            ("/tf_static", "static-c", 164),
            ("/map", "map-1", 165),
            ("/clock", "clock:0", 160),
            ("/clock", "clock:10", 180),
        ]
        messages.extend(extra_messages)
        messages.sort(key=lambda item: item[2])

        class StorageOptions:
            def __init__(self, uri, storage_id):
                self.uri = uri
                self.storage_id = storage_id

        class ConverterOptions:
            def __init__(self, input_serialization_format,
                         output_serialization_format):
                self.input_serialization_format = input_serialization_format
                self.output_serialization_format = output_serialization_format

        class TopicMetadata:
            def __init__(self, **kwargs):
                self.__dict__.update(kwargs)

        class SequentialReader:
            def open(self, storage_options, _converter_options):
                self.messages = list(messages)
                self.index = 0

            def get_all_topics_and_types(self):
                return topics

            def has_next(self):
                return self.index < len(self.messages)

            def read_next(self):
                message = self.messages[self.index]
                self.index += 1
                return message

        class SequentialWriter:
            outputs = {}

            def open(self, storage_options, _converter_options):
                self.uri = Path(storage_options.uri)
                self.uri.mkdir()
                (self.uri / "metadata.yaml").write_text("fake rosbag metadata\n")
                (self.uri / "bag.db3").touch()
                self.outputs[str(self.uri)] = []

            def create_topic(self, _metadata):
                pass

            def write(self, topic, serialized, timestamp):
                record = (topic, serialized, timestamp)
                self.outputs[str(self.uri)].append(record)
                with (self.uri / "bag.db3").open("a", encoding="utf-8") as db:
                    db.write(f"{record!r}\n")

        rosbag2_py = ModuleType("rosbag2_py")
        rosbag2_py.SequentialReader = SequentialReader
        rosbag2_py.SequentialWriter = SequentialWriter
        rosbag2_py.StorageOptions = StorageOptions
        rosbag2_py.ConverterOptions = ConverterOptions
        rosbag2_py.TopicMetadata = TopicMetadata

        rclpy = ModuleType("rclpy")
        serialization = ModuleType("rclpy.serialization")

        def deserialize_message(serialized, _message_type):
            nanoseconds = int(serialized.split(":", 1)[1])
            return SimpleNamespace(
                clock=SimpleNamespace(sec=0, nanosec=nanoseconds))

        serialization.deserialize_message = deserialize_message
        rclpy.serialization = serialization
        rosgraph_msgs = ModuleType("rosgraph_msgs")
        rosgraph_msgs_msg = ModuleType("rosgraph_msgs.msg")
        rosgraph_msgs_msg.Clock = type("Clock", (), {})
        rosgraph_msgs.msg = rosgraph_msgs_msg
        source = directory / "source"
        source.mkdir()
        (source / "metadata.yaml").write_text("source bag\n")
        (source / "source.db3").write_bytes(b"immutable source")
        output = directory / "segments"
        modules = {
            "rosbag2_py": rosbag2_py,
            "rclpy": rclpy,
            "rclpy.serialization": serialization,
            "rosgraph_msgs": rosgraph_msgs,
            "rosgraph_msgs.msg": rosgraph_msgs_msg,
        }
        with patch.dict(sys.modules, modules):
            report = segmenter.split_bag(source, output)
        return report, SequentialWriter.outputs

    def test_rewinds_create_independent_monotonic_epochs(self):
        epochs = segmenter.partition_clock_samples(
            [(100, 0), (110, 10), (120, 20), (130, 0), (140, 10)],
            90, 150)
        self.assertEqual(len(epochs), 2)
        self.assertEqual(
            [(item["clock_start_ns"], item["clock_end_ns"]) for item in epochs],
            [(0, 20), (0, 10)])
        self.assertEqual(epochs[0]["end_bag_ns_exclusive"], 130)
        self.assertEqual(epochs[1]["start_bag_ns"], 130)
        self.assertEqual(epochs[1]["end_bag_ns_exclusive"], 151)

    def test_split_bag_writes_multiple_segments_and_carries_latest_state(self):
        with tempfile.TemporaryDirectory() as temporary:
            report, outputs = self._run_fake_split(Path(temporary))

        self.assertEqual(report["segment_count"], 2)
        first = outputs[report["segments"][0]["path"]]
        second = outputs[report["segments"][1]["path"]]
        self.assertEqual(
            [item[1] for item in first if item[0] == "/clock"],
            ["clock:0", "clock:10"])
        self.assertEqual(
            [item[1] for item in second if item[0] == "/clock"],
            ["clock:0", "clock:10"])
        self.assertEqual(
            [item[1] for item in second if item[0] == "/tf_static"],
            ["static-a", "static-b", "static-c"])
        self.assertEqual(
            [item[1] for item in second if item[0] == "/map"],
            ["map-0", "map-1"])
        self.assertTrue(all(
            item[0] != "/cmd_vel"
            for records in outputs.values() for item in records))
        self.assertEqual(
            report["segments"][1]["message_counts"]["/tf_static"], 3)
        self.assertEqual(report["segments"][1]["message_counts"]["/map"], 2)

    def test_split_bag_rejects_non_clock_message_at_rewind_boundary(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            with self.assertRaisesRegex(ValueError, "shared bag timestamp 160"):
                self._run_fake_split(
                    directory, [("/scan", "ambiguous-scan", 160)])
            self.assertFalse((directory / "segments").exists())

    def test_equal_clock_values_do_not_split_but_multiple_rewinds_do(self):
        epochs = segmenter.partition_clock_samples(
            [(10, 5), (20, 5), (30, 8), (40, 1), (50, 1), (60, 0)], 10, 60)
        self.assertEqual([epoch["clock_messages"] for epoch in epochs], [3, 2, 1])

    def test_rejects_ambiguous_rewind_and_missing_clock(self):
        with self.assertRaisesRegex(ValueError, "shared bag timestamp"):
            segmenter.partition_clock_samples([(10, 9), (10, 0)], 10, 10)
        with self.assertRaisesRegex(ValueError, "no /clock"):
            segmenter.partition_clock_samples([], 0, 1)

    def test_allowlist_contains_inputs_only(self):
        self.assertEqual(
            set(segmenter.TOPIC_TYPES),
            {"/clock", "/tf", "/tf_static", "/scan", "/map", "/odom"})
        self.assertTrue(segmenter.REQUIRED_TOPICS.isdisjoint(
            {"/cmd_vel", "/plan", "/global_costmap/costmap", "/amcl_pose"}))


if __name__ == "__main__":
    unittest.main()
