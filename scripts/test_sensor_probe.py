"""Local regression tests: python3 scripts/test_sensor_probe.py.

ROS imports are intercepted only in the loaded module's private builtins.
No sys.modules entries or installed ROS packages are changed. Fake monotonic
time and spin callbacks keep main() tests deterministic, with no real waits.
"""

import builtins
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock


def load_probe():
    stubs = {name: ModuleType(name) for name in (
        "rclpy", "rclpy.node", "rosgraph_msgs.msg", "sensor_msgs.msg")}
    stubs["rclpy.node"].Node = type("StubNode", (), {})
    for name in ("CameraInfo", "Image", "Imu", "LaserScan"):
        setattr(stubs["sensor_msgs.msg"], name, type(name, (), {}))
    stubs["rosgraph_msgs.msg"].Clock = type("Clock", (), {})

    def isolated_import(name, globals=None, locals=None, fromlist=(), level=0):
        if level == 0 and name in stubs:
            return stubs[name]
        return builtins.__import__(name, globals, locals, fromlist, level)

    spec = importlib.util.spec_from_file_location(
        "sensor_probe_under_test", Path(__file__).with_name("verify_sim_sensors.py"))
    module = importlib.util.module_from_spec(spec)
    module.__dict__["__builtins__"] = dict(vars(builtins), __import__=isolated_import)
    spec.loader.exec_module(module)
    return module


probe_module = load_probe()


def new_probe():
    node = probe_module.SensorProbe.__new__(probe_module.SensorProbe)
    node.samples = {}
    node.received_at = {}
    node.counts = {}
    node.clock_ns = None
    node.clock_advances = 0
    node.clock_rewinds = 0
    node.clock_progress_at = 0.0
    return node


def clock(sec, nanosec=0):
    return SimpleNamespace(clock=SimpleNamespace(sec=sec, nanosec=nanosec))


def image(data, width=640, height=480, frame="camera_optical"):
    return SimpleNamespace(data=data, width=width, height=height, encoding="rgb8",
                           header=SimpleNamespace(frame_id=frame))


class CallbackTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        patch = mock.patch.object(probe_module, "time", SimpleNamespace(monotonic=lambda: self.now))
        patch.start()
        self.addCleanup(patch.stop)
        self.node = new_probe()

    def test_ros_imports_leave_sys_modules_unchanged(self):
        names = ("rclpy", "rclpy.node", "rosgraph_msgs", "rosgraph_msgs.msg",
                 "sensor_msgs", "sensor_msgs.msg")
        before = {name: sys.modules[name] for name in names if name in sys.modules}
        loaded = load_probe()
        after = {name: sys.modules[name] for name in names if name in sys.modules}
        self.assertEqual(before, after)
        self.assertEqual(loaded.SensorProbe.__mro__[1].__name__, "StubNode")

    def test_record_replaces_sample_increments_count_and_refreshes_only_its_key(self):
        self.node.record("scan", "old")
        self.node.record("imu", "imu_frame")
        self.now = 13.5
        self.node.record("scan", "new")
        self.assertEqual(self.node.samples, {"scan": "new", "imu": "imu_frame"})
        self.assertEqual(self.node.counts, {"scan": 2, "imu": 1})
        self.assertEqual(self.node.received_at, {"scan": 13.5, "imu": 10.0})

    def test_clock_initial_equal_advancing_and_backwards_samples(self):
        first = clock(2, 5)
        self.node.clock_cb(first)
        self.assertIs(self.node.samples["clock"], first.clock)
        self.assertEqual(self.node.clock_ns, 2_000_000_005)
        self.assertEqual(self.node.clock_advances, 0)
        self.assertEqual(self.node.clock_progress_at, 0)
        self.now = 11
        self.node.clock_cb(clock(2, 5))
        self.assertEqual(self.node.clock_progress_at, 0)
        self.assertEqual(self.node.received_at["clock"], 11)
        self.now = 12
        self.node.clock_cb(clock(2, 6))
        self.assertEqual(self.node.clock_advances, 1)
        self.assertEqual(self.node.clock_progress_at, 12)
        self.now = 13
        self.node.clock_cb(clock(1, 999_999_999))
        self.assertEqual(self.node.clock_ns, 1_999_999_999)
        self.assertEqual(self.node.clock_rewinds, 1)
        self.assertEqual(self.node.clock_advances, 1)
        self.assertEqual(self.node.clock_progress_at, 12)
        self.assertEqual(self.node.received_at["clock"], 13)
        self.assertEqual(self.node.counts["clock"], 4)

    def test_scan_filters_nonfinite_and_out_of_range_but_includes_bounds(self):
        msg = SimpleNamespace(ranges=[float("nan"), float("inf"), -float("inf"),
                                      0.09, 0.1, 1.0, 10.0, 10.01],
                              range_min=0.1, range_max=10.0,
                              header=SimpleNamespace(frame_id="laser"))
        self.node.scan_cb(msg)
        self.assertEqual(self.node.samples["scan"], (8, 3, "laser"))
        msg.ranges = []
        self.node.scan_cb(msg)
        self.assertEqual(self.node.samples["scan"], (0, 0, "laser"))
        self.assertEqual(self.node.counts["scan"], 2)

    def test_empty_image_does_not_record_or_refresh_previous_image(self):
        self.node.image_cb(image(b""))
        self.assertNotIn("image", self.node.samples)
        self.assertNotIn("image", self.node.counts)
        self.assertNotIn("image", self.node.received_at)
        self.node.image_cb(image(b"\x00\x01"))
        self.now = 20
        self.node.image_cb(image(b""))
        self.assertEqual(self.node.counts["image"], 1)
        self.assertEqual(self.node.received_at["image"], 10)

    def test_image_records_constant_and_nonconstant_payload_metadata(self):
        for data, unique in ((b"\x07" * 32, 1), (bytearray([0, 1, 2, 1]), 3)):
            with self.subTest(unique=unique):
                self.node.image_cb(image(data))
                self.assertEqual(self.node.samples["image"],
                                 (640, 480, "rgb8", unique, len(data), "camera_optical"))

    def test_image_sampling_spans_payload_instead_of_only_prefix(self):
        data = bytearray(16_384)
        data[12_000] = 255  # On the stride, well beyond the first 4096 bytes.
        self.node.image_cb(image(data))
        self.assertEqual(self.node.samples["image"][3:5], (2, len(data)))

    def test_camera_info_copies_intrinsics_and_imu_records_frame(self):
        msg = SimpleNamespace(width=640, height=480, k=[600, 0, 320, 0, 601, 240, 0, 0, 1],
                              header=SimpleNamespace(frame_id="camera_optical"))
        self.node.info_cb(msg)
        msg.k[0] = 0
        self.assertEqual(self.node.samples["camera_info"],
                         (640, 480, (600, 0, 320, 0, 601, 240, 0, 0, 1), "camera_optical"))
        self.node.imu_cb(SimpleNamespace(header=SimpleNamespace(frame_id="imu_link")))
        self.assertEqual(self.node.samples["imu"], "imu_link")
        self.node.imu_cb(SimpleNamespace(header=SimpleNamespace(frame_id="")))
        self.assertEqual(self.node.samples["imu"], "")


class MainTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.node = new_probe()
        self.node.samples = {
            "clock": clock(1).clock, "scan": (360, 300, "laser"), "imu": "imu_link",
            "image": (640, 480, "rgb8", 2, 921600, "camera_optical"),
            "camera_info": (640, 480, (600, 0, 320, 0, 600, 240, 0, 0, 1), "camera_optical"),
        }
        self.node.counts = dict.fromkeys(self.node.samples, 1)
        self.node.received_at = dict.fromkeys(self.node.samples, 0.0)
        self.node.clock_advances = 1
        self.node.destroy_node = mock.Mock()
        self.ros = SimpleNamespace(init=mock.Mock(), shutdown=mock.Mock(),
                                   ok=mock.Mock(return_value=True),
                                   spin_once=mock.Mock(side_effect=self.spin))
        self.factory = mock.Mock(return_value=self.node)
        patches = (
            mock.patch.object(probe_module, "rclpy", self.ros),
            mock.patch.object(probe_module, "SensorProbe", self.factory),
            mock.patch.object(probe_module, "time", SimpleNamespace(monotonic=lambda: self.now)),
        )
        for patch in patches:
            patch.start()
            self.addCleanup(patch.stop)

    def spin(self, node, timeout_sec):
        self.assertIs(node, self.node)
        self.assertEqual(timeout_sec, 0.25)
        self.now += 0.25

    def run_main(self, *args):
        output = io.StringIO()
        with mock.patch.object(sys, "argv", ["sensor-probe"] + list(args)), \
                contextlib.redirect_stdout(output):
            result = probe_module.main()
        self.node.destroy_node.assert_called_once_with()
        self.ros.shutdown.assert_called_once_with()
        return result, output.getvalue()

    def test_invalid_duration_rejected_before_ros_initialization(self):
        for duration in ("-1", "nan", "inf", "-inf"):
            with self.subTest(duration=duration), \
                    mock.patch.object(sys, "argv", ["probe", "--duration=" + duration]), \
                    contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    probe_module.main()
                self.assertEqual(error.exception.code, 2)
                self.ros.init.assert_not_called()
                self.factory.assert_not_called()

    def test_valid_samples_pass_and_cleanup_without_stability_wait(self):
        result, output = self.run_main()
        self.assertEqual(result, 0)
        self.assertIn("PASS: ROS clock", output)
        self.ros.init.assert_called_once_with()
        self.assertEqual(self.ros.spin_once.call_count, 1)

    def test_missing_samples_deadline_is_deterministic(self):
        del self.node.samples["image"]
        self.ros.spin_once.side_effect = lambda *args, **kwargs: setattr(self, "now", self.now + 20)
        result, output = self.run_main()
        self.assertEqual(result, 1)
        self.assertIn("keine ROS-Samples für: image", output)
        self.assertEqual(self.ros.spin_once.call_count, 3)

    def test_static_clock_fails_after_bounded_fake_acquisition(self):
        self.node.clock_advances = 0
        self.ros.spin_once.side_effect = lambda *args, **kwargs: setattr(self, "now", self.now + 20)
        result, output = self.run_main()
        self.assertEqual(result, 1)
        self.assertIn("Simulationszeit steht still", output)
        self.assertEqual(self.ros.spin_once.call_count, 3)

    def test_backwards_clock_fails_even_after_an_advance(self):
        self.node.clock_rewinds = 1
        result, output = self.run_main()
        self.assertEqual(result, 1)
        self.assertIn("springt rückwärts", output)

    def test_invalid_sensor_metadata_is_rejected(self):
        cases = (
            ("scan", (179, 100, "laser"), "ungültiger Scan"),
            ("scan", (360, 9, "laser"), "ungültiger Scan"),
            ("scan", (360, 100, ""), "ungültiger Scan"),
            ("image", (320, 480, "rgb8", 2, 100, "camera_optical"), "Kamera liefert"),
            ("image", (640, 480, "rgb8", 1, 100, "camera_optical"), "Kamera liefert"),
            ("image", (640, 480, "rgb8", 2, 0, "camera_optical"), "Kamera liefert"),
            ("image", (640, 480, "rgb8", 2, 100, ""), "CameraInfo"),
            ("camera_info", (320, 480, (600, 0, 0, 0, 600), "camera_optical"), "CameraInfo"),
            ("camera_info", (640, 480, (0, 0, 0, 0, 600), "camera_optical"), "CameraInfo"),
            ("camera_info", (640, 480, (600, 0, 0, 0, -1), "camera_optical"), "CameraInfo"),
            ("camera_info", (640, 480, (600, 0, 0, 0, 600), "wrong_frame"), "CameraInfo"),
            ("imu", "", "IMU-Frame fehlt"),
        )
        for key, value, diagnostic in cases:
            with self.subTest(key=key, value=value):
                original = self.node.samples[key]
                self.node.samples[key] = value
                self.node.destroy_node.reset_mock()
                self.ros.shutdown.reset_mock()
                result, output = self.run_main()
                self.assertEqual(result, 1)
                self.assertIn(diagnostic, output)
                self.node.samples[key] = original

    def test_stability_accepts_continuously_fresh_advancing_samples(self):
        def fresh_spin(node, timeout_sec):
            self.spin(node, timeout_sec)
            for key in node.samples:
                node.received_at[key] = self.now
                node.counts[key] += 1
            node.clock_progress_at = self.now
            node.clock_advances += 1

        self.ros.spin_once.side_effect = fresh_spin
        result, output = self.run_main("--duration", "1")
        self.assertEqual(result, 0)
        self.assertIn("PASS: 1s kontinuierlicher", output)
        self.assertEqual(self.ros.spin_once.call_count, 5)

    def test_stability_rejects_stale_sensor_even_with_progressing_clock(self):
        def stale_spin(node, timeout_sec):
            if self.ros.spin_once.call_count == 1:
                return self.spin(node, timeout_sec)
            self.now += 11
            for key in node.samples:
                if key != "image":
                    node.received_at[key] = self.now
            node.clock_progress_at = self.now

        self.ros.spin_once.side_effect = stale_spin
        result, output = self.run_main("--duration", "30")
        self.assertEqual(result, 1)
        self.assertIn("stale=['image']", output)

    def test_stability_rejects_static_clock_despite_fresh_messages(self):
        def static_spin(node, timeout_sec):
            self.now += 0.25 if self.ros.spin_once.call_count == 1 else 11
            node.received_at.update(dict.fromkeys(node.samples, self.now))

        self.ros.spin_once.side_effect = static_spin
        result, output = self.run_main("--duration", "30")
        self.assertEqual(result, 1)
        self.assertIn("Daten-/Clock-Stillstand: stale=[]", output)

    def test_stability_rejects_new_clock_rewind(self):
        def rewind_spin(node, timeout_sec):
            self.spin(node, timeout_sec)
            if self.ros.spin_once.call_count > 1:
                node.clock_rewinds += 1

        self.ros.spin_once.side_effect = rewind_spin
        result, output = self.run_main("--duration", "1")
        self.assertEqual(result, 1)
        self.assertIn("clock_rewinds=1", output)

    def test_stability_rejects_ros_context_shutdown(self):
        self.ros.ok.side_effect = [True, False, False]
        result, output = self.run_main("--duration", "1")
        self.assertEqual(result, 1)
        self.assertIn("ROS-Kontext während Stabilitätstest beendet", output)

    def test_spin_exception_still_cleans_up_ros(self):
        self.ros.spin_once.side_effect = RuntimeError("fake spin failure")
        with self.assertRaisesRegex(RuntimeError, "fake spin failure"):
            self.run_main()
        self.node.destroy_node.assert_called_once_with()
        self.ros.shutdown.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()