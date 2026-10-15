"""Tests for the G5 navigation-off/mock equivalence report."""

import json
from pathlib import Path
import tempfile
import unittest

from verify_g5_equivalence import compare_reports


class G5EquivalenceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.artifacts = {}
        for name in ("map", "world", "profile", "common"):
            path = self.root/name
            path.write_text(name, encoding="utf-8")
            self.artifacts[name] = path
        self.nav = {
            "pass": True,
            "scenario": "baseline",
            "hardware_topics_or_devices": "none",
            "map": {"width": 100, "height": 100, "resolution": 0.05},
            "safety_limits": {"linear_x_mps": 0.08, "angular_z_rps": 0.30,
                              "hardware_topics_or_devices": "none"},
            "reachable_goal": {"result": "SUCCEEDED", "distance_to_goal_m": 0.07,
                               "zero_command": {"zero": True,
                                                "values": [0.0, 0.0, 0.0]}},
            "obstacle_avoidance_goal": {
                "result": "SUCCEEDED", "distance_to_goal_m": 0.08,
                "zero_command": {"zero": True, "values": [0.0, 0.0, 0.0]}},
            "narrow_passage_goal": {
                "result": "SUCCEEDED", "distance_to_goal_m": 0.06,
                "zero_command": {"zero": True, "values": [0.0, 0.0, 0.0]}},
        }
        self.transport = {
            "pass": True, "result_messages": 5, "status_messages": 5,
            "command_topics_published": [],
        }

    def tearDown(self):
        self.temp.cleanup()

    def write_json(self, name, value):
        path = self.root/name
        path.write_text(json.dumps(value), encoding="utf-8")
        return path

    def compare(self, off=None, mock=None, off_transport=None, mock_transport=None):
        off_path = self.write_json("off.json", off or self.nav)
        mock_path = self.write_json("mock.json", mock or self.nav)
        off_transport_path = self.write_json("off-transport.json", off_transport or dict(
            self.transport, mode="off", expected_state="unavailable", detection_count=0))
        mock_transport_path = self.write_json("mock-transport.json", mock_transport or dict(
            self.transport, mode="mock", expected_state="ok", detection_count=1))
        return compare_reports(
            off_path, mock_path, off_transport_path, mock_transport_path,
            self.artifacts["map"], self.artifacts["world"],
            self.artifacts["profile"], self.artifacts["common"])

    def test_accepts_same_three_successful_scenarios_and_reports_hashes(self):
        report = self.compare()
        self.assertTrue(report["pass"])
        self.assertEqual(set(report["goals"]), {"free", "avoidance", "narrow_passage"})
        self.assertEqual(report["goals"]["free"]["off"]["result"], "SUCCEEDED")
        self.assertEqual(len(report["identical_artifact_sha256"]), 4)
        self.assertFalse(report["real_inference_used"])

    def test_rejects_failed_goal_and_different_safety_limits(self):
        failed = json.loads(json.dumps(self.nav))
        failed["narrow_passage_goal"]["result"] = "ABORTED"
        with self.assertRaisesRegex(ValueError, "not SUCCEEDED"):
            self.compare(mock=failed)

        changed = json.loads(json.dumps(self.nav))
        changed["safety_limits"]["linear_x_mps"] = 0.081
        with self.assertRaisesRegex(ValueError, "different safety limits"):
            self.compare(mock=changed)

    def test_rejects_incomplete_transport_or_missing_final_zero(self):
        bad_transport = dict(self.transport, mode="mock", expected_state="ok",
                             detection_count=1, result_messages=0)
        with self.assertRaisesRegex(ValueError, "transport evidence is incomplete"):
            self.compare(mock_transport=bad_transport)

        bad_report = json.loads(json.dumps(self.nav))
        bad_report["reachable_goal"]["zero_command"]["zero"] = False
        with self.assertRaisesRegex(ValueError, "final zero command"):
            self.compare(off=bad_report)

    def test_rejects_changed_sensor_derived_map(self):
        changed = json.loads(json.dumps(self.nav))
        changed["map"]["width"] = 101
        with self.assertRaisesRegex(ValueError, "identical map metadata"):
            self.compare(mock=changed)


if __name__ == "__main__":
    unittest.main()
