"""ROS-free tests for the fail-closed lidar scan freshness contract."""

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "linorobot2_bringup/scripts"))
from lidar_monitor_contract import ScanHealth  # noqa: E402


class ScanHealthTests(unittest.TestCase):
    def test_startup_grace_then_failure(self):
        health = ScanHealth(start_time=100.0, startup_grace_period=10.0)
        self.assertIsNone(health.failure_reason(109.999))
        self.assertIn("startup period", health.failure_reason(110.0))

    def test_stream_timeout_is_strictly_greater_than_limit(self):
        health = ScanHealth(start_time=0.0, scan_timeout=15.0)
        health.observe_scan(10.0)
        self.assertIsNone(health.failure_reason(25.0))
        self.assertIn("more than 15.0s", health.failure_reason(25.001))

    def test_new_scan_recovers_freshness(self):
        health = ScanHealth(start_time=0.0, scan_timeout=15.0)
        health.observe_scan(10.0)
        self.assertIn("more than 15.0s", health.failure_reason(25.1))
        health.observe_scan(25.0)
        self.assertIsNone(health.failure_reason(40.0))
        self.assertIn("more than 15.0s", health.failure_reason(40.001))

    def test_nonpositive_budgets_are_rejected(self):
        for grace, timeout in ((0, 15), (10, 0), (-1, 15), (10, -1)):
            with self.subTest(grace=grace, timeout=timeout):
                with self.assertRaises(ValueError):
                    ScanHealth(0, grace, timeout)


if __name__ == "__main__":
    unittest.main()
