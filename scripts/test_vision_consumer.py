"""Consumer tests for interchangeable fixture/off vision backends."""

from pathlib import Path
import sys
import unittest

WORKSPACE = Path(__file__).resolve().parents[1]
PKG = WORKSPACE/"linorobot2_vision"
sys.path.insert(0, str(PKG/"python"))

from linorobot2_vision import (  # noqa: E402
    FixtureVisionBackend,
    VisionOffBackend,
    VisionResultConsumer,
)


class BrokenBackend:
    def get_response(self):
        raise OSError("fixture transport failure")


class BadPayloadBackend:
    def get_response(self):
        return {"schema_version": "99", "status": "ok"}


class VisionConsumerTests(unittest.TestCase):
    def backend(self, fixture):
        return FixtureVisionBackend(PKG/"fixtures"/fixture)

    def test_mock_and_off_backends_share_consumer_interface(self):
        mock = VisionResultConsumer(self.backend("detection.json"), 500_000_000)
        off = VisionResultConsumer(VisionOffBackend(), 500_000_000)
        seen = mock.consume(1_500_000_000, "ros_sim")
        disabled = off.consume(1_500_000_000, "ros_sim")
        self.assertEqual(seen.state, "ok")
        self.assertEqual(len(seen.detections), 1)
        self.assertEqual(seen.age_ns, 250_000_000)
        self.assertEqual(disabled.state, "unavailable")
        self.assertEqual(disabled.error_code, "vision_disabled")
        self.assertEqual(disabled.detections, ())
        self.assertFalse(hasattr(seen, "cmd_vel"))

    def test_valid_empty_result_is_not_an_error_or_unavailable(self):
        consumer = VisionResultConsumer(self.backend("empty.json"), 250_000_000)
        outcome = consumer.consume(2_100_000_000, "ros_sim")
        self.assertEqual(outcome.state, "empty")
        self.assertEqual(outcome.detections, ())
        self.assertEqual(outcome.age_ns, 100_000_000)

    def test_stale_future_and_cross_clock_results_fail_closed(self):
        backend = self.backend("detection.json")
        consumer = VisionResultConsumer(backend, 100_000_000)
        self.assertEqual(consumer.consume(2_000_000_000, "ros_sim").state, "stale")
        self.assertEqual(consumer.consume(1_000_000_000, "ros_sim").state, "future")
        self.assertEqual(consumer.consume(1_500_000_000, "unix_utc").state,
                         "clock_mismatch")
        self.assertEqual(consumer.consume(-1, "ros_sim").state, "error")

    def test_backend_and_contract_errors_never_produce_detections(self):
        for backend, expected_state, expected_code in (
                (BrokenBackend(), "error", "backend_exception"),
                (BadPayloadBackend(), "error", "invalid_response")):
            with self.subTest(code=expected_code):
                outcome = VisionResultConsumer(backend, 100).consume(100, "ros_sim")
                self.assertEqual(outcome.state, expected_state)
                self.assertEqual(outcome.error_code, expected_code)
                self.assertEqual(outcome.detections, ())

    def test_constructor_rejects_invalid_policy_or_backend(self):
        for value in (-1, True, 1.5):
            with self.subTest(max_age_ns=value), self.assertRaises(ValueError):
                VisionResultConsumer(self.backend("detection.json"), value)
        with self.assertRaises(TypeError):
            VisionResultConsumer(object(), 100)


if __name__ == "__main__":
    unittest.main()
