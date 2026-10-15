"""ROS-free contract tests for simulated-camera capture -> mock consumer."""

from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE/"scripts"))
sys.path.insert(0, str(WORKSPACE/"linorobot2_vision"/"python"))

from linorobot2_vision import VisionResultConsumer  # noqa: E402
from verify_vision_sim_camera import EmptyCameraMockBackend, validate_camera_pair  # noqa: E402


def stamp(ns):
    return NS(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000)


def pair():
    data = bytes((index % 251 for index in range(640*480*3)))
    image = NS(width=640, height=480, encoding="rgb8", step=640*3, data=data,
               header=NS(frame_id="camera_link", stamp=stamp(1_000_000_000)))
    info = NS(width=640, height=480, k=[554.383, 0, 320, 0, 554.383, 240, 0, 0, 1],
              header=NS(frame_id="camera_link", stamp=stamp(1_050_000_000)))
    return image, info


class SimulatedCameraConsumerTests(unittest.TestCase):
    def test_valid_pair_becomes_ros_sim_capture_and_empty_consumer_observation(self):
        image, info = pair()
        capture = validate_camera_pair(image, info)
        self.assertEqual(capture["clock_domain"], "ros_sim")
        self.assertEqual(capture["frame_id"], "camera_link")
        self.assertEqual((capture["width"], capture["height"]), (640, 480))
        self.assertEqual(capture["sampled_byte_values"], 251)
        backend = EmptyCameraMockBackend(capture)
        outcome = VisionResultConsumer(backend, 100_000_000).consume(
            1_100_000_000, "ros_sim")
        self.assertEqual(outcome.state, "empty")
        self.assertEqual(outcome.detections, ())
        self.assertEqual(outcome.age_ns, 100_000_000)

    def test_rejects_frame_dimension_encoding_and_pair_time_mismatch(self):
        mutations = (
            (lambda image, info: setattr(info.header, "frame_id", "other"), "frame IDs"),
            (lambda image, info: setattr(info, "width", 320), "dimensions"),
            (lambda image, info: setattr(image, "encoding", "bgr8"), "encoding"),
            (lambda image, info: setattr(image.header, "stamp", stamp(2_000_000_000)), "timestamps"),
            (lambda image, info: setattr(info, "k", [0]*9), "focal lengths"),
        )
        for mutate, message in mutations:
            image, info = pair()
            mutate(image, info)
            with self.subTest(message=message), self.assertRaisesRegex(ValueError, message):
                validate_camera_pair(image, info)

    def test_rejects_bad_step_uniform_pixels_and_short_payload(self):
        image, info = pair()
        image.step = image.width*3-1
        with self.assertRaisesRegex(ValueError, "step"):
            validate_camera_pair(image, info)
        image, info = pair()
        image.data = bytes([7])*(image.width*image.height*3)
        with self.assertRaisesRegex(ValueError, "uniform"):
            validate_camera_pair(image, info)
        image, info = pair()
        image.data = b""
        with self.assertRaisesRegex(ValueError, "byte count"):
            validate_camera_pair(image, info)

    def test_empty_backend_does_not_retain_image_payload(self):
        image, info = pair()
        capture = validate_camera_pair(image, info)
        response = EmptyCameraMockBackend(capture).get_response()
        self.assertEqual(response["status"], "ok")
        self.assertEqual(response["result"]["detections"], [])
        self.assertNotIn("image", response["result"])
        self.assertNotIn("pixel_data", response["result"])


if __name__ == "__main__":
    unittest.main()
