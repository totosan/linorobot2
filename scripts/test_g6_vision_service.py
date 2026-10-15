"""ROS-free tests for the bounded real-Vision single-frame probe."""

from pathlib import Path
import sys
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"scripts"))

from probe_g6_vision_service import (  # noqa: E402
    EXPECTED_CAMERA_FRAME_ID,
    _multipart,
    _require_expected_label,
    _validate_contract_metadata,
    _validate_probe_environment,
    _validate_source_response,
    image_to_ppm,
)


class G6VisionServiceProbeTests(unittest.TestCase):
    def test_probe_requires_the_fortress_optical_camera_frame(self):
        self.assertEqual(EXPECTED_CAMERA_FRAME_ID, "camera_optical_frame")

    def test_attach_mode_uses_domain_42_without_ignition_partition(self):
        _validate_probe_environment(
            "http://127.0.0.1:8082/detect", "42", "", True)
        with self.assertRaisesRegex(ValueError, "isolated.*partition"):
            _validate_probe_environment(
                "http://127.0.0.1:8082/detect", "42", "", False)

    def test_positive_scene_gate_requires_exact_label(self):
        empty_report = {"status": "PASS", "detections": []}
        self.assertEqual(
            _require_expected_label(empty_report, "person")["status"], "FAIL")
        positive_report = {"status": "PASS", "detections": [
            {"label": "person", "confidence": 0.91,
             "bbox_xyxy_px": [1, 2, 20, 30]}]}
        self.assertEqual(
            _require_expected_label(positive_report, "person")["status"], "PASS")

    def test_positive_scene_is_separate_and_contains_person_proxy(self):
        world_path = (ROOT/"linorobot2_gazebo"/"worlds"
                      /"fortress_g6_vision_test.sdf")
        world = ET.parse(world_path).getroot().find("world")
        self.assertEqual(world.attrib["name"], "fortress_g6_vision_test")
        person = world.find("./model[@name='g6_person_proxy']")
        self.assertIsNotNone(person)
        self.assertEqual(person.findtext("static"), "true")
        self.assertEqual(float(person.findtext("pose").split()[0]), 4.0)
        mesh_uri = person.findtext(".//visual[@name='human_mesh']/geometry/mesh/uri")
        self.assertEqual(mesh_uri, "human.dae")
        self.assertTrue((world_path.parent/mesh_uri).is_file())
        self.assertIsNotNone(person.find(".//collision[@name='torso_collision']"))

    def test_default_fortress_world_contains_camera_and_lidar_person_target(self):
        gazebo = ROOT/"linorobot2_gazebo"
        launch = (gazebo/"launch"/"fortress.launch.py").read_text()
        self.assertIn('"worlds", "fortress_test.sdf"', launch)
        self.assertIn("name='vision_view'", launch)
        self.assertIn('"vision_overlay.launch.py"', launch)
        self.assertIn("launch_arguments={'backend': 'mock'}.items()", launch)
        self.assertIn("LaunchConfiguration('vision_view'), \"' == 'false'\"", launch)
        world = ET.parse(gazebo/"worlds"/"fortress_test.sdf").getroot().find("world")
        person = world.find("./model[@name='sim_person_proxy']")
        self.assertIsNotNone(person)
        self.assertEqual(person.findtext("static"), "true")
        self.assertEqual(
            [float(value) for value in person.findtext("pose").split()[:2]],
            [4.0, -1.8])
        obstacle = world.find("./model[@name='test_obstacle']")
        obstacle_pose = [float(value) for value in obstacle.findtext("pose").split()]
        obstacle_size = [
            float(value) for value in
            obstacle.findtext("link/collision/geometry/box/size").split()]
        person_pose = [
            float(value) for value in person.findtext("pose").split()]
        torso = person.find("./link[@name='body']/collision[@name='torso_collision']")
        torso_size = [
            float(value) for value in torso.findtext("geometry/box/size").split()]
        person_near_x = person_pose[0] - torso_size[0]/2
        obstacle_near_x = obstacle_pose[0] - obstacle_size[0]/2
        line_y_at_obstacle = (
            abs(person_pose[1] + torso_size[1]/2)
            * (obstacle_near_x - 0.14) / (person_near_x - 0.14))
        self.assertGreater(line_y_at_obstacle, obstacle_size[1]/2)
        mesh_uri = person.findtext(".//visual[@name='human_mesh']/geometry/mesh/uri")
        self.assertEqual(mesh_uri, "human.dae")
        self.assertTrue((gazebo/"worlds"/mesh_uri).is_file())

    def test_contract_probe_rejects_unverified_or_invalid_metadata(self):
        _validate_contract_metadata("person", "weights-sha256:abc",
                                    "labels-v1", 0)
        for expected in (
                ("person", "", "labels-v1", 0),
                ("person", "weights-v1", "", 0),
                ("person", "weights-v1", "labels-v1", -1),
                ("person", "weights-v1", "labels-v1", True)):
            with self.subTest(expected=expected), self.assertRaises(ValueError):
                _validate_contract_metadata(*expected)

    def test_rgb_ppm_preserves_pixels_and_discards_row_padding(self):
        image = NS(width=2, height=2, encoding="rgb8", step=8,
                   data=bytes([1, 2, 3, 4, 5, 6, 99, 99,
                               7, 8, 9, 10, 11, 12, 88, 88]))
        ppm = image_to_ppm(image)
        self.assertEqual(ppm, b"P6\n2 2\n255\n"+bytes(
            [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]))

    def test_bgr_and_mono_encodings_convert_to_rgb(self):
        bgr = NS(width=1, height=1, encoding="bgr8", step=3,
                 data=bytes([30, 20, 10]))
        mono = NS(width=1, height=1, encoding="mono8", step=1,
                  data=bytes([42]))
        self.assertTrue(image_to_ppm(bgr).endswith(bytes([10, 20, 30])))
        self.assertTrue(image_to_ppm(mono).endswith(bytes([42, 42, 42])))

    def test_rejects_unsupported_or_malformed_ros_image(self):
        image = NS(width=1, height=1, encoding="yuv422", step=2, data=b"ab")
        with self.assertRaisesRegex(ValueError, "unsupported"):
            image_to_ppm(image)
        image = NS(width=2, height=1, encoding="rgb8", step=6, data=b"short")
        with self.assertRaisesRegex(ValueError, "payload length"):
            image_to_ppm(image)

    def test_multipart_contains_single_image_and_confidence_fields(self):
        boundary, body = _multipart(b"P6 test", 0.375)
        self.assertIn(f"--{boundary}".encode(), body)
        self.assertIn(b'name="image"; filename="fortress-camera.ppm"', body)
        self.assertIn(b'name="min_confidence"', body)
        self.assertIn(b"0.375", body)
        self.assertEqual(body.count(b'Content-Disposition: form-data; name="image"'), 1)

    def test_source_success_normalizes_labels_and_pixel_boxes(self):
        payload = {
            "success": True,
            "moduleId": "ObjectDetectionYOLOv11",
            "inferenceDevice": "CPU",
            "inferenceMs": 41,
            "processMs": 45,
            "analysisRoundTripMs": 48,
            "predictions": [{"label": "person", "confidence": 0.91,
                             "x_min": 1, "y_min": 2, "x_max": 20, "y_max": 30}],
            "processedBy": "must-not-be-copied",
            "timestampUTC": "not-capture-time",
        }
        capture = {"width": 640, "height": 480}
        result = _validate_source_response(payload, capture)
        self.assertEqual(result["model_name"], "ObjectDetectionYOLOv11")
        self.assertEqual(result["detections"], [{
            "label": "person", "confidence": 0.91, "bbox_xyxy_px": [1, 2, 20, 30]}])
        self.assertNotIn("processedBy", result)
        self.assertNotIn("timestampUTC", result)

    def test_rejects_out_of_bounds_box_and_failed_inference(self):
        base = {"success": True, "inferenceMs": 0, "processMs": 0,
                "analysisRoundTripMs": 0, "predictions": [{
                    "label": "person", "confidence": 0.9,
                    "x_min": 0, "y_min": 0, "x_max": 641, "y_max": 10}]}
        with self.assertRaisesRegex(ValueError, "outside"):
            _validate_source_response(base, {"width": 640, "height": 480})
        with self.assertRaisesRegex(ValueError, "model offline"):
            _validate_source_response({"success": False, "message": "model offline"},
                                     {"width": 640, "height": 480})


if __name__ == "__main__":
    unittest.main()
