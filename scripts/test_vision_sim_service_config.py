"""ROS-free checks for the live `.150` simulation launch preflight."""

from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(
    ROOT / "linorobot2_vision" / "python"))

from linorobot2_vision.contract import VisionContractError  # noqa: E402
from linorobot2_vision.sim_service_config import (  # noqa: E402
    SIM_VISION_ENDPOINT,
    validate_sim_service_config,
)


class VisionSimServiceConfigTests(unittest.TestCase):
    def test_accepts_authorized_endpoint_and_explicit_provenance(self):
        self.assertTrue(validate_sim_service_config(
            SIM_VISION_ENDPOINT, "weights-sha256:verified",
            "labels-v1-verified", '{"0":"person"}'))

    def test_requires_explicit_model_and_labels_versions(self):
        for model_version, labels_version in (
                ("", "labels-v1"), ("weights-v1", ""), (" ", "labels-v1")):
            with self.subTest(model=model_version, labels=labels_version):
                with self.assertRaises(VisionContractError):
                    validate_sim_service_config(
                        SIM_VISION_ENDPOINT, model_version, labels_version,
                        '{"0":"person"}')

    def test_unverified_sentinels_require_explicit_opt_in_and_allow_empty_map(self):
        with self.assertRaisesRegex(VisionContractError, "allow_unverified_metadata"):
            validate_sim_service_config(
                SIM_VISION_ENDPOINT, "unverified", "unverified", "{}")
        self.assertTrue(validate_sim_service_config(
            SIM_VISION_ENDPOINT, "unverified", "unverified", "{}",
            allow_unverified_metadata=True))
        with self.assertRaises(VisionContractError):
            validate_sim_service_config(
                SIM_VISION_ENDPOINT, "unverified", "unverified", "[]",
                allow_unverified_metadata=True)

    def test_unverified_opt_in_does_not_accept_guessed_versions(self):
        with self.assertRaisesRegex(VisionContractError, "literal 'unverified'"):
            validate_sim_service_config(
                SIM_VISION_ENDPOINT, "ObjectDetectionYOLOv11", "labels-v1", "{}",
                allow_unverified_metadata=True)

    def test_rejects_module_id_as_model_version(self):
        with self.assertRaisesRegex(VisionContractError, "moduleId"):
            validate_sim_service_config(
                SIM_VISION_ENDPOINT, "ObjectDetectionYOLOv11", "labels-v1",
                '{"0":"person"}')

    def test_requires_a_nonempty_reviewed_class_mapping(self):
        for mapping in ("{}", "[]", '{"person":0}', '{"0":" "}'):
            with self.subTest(mapping=mapping), self.assertRaises(VisionContractError):
                validate_sim_service_config(
                    SIM_VISION_ENDPOINT, "weights-v1", "labels-v1", mapping)

    def test_rejects_other_hosts_paths_credentials_and_query_strings(self):
        endpoints = (
            "http://127.0.0.1:8082/v1/vision/detection",
            "http://192.168.0.71:8082/v1/vision/detection",
            "http://192.168.0.150:8082/other",
            "https://192.168.0.150:8082/v1/vision/detection",
            "http://user:pass@192.168.0.150:8082/v1/vision/detection",
            SIM_VISION_ENDPOINT + "?debug=1",
        )
        for endpoint in endpoints:
            with self.subTest(endpoint=endpoint), self.assertRaises(VisionContractError):
                validate_sim_service_config(
                    endpoint, "weights-v1", "labels-v1", '{"0":"person"}')


if __name__ == "__main__":
    unittest.main()
