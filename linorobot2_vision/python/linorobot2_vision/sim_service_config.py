"""Fail-closed configuration checks for live inference in the Fortress overlay."""

from urllib.parse import urlsplit

from .contract import VisionContractError
from .ros_adapter import parse_label_map


SIM_VISION_ENDPOINT = "http://192.168.0.150:8082/v1/vision/detection"
UNVERIFIED_METADATA = "unverified"


def validate_sim_service_config(endpoint_url, model_version, labels_version,
                                label_map_json, *, allow_unverified_metadata=False):
    """Validate explicit service provenance and labels before launching ROS nodes.

    The simulation overlay is intentionally pinned to the previously authorized
    `.150` endpoint. Verified provenance is required by default. An explicit
    unverified mode may use the literal `unverified` sentinel and an empty class
    map; detections with unmapped labels still fail closed in the adapter.
    """
    if not isinstance(endpoint_url, str):
        raise VisionContractError("service endpoint must be the authorized .150 URL")
    try:
        parsed = urlsplit(endpoint_url)
        valid_endpoint = (
            parsed.scheme == "http"
            and parsed.hostname == "192.168.0.150"
            and parsed.port == 8082
            and parsed.path == "/v1/vision/detection"
            and not parsed.username
            and not parsed.password
            and not parsed.query
            and not parsed.fragment
        )
    except ValueError:
        valid_endpoint = False
    if not valid_endpoint:
        raise VisionContractError(
            f"service endpoint must be exactly {SIM_VISION_ENDPOINT}")

    for value, name in ((model_version, "model version"),
                        (labels_version, "labels version")):
        if not isinstance(value, str) or not value.strip():
            raise VisionContractError(f"{name} is required for service mode")

    if allow_unverified_metadata:
        if (model_version.strip() != UNVERIFIED_METADATA
                or labels_version.strip() != UNVERIFIED_METADATA):
            raise VisionContractError(
                "unverified mode requires the literal 'unverified' for both versions")
        parse_label_map(label_map_json, allow_empty=True)
        return True

    if (model_version.strip() == UNVERIFIED_METADATA
            or labels_version.strip() == UNVERIFIED_METADATA):
        raise VisionContractError(
            "unverified metadata requires allow_unverified_metadata:=true")
    if model_version.strip() == "ObjectDetectionYOLOv11":
        raise VisionContractError(
            "moduleId is not a verified model-weights version")

    parse_label_map(label_map_json, allow_empty=False)
    return True


ZMQ_PROFILES = {
    "sim_local": {
        "hosts": ("127.0.0.1", "localhost", "host.docker.internal",
                  "host.internal"),
        "domain_id": "42",
        "image_port": 5555,
        "reply_port": 5556,
    },
    "sim_local_isolated": {
        "hosts": ("host.docker.internal",),
        "domain_id": "42",
        "image_port": 15555,
        "reply_port": 15556,
    },
    "hardware_71": {
        "hosts": ("192.168.0.71",),
        "domain_id": "44",
        "image_port": 5555,
        "reply_port": 5556,
    },
}


def validate_zmq_config(profile, image_endpoint, reply_endpoint, model_version,
                        labels_version, label_map_json, ros_domain_id=None):
    """Fail-closed check of a ``.71``-protocol detector endpoint pair.

    ``sim_local`` allows the normal local simulation container on Domain 42.
    ``sim_local_isolated`` allows only the dedicated OrbStack host route and
    ports 15555/15556 for a collision-free one-shot test. ``hardware_71``
    allows only the Jetson ``.71`` on Domain 44. Both endpoints must use the
    profile's host and port pair.
    """
    if profile not in ZMQ_PROFILES:
        raise VisionContractError(
            f"zmq profile must be one of {sorted(ZMQ_PROFILES)}")
    rules = ZMQ_PROFILES[profile]
    hosts = []
    for endpoint, port, name in (
            (image_endpoint, rules["image_port"], "image"),
            (reply_endpoint, rules["reply_port"], "reply")):
        try:
            parsed = urlsplit(endpoint)
            valid = (parsed.scheme == "tcp" and parsed.port == port
                     and parsed.hostname in rules["hosts"]
                     and parsed.path in ("", "/")
                     and not parsed.username and not parsed.password)
            hosts.append(parsed.hostname)
        except (ValueError, TypeError, AttributeError):
            valid = False
        if not valid:
            raise VisionContractError(
                f"zmq {name} endpoint for profile {profile} must be "
                f"tcp://<{'|'.join(rules['hosts'])}>:{port}")
    if hosts[0] != hosts[1]:
        raise VisionContractError("zmq image and reply endpoints must use the same host")
    if ros_domain_id is not None and str(ros_domain_id) != rules["domain_id"]:
        raise VisionContractError(
            f"profile {profile} requires ROS_DOMAIN_ID={rules['domain_id']}")

    for value, name in ((model_version, "model version"),
                        (labels_version, "labels version")):
        if (not isinstance(value, str) or not value.strip()
                or value.strip() == UNVERIFIED_METADATA):
            raise VisionContractError(f"{name} must be explicitly verified for zmq mode")
    parse_label_map(label_map_json, allow_empty=False)
    return True
