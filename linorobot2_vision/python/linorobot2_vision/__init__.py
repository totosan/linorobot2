"""ROS-independent vision contract for linorobot2."""

from .adapters import normalize_jetson_detection_v1
from .contract import (
    CLOCK_DOMAINS,
    SCHEMA_VERSION,
    FixtureVisionBackend,
    VisionClockDomainError,
    VisionContractError,
    VisionFutureResultError,
    VisionNoResultError,
    VisionStaleResultError,
    load_response,
    require_fresh_result,
    validate_response,
)
from .replay import ReplayCase, VisionReplayError, load_replay_case, load_svg_scene
from .consumer import ConsumerOutcome, VisionOffBackend, VisionResultConsumer
from .ros_adapter import (
    bind_fixture_capture,
    capture_from_camera_pair,
    error_response,
    parse_label_map,
    response_for_outcome,
    status_payload,
)
from .service_backend import VisionHTTPServiceBackend

__all__ = [
    "CLOCK_DOMAINS",
    "SCHEMA_VERSION",
    "FixtureVisionBackend",
    "VisionClockDomainError",
    "VisionContractError",
    "VisionFutureResultError",
    "VisionNoResultError",
    "VisionStaleResultError",
    "ReplayCase",
    "VisionReplayError",
    "ConsumerOutcome",
    "VisionOffBackend",
    "VisionResultConsumer",
    "VisionHTTPServiceBackend",
    "bind_fixture_capture",
    "capture_from_camera_pair",
    "error_response",
    "load_response",
    "load_replay_case",
    "load_svg_scene",
    "normalize_jetson_detection_v1",
    "parse_label_map",
    "require_fresh_result",
    "response_for_outcome",
    "status_payload",
    "validate_response",
]