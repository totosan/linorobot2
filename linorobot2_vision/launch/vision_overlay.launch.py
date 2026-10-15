"""Live simulated-camera view with explicitly synthetic mock detections."""

import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from linorobot2_vision.sim_service_config import (
    SIM_VISION_ENDPOINT,
    UNVERIFIED_METADATA,
    validate_sim_service_config,
    validate_zmq_config,
)
from linorobot2_vision.zmq_backend import coco80_label_map


def validate_backend_configuration(context):
    backend = LaunchConfiguration("backend").perform(context)
    if backend == "zmq":
        try:
            validate_zmq_config(
                LaunchConfiguration("zmq_profile").perform(context),
                LaunchConfiguration("zmq_image_endpoint").perform(context),
                LaunchConfiguration("zmq_reply_endpoint").perform(context),
                LaunchConfiguration("zmq_model_version").perform(context),
                LaunchConfiguration("zmq_labels_version").perform(context),
                LaunchConfiguration("zmq_label_map_json").perform(context),
                ros_domain_id=os.environ.get("ROS_DOMAIN_ID"),
            )
        except ValueError as exc:
            raise RuntimeError(f"zmq inference preflight failed: {exc}") from exc
        return []
    if backend != "service":
        return []
    allow_unverified = (
        LaunchConfiguration("allow_unverified_metadata").perform(context) == "true")
    try:
        validate_sim_service_config(
            LaunchConfiguration("service_endpoint_url").perform(context),
            LaunchConfiguration("service_model_version").perform(context),
            LaunchConfiguration("service_labels_version").perform(context),
            LaunchConfiguration("service_label_map_json").perform(context),
            allow_unverified_metadata=allow_unverified,
        )
    except ValueError as exc:
        raise RuntimeError(f"live simulation inference preflight failed: {exc}") from exc
    return []


def generate_launch_description():
    share = Path(get_package_share_directory("linorobot2_vision"))
    fixture = str(share / "fixtures" / "detection.json")
    rviz_config = str(share / "rviz" / "vision_overlay.rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "backend",
            default_value="mock",
            choices=["mock", "service", "zmq"],
            description="Synthetic mock, legacy HTTP service backend, or a .71-protocol ZeroMQ detector (see zmq_profile)",
        ),
        DeclareLaunchArgument(
            "service_endpoint_url",
            default_value=SIM_VISION_ENDPOINT,
            description="Legacy HTTP service endpoint (.150; not the current .71 inference target)",
        ),
        DeclareLaunchArgument(
            "service_model_version",
            default_value=UNVERIFIED_METADATA,
            description="Verified weights version or literal 'unverified' with explicit opt-in",
        ),
        DeclareLaunchArgument(
            "service_labels_version",
            default_value=UNVERIFIED_METADATA,
            description="Verified label schema or literal 'unverified' with explicit opt-in",
        ),
        DeclareLaunchArgument(
            "service_label_map_json",
            default_value="{}",
            description="Reviewed JSON mapping from numeric class IDs to exact labels",
        ),
        DeclareLaunchArgument(
            "allow_unverified_metadata",
            default_value="false",
            choices=["false", "true"],
            description="Explicitly allow inference with unverified versions and no class map",
        ),
        DeclareLaunchArgument(
            "zmq_profile", default_value="sim_local",
            choices=["sim_local", "hardware_71"],
            description="sim_local: local sim detector container (Domain 42); "
                        "hardware_71: Jetson .71 (Domain 44)"),
        DeclareLaunchArgument(
            "zmq_image_endpoint", default_value="tcp://host.docker.internal:5555",
            description="Detector image PULL endpoint (port 5555)"),
        DeclareLaunchArgument(
            "zmq_reply_endpoint", default_value="tcp://host.docker.internal:5556",
            description="Detector detect REP endpoint (port 5556)"),
        DeclareLaunchArgument(
            "zmq_model_name", default_value="yolo12n",
            description="Model name reported in contract v1"),
        DeclareLaunchArgument(
            "zmq_model_version", default_value="",
            description="Verified weights version, e.g. weights-sha256:<hash>; required"),
        DeclareLaunchArgument(
            "zmq_labels_version", default_value="coco80-ultralytics",
            description="Label schema version matching zmq_label_map_json"),
        DeclareLaunchArgument(
            "zmq_label_map_json",
            default_value=json.dumps(coco80_label_map(), separators=(",", ":")),
            description="Reviewed class-ID to label map (default COCO-80, person=0)"),
        DeclareLaunchArgument(
            "zmq_timeout_sec", default_value="60.0",
            description="Per-request timeout; the first request loads the model lazily on .71"),
        OpaqueFunction(function=validate_backend_configuration),
        Node(
            package="linorobot2_vision",
            executable="vision_adapter_node",
            name="vision_adapter_sim",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "backend": ParameterValue(
                    LaunchConfiguration("backend"), value_type=str),
                "fixture_path": fixture,
                "expected_frame_id": "camera_optical_frame",
                "clock_domain": "ros_sim",
                "label_map_json": '{"1":"test_object"}',
                "service_endpoint_url": ParameterValue(
                    LaunchConfiguration("service_endpoint_url"), value_type=str),
                "service_model_version": ParameterValue(
                    LaunchConfiguration("service_model_version"), value_type=str),
                "service_labels_version": ParameterValue(
                    LaunchConfiguration("service_labels_version"), value_type=str),
                "service_label_map_json": ParameterValue(
                    LaunchConfiguration("service_label_map_json"), value_type=str),
                "zmq_image_endpoint": ParameterValue(
                    LaunchConfiguration("zmq_image_endpoint"), value_type=str),
                "zmq_reply_endpoint": ParameterValue(
                    LaunchConfiguration("zmq_reply_endpoint"), value_type=str),
                "zmq_model_name": ParameterValue(
                    LaunchConfiguration("zmq_model_name"), value_type=str),
                "zmq_model_version": ParameterValue(
                    LaunchConfiguration("zmq_model_version"), value_type=str),
                "zmq_labels_version": ParameterValue(
                    LaunchConfiguration("zmq_labels_version"), value_type=str),
                "zmq_label_map_json": ParameterValue(
                    LaunchConfiguration("zmq_label_map_json"), value_type=str),
                "zmq_timeout_sec": ParameterValue(
                    LaunchConfiguration("zmq_timeout_sec"), value_type=float),
                "service_min_interval_sec": 1.0,
                "service_timeout_sec": 5.0,
                "service_max_age_ns": 1_000_000_000,
                "service_max_requests": 0,
            }],
        ),
        Node(
            package="linorobot2_vision",
            executable="vision_image_overlay_node",
            name="vision_image_overlay",
            output="screen",
            parameters=[{
                "image_topic": "/camera/image_raw",
                "result_topic": "/vision/result",
                "overlay_topic": "/vision/overlay/image_raw",
            }],
        ),
        Node(
            package="linorobot2_vision",
            executable="vision_lidar_fusion_node",
            name="vision_lidar_fusion_sim",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz_vision_overlay",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": True}],
        ),
    ])
