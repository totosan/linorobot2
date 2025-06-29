# linorobot2_vision

This package is responsible for the vision capabilities of the Linorobot2. It includes nodes for:

*   **Object Detection and Laser Fusion (`vision_node.py`)**:
    *   Subscribes to synchronized laser scan data (e.g., `/scan`) and compressed image data (e.g., `/image_raw/compressed`).
    *   Performs object detection on the image frames (currently utilizing an external API via ZeroMQ for detection results).
    *   Projects laser scan points into the 3D world and then into the 2D image plane.
    *   Correlates detected objects with laser scan data to estimate their distance and position.
    *   Publishes a "marked" laser scan (`/marked_scan`) where scan points corresponding to detected objects can have different intensity values.
    *   Publishes an array of `DetectedObject` messages (`vision/objects`) containing information about each detected object, including its label, confidence, distance, and the 3D points associated with it.
    *   Requires calibration data (`calibration_result.txt`) for the extrinsic transformation between the camera and laser scanner, and a camera configuration file (`camera_fusion_config.yaml`) for intrinsic camera parameters and lens type.
    *   Parameters for topics, file paths, processing rates, and filtering thresholds can be configured via launch file parameters.

*   **Object Visualization (`object_visualization_node.py`)**:
    *   Subscribes to the `DetectedObjectArray` messages (`vision/objects`) published by the `vision_node`.
    *   Publishes `MarkerArray` messages for RViz to visualize the detected objects. This includes:
        *   Bounding boxes around objects.
        *   Text labels with object names and distances.
        *   The 3D points associated with each object.
    *   Launch arguments control aspects like marker lifetime, text size, point size, and whether to show points, labels, or distance information.

## Dependencies

*   `rclpy`
*   `sensor_msgs` (Image, CompressedImage, LaserScan, PointCloud2)
*   `cv_bridge`
*   `geometry_msgs` (Point, TransformStamped, Quaternion)
*   `std_msgs`
*   `visualization_msgs` (Marker, MarkerArray)
*   `tf2_ros`
*   `laser_geometry`
*   `PyYAML`
*   `numpy`
*   `opencv-python`
*   `pyzmq`
*   `python-quaternion`
*   `rosidl_default_generators` (for custom message generation)
*   Custom messages: `DetectedObject.msg`, `DetectedObjectArray.msg`

## Setup and Usage

1.  **Calibration**:
    *   Ensure you have a `calibration_result.txt` file in the `linorobot2_vision/data` directory. This file contains the extrinsic calibration parameters (translation and quaternion) between your camera and laser scanner.
    *   Ensure you have a `camera_fusion_config.yaml` file in the `linorobot2_vision/config` directory. This file contains the intrinsic camera parameters (fx, fy, cx, cy, k1, k2, p1/k3, p2/k4) and the lens type (`pinhole` or `fisheye`).

2.  **Build the Package**:
    ```bash
    cd ~/linorobot2_ws # Or your ROS2 workspace
    colcon build --packages-select linorobot2_vision
    source install/setup.bash
    ```

3.  **Launch**:
    The vision and visualization nodes are typically launched as part of the main Linorobot2 bringup, specifically via the `linorobot2_bringup/launch/vision.launch.py` file. This launch file handles passing the necessary parameters to the nodes.

    Example of how it might be included in a higher-level launch:
    ```python
    # In a top-level launch file
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.substitutions import FindPackageShare

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('linorobot2_bringup'),
                'launch',
                'vision.launch.py'
            ])
        ]),
        launch_arguments={
            'enable_object_visualization': 'true', # or 'false'
            # ... other arguments for vision.launch.py
        }.items()
    )
    ```

    To run the object visualization in RViz, you can use the provided RViz configuration file:
    `linorobot2_vision/rviz/object_visualization.rviz`

    The `vision_node` expects an external object detection service to be running and accessible via ZeroMQ sockets (configurable via parameters `image_sender_endpoint` and `results_receiver_endpoint`).

## Nodes

### `vision_node` (linorobot2_vision.vision_node:main)

#### Subscribed Topics

*   **`~image_topic`** (default: `/image_raw/compressed`, type: `sensor_msgs/CompressedImage`): Compressed image stream from the camera.
*   **`~scan_topic`** (default: `/scan`, type: `sensor_msgs/LaserScan`): Laser scan data.

#### Published Topics

*   **`/marked_scan`** (type: `sensor_msgs/LaserScan`): Original laser scan with intensities modified for points corresponding to detected objects.
*   **`vision/objects`** (type: `linorobot2_vision/DetectedObjectArray`): Array of detected objects with their properties (label, confidence, distance, points).
*   **`/tf_static`** (type: `tf2_msgs/TFMessage`): Publishes the static transform from camera to laser if `publish_tf` parameter is true (Note: this functionality might be implicit or handled by other parts of the system, check `vision_node.py` for exact TF publishing behavior).
*   **`/image_republish`** (type: `sensor_msgs/Image`): Republishes the processed image with visual annotations (e.g., bounding boxes, laser points). The exact topic name might vary based on implementation details within `vision_node.py`.

#### Parameters

*   `scan_topic`: (string, default: "/scan") Topic for laser scan messages.
*   `image_topic`: (string, default: "/image_raw") Topic for image messages.
*   `calib_file`: (string, default: "calib.yaml") Path to the camera-laser extrinsic calibration file.
*   `config_file`: (string, default: "config.yaml") Path to the camera intrinsic configuration file.
*   `laser_point_radius`: (int, default: 3) Radius for drawing laser points on the image.
*   `time_diff`: (double, default: 0.1) Maximum allowed time difference (seconds) for message synchronization.
*   `max_laser_distance`: (double, default: 5.0) Maximum distance (meters) for laser points to be visualized/processed.
*   `min_points_for_depth_filter`: (int, default: 5) Minimum number of laser points within an object's bounding box to apply median depth filtering.
*   `object_depth_filter_tolerance`: (double, default: 0.25) Tolerance (meters) around the median depth for filtering object points.
*   `image_sender_endpoint`: (string, default: "tcp://localhost:5555") ZeroMQ PUSH socket endpoint for sending images to the detection API.
*   `results_receiver_endpoint`: (string, default: "tcp://localhost:5556") ZeroMQ REQ socket endpoint for receiving detection results from the API.
*   `api_confidence_threshold`: (double, default: 0.3) Minimum confidence score from the detection API to consider a detection valid.

### `object_visualization_node` (linorobot2_vision.object_visualization_node:main)

#### Subscribed Topics

*   **`vision/objects`** (type: `linorobot2_vision/DetectedObjectArray`): Detected objects from the `vision_node`.

#### Published Topics

*   **`vision/visualization_marker_array`** (type: `visualization_msgs/MarkerArray`): Markers for visualizing detected objects in RViz.

#### Parameters

*   `marker_lifetime`: (double, default: 2.0) Lifetime of the visualization markers in seconds.
*   `text_size`: (double, default: 0.2) Size of the text labels for objects.
*   `point_size`: (double, default: 0.05) Size of the LiDAR points visualized for each object.
*   `show_points`: (bool, default: true) Whether to display the LiDAR points associated with detected objects.
*   `show_labels`: (bool, default: true) Whether to display text labels for detected objects.
*   `show_distance`: (bool, default: true) Whether to display the estimated distance to detected objects.

## Custom Messages

### `DetectedObject.msg`
```c++
std_msgs/Header header
string name
float32 confidence
float32 distance
geometry_msgs/Point[] points # 3D points in laser_frame associated with the object
uint32 tracking_id # Optional tracking ID
```

### `DetectedObjectArray.msg`
```c++
std_msgs/Header header
linorobot2_vision/DetectedObject[] objects
```

This README provides a high-level overview. For detailed implementation, refer to the source code within the package and the main Linorobot2 documentation.
The ROS distribution targeted is ROS Humble.
