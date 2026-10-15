# linorobot2_vision

Phase-5 contract and adapter. The core contract remains transport-neutral and
imports no ROS, OpenCV, ML framework, or robot-control interface. A separate
optional ROS 2 node transports only validated contract-v1 JSON results.

## Response v1

  failed responses carry a separate `error` and no result.
  compare ages across these clock domains.
  `[x_min, y_min, x_max, y_max]` with exclusive right/bottom bounds.

## ROS 2 off/mock/service adapter

`ros2 run linorobot2_vision vision_adapter_node` starts the local adapter. It
supports `backend:=off`, `backend:=mock`, and the optional `backend:=service`.
It subscribes to paired `sensor_msgs/msg/Image` and
`sensor_msgs/msg/CameraInfo` messages (default `/camera/image_raw` and
`/camera/camera_info`) and publishes:

- `/vision/result` (`std_msgs/msg/String`): compact JSON contract-v1 response.
- `/vision/status` (`std_msgs/msg/String`): JSON state (`ok`, `empty`,
  `unavailable`, `stale`, `future`, `clock_mismatch`, or `error`) with stable
  error code/message and capture metadata when available.

Parameters include `backend` (`off` by default), `fixture_path` (absolute path;
defaults to the valid empty fixture), `result_topic`, `status_topic`,
`expected_frame_id`, `clock_domain` (`ros_sim` by default), `max_age_ns`,
`pair_slop_ns`, and `label_map_json`. Image/CameraInfo frames and dimensions must
match; timestamps must be within the configured pair slop. Mock detections are
bound to the paired image's timestamp, frame and dimensions, then checked against
the explicit class-ID-to-label map. The fixture's archived capture stamp is not
reused as if it came from the live image.

`off` publishes `unavailable` with `vision_disabled`; it does not publish
detections. Invalid metadata, unknown class IDs, stale/future capture times and
clock-domain mismatches fail closed as contract errors plus a status diagnostic.
The adapter has no `/cmd_vel` publisher, Nav2 dependency or actuator interface.
The off/mock node and Nav2 equivalence are the G5 acceptance; the remote service
backend is an isolated G6 feature and must not be enabled in Nav2 without a
separate review.

`FixtureVisionBackend` reads deterministic JSON responses for offline tests.
The `normalize_jetson_detection_v1` adapter maps the archived
`/v1/vision/detection` handler's `success`/`predictions` response shape into
contract v1. The adapter requires capture metadata and an explicit reviewed
label-to-class-ID map from its caller. It ignores the source's `timestampUTC`,
which the archived handler generates at response time rather than capture time.
The source model version is not returned by that handler and must be supplied
explicitly; it is never guessed from `moduleId`.

### Legacy HTTP service backend (not the `.71` inference target)

`backend:=service` sends one image per request to the configured endpoint using
`multipart/form-data` field `image`; the image is encoded in memory as PPM, so no
extra image or ML package is installed in the ROS VM. The legacy default
endpoint is `http://192.168.0.150:8082/v1/vision/detection`. It was used in
historical transport tests but is not the correct remote inference target for
the current G6 purpose; do not use this backend for a real `.71` test. For the
Jetson `.71`, use the separate ZeroMQ path documented in
[`sim_detector/README.md`](sim_detector/README.md), with
`zmq_profile:=hardware_71` and Domain 44. The HTTP service confidence threshold is `0.3`,
timeout `5 s`, and minimum request interval `1 s` (at most 60/minute, below the
service's documented 100/minute cap). `service_max_age_ns` bounds capture-to-use
age; old results fail closed.

Service mode requires explicit non-empty `service_model_version` and
`service_labels_version` values. The live API reports `moduleId` but no verified
weight version or stable class IDs. `service_label_map_json` maps reviewed
numeric IDs to exact response labels; it may be `{}` for empty-frame transport
tests, but any returned label without a mapping becomes a contract error. Do not
derive model versions or IDs from `moduleId`. The endpoint's `timestampUTC` is
response time, not image capture time; Contract v1 retains the ROS `ros_sim`
capture stamp, `camera_optical_frame`, and dimensions. `service_max_requests`
defaults to `0` (unlimited); a positive value caps submissions for bounded
acceptance runs.

The bounded probe `scripts/test_g6_vision_service.sh` submits exactly one
simulated image and records sanitized detections/latency; it starts no Nav2 and
emits no robot command. By default it starts headless Fortress in a unique
Ignition partition and refuses startup if Domain 42 already has camera or clock
publishers. Use `scripts/test_g6_vision_service.sh --attach` to reuse an existing
Domain-42 camera/CameraInfo/clock graph without starting or stopping simulation.

### Simulated person in the default Fortress world

Every standard `fortress.launch.py` start uses `fortress_test.sdf`, which now
includes a static `sim_person_proxy` at `(4.0, -1.8, 0)` beside the existing
test obstacle. Its visual is the untextured T-pose mesh `worlds/human.dae`
(~1.65 m, yaw set so it faces the robot; front axis unverified until rendered);
the collision is a simplified torso/leg box, so the planar LiDAR sees the same
object as the camera. This provides a shared target for camera and
planar-LiDAR sensor-fusion experiments. A headless run verified
the target in the camera image and LiDAR at this pose; the synthetic target is
not a detector-quality or sensor-fusion-algorithm validation. Start the regular
Fortress simulation as usual:

```bash
colcon build --packages-up-to linorobot2_gazebo --cmake-clean-cache
source install/setup.bash
ros2 launch linorobot2_gazebo fortress.launch.py
```

To open RViz with the Vision camera overlay and planar-fusion view in the same
simulation launch, use:

```bash
ros2 launch linorobot2_gazebo fortress.launch.py vision_view:=true
```

This option starts `vision_overlay.launch.py` with its default synthetic mock
backend and its own RViz configuration. It does not contact the inference
service; the ordinary Fortress RViz window is suppressed to avoid opening two
RViz instances. The Vision RViz image is `/vision/overlay/image_raw` and falls
back to the live camera stream when no capture-matched mock result is available.

This static proxy is a repeatable synthetic target, not a photorealistic or
moving human, and does not guarantee detector recognition. Its collision
geometry also means it can affect sensor-derived maps and navigation. Specialized
G4/G6 harnesses may intentionally select their own worlds instead. For an
isolated positive-label Vision check, use
`scripts/test_g6_vision_service.sh --positive-person` as described below; that
scene requires verified model/label metadata and sends one image.

For a candidate positive test use
`scripts/test_g6_vision_service.sh --positive-person`; this starts the separate
`fortress_g6_vision_test.sdf` scene and fails unless the response contains the
exact `person` label. Its target is the untextured T-pose mesh
`worlds/human.dae` with a simplified torso/leg collision box, not a
photorealistic model or guaranteed detector positive, so an empty result is an
explicit failed positive gate rather than a PASS. This mode requires verified
`G6_VISION_MODEL_VERSION`, `G6_VISION_LABELS_VERSION`, and numeric
`G6_VISION_PERSON_CLASS_ID`; it caps requests at one and routes Contract v1 to
`/vision/result` with diagnostics on `/vision/status`. Missing metadata fails
before simulator startup.
The first live call returned `ObjectDetectionYOLOv11` on CPU in about 498 ms and
found no objects in the Fortress scene. This proves one live inference request,
not recognition quality. A known-positive scene and verified model/label
metadata remain required for full G6 acceptance. The ROS service backend was
runtime-tested in the Humble VM against a loopback fake HTTP service, including
a synthetic positive response. This validates ROS transport, normalization and
publication, not connectivity to the Jetson service or real detector
recognition. The isolated one-frame probe expects the current Fortress optical
frame, `camera_optical_frame`.

The source reference is the protected local backup described in
[`SOURCE_MANIFEST.md`](../../../docs/reference/SOURCE_MANIFEST.md) (the last Jetson runtime
source snapshot). The fixture payloads are synthetic examples based on that
handler, not recorded Jetson responses. The default `off` and fixture `mock`
backends never connect to a device; the optional service backend performs a
bounded HTTP request and has no motor or navigation interface.

### Historical `.150` HTTP service path (not the current G6 target)

The `backend:=service` integration and endpoint
`http://192.168.0.150:8082/v1/vision/detection` describe historical transport
work only. `.150` is not the correct inference endpoint for the current purpose;
do not use this path for an external G6 test. Its old `unverified` metadata mode
can send images without establishing model/label provenance and is not a
positive G6 acceptance.

For an optional real inference comparison, the user can start Jetson `.71` when
needed. It uses the separate ZeroMQ protocol and the `hardware_71` profile
(Domain 44), not `backend:=service`; follow
[`sim_detector/README.md`](sim_detector/README.md). Any live image request must
be bounded to the specifically approved test and must not be connected to Nav2
or `/cmd_vel`. The default backend remains `mock` for reproducible G5 runs.

Do not start a second overlay on an existing Domain-42 graph: both adapters
would publish `/vision/result`. Stop only the current `vision_overlay.launch.py`
process group, leave Fortress running, then relaunch the overlay with
`backend:=service` and the verified metadata. The live service mode has only
passed configuration/launch preflight here; no service-mode node was started
and no new `.150` request was sent.

## Live camera overlay (synthetic fixture)

`ros2 launch linorobot2_vision vision_overlay.launch.py` starts the camera
adapter with the explicitly synthetic `detection.json` fixture, a
capture-matched pixel-box renderer, the 2D LiDAR association node, and RViz
Image/MarkerArray displays. The output topic
`/vision/overlay/image_raw` remains a live passthrough when no matching result
is available. A box is drawn only when capture stamp, frame, and dimensions
match that exact image. The RViz display name identifies it as a mock fixture.

The fusion node consumes `/vision/result`, `/camera/camera_info`, and `/scan`,
uses bounded timestamp caches and TF at scan time, and publishes Contract-v1
association diagnostics on `/vision/fused_planar` (`std_msgs/String`) plus
short-lived markers on `/vision/fused_planar_markers` (`MarkerArray`). A
matched position is the median **LiDAR surface return in the scan plane** in
`base_link`; `nearest_cluster` means multiple projected clusters were present.
Unmatched, stale, future, invalid, and TF-unavailable cases are reported rather
than represented as zero-valued positions. This output is not an object center,
not a 3D detection, and is not connected to Nav2 or `/cmd_vel`.

For projection, the Fortress camera publishes its image/CameraInfo in
`camera_optical_frame`; Xacro provides the fixed REP-style optical transform
from the physical `camera_link`. The simulated transform/configuration must be
reloaded after model changes. Fixture launch results validate transport and
geometry only; they do not prove a detector recognized a real scene object.

This demonstrates **2D image-plane box projection only**. It does not project
detections into robot/world coordinates: contract v1 carries no depth or 3D
position, and the camera is monocular. The box (`test_object`) is synthetic,
not a claim that inference recognized the red Gazebo obstacle. No Nav2 or
command interface is started by this launch.

## RDP Fixture Monitor

`scripts/x11_vision_monitor.py` opens a native X11 window on the current Linux
`DISPLAY`; it uses the system `libX11` and does not require Tkinter. It loads
`fixtures/replay_manifest.json`, renders the referenced local SVG scene and
overlays the paired contract-v1 detection box and metadata. Keys: `1` detection,
`2` valid empty result, `3` unavailable, `q` quit; the window-manager close
button also exits. The unavailable case deliberately has no fabricated image.

In the current OrbStack setup launch it from the Mac terminal with:

    orb -m ros-humble env DISPLAY=:10.0 python3 /Users/toto/Projects/linobot2_ws/scripts/x11_vision_monitor.py

`scripts/vision_replay_monitor.py` is the colored XFCE-terminal fallback. Both
views use synthetic fixtures only; neither is a live camera viewer. Installing
`python3-tk` and `x11-utils` was attempted but blocked by the environment's
execution security check; the native X11 monitor avoids those dependencies.

### Live simulated-camera view

The Phase-5 harness starts Gazebo plus RViz with `fortress_camera.rviz`; that
RViz preset shows the actual simulated `/camera/image_raw` stream. The RDP
desktop therefore has separate Gazebo and RViz windows. Status/PASS output is
printed in whichever terminal launched the harness, not in Gazebo/RViz.

**From a terminal inside the RDP desktop:**

  cd /mnt/mac/Users/toto/Projects/linobot2_ws
  source scripts/enter_sim_env.sh
  export DISPLAY=:10.0 LIBGL_ALWAYS_SOFTWARE=1 P5_KEEP_OPEN=1
  bash scripts/test_phase5_sim_camera.sh

**From the Mac terminal:**

  orb -m ros-humble env DISPLAY=:10.0 LIBGL_ALWAYS_SOFTWARE=1 P5_KEEP_OPEN=1 bash /Users/toto/Projects/linobot2_ws/scripts/test_phase5_sim_camera.sh

After five valid Image/CameraInfo pairs pass, interactive runs leave Gazebo and
RViz open. Press Enter in the launching terminal (or Ctrl+C) to stop them
cleanly. Without a TTY the harness cleans up immediately. The image is a
simulated camera and the consumer intentionally uses an empty mock; no inference,
detections, G4/Nav2 goal, or physical camera is involved.

## Simulated camera consumer probe

`bash scripts/test_phase5_sim_camera.sh` runs a bounded Fortress GUI session in
Domain 42 and checks actual `/camera/image_raw`, `/camera/camera_info`, and
`/clock` samples through `VisionResultConsumer` with an intentional empty mock.
The probe validates frame, dimensions, encoding, intrinsics, paired timestamps,
and nonuniform RGB bytes; it performs no inference and publishes no robot command.
G5 Off/Mock and the Nav2 comparison have passed; this camera probe remains an
empty-mock sensor check and does not demonstrate real inference.
## ZeroMQ detector backend (`.71` protocol)

`backend:=zmq` talks the Jetson `.71` ZeroMQ protocol (PUSH 5555 image,
REQ `detect` on 5556). Profile `sim_local` targets the local simulation
container in [sim_detector/](sim_detector/README.md) (Domain 42);
`hardware_71` targets only `192.168.0.71` (Domain 44, separate approval).
The reply carries no capture time, class ID or model version, so those come
from the camera pair and explicit `zmq_model_version`/`zmq_labels_version`/
`zmq_label_map_json` (default COCO-80, person=0). Unknown labels, malformed
boxes and detector `error` replies fail closed. One-frame probe:
`scripts/probe_zmq_vision_service.py`.
