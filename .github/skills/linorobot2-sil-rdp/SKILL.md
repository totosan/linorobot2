---
name: linorobot2-sil-rdp
description: "Use when working in the Linorobot2 ROS 2 Humble/Gazebo Fortress workspace: prepare, activate, or roll back reproducible simulation releases; start, stop, or troubleshoot RDP Gazebo/RViz; add simulated targets; validate SDF/DAE assets, camera and LiDAR visibility; open the Vision live camera view; run the local .71-protocol detector or diagnose camera/LiDAR fusion; diagnose launch/build/parser errors; run replay; source the VM environment; control simulated teleop; investigate CPU/TF/clock issues; or perform a separately authorized bounded Jetson .71 check. The .150 HTTP service is not the current inference target. Never access hardware without separate authorization."
user-invocable: true
---

# Linorobot2 SIL and RDP workflow

Use this skill for repeatable work in the Linorobot2 simulation workspace. The simulation is a test fixture, not a hardware model or safety certification. Always read the current project state before acting; gate status and launch details can change.

## Read current project state first

Read these files before changing runtime/configuration or claiming a gate result:

- `docs/DEVELOPMENT_PLAN.md` — authoritative sequence and acceptance gates.
- `docs/OPEN_TASKS.md` — current prioritized work.
- `docs/IMPLEMENTATION_STATUS.md` — measured results, failures, limits, and known blockers.
- `docs/RDP_GUIDE.md` — current RDP and Gazebo/RViz usage.

The canonical Git root is `src/linorobot2`; active project docs, scripts,
instructions, skills, and the RDP profile are inside it. Preserve all existing
tracked and untracked changes; never run `git clean`, reset, or blanket
cleanup.

## Environment contract

- Runtime host: OrbStack VM `ros-humble`, Ubuntu 22.04 ARM64, ROS 2 Humble, Gazebo Fortress.
- Simulation profile: `ROS_DOMAIN_ID=42`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `LINOROBOT2_BASE=2wd`.
- Prefer `source scripts/enter_sim_env.sh` from the canonical checkout. It
  resolves `/home/toto/linobot2_sim/active` and falls back to the legacy
  `/home/toto/linobot2_sim/install` only while no active release exists.
- Simulation time is node-specific (`use_sim_time=true` for simulated consumers); do not globally substitute wall time for `/clock`.
- The top-level Fortress launch enforces `ros_profile:=sim`, `ros_domain_id:=42`; the bringup launch is hardware-only `hardware/44`. Domain 43 has separate vision-transport and input-only Nav2 replay harnesses; use `docs/REPLAY_ROS_CONTRACT.md`, `scripts/test_replay_gate.sh`, and `scripts/test_nav2_replay_gate.sh`. Do not imply that a costmap-input replay is navigation-goal replay or a new G4/G5 acceptance. ROS domains are not security boundaries.

## Reproducible simulation releases

Use `docs/G9_LIFECYCLE.md` as the authoritative fresh-build, activation,
recovery, and rollback procedure. Releases under
`/home/toto/linobot2_sim/releases/<id>` are immutable evidence-bearing
checkouts with separate build/install/log paths. Build only a pushed commit
from its configured remote; do not treat the dirty development checkout as a
release.

Activation and rollback use the guarded `scripts/activate_sim_release.sh` and
`scripts/rollback_sim_release.sh` entry points. Do not rewrite `active` or
`previous` manually, activate an unvalidated manifest, or switch releases
while owned simulator processes are running. After a switch, source
`scripts/enter_sim_env.sh`, verify the resolved package prefix, and use the
bounded G9 smoke/cleanup contract rather than inferring success from a build.

## Nav2 input-only bag replay

The Nav2 input replay starts no Fortress, localization, goal, hardware driver,
or base controller. Read `docs/REPLAY_ROS_CONTRACT.md` and the current
`docs/IMPLEMENTATION_STATUS.md` before a run; a pass proves only input delivery
and Nav2 costmap publication for the selected segment.

The segmenter reads the original sqlite3 bag without modifying it and emits
hash-pinned segments using only `/clock`, `/tf`, `/tf_static`, `/scan`, `/map`,
and `/odom`. It splits on backwards `/clock` and carries earlier static TF and
the latest map into subsequent segments. Use a new, non-existing output path:

```bash
python3 scripts/segment_nav2_bag.py \
  --bag log/g5_runtime/SELECTED_RUN/off/rosbag \
  --output log/replay_runtime/nav2/NEW_SEGMENT_SET
```

Before replay, require an empty Domain-43 graph and validate that the Linux
Humble overlay has `nav2_bringup`, the replay profile, `navigation.launch.py`,
`navigation_common.yaml`, and `navigation_fortress.yaml`. A Mac source file
does not prove it is installed in the VM overlay. If a package/config is
missing, report the exact artifact, expected install path and impact; recommend
a targeted package build and ask permission before modifying the overlay. The
established VM workspace is mounted at `/Users/toto/Projects/linobot2_ws`;
`colcon --log-base ...` is a global option and precedes `build`. After an
authorized build, source the same `/home/toto/linobot2_sim/install` overlay and
check the installed file and composed parameters before launching Nav2.
For example, install only the navigation package into that established overlay:

```bash
source /opt/ros/humble/setup.bash
cd /Users/toto/Projects/linobot2_ws
colcon --log-base /home/toto/linobot2_sim/log build \
  --base-paths src/linorobot2 \
  --build-base /home/toto/linobot2_sim/build \
  --install-base /home/toto/linobot2_sim/install \
  --packages-select linorobot2_navigation
source /home/toto/linobot2_sim/install/setup.bash
ros2 pkg prefix --share linorobot2_navigation
```

Run only a validated segment, in Domain 43, after verifying no other replay or
ROS graph is active:

```bash
ROS_DOMAIN_ID=43 bash scripts/test_nav2_replay_gate.sh \
  --segment="$(realpath log/replay_runtime/nav2/NEW_SEGMENT_SET/segment-000)"
```

The gate composes the installed common and Fortress profiles through the
navigation helper. Its order is significant: start the six one-shot input
probes before the player, start playback, observe input publishers, then wait
for Nav2 lifecycle activation and inspect Nav2 subscriptions after the probes
finish. Nav2 may need replayed `/tf` before lifecycle activation; before playback
the `/tf` graph endpoint may not yet be discoverable. Delaying the subscriber
snapshot until probes finish also prevents the CLI probes themselves from
being mistaken for Nav2 consumers.

The gate checks delivered samples, both costmaps, no `/cmd_vel` subscriber, and
graph cleanup. A PASS writes `replay-report.json` with the segment-manifest
hash, sample references and cleanup observations; it records one-or-more
samples, not complete message totals. Retain the per-topic verbose endpoint
snapshots to assess offered/requested QoS. It issues no goals and replays no
computed outputs. On failure, inspect saved player/Nav2 logs, stop/verify owned
processes and graph residue, update the implementation status, and do not retry
unless the user separately authorizes a new run. One successful segment does
not validate clock-rewind splitting, G4/G5-Mock bags, goal replay, or the passed
G4/G5 gates themselves.

## Standard-world targets and Vision live view

For a human/person in the default simulation, future sensor-fusion objects,
SDF/DAE loading errors, or the integrated RViz camera view, follow
[Standard-world targets, camera view, and replay](./references/simulation-targets-and-vision.md).
It covers source-versus-installed resources, sensor visibility, launch ownership,
and separating historical fixture measurements from the current asset.

The current integrated entry point is:

```bash
ros2 launch linorobot2_gazebo fortress.launch.py vision_view:=true
```

This includes the Vision overlay with `backend=mock` and replaces the ordinary
Fortress RViz window with the Vision RViz view. It is not a live detector test
and must not silently enable external inference.

## Optional real Jetson `.71` inference

The local G6 camera-to-inference-to-Contract-v1 path has already passed with
the Mac-hosted CPU `sim_detector`, which imitates the `.71` ZeroMQ protocol; it
is neither the fixture-only ROS `backend:=mock` nor the real Jetson. A real
TensorRT comparison is optional. The user starts the `.71` service when wanted
and must separately authorize the specific image request. Do not start, probe,
or send images to `.71` just because it is reachable.

`.71` uses ZeroMQ PULL `tcp://192.168.0.71:5555` for images and REP
`tcp://192.168.0.71:5556` for `detect`, on the `hardware_71` profile and ROS
Domain 44. Before any request, verify the user-started service identity and
current engine/weights hash plus label schema from its startup/export evidence;
do not reuse a hash from an earlier service start or guess metadata from a
module name. The prior one-image `.71` test had zero detections and did not
prove positive recognition or ROS Contract-v1 transport.

For an explicitly approved one-frame check, use only a bounded one-shot
procedure such as `scripts/probe_zmq_vision_service.py`, which sends one
synchronized Image/CameraInfo capture, does not retry, start a simulator,
launch Nav2, or publish robot commands. It must target the `hardware_71`
profile in Domain 44 with the verified engine version and reviewed label map.
The probe requires a matching camera stream in Domain 44. Do not change the
standard simulator's enforced Domain 42 or attach to a hardware camera graph
to work around a missing approved source. If a safe approved Domain-44 capture
source is unavailable, stop before inference and ask for direction.

Do not use `backend:=service` or the legacy `.150` HTTP URL for `.71`. Do not
use the ordinary `backend:=zmq` live overlay for a one-shot test: it can keep
sending camera frames and has no one-request cap. Any continuous live
streaming would require distinct, explicit authorization and a verified stop
procedure. Keep Nav2 and `/cmd_vel` disconnected from detections.

## Local `.71`-protocol detector and camera/LiDAR fusion

Use the local `simdet` CPU container when simulated-camera inference must not
depend on `.71`. It implements the Jetson ZeroMQ wire protocol but uses
PyTorch CPU weights; inference accuracy and latency are not hardware-equivalent.
The adapter's `sim_local` profile is for Domain 42 and the container reachable
from the VM at `host.docker.internal:5555/5556`. Install `pyzmq` in the ROS
environment, start the container as described in
`linorobot2_vision/sim_detector/README.md` from the canonical checkout,
and use the weights SHA-256 printed by its startup log as the version. Do not
reuse a stale hash after changing weights.

With the simulation already running and ownership verified, start exactly one
Vision overlay in a separate terminal:

```bash
ROS_DOMAIN_ID=42 ros2 launch linorobot2_vision vision_overlay.launch.py backend:=zmq zmq_model_version:=weights-sha256:<printed-hash>
```

The launch starts the adapter, capture-matched image overlay, planar fusion
node, and dedicated Vision RViz. Do not start this alongside an existing
`vision_view:=true` parent launch: first inspect node/topic publishers and
launch ownership. The `hardware_71` profile uses Domain 44 and the real Jetson service; it
requires the specific inference request to be separately authorized and must
not be substituted into a Domain-42 simulation workflow.

Diagnose in data-path order rather than relying on the RViz picture:

1. Check that `/vision/result` has one publisher and inspect the full message,
   `/vision/status`, and `/vision/overlay/image_raw`. Verify captured
   `frame_id`, dimensions, and ROS simulation timestamp. Detection on the
   image does not prove LiDAR association.
2. Check `/scan`, `/camera/camera_info`, and transforms at the capture time.
   Fusion matches results, camera info, and scans within configured timestamp
   tolerances (default 50 ms) and requires transforms from the scan frame to
   `camera_optical_frame` and `base_link`.
3. Inspect the fused diagnostic and marker topics:

   ```bash
   ros2 topic echo --once --full-length /vision/fused_planar
   ros2 topic echo --once /vision/fused_planar_markers
   ```

   `/vision/fused_planar` is JSON diagnostics; successful association reports
   `state: ok` and per-detection LiDAR status such as `nearest_cluster`.
   `/vision/fused_planar_markers` carries short-lived RViz markers. Empty or
   delete-only markers can mean no valid fused hit, even while detections exist.
4. In Vision RViz set Fixed Frame to `base_link`, enable
   `/vision/fused_planar_markers`, and check marker status/errors. If displaying
   `/scan`, use Best Effort reliability because LaserScan is a sensor-data
   topic. The camera panel shows `/vision/overlay/image_raw`; the 3D panel is a
   separate view, not an overlay of the camera box in world coordinates.

Fusion is a 2D scan-plane association at capture time, not 3D reconstruction.
The reported position is a LiDAR surface return in `base_link`, not an object's
center or a camera-derived metric position. The target needs a collision shape
intersecting the LiDAR scan plane; a visible visual mesh alone is insufficient.
Do not feed detections or fused markers into `/cmd_vel`, Nav2, or a safety path.
For current SDF assets, RViz settings, isolated positive-test constraints and
source/install checks, follow
[Standard-world targets, camera view, and replay](./references/simulation-targets-and-vision.md)
and the Vision README. Record each local run as simulation evidence only; do not
claim `.71`, TensorRT, G5, or hardware performance from the local detector.

## RDP and visualization

1. RDP needs no Mac-side tunnel. A VM systemd bridge
   (`linorobot2-rdp-bridge.socket`/`.service`) starts with the VM, listens on
   port 13389 and proxies to loopback-only xrdp `127.0.0.1:3389`; OrbStack
   mirrors that port to the Mac. Connect the versioned `ros-humble-rdp.rdp`
   profile in Windows App to `127.0.0.1:13389`, without a gateway. Manage the
   bridge inside the VM with
   `bash scripts/configure_rdp_bridge.sh --status | --enable | --disable | --check-exposure`.
   Never add `BindToDevice=` to the socket: OrbStack does not activate network
   device units, so the dependency makes the socket fail at boot.
   The access boundary depends on the OrbStack setting
   `machines.expose_ports_to_lan`. It must stay `false`, otherwise the Mac
   binds the forward on all interfaces and exposes the RDP login to the local
   network. `--enable` now refuses to activate the bridge when the host
   reports `true`, querying it from the VM via
   `mac orb config get machines.expose_ports_to_lan`; an undeterminable value
   warns and must never be treated as safe. `--status` reports the setting too.
   For a raw VM, `bash scripts/setup_rdp_env.sh` installs the desktop and xrdp
   and enables the bridge in step 5; the OrbStack setting and the Linux
   password stay manual and are not restorable from a clone.
   `scripts/connect_rdp_mac.sh` remains a manual SSH-tunnel fallback and exits
   cleanly instead of starting a second tunnel when port 13389 is taken.
2. Diagnose availability in layers before changing configuration:
   - Mac listener: `lsof -nP -iTCP:13389 -sTCP:LISTEN` (expect `127.0.0.1` only)
   - Guest bridge: `orb -m ros-humble bash -lc 'bash scripts/configure_rdp_bridge.sh --status'`
   - Guest services/listeners:
     `orb -m ros-humble python3 /Users/toto/Projects/linobot2_ws/src/linorobot2/scripts/check_rdp_ready.py`
   - Existing Xorg session/display: inspect `loginctl`, xrdp/Xorg processes,
     `/tmp/.X11-unix/X*`, then `DISPLAY=:10.0 xrandr --current`.

   The readiness script deliberately checks listener state without opening a
   raw TCP connection. `nc`, a bare socket connect, or similar non-RDP probes
   can create alarming xrdp protocol errors even when the service is healthy.
3. The guest desktop uses local X11 `DISPLAY=:10.0`, XFCE and Mesa/llvmpipe.
   Keep xrdp and sesman loopback-only; the documented bridge socket is the only
   exception. Never use `xhost +` or firewall changes as workarounds. The
   profile pins 1280×800 with `dynamic resolution:i:0` to cut llvmpipe CPU
   cost, not because of a protocol limit; xorgxrdp refuses a later RandR
   framebuffer change, so the size is only settled at connection setup.
4. In an XFCE terminal inside the RDP desktop, start the stationary Gazebo+RViz view:

   ```bash
   bash scripts/start_gazebo_rdp.sh
   ```

   From a Mac workspace terminal, the equivalent is:

   ```bash
   orb -m ros-humble env DISPLAY=:10.0 LIBGL_ALWAYS_SOFTWARE=1 bash /Users/toto/Projects/linobot2_ws/src/linorobot2/scripts/start_gazebo_rdp.sh
   ```

   The launcher validates local display/domain, selects a unique `IGN_PARTITION`, uses `LP_NUM_THREADS=2` by default, enables Gazebo GUI and `fortress.rviz`, starts no Nav2, and sends no navigation goal. It records logs under `/home/toto/linobot2_sim/log/rdp-gazebo-*`; latest path is in `rdp-gazebo-current.txt`.
5. Prefer stopping from the launch terminal with `Ctrl+C`; the supervisor cleans up its process tree. For stopping multiple or detached VM simulation sessions, use the shutdown procedure below. Before starting again, verify no older Fortress launch is active. Never start a second world in Domain 42.
6. XDG Gazebo autostart is optional and is not assumed enabled. Configure only with `scripts/configure_rdp_gazebo_autostart.sh --enable`; undo with `--disable`. This applies on the next XFCE login, not to the current desktop.
7. The `.rdp` profile requests a fixed 1280×800 with dynamic resize disabled.
   A disconnected Xorg session may persist at its old size; fully disconnect
   the client and reopen the profile, then verify the negotiated mode with
   `xrandr --current`. Reuse of the existing session is normal. xorgxrdp may
   reject forced framebuffer resizing—do not force `xrandr` and do not
   terminate the desktop merely to change its resolution.

## Stop simulation sessions in the VM

Use this when asked to stop/close VM sessions or when launches have become detached. “Stop the simulation sessions” does **not** mean shut down OrbStack, the VM, XFCE, xrdp, or the user's RDP connection.

1. Inventory before signaling. In the VM, inspect `ps -eo user,pid,ppid,pgid,sid,etime,args` and identify every relevant Fortress server/GUI, `ros2 launch`, `supervised_process.py`, service/vision overlay node, and separately launched RViz. Also inspect the applicable ROS domain's `ros2 node list --no-daemon` and publisher counts for `/clock`, camera, and vision topics. Include standalone RViz: it may not be a child of the Fortress launch.
2. Build an explicit target list from command lines and parent-child relationships. A launch manager, its ROS nodes, Gazebo server/GUI, a separate vision launch, and standalone RViz can be distinct processes/sessions. Process groups and sessions may differ; do not assume the launcher's process group owns every Gazebo child.
3. Prefer `Ctrl+C` in each known launch terminal. For a detached launch, immediately re-read the selected PID's command line, require an exact expected launch/supervisor identity, then send `SIGINT` to that PID only. Re-check each PID immediately before signaling; if it exited or its command changed, stop and re-inventory. Never use `killall`, broad `pkill`, `killpg`, or a process-name-only kill. Stop standalone RViz only after separately verifying its exact PID and command.
4. Let the launch/supervisor perform graceful cleanup, then inspect its terminal output and active launch log before taking further action. ROS launch may escalate its children to `SIGTERM` after its grace period; RViz can abort during shutdown. An error exit code (for example 255) or `pthread_mutex` assertion is an abnormal shutdown, not proof that children remain: do not retry or send stronger signals blindly.
5. Verify by process identity, not only the ROS graph: repeat the process inventory and confirm no targeted Gazebo, ROS launch, simulation-node, supervisor, or RViz processes remain. Then check that `/clock`, camera, and `/vision/*` no longer have publishers in the relevant domain. A lingering node name with no matching process or topic publishers can be DDS/CLI discovery residue; do not kill an unrelated process to clear it. Once graph inspection is done, `ros2 daemon stop` may stop the CLI discovery daemon; confirm the final process inventory afterward.
6. Preserve the VM/RDP desktop and unrelated services. Report which simulation processes stopped, any abnormal exit/log evidence, remaining sessions (if any), and whether verification found process or graph residue. Do not claim “all sessions stopped” until the process scan is clean.

## Simulated keyboard teleop

Only start teleop after confirming the correct simulation is running in Domain 42 and no unexpected `/cmd_vel` publisher exists. In a focused XFCE terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.08 -p turn:=0.30
```

This publishes to `/cmd_vel`. `i` forward, `,` reverse, `j`/`l` rotate, `k` stop; any other key publishes zero. `Ctrl+C` sends a final zero and exits. Do not use the teleop speed-increase keys for the bounded SIL profile. RViz visualizes; it does not control the robot. This procedure is simulation-only.

## Runtime diagnosis and evidence

### Before interpreting RViz or CPU symptoms

1. Capture `uptime`, `ps --sort=-%cpu`, simulator/launch PIDs and the exact log directory.
2. In Domain 42 inspect `ros2 node list --no-daemon`, `/clock` publisher count, `/scan`, `/odom/unfiltered`, `/odom`, `/tf`, and `ros2 topic info --verbose` as needed.
3. Check `use_sim_time`, RViz Fixed Frame (`odom` in `fortress.rviz`), and whether the required `odom -> base_footprint` transform actually exists. Raw odometry alone does not prove filtered `/odom` or TF is available.
4. Read the active launch log before changing configuration. Deprecated `ign_*` bridge warnings are known migration warnings; distinguish them from actual transport failures.

### Duplicate launches and clock/TF regressions

- Duplicate Fortress servers or bridges in the same ROS domain can create multiple `/clock` publishers, duplicated node names, backward simulation-time jumps, `TF_OLD_DATA`, cleared TF buffers, and invalid RViz displays.
- Treat duplicate node names, more than one `/clock` publisher, or repeated backward-time warnings as a failed run. Do not tune RViz/TF or continue teleop through a time reset.
- Identify the exact old supervisor and verify its command/ownership before stopping it. Send SIGINT to that supervisor only; do not use broad `pkill`, process-name kills, or automatic retries. Confirm cleanup, then start one isolated launch with a fresh `IGN_PARTITION`.
- After a time jump, EKF may fail to recover even after duplicates are removed. Prefer a clean single-launch restart when authorized; verify `/odom` and `odom -> base_footprint` before judging RViz.

### llvmpipe CPU load

- `LIBGL_ALWAYS_SOFTWARE=1` means software rendering; high Gazebo GUI CPU is expected on llvmpipe. Inspect process CPU and host load before attributing the cause.
- `LP_NUM_THREADS=2` is the default in the RDP launcher and can be overridden for a measured experiment, e.g. `LP_NUM_THREADS=4 bash scripts/start_gazebo_rdp.sh`. Fewer render threads may lower CPU peaks but reduce frame rate. Record resolution, thread count, Gazebo/Server/RViz CPU, load average, and visual responsiveness; a point sample is not a performance gate.
- If CPU remains high, lower the Windows App/RDP desktop resolution and reconnect; don't force an unsupported xorgxrdp `xrandr` mode.

### Visual evidence

- `scripts/gui_x11_probe.py` can discover and ping only GUI windows owned by the specified supervisor; a mapped window or successful ping is not proof of rendered scene pixels.
- Fortress `/gui/screenshot` expects a unique empty directory. Use the current `IGN_PARTITION`, request the screenshot service, and validate the generated PNG with `verify_camera_gui.inspect_png` (CRC, dimensions, decompression, pixel diversity). Do not move scene entities for a display check.

## Safety and reporting boundaries

- No Pi/ESP32/Jetson access, firmware changes, real motor commands, physical tests, or real inference without separate explicit authorization and a verified safe state.
- A user report that a device is online does not authenticate it. If SSH host-key verification fails, stop: do not edit `known_hosts`, disable checking, guess an IP, or route around the failure.
- Motorless HIL requires independent confirmation that motor power is physically disconnected/disabled; software or ROS state cannot prove that.
- Keep SIL, mock, replay, HIL, and physical evidence distinct. Report failed runs and unknowns without relabeling them as PASS. No automatic retry after runtime failure.
- After changes, run the narrow ROS-free tests, syntax/diff checks, then the smallest approved SIL validation. Never claim a hardware or G4/G5 gate from a build or unit test alone.
