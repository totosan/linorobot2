# Standard-world targets, camera view, and replay

Use for repeatable simulated-person and future sensor-fusion-fixture work.
Run commands from the workspace in the Humble Linux environment, not native
macOS. Read the current project state as required by the parent skill.

## 1. Identify the actual launch and fixture

- The active standard path is
  `src/linorobot2/linorobot2_gazebo/launch/fortress.launch.py`, which defaults to
  `worlds/fortress_test.sdf`.
- Explicit `world:=...` overrides, dedicated G4/G6 worlds, and the legacy
  Classic `gazebo.launch.py` are separate surfaces. Do not add a target to every
  acceptance scene merely because the standard simulation should include it.
- At extraction time the standard target is static `sim_person_proxy` at
  `(4.0, -1.8, 0)`, with `human.dae` as its visual and a simplified torso
  collision box. Re-read the current SDF before relying on these details.
- The separate `fortress_g6_vision_test.sdf` is a positive-test candidate, not
  the standard world or a guaranteed detector-positive scene.

For new objects, follow the existing world's SDF pattern. Specify reproducible
name, pose, dimensions, units and orientation; distinguish visual geometry from
collision geometry. Planar LiDAR needs a collision shape at scan height, while
camera visibility depends on the rendered visual. A simplified collision does
not establish exact human geometry or detector quality.

## 2. Validate resources and diagnose startup failures

1. Read the full first failing launch/build log. A reported "parser error"
   may originate in Xacro expansion or a missing installed file, not world XML.
2. Check `linorobot2_description` and `linorobot2_vision` dependencies in
   `linorobot2_gazebo/package.xml`, plus installation of worlds, meshes and launch
   resources in `CMakeLists.txt`. Inspect the active package share with
   `ros2 pkg prefix --share linorobot2_gazebo` and the description equivalent.
   Source files existing on the Mac do not prove the Linux overlay contains them.
3. Inspect large DAE assets with bounded metadata/URI reads, not a full dump.
   Check meter scale, up-axis, local pose, mesh URI and any texture references.
   Preserve asset provenance/license; do not infer either from a filename or
   import a browser image merely because it is attached.
4. Build into the intended Linux overlay and source that same install tree.
   Do not mix workspace `install/` with `/home/toto/linobot2_sim/install`.
   A prior stale CMake cache referenced a nonexistent Miniconda `make`; use
   `--cmake-clean-cache` after diagnosing that cause, not a blanket deletion.

For a workspace-local Linux install:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to linorobot2_gazebo --cmake-clean-cache
source install/setup.bash
ros2 launch linorobot2_gazebo fortress.launch.py --show-args
ign sdf -k "$(ros2 pkg prefix --share linorobot2_gazebo)/worlds/fortress_test.sdf"
```

For the established external VM overlay, follow the project's current build
procedure and inspect its installed resources instead. `colcon --log-base ...`
is a global option and belongs before `build`. Missing installed
`2wd_fortress.urdf.xacro` requires rebuilding the description dependency, not
changing SDF geometry.

## 3. Select the RViz view without duplicate ownership

```bash
ros2 launch linorobot2_gazebo fortress.launch.py vision_view:=true
```

- `vision_view` defaults to `false`. When true, Fortress includes
  `vision_overlay.launch.py` with `backend=mock`, starts the adapter, image
  renderer, planar LiDAR association and Vision RViz, and suppresses ordinary
  Fortress RViz. Do not start another overlay on this graph.
- The live image topic is `/vision/overlay/image_raw`, derived from
  `/camera/image_raw`. Without a capture-matched result, it is a camera
  passthrough; a synthetic box is not a recognition of the person.
- Input camera metadata uses `camera_optical_frame` and `ros_sim` capture time.
  Retain matching Image/CameraInfo dimensions, frame and timestamps.
- For raw camera-only viewing, the existing preset is
  `linorobot2_description/rviz/fortress_camera.rviz`; select it through the
  ordinary launch's `rviz_config` argument without enabling `vision_view`.
- Run on the RDP desktop's authorized `DISPLAY` using the parent skill's
  environment. `--show-args` and a successful build do not prove rendered pixels
  or a live camera stream.
- External `.71` inference is an optional, separately authorized one-frame
  operation under the parent skill; `.150` is not the current inference target.
  Do not change this combined start's mock default or replace a parent-owned
  adapter by blindly killing the simulator.

## 4. Verify camera and LiDAR independently

First run the narrow source regression from the workspace:

```bash
python3 scripts/test_g6_vision_service.py
```

Then, only for an authorized runtime test with a defined contract:

1. Require one simulator/clock and unique camera/result ownership in Domain 42.
   Use a fresh `IGN_PARTITION`; do not start Nav2 or issue robot commands for a
   stationary visibility check.
2. Record actual Image/CameraInfo pairing, encoding, dimensions, optical frame,
   capture timing, discovered DDS QoS and successful receipt.
3. Save a complete camera frame before selecting an ROI. The existing box near
   `x=2` previously occluded a person near `x=4`; image edges, camera field of
   view and occlusion must be checked rather than assumed.
4. Attribute LiDAR returns using the target's scan-plane geometry, range and
   bearing. A sector's minimum can belong to the foreground box; receiving a
   scan alone proves neither a person return nor camera visibility.
5. Preserve logs, measurements and image links, then verify process and graph
   cleanup. On failure inspect the complete evidence; no automatic runtime retry
   or threshold/ROI changes merely to produce PASS.

Historical `10,841` blue pixels and six LiDAR returns around `4.17-4.38 m`
belong to the earlier geometric proxy run. They do not validate a subsequently
changed DAE visual or collision model. Revalidate changed geometry using a
contract suited to the actual asset; do not retain a blue-pixel gate for an
unrelated mesh appearance.

Report separately: source implementation, installation, parser/build checks,
rendered camera visibility, LiDAR observation, detector recognition, fusion
accuracy, and hardware transfer. Update `IMPLEMENTATION_STATUS.md` for evidence,
`OPEN_TASKS.md` for remaining work, and the Vision README for user commands.
Keep historical failures identifiable rather than silently rewriting them.

## 5. Reuse the implemented replay harness

Read `docs/REPLAY_ROS_CONTRACT.md` before using either Domain-43 replay:

- For recorded vision transport, use
  `scripts/test_replay_gate.sh --segment=<segment-path> --mode=mock`;
  `off` is also supported. Replay recorded `/clock` only once: do not add
  `ros2 bag play --clock` when the segment already carries `/clock`. Verify
  paired inputs, result/status delivery, single clock ownership and cleanup.
- For Nav2 input-only replay, use `scripts/segment_nav2_bag.py` to create a new
  hash-pinned segment set, then run
  `scripts/test_nav2_replay_gate.sh --segment=<segment-path>` in a clean
  Domain-43 graph. This uses only `/clock`, `/tf`, `/tf_static`, `/scan`, `/map`
  and `/odom`; it starts no simulator, localization, base controller or goal.
  Verify input receipt, lifecycle activation, both costmaps, no `/cmd_vel`
  subscriber and cleanup. Inspect the installed Humble overlay for the common
  and Fortress navigation configs before starting.

Both replays are simulation/offline evidence only. The Nav2 input replay is not
navigation-goal replay, a new G4/G5 acceptance, a physical camera test, or real
inference. Read the current `IMPLEMENTATION_STATUS.md`; one passing segment does
not prove clock-rewind multi-segment handling.
