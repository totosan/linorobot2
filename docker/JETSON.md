# Manual Jetson hardware-container profile

This is a local deployment profile for the inspected Jetson image, not evidence
that hardware startup has been validated. It deliberately does not change the
general development/simulation Compose file.

## Prepare on the Jetson

1. Transfer/review the intended source changes; do not overwrite the Jetson
   workspace wholesale because its checkout may contain local changes.
2. Copy `src/linorobot2/docker/.env.jetson.example` to
   `src/linorobot2/docker/.env.jetson`. Verify the image/workspace paths and
   transport values. `.env.jetson` is ignored by Git.
3. From the workspace root, use the opt-in builder service to rebuild both
   changed packages into the sourced `install` overlay:

   ```sh
   docker compose --env-file src/linorobot2/docker/.env.jetson -f src/linorobot2/docker/docker-compose.jetson.yaml --profile build run --rm --no-deps builder
   ```

   This service has no network or device passthrough, a read-only container
   filesystem, and a writable workspace mount for colcon's `build`, `install`,
   and `log` outputs. It loads bringup arguments and runs a bounded no-scan
   monitor fixture in isolation; it does not start the robot service or connect
   to physical sensors.

4. Confirm the physical-port udev aliases are present and match the labeled
   LiDAR and ESP32 cables. The profile expects `/dev/rplidar` and
   `/dev/ttyESP32` to remain in their inspected physical ports.
5. Start manually from any directory:

   ```sh
   bash /home/jetson/linorobot2_ws/src/linorobot2/docker/start_jetson_container.sh
   ```

   The wrapper checks the two aliases are character devices, resolve to
   different device nodes, and that `install/setup.bash` exists before calling
   Compose. Use Ctrl-C to stop the foreground Compose session.

## Fixed profile choices and limits

- ROS hardware profile/domain `44`; image `l4t-ros2:humble-v2.4`.
- ESP32 alias → container `/dev/ttyUSB0` (base); LiDAR alias → container
  `/dev/ttyUSB1` (A1). Only these serial devices are passed through; there is
  no `privileged` mode, whole-`/dev` mount, restart policy, camera, or autostart.
- micro-ROS transport defaults to `serial` and baudrate `921600`, matching the
  current launch/script assumption. These values must be checked against the
  firmware actually flashed to the ESP32 before hardware use.
- The isolated container inspection did not establish `RMW_IMPLEMENTATION`.
  This profile does not force an RMW; verify the deployed image's middleware and
  network behavior before treating multi-host ROS discovery as ready.
- Udev identifies the generic USB-UART devices by physical port, not by role.
  Preflight detects missing aliases and aliases resolving to one device, but
  cannot detect cables swapped between the two ports.
- The bringup retains `lidar_required:=true` by default. Its monitor allows a
   10-second startup grace and fails on more than 15 seconds without `/scan`.
   Failure shuts down only the monitor's ROS context; a launch `OnProcessExit`
   handler requests orderly launch shutdown. This replaces the old global
   `killall`/process-group termination behavior. ROS-free timing tests cover the
   grace, timeout, and recovery boundaries.

On 2026-10-12, the reviewed source files were transferred to the Jetson checkout
without touching its existing `generic_laser.urdf.xacro` edit. The isolated
builder completed `linorobot2_base` and `linorobot2_bringup`; its post-build
domain import/install checks passed, and `ros2 launch ... --show-args` loaded the
installed bringup description and listed `lidar_required:=true`. Builder source
mount is read-only; network and device access are disabled. The isolated no-scan
monitor test also passed: after the 10-second startup grace the monitor exited
with status 1, the launch observed its process-exit event, and the test launch
shut down within 25 seconds. No physical sensor, robot service, micro-ROS agent,
motor, or firmware was started. Firmware transport/
baudrate, RMW selection, cable-to-role placement, and attached-device operation
remain hardware-validation items.
