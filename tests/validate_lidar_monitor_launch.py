"""Run the lidar timeout launch fixture in isolation and assert orderly exit."""

import subprocess
import sys


command = [
    "ros2",
    "launch",
    "linorobot2_bringup",
    "lidar_monitor_guard_test.launch.py",
]
try:
    result = subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=25,
        check=False,
    )
except subprocess.TimeoutExpired as exc:
    output = exc.stdout or ""
    if isinstance(output, bytes):
        output = output.decode(errors="replace")
    print(output, file=sys.stderr)
    raise SystemExit("lidar monitor guard launch did not shut down within 25s")

print(result.stdout)
expected_failure = "Required lidar failed: No scan data received after 10.0s startup period"
if expected_failure not in result.stdout:
    raise SystemExit("monitor did not report the expected missing-scan failure")
if "LIDAR_MONITOR_EXIT_OBSERVED" not in result.stdout:
    raise SystemExit("launch did not observe the monitor process exit")
if result.returncode not in (0, 1):
    raise SystemExit(f"unexpected ros2 launch exit status: {result.returncode}")

print("isolated lidar monitor shutdown test passed")
