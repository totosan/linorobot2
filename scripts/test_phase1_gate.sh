#!/usr/bin/env bash
# Partieller Laufzeit-Smoke-Test für Phase 1 (kein vollständiger G1-Nachweis).
# Vollständiges G1 verlangt zusätzlich zwei saubere Starts und 15 Minuten Stabilität.
set -euo pipefail

WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "${WORKSPACE_DIR}/scripts/lib/sim_paths.sh"
STABILITY_SECONDS="${G1_STABILITY_SECONDS:-0}"
CAMERA_GUI_SECONDS="${G1_CAMERA_GUI_SECONDS:-}"
if [[ ! "$STABILITY_SECONDS" =~ ^[0-9]+$ ]]; then
    echo "FAIL: G1_STABILITY_SECONDS muss eine nichtnegative ganze Zahl sein."
    exit 1
fi
if [[ -n "$CAMERA_GUI_SECONDS" && ! "$CAMERA_GUI_SECONDS" =~ ^[0-9]+$ ]]; then
    echo "FAIL: G1_CAMERA_GUI_SECONDS muss eine nichtnegative ganze Zahl sein."
    exit 1
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
if [[ "$ROS_DOMAIN_ID" != "42" ]]; then
    echo "FAIL: Simulationstest nur in ROS_DOMAIN_ID=42 erlaubt."
    exit 1
fi
if [[ ! "${DISPLAY:-}" =~ ^:[0-9]+(\.[0-9]+)?$ ]]; then
    echo "FAIL: DISPLAY muss auf einen lokalen Linux-X11-Server zeigen (z. B. :10.0), nicht auf XQuartz."
    exit 1
fi
LOG_DIR="${HOME}/linobot2_sim/log/g1-smoke-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$LOG_DIR"

# Isolate Gazebo discovery from unrelated simulators, including stale sessions.
export IGN_PARTITION="linobot2_g1_${UID}_$(date +%Y%m%d_%H%M%S)_$$"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
set +u
source /opt/ros/humble/setup.bash
linorobot2_source_overlay
set -u

SIM_PID=""
LISTENER_PID=""
TALKER_PID=""
cleanup() {
    [[ -z "$TALKER_PID" ]] || { kill "$TALKER_PID" 2>/dev/null || true; wait "$TALKER_PID" 2>/dev/null || true; }
    [[ -z "$LISTENER_PID" ]] || { kill "$LISTENER_PID" 2>/dev/null || true; wait "$LISTENER_PID" 2>/dev/null || true; }
    if [[ -n "$SIM_PID" ]]; then
        # Supervisor owns every descendant, even Gazebo children with setsid().
        # TERM/KILL escalation and reaping inside it have a bounded deadline.
        kill -TERM "$SIM_PID" 2>/dev/null || true
        wait "$SIM_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

printf '%s\n' "[1/3] ROS Talker/Listener (Domain ${ROS_DOMAIN_ID})"
python3 "$WORKSPACE_DIR/scripts/supervised_process.py" -- \
    ros2 run demo_nodes_cpp listener --ros-args --remap chatter:=/g1_test_chatter \
    >"$LOG_DIR/listener.log" 2>&1 &
LISTENER_PID=$!
sleep 1
python3 "$WORKSPACE_DIR/scripts/supervised_process.py" -- \
    ros2 run demo_nodes_cpp talker --ros-args --remap chatter:=/g1_test_chatter \
    >"$LOG_DIR/talker.log" 2>&1 &
TALKER_PID=$!
sleep 3
kill "$TALKER_PID" "$LISTENER_PID" 2>/dev/null || true
wait "$TALKER_PID" 2>/dev/null || true
wait "$LISTENER_PID" 2>/dev/null || true
TALKER_PID=""
LISTENER_PID=""
grep -q "I heard:" "$LOG_DIR/listener.log" || {
    echo "FAIL: ROS-Nachrichtenfluss fehlt; Logs: $LOG_DIR"
    exit 1
}
echo "PASS: Talker/Listener"

printf '%s\n' "[2/3] Fortress-Roboter, GUI, Bridge und Sensoren starten"
python3 "$WORKSPACE_DIR/scripts/supervised_process.py" -- \
    ros2 launch linorobot2_gazebo fortress.launch.py >"$LOG_DIR/launch.log" 2>&1 &
SIM_PID=$!
python3 "$WORKSPACE_DIR/scripts/wait_for_fortress.py" --pid "$SIM_PID" \
    | tee "$LOG_DIR/readiness.log" || {
    echo "FAIL: Fortress-World-Service nicht bereit; Log: $LOG_DIR/launch.log"
    tail -n 80 "$LOG_DIR/launch.log"
    exit 1
}

printf '%s\n' "[3/3] ROS-Sensordaten und CameraInfo prüfen"
timeout "$((75 + 10#$STABILITY_SECONDS))" python3 -u \
    "$WORKSPACE_DIR/scripts/verify_sim_sensors.py" --duration "$STABILITY_SECONDS" \
    | tee "$LOG_DIR/sensors.log" || {
    echo "FAIL: Sensor-Smoke-Test fehlgeschlagen; Log: $LOG_DIR/launch.log"
    tail -n 80 "$LOG_DIR/launch.log"
    exit 1
}
# Consume the complete listing: grep -q can close early and make ros2 fail
# with BrokenPipeError under pipefail, despite an existing RViz node.
timeout 10 ros2 node list --no-daemon | grep '^/rviz2$' >/dev/null || {
    echo "FAIL: rviz2-Node ist nicht gestartet; Log: $LOG_DIR/launch.log"
    exit 1
}
printf '%s\n' "[G1] Lidar-Testgeometrie, tatsächliche TF/Pose und DDS-QoS prüfen"
timeout 70 python3 -u "$WORKSPACE_DIR/scripts/verify_lidar_qos.py" \
    --report "$LOG_DIR/lidar-qos.json" | tee "$LOG_DIR/lidar-qos.log" || {
    echo "FAIL: Lidar-Geometrie/QoS; Nachweis: $LOG_DIR/lidar-qos.json"
    exit 1
}
if [[ -n "$CAMERA_GUI_SECONDS" ]]; then
    printf '%s\n' "[G1] Kamerabildwechsel und GUI-Responsivität (${CAMERA_GUI_SECONDS}s) prüfen"
    timeout "$((120 + 10#$CAMERA_GUI_SECONDS))" python3 -u \
        "$WORKSPACE_DIR/scripts/verify_camera_gui.py" --supervisor-pid "$SIM_PID" \
        --duration "$CAMERA_GUI_SECONDS" --report "$LOG_DIR/camera-gui.json" \
        | tee "$LOG_DIR/camera-gui.log" || {
        echo "FAIL: Kamera/GUI; Nachweis: $LOG_DIR/camera-gui.json"
        exit 1
    }
fi
echo "PASS: partieller G1-Laufzeit-Smoke-Test. Logs: $LOG_DIR"
echo "G1 separat abzunehmen: zwei saubere Starts, 15 Minuten GUI/Server-Stabilität und GUI-Sichtprüfung."
