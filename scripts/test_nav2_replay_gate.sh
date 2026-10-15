#!/usr/bin/env bash
# Run Nav2 against one validated input-only bag segment in replay/43.
set -euo pipefail

WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "${WORKSPACE_DIR}/scripts/lib/sim_paths.sh"
LOG_PARENT="${NAV2_REPLAY_LOG_PARENT:-${WORKSPACE_DIR}/log/replay_runtime/nav2}"
SEGMENT=""
for arg in "$@"; do
    case "$arg" in
        --segment=*) SEGMENT="${arg#*=}" ;;
        -h|--help)
            echo "Usage: bash $0 --segment=/absolute/path/to/segment-NNN"
            echo "Runs Nav2 without simulator, localization, goals, or a base controller."
            exit 0
            ;;
        *) echo "FAIL: unsupported argument: $arg"; exit 2 ;;
    esac
done

if [[ -n "${ROS_DOMAIN_ID:-}" && "$ROS_DOMAIN_ID" != "43" ]]; then
    echo "FAIL: Nav2 replay is restricted to ROS_DOMAIN_ID=43."
    exit 1
fi
if [[ -z "$SEGMENT" || "$SEGMENT" != /* || ! -s "$SEGMENT/metadata.yaml" ]]; then
    echo "FAIL: provide an existing absolute rosbag2 segment with metadata.yaml."
    exit 2
fi
MANIFEST="$SEGMENT.manifest.json"
if [[ ! -s "$MANIFEST" ]]; then
    echo "FAIL: validated segment manifest missing: $MANIFEST"
    exit 1
fi
python3 - "$MANIFEST" "$SEGMENT" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""):
            digest.update(block)
    return digest.hexdigest()

manifest = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
expected_topics = {
    "/clock": "rosgraph_msgs/msg/Clock",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
    "/scan": "sensor_msgs/msg/LaserScan",
    "/map": "nav_msgs/msg/OccupancyGrid",
    "/odom": "nav_msgs/msg/Odometry",
}
if (manifest.get("status") != "PASS"
        or manifest.get("scope") != "NAV2_SENSOR_INPUT_SEGMENT_V1"
        or manifest.get("capture_clock_domain") != "ros_sim"
        or manifest.get("topics") != expected_topics
        or manifest.get("hardware_topics_or_devices") != "none"):
    raise SystemExit("FAIL: manifest is not a validated Nav2 input-only segment")
root = Path(sys.argv[2])
hashes = manifest.get("file_sha256")
if not isinstance(hashes, dict) or not hashes:
    raise SystemExit("FAIL: manifest lacks segment file hashes")
for name, expected in hashes.items():
    if not isinstance(name, str) or Path(name).name != name:
        raise SystemExit(f"FAIL: invalid path in segment file hashes: {name!r}")
    path = root/name
    if not path.is_file() or sha256(path) != expected:
        raise SystemExit(f"FAIL: segment file hash mismatch: {name}")
PY

set +u
source /opt/ros/humble/setup.bash
linorobot2_source_overlay
set -u
export ROS_DOMAIN_ID=43
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
python3 -c 'from linorobot2_base.domain_profile import resolve_ros_domain; assert resolve_ros_domain("replay", "43") == "43"' || {
    echo "FAIL: installed replay/43 domain profile is unavailable."
    exit 1
}

NAVIGATION_SHARE="$(ros2 pkg prefix --share linorobot2_navigation)"
NAV2_PROFILE="$NAVIGATION_SHARE/config/navigation_fortress.yaml"
NAV2_COMMON="$NAVIGATION_SHARE/config/navigation_common.yaml"
NAV2_LAUNCH="$NAVIGATION_SHARE/launch/navigation.launch.py"
for package in nav2_bringup rosbag2_transport; do
    ros2 pkg prefix "$package" >/dev/null 2>&1 || {
        echo "FAIL: required package is unavailable: $package"
        exit 1
    }
done
for required in "$NAV2_PROFILE" "$NAV2_COMMON" "$NAV2_LAUNCH"; do
    [[ -s "$required" ]] || {
        echo "FAIL: installed Nav2 configuration artifact is missing: $required"
        exit 1
    }
done

NODES="$(ros2 node list --no-daemon)"
if printf '%s\n' "$NODES" | grep -q '[^[:space:]]'; then
    echo "FAIL: replay/43 graph is not empty before Nav2 replay:"
    printf '%s\n' "$NODES"
    exit 1
fi

mkdir -p "$LOG_PARENT"
LOG_DIR="$(mktemp -d "${LOG_PARENT}/nav2-replay-$(date +%Y%m%d-%H%M%S)-XXXXXX")"
NAV2_PID=""
PLAYER_PID=""
SAMPLE_PROBE_PIDS=()
PUBLISHER_WAIT_PIDS=()
stop_owned() {
    local pid="$1"
    if [[ -n "$pid" ]]; then
        kill -INT "$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    fi
}
cleanup() {
    stop_owned "$PLAYER_PID"
    stop_owned "$NAV2_PID"
    for pid in "${SAMPLE_PROBE_PIDS[@]}" "${PUBLISHER_WAIT_PIDS[@]}"; do
        [[ -n "$pid" ]] && wait "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

NAV2_PARAMS="$LOG_DIR/navigation-composed.yaml"
python3 - "$NAV2_LAUNCH" "$NAV2_COMMON" "$NAV2_PROFILE" "$NAV2_PARAMS" <<'PY'
import importlib.util
import sys

spec = importlib.util.spec_from_file_location("navigation_parameters", sys.argv[1])
navigation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(navigation)
navigation._compose_parameter_file(sys.argv[2], sys.argv[3], sys.argv[4])
PY

printf 'domain=43\nprofile=replay\nsegment=%s\nnav2_params=%s\n' \
    "$SEGMENT" "$NAV2_PARAMS" >"$LOG_DIR/run-profile.txt"
cp "$MANIFEST" "$LOG_DIR/input-segment.manifest.json"
sha256sum "$SEGMENT/metadata.yaml" "$MANIFEST" "$NAV2_COMMON" "$NAV2_PROFILE" \
    "$NAV2_PARAMS" >"$LOG_DIR/input-sha256.txt"
ros2 bag info "$SEGMENT" >"$LOG_DIR/segment-info.txt" 2>&1 || {
    echo "FAIL: ros2 bag info could not read the segment."
    exit 1
}

echo "[Nav2 replay] Starting navigation-only stack (no localization or base driver)."
python3 "$WORKSPACE_DIR/scripts/supervised_process.py" -- \
    ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=True autostart:=True params_file:="$NAV2_PARAMS" \
    >"$LOG_DIR/nav2.log" 2>&1 &
NAV2_PID=$!

wait_lifecycle_active() {
    local node="$1"
    for _ in $(seq 1 120); do
        if timeout 5 ros2 lifecycle get "$node" 2>/dev/null | grep -Eq '^active \[3\]$'; then
            return 0
        fi
        if grep -Eq '\[FATAL\]|process has died' "$LOG_DIR/nav2.log"; then
            echo "FAIL: Nav2 reported a fatal error during lifecycle activation."
            return 1
        fi
        if ! kill -0 "$NAV2_PID" 2>/dev/null; then
            echo "FAIL: Nav2 launch exited before lifecycle activation."
            return 1
        fi
        sleep 0.5
    done
    echo "FAIL: lifecycle node did not become active: $node"
    return 1
}

wait_for_publisher() {
    local topic="$1"
    local minimum="$2"
    local count
    local safe_name="${topic//\//_}"
    for _ in $(seq 1 100); do
        count="$(ros2 topic info "$topic" 2>/dev/null | sed -n 's/^Publisher count: //p' | head -n1 || true)"
        if [[ "$count" =~ ^[0-9]+$ ]] && (( count >= minimum )); then
            ros2 topic info "$topic" --verbose \
                >"$LOG_DIR/${safe_name}.topic-info.txt" 2>&1 || return 1
            printf '%s\n' "$count"
            return 0
        fi
        if ! kill -0 "$PLAYER_PID" 2>/dev/null; then
            break
        fi
        sleep 0.2
    done
    echo "FAIL: expected publisher on $topic while the bag was playing." >&2
    return 1
}

check_cmd_vel_unsubscribed() {
    local info
    if info="$(ros2 topic info /cmd_vel --verbose 2>&1)"; then
        :
    elif ! grep -qi 'unknown topic' <<<"$info"; then
        printf '%s\n' "$info" >"$LOG_DIR/cmd-vel-ownership.txt"
        echo "FAIL: could not inspect /cmd_vel ownership."
        return 1
    fi
    printf '%s\n' "$info" >"$LOG_DIR/cmd-vel-ownership.txt"
    if grep -Eq '^Subscription count: [1-9][0-9]*$' "$LOG_DIR/cmd-vel-ownership.txt"; then
        echo "FAIL: unexpected /cmd_vel subscriber exists in replay/43."
        return 1
    fi
    if ! grep -q '^Subscription count: 0$' "$LOG_DIR/cmd-vel-ownership.txt" \
            && ! grep -qi 'unknown topic' "$LOG_DIR/cmd-vel-ownership.txt"; then
        echo "FAIL: /cmd_vel subscriber count is unavailable."
        return 1
    fi
}
check_cmd_vel_unsubscribed || exit 1

start_sample_probe() {
    local topic="$1"
    local message_type="$2"
    shift 2
    local safe_name="${topic//\//_}"
    timeout 15 ros2 topic echo --once "$@" "$topic" "$message_type" \
        >"$LOG_DIR/${safe_name}.sample.txt" \
        2>"$LOG_DIR/${safe_name}.sample.err" &
    SAMPLE_PROBE_PIDS+=("$!")
}

echo "[Nav2 replay] Starting concurrent input sample probes."
start_sample_probe /clock rosgraph_msgs/msg/Clock --qos-reliability reliable
start_sample_probe /tf tf2_msgs/msg/TFMessage --qos-reliability reliable
start_sample_probe /tf_static tf2_msgs/msg/TFMessage \
    --qos-reliability reliable --qos-durability transient_local
start_sample_probe /scan sensor_msgs/msg/LaserScan --qos-reliability best_effort
start_sample_probe /map nav_msgs/msg/OccupancyGrid \
    --qos-reliability reliable --qos-durability transient_local
start_sample_probe /odom nav_msgs/msg/Odometry --qos-reliability reliable

echo "[Nav2 replay] Playing only recorded clock/sensor/TF/map/odometry inputs."
python3 "$WORKSPACE_DIR/scripts/supervised_process.py" -- \
    ros2 bag play "$SEGMENT" \
    --topics /clock /tf /tf_static /scan /map /odom \
    --disable-keyboard-controls \
    >"$LOG_DIR/player.log" 2>&1 &
PLAYER_PID=$!

start_sample_probe /global_costmap/costmap nav_msgs/msg/OccupancyGrid
start_sample_probe /local_costmap/costmap nav_msgs/msg/OccupancyGrid

for topic in /clock /tf /tf_static /scan /map /odom; do
    safe_name="${topic//\//_}"
    wait_for_publisher "$topic" 1 >"$LOG_DIR/${safe_name}.publisher-count.txt" &
    PUBLISHER_WAIT_PIDS+=("$!")
done
PUBLISHER_CHECK_FAILED=0
for pid in "${PUBLISHER_WAIT_PIDS[@]}"; do
    wait "$pid" || PUBLISHER_CHECK_FAILED=1
done
PUBLISHER_WAIT_PIDS=()
if [[ "$PUBLISHER_CHECK_FAILED" != "0" ]]; then
    echo "FAIL: one or more replay input publishers were not observed."
    exit 1
fi
[[ "$(cat "$LOG_DIR/_clock.publisher-count.txt")" == "1" ]] || {
    echo "FAIL: expected exactly one replay /clock publisher."
    exit 1
}
for node in /planner_server /controller_server /global_costmap/global_costmap /local_costmap/local_costmap; do
    wait_lifecycle_active "$node" || {
        cat "$LOG_DIR/nav2.log"
        exit 1
    }
done

SAMPLE_CHECK_FAILED=0
for pid in "${SAMPLE_PROBE_PIDS[@]}"; do
    wait "$pid" || SAMPLE_CHECK_FAILED=1
done
SAMPLE_PROBE_PIDS=()
if [[ "$SAMPLE_CHECK_FAILED" != "0" ]]; then
    echo "FAIL: one or more concurrent replay sample probes failed."
    exit 1
fi
for topic in /tf /tf_static /scan /map /odom; do
    safe_name="${topic//\//_}"
    ros2 topic info "$topic" --verbose \
        >"$LOG_DIR/${safe_name}.nav2-subscribers.txt" 2>&1 || {
        echo "FAIL: could not inspect Nav2 subscriptions for $topic after playback."
        exit 1
    }
    if ! grep -Eq '^Subscription count: [1-9][0-9]*$' \
            "$LOG_DIR/${safe_name}.nav2-subscribers.txt"; then
        echo "FAIL: Nav2 has no subscriber for required input $topic after playback."
        exit 1
    fi
done
for topic in /clock /tf /tf_static /scan /map /odom \
        /global_costmap/costmap /local_costmap/costmap; do
    safe_name="${topic//\//_}"
    [[ -s "$LOG_DIR/${safe_name}.sample.txt" ]] || {
        echo "FAIL: no replayed message received on $topic."
        exit 1
    }
done

wait "$PLAYER_PID" || {
    PLAYER_PID=""
    echo "FAIL: rosbag2 player exited unsuccessfully."
    exit 1
}
PLAYER_PID=""
check_cmd_vel_unsubscribed || exit 1
stop_owned "$NAV2_PID"
NAV2_PID=""
GRAPH_CLEAN=0
for _ in $(seq 1 25); do
    REMAINING_NODES="$(ros2 node list --no-daemon)"
    if ! printf '%s\n' "$REMAINING_NODES" | grep -q '[^[:space:]]'; then
        GRAPH_CLEAN=1
        break
    fi
    sleep 0.2
done
if [[ "$GRAPH_CLEAN" != "1" ]]; then
    printf 'FAIL: ROS nodes remain after owned Nav2 shutdown:\n%s\n' "$REMAINING_NODES"
    exit 1
fi
python3 - "$LOG_DIR" "$SEGMENT" "$MANIFEST" <<'PY'
import hashlib
import json
from pathlib import Path
import re
import sys

log_dir, segment, manifest = map(Path, sys.argv[1:])
topics = [
    "/clock", "/tf", "/tf_static", "/scan", "/map", "/odom",
    "/global_costmap/costmap", "/local_costmap/costmap",
]
samples = {}
for topic in topics:
    safe_name = topic.replace("/", "_")
    sample = log_dir/f"{safe_name}.sample.txt"
    if not sample.is_file() or sample.stat().st_size == 0:
        raise SystemExit(f"FAIL: sample evidence missing for {topic}")
    samples[topic] = {
        "received_one_or_more": True,
        "sample_file": sample.name,
        "topic_info_file": (
            f"{safe_name}.topic-info.txt" if topic in topics[:6] else None
        ),
    }

cmd_vel_info = (log_dir/"cmd-vel-ownership.txt").read_text(encoding="utf-8")
cmd_vel_match = re.search(r"^Subscription count: ([0-9]+)$", cmd_vel_info, re.M)
if cmd_vel_match:
    cmd_vel_count = int(cmd_vel_match.group(1))
    if cmd_vel_count != 0:
        raise SystemExit("FAIL: report cannot record a connected /cmd_vel subscriber")
    cmd_vel_observation = "zero_subscribers"
elif "unknown topic" in cmd_vel_info.lower():
    cmd_vel_count = None
    cmd_vel_observation = "topic_not_discovered"
else:
    raise SystemExit("FAIL: /cmd_vel subscriber observation is unavailable")

report = {
    "status": "PASS",
    "scope": "NAV2_SENSOR_INPUT_SEGMENT_V1",
    "domain_id": 43,
    "segment": str(segment),
    "segment_manifest": str(manifest),
    "segment_manifest_sha256": hashlib.sha256(manifest.read_bytes()).hexdigest(),
    "clock_publisher_count": 1,
    "samples": samples,
    "cmd_vel_subscriber_count": cmd_vel_count,
    "cmd_vel_observation": cmd_vel_observation,
    "ros_graph_clean_after_cleanup": True,
    "limits": [
        "one sample per topic is evidence of delivery, not total message counts",
        "input/costmap replay only; no goals or navigation acceptance",
    ],
}
(log_dir/"replay-report.json").write_text(
    json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
PY
echo "PASS [NAV2_REPLAY_INPUTS_ONLY]: Nav2 consumed isolated segment and published both costmaps. Evidence: $LOG_DIR"
