#!/usr/bin/env bash
# Replay gate (Domain 43): replay a validated camera/clock segment into the
# vision adapter. No simulator, no Nav2, no live camera, no hardware.
set -euo pipefail

WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_PARENT="${REPLAY_LOG_PARENT:-${WORKSPACE_DIR}/log/replay_runtime}"
SEGMENT=""
MODE="off"
for arg in "$@"; do
    case "$arg" in
        --segment=*) SEGMENT="${arg#*=}" ;;
        --mode=off) MODE="off" ;;
        --mode=mock) MODE="mock" ;;
        -h|--help)
            echo "Usage: bash $0 --segment=/absolute/path/to/segment_dir [--mode=off|mock]"
            echo "Replays /clock, /camera/image_raw, /camera/camera_info in Domain 43."
            exit 0
            ;;
        *) echo "FAIL: unsupported argument: $arg"; exit 2 ;;
    esac
done

if [[ -n "${ROS_DOMAIN_ID:-}" && "$ROS_DOMAIN_ID" != "43" ]]; then
    echo "FAIL: replay profile requires ROS_DOMAIN_ID=43 (got $ROS_DOMAIN_ID)."
    exit 1
fi
export ROS_DOMAIN_ID=43
python3 -c 'import sys; sys.path.insert(0, sys.argv[1]); from linorobot2_base.domain_profile import resolve_ros_domain; assert resolve_ros_domain("replay", "43") == "43"' "${WORKSPACE_DIR}/linorobot2_base" || {
    echo "FAIL: replay/43 domain contract could not be resolved."
    exit 1
}
if [[ -z "$SEGMENT" || "$SEGMENT" != /* || ! -s "$SEGMENT/metadata.yaml" ]]; then
    echo "FAIL: provide an existing replay segment directory with metadata.yaml via --segment."
    exit 2
fi
MANIFEST="$SEGMENT.manifest.json"
if [[ ! -s "$MANIFEST" ]]; then
    echo "FAIL: segment manifest missing: $MANIFEST"
    exit 1
fi
python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); assert d.get("status")=="PASS" and d.get("capture_clock_domain")=="ros_sim" and d.get("scope")=="REPLAY_SEGMENT_VISION_TRANSPORT_ONLY"' "$MANIFEST" || {
    echo "FAIL: segment manifest is not a validated ros_sim replay segment."
    exit 1
}
python3 -c 'import hashlib,json,sys; from pathlib import Path; d=json.load(open(sys.argv[1])); root=Path(sys.argv[2]); bad=[(n,h) for n,h in d["file_sha256"].items() if hashlib.sha256((root/n).read_bytes()).hexdigest()!=h]; assert not bad, bad' "$MANIFEST" "$SEGMENT" || {
    echo "FAIL: segment files do not match manifest hashes."
    exit 1
}

set +u
source /opt/ros/humble/setup.bash
source "${HOME}/linorobot2_sim/install/setup.bash"
set -u
export ROS_DOMAIN_ID=43
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
python3 -c 'from linorobot2_base.domain_profile import resolve_ros_domain; assert resolve_ros_domain("replay", "43")=="43"' || {
    echo "FAIL: installed linorobot2_base replay/43 domain guard is unavailable."
    exit 1
}

VISION_SHARE="${HOME}/linorobot2_sim/install/linorobot2_vision/share/linorobot2_vision"
VISION_NODE="${HOME}/linorobot2_sim/install/linorobot2_vision/lib/linorobot2_vision/vision_adapter_node"
MOCK_FIXTURE="${VISION_SHARE}/fixtures/detection.json"
for required in "$VISION_NODE" "$MOCK_FIXTURE"; do
    [[ -s "$required" ]] || { echo "FAIL: installed replay artifact missing: $required"; exit 1; }
done

mkdir -p "$LOG_PARENT"
LOG_DIR="$(mktemp -d "${LOG_PARENT}/replay-$(date +%Y%m%d-%H%M%S)-XXXXXX")"
PLAYER_PID=""
VISION_PID=""

stop_owned() {
    local pid="$1"
    if [[ -n "$pid" ]]; then
        kill -TERM "$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    fi
}
cleanup() {
    stop_owned "$VISION_PID"
    stop_owned "$PLAYER_PID"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

printf 'mode=%s\nsegment=%s\n' "$MODE" "$SEGMENT" >"$LOG_DIR/run-profile.txt"
sha256sum "$SEGMENT/metadata.yaml" "$MANIFEST" >"$LOG_DIR/input-sha256.txt"
ros2 bag info "$SEGMENT" >"$LOG_DIR/segment-info.txt" 2>&1 || {
    echo "FAIL: ros2 bag info could not read the segment."
    exit 1
}

echo "[Replay] Starting bag player with topic allowlist (no --clock generator)."
python3 "$WORKSPACE_DIR/scripts/supervised_process.py" -- \
    ros2 bag play "$SEGMENT" \
    --topics /clock /camera/image_raw /camera/camera_info \
    --disable-keyboard-controls \
    >"$LOG_DIR/player.log" 2>&1 &
PLAYER_PID=$!

echo "[Replay] Verifying single /clock publisher before starting consumers."
CLOCK_READY=0
for _ in $(seq 1 75); do
    PUBLISHER_COUNT="$(ros2 topic info /clock 2>/dev/null | sed -n 's/^Publisher count: //p' | head -n1)"
    if [[ "$PUBLISHER_COUNT" == "1" ]]; then
        CLOCK_READY=1
        break
    fi
    sleep 0.2
done
if [[ "$CLOCK_READY" != "1" ]]; then
    echo "FAIL: expected exactly one /clock publisher from the player."
    ros2 topic info /clock >>"$LOG_DIR/clock-ownership.txt" 2>&1 || true
    exit 1
fi
ros2 topic info /clock --verbose >"$LOG_DIR/clock-ownership.txt" 2>&1 || true

echo "[Replay] Starting vision adapter mode=$MODE."
ARGS=(--ros-args
    -p use_sim_time:=true
    -p clock_domain:=ros_sim
    -p expected_frame_id:=camera_link
    -p max_age_ns:=1500000000
    -p pair_slop_ns:=50000000)
if [[ "$MODE" == "mock" ]]; then
    ARGS+=(-p backend:=mock -p "fixture_path:=$MOCK_FIXTURE")
fi
python3 "$WORKSPACE_DIR/scripts/supervised_process.py" -- \
    ros2 run linorobot2_vision vision_adapter_node "${ARGS[@]}" \
    >"$LOG_DIR/vision-adapter.log" 2>&1 &
VISION_PID=$!

python3 "$WORKSPACE_DIR/scripts/verify_vision_ros_transport.py" \
    --mode "$MODE" --domain 43 --min-pairs 3 --timeout 35 \
    --report "$LOG_DIR/vision-transport.json" \
    | tee "$LOG_DIR/vision-transport.log" || {
    echo "FAIL: replay transport probe; inspect $LOG_DIR/vision-transport.json"
    exit 1
}
stop_owned "$VISION_PID"
VISION_PID=""
stop_owned "$PLAYER_PID"
PLAYER_PID=""
echo "PASS: replay/43 camera+clock segment consumed by vision adapter ($MODE). Evidence: $LOG_DIR"
