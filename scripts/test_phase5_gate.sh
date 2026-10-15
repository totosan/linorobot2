#!/usr/bin/env bash
# Compare Nav2 baseline goals with the G5 vision adapter off vs. fixture mock.
set -euo pipefail

WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_PARENT="${G5_LOG_PARENT:-${WORKSPACE_DIR}/log/g5_runtime}"
MAP_YAML="${G5_MAP_YAML:-}"

for arg in "$@"; do
    case "$arg" in
        --map=*) MAP_YAML="${arg#*=}" ;;
        -h|--help)
            echo "Usage: bash $0 --map=/absolute/path/to/sensor-derived-nav-map.yaml"
            echo "Runs the same frozen map/G4 profile in Domain 42 with vision off and mock."
            exit 0
            ;;
        *) echo "FAIL: unsupported argument: $arg"; exit 2 ;;
    esac
done

if [[ -n "${ROS_DOMAIN_ID:-}" && "$ROS_DOMAIN_ID" != "42" ]]; then
    echo "FAIL: G5 SIL requires ROS_DOMAIN_ID=42 (got $ROS_DOMAIN_ID)."
    exit 1
fi
if [[ -z "$MAP_YAML" || "$MAP_YAML" != /* || ! -s "$MAP_YAML" ]]; then
    echo "FAIL: supply an existing absolute sensor-derived Nav2 map via --map or G5_MAP_YAML."
    exit 2
fi
MAP_PREPARE="${MAP_YAML%.yaml}.prepare.json"
if [[ ! -s "$MAP_PREPARE" ]]; then
    echo "FAIL: map preparation evidence is required next to map: $MAP_PREPARE"
    exit 1
fi
python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); assert d.get("status")=="PASS" and d.get("ground_truth_used_for_navigation_map") is False' "$MAP_PREPARE" || {
    echo "FAIL: map must have passing sensor-seeded provenance without Ground Truth navigation input."
    exit 1
}

set +u
source /opt/ros/humble/setup.bash
source "${HOME}/linorobot2_sim/install/setup.bash"
set -u
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export LINOROBOT2_BASE=2wd
python3 -c 'from linorobot2_base.domain_profile import resolve_ros_domain; assert resolve_ros_domain("sim", "42")=="42"' || {
    echo "FAIL: installed linorobot2_base sim/42 domain guard is unavailable."
    exit 1
}

TF2_FIX_SETUP="${HOME}/linobot2_tf2_fix/install/local_setup.bash"
if [[ ! -r "$TF2_FIX_SETUP" ]]; then
    echo "FAIL: tested Humble TF2 overlay is required: $TF2_FIX_SETUP"
    exit 1
fi
set +u
source "$TF2_FIX_SETUP"
set -u

NAV_PACKAGE="${HOME}/linorobot2_sim/install/linorobot2_navigation/share/linorobot2_navigation"
VISION_PACKAGE="${HOME}/linorobot2_sim/install/linorobot2_vision/share/linorobot2_vision"
GZ_PACKAGE="${HOME}/linorobot2_sim/install/linorobot2_gazebo/share/linorobot2_gazebo"
for package in linorobot2_base linorobot2_description linorobot2_gazebo \
        linorobot2_navigation linorobot2_vision; do
    ros2 pkg prefix "$package" >/dev/null 2>&1 || {
        echo "FAIL: required SIL package is absent from the active overlay: $package"
        exit 1
    }
done
WORLD="${GZ_PACKAGE}/worlds/fortress_g4_test.sdf"
NAV_PROFILE="${NAV_PACKAGE}/config/navigation_fortress.yaml"
COMMON_PROFILE="${NAV_PACKAGE}/config/navigation_common.yaml"
VISION_FIXTURE="${VISION_PACKAGE}/fixtures/detection.json"
for required in "$WORLD" "$NAV_PROFILE" "$COMMON_PROFILE" "$VISION_FIXTURE"; do
    [[ -s "$required" ]] || { echo "FAIL: required installed SIL artifact missing: $required"; exit 1; }
done

mkdir -p "$LOG_PARENT"
LOG_ROOT="$(mktemp -d "${LOG_PARENT}/g5-nav-$(date +%Y%m%d-%H%M%S)-XXXXXX")"
SIM_PID=""
NAV_PID=""
VISION_PID=""
BAG_PID=""
G5_PARTITION=""
G5_TOPICS=(/clock /tf /tf_static /scan /map /odom /odom/unfiltered /joint_states
    /cmd_vel /plan /global_costmap/costmap /local_costmap/costmap
    /vision/result /vision/status)

stop_owned() {
    local pid="$1"
    if [[ -n "$pid" ]]; then
        kill -TERM "$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    fi
}
cleanup() {
    stop_owned "$NAV_PID"
    stop_owned "$VISION_PID"
    stop_owned "$SIM_PID"
    stop_owned "$BAG_PID"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_bag() {
    python3 "$WORKSPACE_DIR/scripts/supervised_process.py" -- \
        ros2 bag record --storage sqlite3 --output "$MODE_LOG/rosbag" \
        "${G5_TOPICS[@]}" >"$MODE_LOG/rosbag-record.log" 2>&1 &
    BAG_PID=$!
}
stop_bag() {
    stop_owned "$BAG_PID"
    BAG_PID=""
    ros2 bag info "$MODE_LOG/rosbag" >"$MODE_LOG/rosbag-info.txt" 2>&1 || {
        echo "FAIL: invalid G5 ${CURRENT_MODE} rosbag; inspect $MODE_LOG/rosbag-info.txt"
        return 1
    }
    grep -Eq '^[[:space:]]*Messages:[[:space:]]*[1-9][0-9]*' "$MODE_LOG/rosbag-info.txt" || {
        echo "FAIL: G5 ${CURRENT_MODE} bag contains no messages."
        return 1
    }
}

start_fortress() {
    G5_PARTITION="linobot2_g4_${UID}_g5_${CURRENT_MODE}_$(date +%Y%m%d_%H%M%S)_${RANDOM}_$$"
    export IGN_PARTITION="$G5_PARTITION"
    if [[ "$IGN_PARTITION" != linobot2_g4_* || "$ROS_DOMAIN_ID" != "42" ]]; then
        echo "FAIL: invalid G5 simulation profile: ROS_DOMAIN_ID=$ROS_DOMAIN_ID IGN_PARTITION=$IGN_PARTITION"
        return 1
    fi
    python3 "$WORKSPACE_DIR/scripts/supervised_process.py" -- \
        ros2 launch linorobot2_gazebo fortress.launch.py \
        ros_profile:=sim ros_domain_id:=42 gui:=false rviz:=false world:="$WORLD" \
        >"$MODE_LOG/fortress.log" 2>&1 &
    SIM_PID=$!
    python3 "$WORKSPACE_DIR/scripts/wait_for_fortress.py" \
        --pid "$SIM_PID" --world fortress_test \
        | tee "$MODE_LOG/fortress-readiness.log"
}

start_vision() {
    local mode="$1"
    local args=(--ros-args
        -p use_sim_time:=true
        -p clock_domain:=ros_sim
        -p expected_frame_id:=camera_link
        -p max_age_ns:=250000000
        -p pair_slop_ns:=50000000)
    if [[ "$mode" == "mock" ]]; then
        args+=(-p backend:=mock -p "fixture_path:=$VISION_FIXTURE")
    fi
    python3 "$WORKSPACE_DIR/scripts/supervised_process.py" -- \
        ros2 run linorobot2_vision vision_adapter_node "${args[@]}" \
        >"$MODE_LOG/vision-adapter.log" 2>&1 &
    VISION_PID=$!
    python3 "$WORKSPACE_DIR/scripts/verify_vision_ros_transport.py" \
        --mode "$mode" --min-pairs 3 --timeout 35 \
        --report "$MODE_LOG/vision-transport.json" \
        | tee "$MODE_LOG/vision-transport.log"
}

start_nav2() {
    export ROS_DOMAIN_ID=42
    export IGN_PARTITION="$G5_PARTITION"
    printf 'ROS_DOMAIN_ID=%s\nIGN_PARTITION=%s\n' \
        "$ROS_DOMAIN_ID" "$IGN_PARTITION" >"$MODE_LOG/runtime-profile.txt"
    if [[ "$IGN_PARTITION" != linobot2_g4_* ]]; then
        echo "FAIL: Nav2 verifier requires an isolated linorobot2_g4_ partition."
        return 1
    fi
    python3 "$WORKSPACE_DIR/scripts/supervised_process.py" -- \
        ros2 launch linorobot2_navigation navigation.launch.py \
        sim:=true rviz:=false map:="$MAP_YAML" \
        sim_params_file:="$NAV_PROFILE" common_params_file:="$COMMON_PROFILE" \
        >"$MODE_LOG/nav2.log" 2>&1 &
    NAV_PID=$!
    timeout --kill-after=5s 75s env ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
        IGN_PARTITION="$IGN_PARTITION" python3 -u "$WORKSPACE_DIR/scripts/verify_phase4_nav.py" \
        --timeout 60 --report "$MODE_LOG/nav2-readiness.json" \
        | tee "$MODE_LOG/nav2-readiness.log"
}

run_mode() {
    CURRENT_MODE="$1"
    MODE_LOG="$LOG_ROOT/$CURRENT_MODE"
    mkdir -p "$MODE_LOG"
    printf '%s\n' "$CURRENT_MODE" >"$MODE_LOG/vision-mode.txt"
    sha256sum "$MAP_YAML" "${MAP_YAML%.yaml}.pgm" "$MAP_PREPARE" \
        "$WORLD" "$NAV_PROFILE" "$COMMON_PROFILE" >"$MODE_LOG/input-sha256.txt"
    start_bag
    start_fortress
    start_vision "$CURRENT_MODE"
    start_nav2
    timeout --kill-after=10s 510s env ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
        IGN_PARTITION="$IGN_PARTITION" python3 -u "$WORKSPACE_DIR/scripts/verify_phase4_goal.py" \
        --supervisor-pid "$SIM_PID" --readiness-timeout 45 --goal-timeout 180 \
        --report "$MODE_LOG/reachable-goal.json" \
        | tee "$MODE_LOG/reachable-goal.log"
    stop_owned "$NAV_PID"
    NAV_PID=""
    stop_owned "$VISION_PID"
    VISION_PID=""
    stop_owned "$SIM_PID"
    SIM_PID=""
    stop_bag
}

CURRENT_MODE=""
MODE_LOG=""
run_mode off
run_mode mock
python3 "$WORKSPACE_DIR/scripts/verify_g5_equivalence.py" \
    --off-report "$LOG_ROOT/off/reachable-goal.json" \
    --mock-report "$LOG_ROOT/mock/reachable-goal.json" \
    --off-transport "$LOG_ROOT/off/vision-transport.json" \
    --mock-transport "$LOG_ROOT/mock/vision-transport.json" \
    --map "$MAP_YAML" --world "$WORLD" --profile "$NAV_PROFILE" \
    --common-profile "$COMMON_PROFILE" \
    --report "$LOG_ROOT/g5-equivalence.json"
echo "PASS: vision off/mock Nav2 equivalence on identical sensor-derived map and G4 profile. Evidence: $LOG_ROOT"
