#!/usr/bin/env bash
# Start the stationary Fortress world in the existing Linux RDP desktop.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/sim_paths.sh
source "${SCRIPT_DIR}/lib/sim_paths.sh"
REPO_ROOT="$(linorobot2_repo_root)"

if [[ ! "${DISPLAY:-}" =~ ^:[0-9]+(\.[0-9]+)?$ ]]; then
    echo "FAIL: DISPLAY must be a local Linux X11 display (for example :10.0)." >&2
    exit 2
fi

if [[ -n "${ROS_DOMAIN_ID:-}" && "$ROS_DOMAIN_ID" != "42" ]]; then
    echo "FAIL: Fortress RDP view requires ROS_DOMAIN_ID=42; refusing $ROS_DOMAIN_ID." >&2
    exit 2
fi

[[ -f /opt/ros/humble/setup.bash ]] || {
    echo "FAIL: ROS Humble is not installed in this environment." >&2
    exit 2
}

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export LIBGL_ALWAYS_SOFTWARE=1
export LP_NUM_THREADS="${LP_NUM_THREADS:-2}"
export IGN_PARTITION="${IGN_PARTITION:-linorobot2_rdp_view_$(date +%Y%m%d_%H%M%S)_$$}"

set +u
source /opt/ros/humble/setup.bash
linorobot2_source_overlay
set -u

RELEASE_ROOT="$(linorobot2_release_root)"
LOG_DIR="${RELEASE_ROOT}/log/rdp-gazebo-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$LOG_DIR"
printf '%s\n' "$LOG_DIR" > "${RELEASE_ROOT}/log/rdp-gazebo-current.txt"
exec > >(tee -a "${LOG_DIR}/launch.log") 2>&1

exec python3 "${REPO_ROOT}/scripts/supervised_process.py" -- \
    ros2 launch linorobot2_gazebo fortress.launch.py \
    ros_profile:=sim ros_domain_id:=42 gui:=true rviz:=true
