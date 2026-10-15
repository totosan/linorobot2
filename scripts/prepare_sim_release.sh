#!/usr/bin/env bash
# Clone and build a new simulation release without changing the active release.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/sim_paths.sh
source "${SCRIPT_DIR}/lib/sim_paths.sh"

REMOTE="https://github.com/totosan/linorobot2.git"
COMMIT=""
RELEASE_ID=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --remote) REMOTE="$2"; shift 2 ;;
        --commit) COMMIT="$2"; shift 2 ;;
        --release-id) RELEASE_ID="$2"; shift 2 ;;
        *) echo "FAIL: unknown argument: $1" >&2; exit 2 ;;
    esac
done

[[ -n "$COMMIT" ]] || { echo "FAIL: --commit is required." >&2; exit 2; }
[[ "$RELEASE_ID" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,79}$ ]] || {
    echo "FAIL: --release-id must contain only letters, digits, dot, underscore, or hyphen." >&2
    exit 2
}
[[ "$(uname -s)" == Linux && -f /opt/ros/humble/setup.bash ]] || {
    echo "FAIL: release builds require Linux with ROS Humble." >&2
    exit 2
}

SIM_ROOT="$(linorobot2_sim_root)"
RELEASE="${SIM_ROOT}/releases/${RELEASE_ID}"
[[ ! -e "$RELEASE" ]] || {
    echo "FAIL: refusing to overwrite existing release: $RELEASE" >&2
    exit 2
}
mkdir -p "$RELEASE"
printf '%s\n' "pending" >"${RELEASE}/BUILD_STATE"

git clone --no-checkout "$REMOTE" "${RELEASE}/source"
git -C "${RELEASE}/source" checkout --detach "$COMMIT"
[[ "$(git -C "${RELEASE}/source" rev-parse HEAD)" == "$(git -C "${RELEASE}/source" rev-parse "$COMMIT^{commit}")" ]] || {
    echo "FAIL: checked-out commit does not match requested commit." >&2
    exit 1
}
[[ -z "$(git -C "${RELEASE}/source" status --porcelain)" ]] || {
    echo "FAIL: fresh source checkout is unexpectedly dirty." >&2
    exit 1
}

export LINOROBOT2_SIM_RELEASE="$RELEASE"
bash "${RELEASE}/source/scripts/build_workspace.sh"
python3 -m unittest discover -s "${RELEASE}/source/scripts" -p 'test_*.py'

set +u
source /opt/ros/humble/setup.bash
set -u
python3 "${RELEASE}/source/scripts/sim_release.py" manifest \
    --release "$RELEASE" \
    --source "${RELEASE}/source" \
    --validation-status pending \
    --validation-command "bash scripts/build_workspace.sh" \
    --validation-command "python3 -m unittest discover -s scripts -p test_*.py"
printf '%s\n' "built-awaiting-runtime-smoke" >"${RELEASE}/BUILD_STATE"
echo "PASS: release built and ROS-free tests passed; runtime smoke still required: $RELEASE"
