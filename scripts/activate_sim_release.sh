#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/sim_paths.sh
source "${SCRIPT_DIR}/lib/sim_paths.sh"

[[ $# -eq 1 ]] || {
    echo "Usage: $0 RELEASE_ID" >&2
    exit 2
}
RELEASE_ID="$1"
[[ "$RELEASE_ID" =~ ^[A-Za-z0-9][A-Za-z0-9._-]{0,79}$ ]] || {
    echo "FAIL: invalid release id." >&2
    exit 2
}
exec python3 "${SCRIPT_DIR}/sim_release.py" activate \
    --sim-root "$(linorobot2_sim_root)" \
    --release "$(linorobot2_sim_root)/releases/${RELEASE_ID}"
