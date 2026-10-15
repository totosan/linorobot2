#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/sim_paths.sh
source "${SCRIPT_DIR}/lib/sim_paths.sh"

exec python3 "${SCRIPT_DIR}/sim_release.py" rollback \
    --sim-root "$(linorobot2_sim_root)"
