#!/usr/bin/env bash

linorobot2_repo_root() {
    cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd
}

linorobot2_sim_root() {
    printf '%s\n' "${LINOROBOT2_SIM_ROOT:-${HOME}/linobot2_sim}"
}

linorobot2_release_root() {
    if [[ -n "${LINOROBOT2_SIM_RELEASE:-}" ]]; then
        printf '%s\n' "$LINOROBOT2_SIM_RELEASE"
        return
    fi

    local sim_root
    sim_root="$(linorobot2_sim_root)"
    if [[ -L "${sim_root}/active" ]]; then
        readlink -f "${sim_root}/active"
    else
        printf '%s\n' "$sim_root"
    fi
}

linorobot2_install_base() {
    printf '%s/install\n' "$(linorobot2_release_root)"
}

linorobot2_source_overlay() {
    local setup
    setup="$(linorobot2_install_base)/setup.bash"
    if [[ ! -f "$setup" ]]; then
        echo "FAIL: simulation overlay is missing: $setup" >&2
        return 1
    fi

    set +u
    # shellcheck disable=SC1090
    source "$setup"
    set -u
}
