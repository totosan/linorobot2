#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="${JETSON_ENV_FILE:-${SCRIPT_DIR}/.env.jetson}"
COMPOSE_FILE="${SCRIPT_DIR}/docker-compose.jetson.yaml"

if [[ ! -f "${ENV_FILE}" ]]; then
    echo "Missing ${ENV_FILE}. Copy .env.jetson.example to .env.jetson and review its values." >&2
    exit 2
fi

# This is a trusted local config file; source it so host-side checks and Compose
# use the same paths and transport settings.
set -a
source "${ENV_FILE}"
set +a

: "${JETSON_WORKSPACE:=/home/jetson/linorobot2_ws}"
: "${JETSON_ESP32_DEVICE:=/dev/ttyESP32}"
: "${JETSON_LIDAR_DEVICE:=/dev/rplidar}"

for device in "${JETSON_ESP32_DEVICE}" "${JETSON_LIDAR_DEVICE}"; do
    if [[ ! -c "${device}" ]]; then
        echo "Required serial character device is unavailable: ${device}" >&2
        exit 3
    fi
done

esp32_real="$(readlink -f -- "${JETSON_ESP32_DEVICE}")"
lidar_real="$(readlink -f -- "${JETSON_LIDAR_DEVICE}")"
if [[ "${esp32_real}" == "${lidar_real}" ]]; then
    echo "ESP32 and LiDAR aliases resolve to the same device: ${esp32_real}" >&2
    exit 4
fi

if [[ ! -f "${JETSON_WORKSPACE}/install/setup.bash" ]]; then
    echo "Workspace overlay is missing: ${JETSON_WORKSPACE}/install/setup.bash" >&2
    exit 5
fi

exec docker compose --env-file "${ENV_FILE}" -f "${COMPOSE_FILE}" up --abort-on-container-exit
