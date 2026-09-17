#!/usr/bin/env bash
# Source this file after activating the pdms venv.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "Run: source scripts/setup_pdms_docker_env.sh" >&2
    exit 1
fi

_PDMS_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PDMS_WS_ROOT="$(cd -- "${_PDMS_SCRIPT_DIR}/.." && pwd)"

# Dataset directory layout follows the supplied setup_vis scripts.
export DATASET_ROOT="/data"
export ETRI_DATA_ROOT="${DATASET_ROOT}/etri"
export ETRI_CACHE_PATH="${ETRI_DATA_ROOT}/cache"
export ETRI_PREDICTION_CACHE="${ETRI_DATA_ROOT}/result/stage2"

# Edit directories only. All matching top-level PKLs are discovered by schema.
export PDMS_DATA_ROOT="${ETRI_DATA_ROOT}/val"
# Optional: source this script nuscenes v1.0-mini (or v1.0-trainval).
export PDMS_DATASET="${1:-etri}"
if [[ "$PDMS_DATASET" != "etri" && "$PDMS_DATASET" != "nuscenes" ]]; then
    echo "Dataset must be etri or nuscenes" >&2
    return 1
fi
export NUSCENES_VERSION="${2:-v1.0-mini}"
export NUSCENES_DATA_ROOT="${DATASET_ROOT}/nuscenes/${NUSCENES_VERSION}"
export NUSCENES_CACHE_PATH="${NUSCENES_DATA_ROOT}/cache"
export NUSCENES_PREDICTION_CACHE="${NUSCENES_DATA_ROOT}/result/stage2"
export NUSCENES_PREDICTION_FRAME="lidar"
export NUSCENES_MAP_ROOT="${NUSCENES_DATA_ROOT}"
if [[ "$PDMS_DATASET" == "nuscenes" ]]; then
    export PDMS_DATA_ROOT="$NUSCENES_DATA_ROOT"
fi
# Prevent stale filename settings from an earlier setup script.
unset PDMS_RAW_PKL PDMS_PLANNING_PKL

export PDMS_HOST="0.0.0.0"
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1

printf '%s\n' "PDMS_DATASET=${PDMS_DATASET}" "NUSCENES_CACHE_PATH=${NUSCENES_CACHE_PATH}" "NUSCENES_PREDICTION_CACHE=${NUSCENES_PREDICTION_CACHE}" "PDMS_WS_ROOT=${PDMS_WS_ROOT}" \
    "PDMS_DATA_ROOT=${PDMS_DATA_ROOT}" \
    "ETRI_CACHE_PATH=${ETRI_CACHE_PATH}" \
    "ETRI_PREDICTION_CACHE=${ETRI_PREDICTION_CACHE}" \
    "PDMS_HOST=${PDMS_HOST}; port=7200"
unset _PDMS_SCRIPT_DIR
