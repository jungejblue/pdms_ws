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

# EDIT THESE THREE PATHS to match your original parquet split and PKL filenames.
# The two filenames below are examples, not automatically discovered files.
export PDMS_DATA_ROOT="${ETRI_DATA_ROOT}/val"
export PDMS_RAW_PKL="${ETRI_CACHE_PATH}/etri_infos_temporal_val.pkl"
export PDMS_PLANNING_PKL="${ETRI_PREDICTION_CACHE}/planning.pkl"

export PDMS_HOST="0.0.0.0"
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1

printf '%s\n' "PDMS_WS_ROOT=${PDMS_WS_ROOT}" \
    "PDMS_DATA_ROOT=${PDMS_DATA_ROOT}" \
    "PDMS_RAW_PKL=${PDMS_RAW_PKL}" \
    "PDMS_PLANNING_PKL=${PDMS_PLANNING_PKL}" \
    "PDMS_HOST=${PDMS_HOST}; port=7200"
unset _PDMS_SCRIPT_DIR
