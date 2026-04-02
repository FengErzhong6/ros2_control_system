#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"

tool_src="${script_dir}/tracking_ik_first_frame_window_sweep.cpp"
tracking_ik_cpp="${TRACKING_IK_CPP:-/home/mmlab/codes/huangshzh/elbow_shift_IK/TrackingIk.cpp}"
kine_include_dir="${repo_root}/marvin_system/third_party/marvinKine"
kine_lib_dir="${repo_root}/marvin_system/third_party/marvinKine"
controllers_yaml="${repo_root}/marvin_system/bringup/config/marvin_tracker_teleop_controllers.yaml"
debug_file="${repo_root}/marvin_system/test/debug_teleop_first_frame.txt"
kine_config="${repo_root}/marvin_system/third_party/marvinCfg/ccs_m6_40.MvKDCfg"
build_dir="${TMPDIR:-/tmp}/marvin_tracking_ik_window_sweep"
binary="${build_dir}/tracking_ik_first_frame_window_sweep"
cxx="${CXX:-g++}"

mkdir -p "${build_dir}"

if [[ ! -f "${tracking_ik_cpp}" ]]; then
  echo "TrackingIk.cpp not found: ${tracking_ik_cpp}" >&2
  echo "Set TRACKING_IK_CPP=/path/to/TrackingIk.cpp and rerun." >&2
  exit 1
fi

if [[ ! -f "${controllers_yaml}" ]]; then
  echo "Controller yaml not found: ${controllers_yaml}" >&2
  exit 1
fi

if [[ ! -f "${debug_file}" ]]; then
  echo "Debug file not found: ${debug_file}" >&2
  exit 1
fi

if [[ ! -f "${kine_config}" ]]; then
  echo "Kinematics config not found: ${kine_config}" >&2
  exit 1
fi

if [[ ! -x "${binary}" || "${tool_src}" -nt "${binary}" || "${tracking_ik_cpp}" -nt "${binary}" ]]; then
  "${cxx}" \
    -std=c++17 \
    -O2 \
    -Wall \
    -Wextra \
    -pedantic \
    -Wno-unused-function \
    -I"${kine_include_dir}" \
    "${tool_src}" \
    "${tracking_ik_cpp}" \
    -L"${kine_lib_dir}" \
    -lKine \
    -Wl,-rpath,"${kine_lib_dir}" \
    -o "${binary}"
fi

exec "${binary}" \
  --controllers-yaml "${controllers_yaml}" \
  --debug-file "${debug_file}" \
  --kine-config "${kine_config}" \
  "$@"
