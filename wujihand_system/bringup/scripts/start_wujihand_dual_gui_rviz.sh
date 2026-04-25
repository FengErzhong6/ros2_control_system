#!/usr/bin/env bash
set -Eeuo pipefail

# SCRIPT INFO
# Launch dual WujiHand control with per-hand slider GUIs and RViz.
# Use --mock to visualize without real hardware.

# USAGE
#   ./wujihand_system/bringup/scripts/start_wujihand_dual_gui_rviz.sh [--mock]
#   ./wujihand_system/bringup/scripts/start_wujihand_dual_gui_rviz.sh --help

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"
readonly ROS_SETUP_FILE="/opt/ros/jazzy/setup.bash"
readonly WORKSPACE_SETUP_FILE="${WORKSPACE_DIR}/install/setup.bash"
readonly RVIZ_CONFIG_FILE="${WORKSPACE_DIR}/wujihand_system/bringup/rviz/wujihand_dual_teleop.rviz"
readonly LAUNCH_FILE="wujihand_dual_control.launch.py"

G_USE_MOCK_HARDWARE="false"
G_LAUNCH_PID=""

fct_print_usage() {
    printf '%s\n' \
        "Launch dual WujiHand control with left/right slider GUIs and RViz." \
        "" \
        "Options:" \
        "  --mock    Use mock hardware instead of real WujiHand devices" \
        "  --help    Show this help message"
}

fct_parse_args() {
    while (($# > 0)); do
        case "$1" in
            --mock)
                G_USE_MOCK_HARDWARE="true"
                ;;
            --help|-h)
                fct_print_usage
                exit 0
                ;;
            *)
                printf 'Unknown argument: %s\n' "$1" >&2
                fct_print_usage >&2
                exit 1
                ;;
        esac
        shift
    done
}

fct_require_file() {
    local path="$1"
    if [[ ! -f "${path}" ]]; then
        printf 'Required file not found: %s\n' "${path}" >&2
        exit 1
    fi
}

fct_source_environment() {
    fct_require_file "${ROS_SETUP_FILE}"
    fct_require_file "${WORKSPACE_SETUP_FILE}"
    fct_require_file "${RVIZ_CONFIG_FILE}"

    # shellcheck disable=SC1091
    source "${ROS_SETUP_FILE}"
    # shellcheck disable=SC1091
    source "${WORKSPACE_SETUP_FILE}"
}

fct_cleanup() {
    local exit_code="$1"

    if [[ -n "${G_LAUNCH_PID}" ]] && kill -0 "${G_LAUNCH_PID}" 2>/dev/null; then
        kill "${G_LAUNCH_PID}" 2>/dev/null || true
        wait "${G_LAUNCH_PID}" 2>/dev/null || true
    fi

    exit "${exit_code}"
}

fct_execute_this() {
    ros2 launch wujihand_system "${LAUNCH_FILE}" \
        use_left_gui:=true \
        use_right_gui:=true \
        left_namespace:=left_hand \
        right_namespace:=right_hand \
        activate_forward_controller:=true \
        use_mock_hardware:="${G_USE_MOCK_HARDWARE}" &
    G_LAUNCH_PID="$!"

    if ! kill -0 "${G_LAUNCH_PID}" 2>/dev/null; then
        printf 'Failed to start ros2 launch process.\n' >&2
        exit 1
    fi

    rviz2 -d "${RVIZ_CONFIG_FILE}"
}

fct_main() {
    fct_parse_args "$@"
    fct_source_environment
    trap 'fct_cleanup $?' EXIT INT TERM
    fct_execute_this
}

fct_main "$@"
