#ifndef ROS2_CONTROL_MARVIN__TELEOP_DIAGNOSTICS_HPP_
#define ROS2_CONTROL_MARVIN__TELEOP_DIAGNOSTICS_HPP_

#include <cstdint>
#include <algorithm>
#include <cctype>
#include <string>

#include "marvin_system/MarvinSDK.h"

namespace marvin_system {

enum class ControlProfile : int32_t {
    kUnknown = 0,
    kPositionFollow = 1,
    kJointImpedance = 2,
};

inline constexpr char kSdkCommandPositionStateInterface[] = "sdk_command_position";
inline constexpr char kActiveControlProfileStateInterface[] = "active_control_profile";
inline constexpr char kRequestedControlProfileStateInterface[] = "requested_control_profile";
inline constexpr char kSdkCurrentStateStateInterface[] = "sdk_cur_state";
inline constexpr char kSdkCommandStateStateInterface[] = "sdk_cmd_state";
inline constexpr char kSdkErrorCodeStateInterface[] = "sdk_err_code";
inline constexpr char kSdkImpTypeStateInterface[] = "sdk_imp_type";
inline constexpr char kSdkInFrameSerialStateInterface[] = "sdk_in_frame_serial";
inline constexpr char kSdkOutFrameSerialStateInterface[] = "sdk_out_frame_serial";

inline std::string joint_state_if(const std::string &joint_name, const std::string &interface_name)
{
    return joint_name + "/" + interface_name;
}

inline const char *control_profile_to_string(ControlProfile profile)
{
    switch (profile) {
        case ControlProfile::kPositionFollow:
            return "POSITION_FOLLOW";
        case ControlProfile::kJointImpedance:
            return "JOINT_IMPEDANCE";
        case ControlProfile::kUnknown:
        default:
            return "UNKNOWN";
    }
}

inline ControlProfile control_profile_from_code(int code)
{
    switch (code) {
        case static_cast<int>(ControlProfile::kPositionFollow):
            return ControlProfile::kPositionFollow;
        case static_cast<int>(ControlProfile::kJointImpedance):
            return ControlProfile::kJointImpedance;
        case static_cast<int>(ControlProfile::kUnknown):
        default:
            return ControlProfile::kUnknown;
    }
}

inline std::string normalize_control_profile_name(std::string value)
{
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), [](unsigned char ch) {
                    return !std::isspace(ch);
                }));
    value.erase(std::find_if(value.rbegin(), value.rend(), [](unsigned char ch) {
                    return !std::isspace(ch);
                }).base(),
                value.end());
    for (char &ch : value) {
        if (ch == '-' || std::isspace(static_cast<unsigned char>(ch))) {
            ch = '_';
            continue;
        }
        ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    }
    return value;
}

inline ControlProfile control_profile_from_string(const std::string &raw_name)
{
    const std::string normalized = normalize_control_profile_name(raw_name);
    if (normalized == "POSITION_FOLLOW" || normalized == "POSITIONFOLLOW") {
        return ControlProfile::kPositionFollow;
    }
    if (normalized == "JOINT_IMPEDANCE" || normalized == "JOINTIMPEDANCE") {
        return ControlProfile::kJointImpedance;
    }
    return ControlProfile::kUnknown;
}

inline const char *arm_state_code_to_string(int state)
{
    switch (state) {
        case ARM_STATE_IDLE:
            return "IDLE";
        case ARM_STATE_POSITION:
            return "POSITION";
        case ARM_STATE_PVT:
            return "PVT";
        case ARM_STATE_TORQ:
            return "TORQ";
        case ARM_STATE_RELEASE:
            return "RELEASE";
        case ARM_STATE_ERROR:
            return "ERROR";
        case ARM_STATE_TRANS_TO_POSITION:
            return "TRANS_TO_POSITION";
        case ARM_STATE_TRANS_TO_PVT:
            return "TRANS_TO_PVT";
        case ARM_STATE_TRANS_TO_TORQ:
            return "TRANS_TO_TORQ";
        case ARM_STATE_TRANS_TO_RELEASE:
            return "TRANS_TO_RELEASE";
        case ARM_STATE_TRANS_TO_IDLE:
            return "TRANS_TO_IDLE";
        default:
            return "UNKNOWN";
    }
}

}  // namespace marvin_system

#endif  // ROS2_CONTROL_MARVIN__TELEOP_DIAGNOSTICS_HPP_
