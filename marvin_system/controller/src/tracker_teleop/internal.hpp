#ifndef ROS2_CONTROL_MARVIN__CONTROLLER__TRACKER_TELEOP_CONTROLLER_INTERNAL_HPP_
#define ROS2_CONTROL_MARVIN__CONTROLLER__TRACKER_TELEOP_CONTROLLER_INTERNAL_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "marvin_system/tracker_teleop_controller.hpp"

namespace marvin_system {

using config_type = controller_interface::interface_configuration_type;

inline constexpr std::array<const char *, kArmCount> kSideLabels{{"left", "right"}};
inline constexpr std::array<const char *, kArmCount> kSideTags{{"LEFT", "RIGHT"}};

template <typename LoanedInterfaceT>
inline LoanedInterfaceT *find_loaned_interface(
    std::vector<LoanedInterfaceT> &interfaces, const std::string &full_name)
{
    for (auto &iface : interfaces) {
        if (iface.get_name() == full_name) {
            return &iface;
        }
    }
    return nullptr;
}

inline bool normalizeVector(std::array<double, 3> &vector)
{
    const double norm = std::sqrt(
        vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
    if (norm < 1e-12) {
        return false;
    }
    vector[0] /= norm;
    vector[1] /= norm;
    vector[2] /= norm;
    return true;
}

inline double clampUnit(double value)
{
    return std::clamp(value, -1.0, 1.0);
}

inline double angleBetweenVectorsDeg(
    const std::array<double, 3> &lhs, const std::array<double, 3> &rhs)
{
    return std::acos(clampUnit(
        lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2])) * kRad2Deg;
}

inline double pointDistanceMeters(
    const geometry_msgs::msg::Point &lhs,
    const geometry_msgs::msg::Point &rhs)
{
    const double dx = lhs.x - rhs.x;
    const double dy = lhs.y - rhs.y;
    const double dz = lhs.z - rhs.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline double quaternionAngularDistanceDeg(
    const geometry_msgs::msg::Quaternion &lhs,
    const geometry_msgs::msg::Quaternion &rhs)
{
    tf2::Quaternion q_l(lhs.x, lhs.y, lhs.z, lhs.w);
    tf2::Quaternion q_r(rhs.x, rhs.y, rhs.z, rhs.w);
    q_l.normalize();
    q_r.normalize();

    const double dot = std::abs(
        q_l.x() * q_r.x() +
        q_l.y() * q_r.y() +
        q_l.z() * q_r.z() +
        q_l.w() * q_r.w());
    return 2.0 * std::acos(clampUnit(dot)) * kRad2Deg;
}

inline bool extractSolvedUpperArmDir(
    FX_INT32L arm,
    const std::array<double, kJointsPerArm> &joints_rad,
    std::array<double, 3> &upper_arm_dir)
{
    FX_DOUBLE joints_deg[kJointsPerArm];
    Matrix4 fk_pose{};
    Matrix3 nsp_pose{};

    for (size_t j = 0; j < kJointsPerArm; ++j) {
        joints_deg[j] = joints_rad[j] * kRad2Deg;
    }

    if (!FX_Robot_Kine_FK_NSP(arm, joints_deg, fk_pose, nsp_pose)) {
        return false;
    }

    upper_arm_dir = {{nsp_pose[0][1], nsp_pose[1][1], nsp_pose[2][1]}};
    return normalizeVector(upper_arm_dir);
}

}  // namespace marvin_system

#endif  // ROS2_CONTROL_MARVIN__CONTROLLER__TRACKER_TELEOP_CONTROLLER_INTERNAL_HPP_
