#ifndef ROS2_CONTROL_MARVIN__CONTROLLER__MARVIN_KINE_UTILS_HPP_
#define ROS2_CONTROL_MARVIN__CONTROLLER__MARVIN_KINE_UTILS_HPP_

#include <cmath>
#include <cstring>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "FxRobot.h"

namespace marvin_system {

constexpr size_t kJointsPerArm = 7;
constexpr size_t kArmCount = 2;
constexpr size_t kTotalJoints = kJointsPerArm * kArmCount;
constexpr double kDeg2Rad = 0.01745329251994329576923690768489;
constexpr double kRad2Deg = 57.295779513082320876798154814105;
constexpr double kMm2M = 0.001;
constexpr double kM2Mm = 1000.0;

struct MarvinKineData {
    FX_INT32L type[2]{};
    FX_DOUBLE grv[2][3]{};
    FX_DOUBLE dh[2][8][4]{};
    FX_DOUBLE pnva[2][7][4]{};
    FX_DOUBLE bd[2][4][3]{};
    FX_DOUBLE mass[2][7]{};
    FX_DOUBLE mcp[2][7][3]{};
    FX_DOUBLE inertia[2][7][6]{};
};

inline bool initMarvinKinematics(const std::string &config_path,
                                 MarvinKineData &kd,
                                 const rclcpp::Logger &logger)
{
    FX_LOG_SWITCH(0);

    FX_BOOL ret = LOADMvCfg(
        const_cast<FX_CHAR *>(config_path.c_str()),
        kd.type, kd.grv, kd.dh, kd.pnva, kd.bd,
        kd.mass, kd.mcp, kd.inertia);

    if (!ret) {
        RCLCPP_ERROR(logger, "LOADMvCfg failed for '%s'", config_path.c_str());
        return false;
    }

    RCLCPP_INFO(logger, "Loaded kinematic config: type_L=%ld, type_R=%ld",
                static_cast<long>(kd.type[0]), static_cast<long>(kd.type[1]));

    for (int arm = 0; arm < static_cast<int>(kArmCount); ++arm) {
        if (!FX_Robot_Init_Type(arm, kd.type[arm])) {
            RCLCPP_ERROR(logger, "FX_Robot_Init_Type failed for arm %d", arm);
            return false;
        }
        if (!FX_Robot_Init_Kine(arm, kd.dh[arm])) {
            RCLCPP_ERROR(logger, "FX_Robot_Init_Kine failed for arm %d", arm);
            return false;
        }
        if (!FX_Robot_Init_Lmt(arm, kd.pnva[arm], kd.bd[arm])) {
            RCLCPP_ERROR(logger, "FX_Robot_Init_Lmt failed for arm %d", arm);
            return false;
        }
        RCLCPP_INFO(logger, "Kinematics initialized for arm %d", arm);
    }

    return true;
}

inline void poseToMatrix4(const geometry_msgs::msg::PoseStamped &pose, Matrix4 mat)
{
    std::memset(mat, 0, sizeof(Matrix4));

    const auto &p = pose.pose.position;
    const auto &q = pose.pose.orientation;

    const double qx = q.x, qy = q.y, qz = q.z, qw = q.w;
    const double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    const double s = (n > 1e-12) ? (2.0 / (n * n)) : 0.0;

    const double wx = s * qw * qx, wy = s * qw * qy, wz = s * qw * qz;
    const double xx = s * qx * qx, xy = s * qx * qy, xz = s * qx * qz;
    const double yy = s * qy * qy, yz = s * qy * qz, zz = s * qz * qz;

    mat[0][0] = 1.0 - (yy + zz);
    mat[0][1] = xy - wz;
    mat[0][2] = xz + wy;
    mat[0][3] = p.x * kM2Mm;

    mat[1][0] = xy + wz;
    mat[1][1] = 1.0 - (xx + zz);
    mat[1][2] = yz - wx;
    mat[1][3] = p.y * kM2Mm;

    mat[2][0] = xz - wy;
    mat[2][1] = yz + wx;
    mat[2][2] = 1.0 - (xx + yy);
    mat[2][3] = p.z * kM2Mm;

    mat[3][0] = 0.0;
    mat[3][1] = 0.0;
    mat[3][2] = 0.0;
    mat[3][3] = 1.0;
}

inline void matrix4ToPose(const Matrix4 mat, geometry_msgs::msg::PoseStamped &pose)
{
    pose.pose.position.x = mat[0][3] * kMm2M;
    pose.pose.position.y = mat[1][3] * kMm2M;
    pose.pose.position.z = mat[2][3] * kMm2M;

    const double trace = mat[0][0] + mat[1][1] + mat[2][2];
    double qw, qx, qy, qz;

    if (trace > 0.0) {
        double s = 0.5 / std::sqrt(trace + 1.0);
        qw = 0.25 / s;
        qx = (mat[2][1] - mat[1][2]) * s;
        qy = (mat[0][2] - mat[2][0]) * s;
        qz = (mat[1][0] - mat[0][1]) * s;
    } else if (mat[0][0] > mat[1][1] && mat[0][0] > mat[2][2]) {
        double s = 2.0 * std::sqrt(1.0 + mat[0][0] - mat[1][1] - mat[2][2]);
        qw = (mat[2][1] - mat[1][2]) / s;
        qx = 0.25 * s;
        qy = (mat[0][1] + mat[1][0]) / s;
        qz = (mat[0][2] + mat[2][0]) / s;
    } else if (mat[1][1] > mat[2][2]) {
        double s = 2.0 * std::sqrt(1.0 + mat[1][1] - mat[0][0] - mat[2][2]);
        qw = (mat[0][2] - mat[2][0]) / s;
        qx = (mat[0][1] + mat[1][0]) / s;
        qy = 0.25 * s;
        qz = (mat[1][2] + mat[2][1]) / s;
    } else {
        double s = 2.0 * std::sqrt(1.0 + mat[2][2] - mat[0][0] - mat[1][1]);
        qw = (mat[1][0] - mat[0][1]) / s;
        qx = (mat[0][2] + mat[2][0]) / s;
        qy = (mat[1][2] + mat[2][1]) / s;
        qz = 0.25 * s;
    }

    double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    pose.pose.orientation.x = qx / norm;
    pose.pose.orientation.y = qy / norm;
    pose.pose.orientation.z = qz / norm;
    pose.pose.orientation.w = qw / norm;
}

inline bool isQuaternionValid(const geometry_msgs::msg::Quaternion &q)
{
    if (!std::isfinite(q.x) || !std::isfinite(q.y) ||
        !std::isfinite(q.z) || !std::isfinite(q.w)) {
        return false;
    }
    const double norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::abs(norm_sq - 1.0) < 0.01;
}

}  // namespace marvin_system

#endif  // ROS2_CONTROL_MARVIN__CONTROLLER__MARVIN_KINE_UTILS_HPP_
