#include "marvin_system/workspace_guard.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace {

constexpr double kDeg2Rad = M_PI / 180.0;

using marvin_system::M4;

// ── 4×4 homogeneous-transform helpers ────────────────────────────────────

M4 m4_from_rpy_xyz(double roll, double pitch, double yaw,
                    double tx, double ty, double tz)
{
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw),  sy = std::sin(yaw);
    return {{
        {cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr,  tx},
        {sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr,  ty},
        {-sp,    cp*sr,             cp*cr,              tz},
        {0,      0,                 0,                  1}
    }};
}

std::string param_str(const std::unordered_map<std::string, std::string> &m,
                      const std::string &key, const std::string &def = {})
{
    const auto it = m.find(key);
    return (it != m.end()) ? it->second : def;
}

}  // namespace

// =========================================================================

namespace marvin_system {

// ---------------------------------------------------------------------------
// configure
// ---------------------------------------------------------------------------
bool WorkspaceGuard::configure(
    const std::unordered_map<std::string, std::string> &p,
    const rclcpp::Logger &logger)
{
    has_tool_ = false;
    tool_tf_ = {};
    for (auto &s : has_safe_) s = false;
    for (auto &v : violation_streak_) v = 0;

    auto it_z = p.find("workspace_z_min");
    if (it_z == p.end()) {
        enabled_ = false;
        return false;
    }

    const double z_min = std::stod(it_z->second);
    auto it_m = p.find("workspace_safety_margin");
    const double margin = (it_m != p.end()) ? std::stod(it_m->second) : 0.05;
    z_threshold_ = z_min + margin;

    auto parse_tf = [&](const std::string &xyz_key, const std::string &rpy_key,
                        const std::string &xyz_def,
                        const std::string &rpy_def) -> M4 {
        auto xyz_s = param_str(p, xyz_key, xyz_def);
        auto rpy_s = param_str(p, rpy_key, rpy_def);
        double x = 0.0, y = 0.0, z = 0.0, r = 0.0, pi = 0.0, ya = 0.0;
        if (std::sscanf(xyz_s.c_str(), "%lf %lf %lf", &x, &y, &z) != 3) {
            RCLCPP_WARN(logger,
                "Invalid %s='%s', falling back to default '%s'.",
                xyz_key.c_str(), xyz_s.c_str(), xyz_def.c_str());
            if (std::sscanf(xyz_def.c_str(), "%lf %lf %lf", &x, &y, &z) != 3) {
                x = y = z = 0.0;
            }
        }
        if (std::sscanf(rpy_s.c_str(), "%lf %lf %lf", &r, &pi, &ya) != 3) {
            RCLCPP_WARN(logger,
                "Invalid %s='%s', falling back to default '%s'.",
                rpy_key.c_str(), rpy_s.c_str(), rpy_def.c_str());
            if (std::sscanf(rpy_def.c_str(), "%lf %lf %lf", &r, &pi, &ya) != 3) {
                r = pi = ya = 0.0;
            }
        }
        return m4_from_rpy_xyz(r, pi, ya, x, y, z);
    };

    mount_tf_[0] = parse_tf("mount_xyz_L", "mount_rpy_L",
                            "0 0.037 0.3618964", "-1.5707963 0 0");
    mount_tf_[1] = parse_tf("mount_xyz_R", "mount_rpy_R",
                            "0 -0.037 0.3618964", "1.5707963 0 0");

    auto tool_s = param_str(p, "tool_offset", "");
    if (!tool_s.empty()) {
        double tx = 0.0, ty = 0.0, tz = 0.0;
        if (std::sscanf(tool_s.c_str(), "%lf %lf %lf", &tx, &ty, &tz) == 3) {
            tool_tf_ = m4_from_rpy_xyz(0, 0, 0, tx, ty, tz);
            has_tool_ = true;
            RCLCPP_INFO(logger,
                "Workspace check: tool offset (%.3f, %.3f, %.3f) enabled.",
                tx, ty, tz);
        } else {
            RCLCPP_WARN(logger,
                "Invalid tool_offset='%s', ignoring tool tip extension.",
                tool_s.c_str());
        }
    }

    enabled_ = true;
    RCLCPP_INFO(logger,
        "Workspace z-floor check enabled: z >= %.3f m "
        "(z_min=%.3f + margin=%.3f).",
        z_threshold_, z_min, margin);
    return true;
}

// ---------------------------------------------------------------------------
// seed
// ---------------------------------------------------------------------------
void WorkspaceGuard::seed(size_t arm, const double feedback_deg[kJointsPerArm])
{
    std::copy(feedback_deg, feedback_deg + kJointsPerArm,
              last_safe_deg_[arm].begin());
    has_safe_[arm] = true;
    violation_streak_[arm] = 0;
}

// ---------------------------------------------------------------------------
// filter
// ---------------------------------------------------------------------------
bool WorkspaceGuard::filter(size_t arm, double cmd_deg[kJointsPerArm],
                            const rclcpp::Logger &logger)
{
    if (!checkArmSafe(arm, cmd_deg)) {
        // Always replace with last_safe so we never send a violating command.
        // If not seeded yet, last_safe_deg_ is zero-initialized (fallback).
        std::copy(last_safe_deg_[arm].begin(),
                  last_safe_deg_[arm].end(), cmd_deg);
        if (!has_safe_[arm]) {
            if (violation_streak_[arm] == 0 || violation_streak_[arm] % 1000 == 0) {
                RCLCPP_ERROR(logger,
                    "Arm %zu: workspace violation but no safe position seeded yet; "
                    "applying fallback to avoid sending violation.", arm);
            }
        }
        violation_streak_[arm]++;
        if (violation_streak_[arm] == 1 ||
            violation_streak_[arm] % 1000 == 0) {
            RCLCPP_WARN(logger,
                "Arm %zu: command violates z >= %.3f, "
                "holding safe position (count=%d).",
                arm, z_threshold_, violation_streak_[arm]);
        }
        return false;
    }

    if (violation_streak_[arm] > 0) {
        RCLCPP_INFO(logger,
            "Arm %zu: z-floor check OK, resuming "
            "(was held for %d cycles).",
            arm, violation_streak_[arm]);
    }
    violation_streak_[arm] = 0;
    std::copy(cmd_deg, cmd_deg + kJointsPerArm,
              last_safe_deg_[arm].begin());
    has_safe_[arm] = true;
    return true;
}

// ---------------------------------------------------------------------------
// checkArmSafe – FK all joint origins + tool tip, verify z >= threshold
// ---------------------------------------------------------------------------
bool WorkspaceGuard::checkArmSafe(
    size_t arm, const double cmd_deg[kJointsPerArm]) const
{
    // Marvin M6-S joint origin transforms (from URDF, identical for L/R arms).
    static const M4 jt[kJointsPerArm] = {
        m4_from_rpy_xyz(0,        0,        0,      0,     0,      0.1745),
        m4_from_rpy_xyz(M_PI_2,   0,        0,      0,     0,      0),
        m4_from_rpy_xyz(-M_PI_2,  0,        0,      0,     0.287,  0),
        m4_from_rpy_xyz(-M_PI_2,  0,        M_PI,   0.018, 0,      0),
        m4_from_rpy_xyz(-M_PI_2,  0,        M_PI,   0.018, -0.314, 0),
        m4_from_rpy_xyz(M_PI_2,  -M_PI_2,   0,      0,     0,      0),
        m4_from_rpy_xyz(M_PI_2,  -M_PI_2,   0,      0,     0,      0),
    };

    double T[3][4];
    for (int i = 0; i < 3; ++i)
        for (int c = 0; c < 4; ++c)
            T[i][c] = mount_tf_[arm].d[i][c];

    for (size_t j = 0; j < kJointsPerArm; ++j) {
        const double q = cmd_deg[j] * kDeg2Rad;
        const double cq = std::cos(q), sq = std::sin(q);
        const auto &J = jt[j].d;

        const double j00 = J[0][0]*cq + J[0][1]*sq;
        const double j10 = J[1][0]*cq + J[1][1]*sq;
        const double j20 = J[2][0]*cq + J[2][1]*sq;
        const double j01 = J[0][1]*cq - J[0][0]*sq;
        const double j11 = J[1][1]*cq - J[1][0]*sq;
        const double j21 = J[2][1]*cq - J[2][0]*sq;

        for (int i = 0; i < 3; ++i) {
            const double r0 = T[i][0], r1 = T[i][1], r2 = T[i][2], r3 = T[i][3];
            T[i][0] = r0*j00 + r1*j10 + r2*j20;
            T[i][1] = r0*j01 + r1*j11 + r2*j21;
            T[i][2] = r0*J[0][2] + r1*J[1][2] + r2*J[2][2];
            T[i][3] = r0*J[0][3] + r1*J[1][3] + r2*J[2][3] + r3;
        }

        if (__builtin_expect(T[2][3] < z_threshold_, 0))
            return false;
    }

    if (has_tool_) {
        const double z = T[2][0]*tool_tf_.d[0][3] + T[2][1]*tool_tf_.d[1][3]
                       + T[2][2]*tool_tf_.d[2][3] + T[2][3];
        if (__builtin_expect(z < z_threshold_, 0))
            return false;
    }

    return true;
}

}  // namespace marvin_system
