#ifndef ROS2_CONTROL_MARVIN__HARDWARE__WORKSPACE_GUARD_HPP_
#define ROS2_CONTROL_MARVIN__HARDWARE__WORKSPACE_GUARD_HPP_

#include <array>
#include <string>
#include <unordered_map>

#include "rclcpp/logger.hpp"

namespace marvin_system {

struct M4 { double d[4][4] = {}; };

class WorkspaceGuard {
public:
    static constexpr size_t kJointsPerArm = 7;
    static constexpr size_t kArmCount = 2;

    /// Parse hardware parameters. Call in on_configure().
    /// Returns true if workspace checking is configured (workspace_z_min present).
    /// Runtime enforcement stays disarmed until arm() is called.
    bool configure(const std::unordered_map<std::string, std::string> &params,
                   const rclcpp::Logger &logger);

    /// Enable runtime enforcement after the startup sequence has completed.
    void arm();

    /// Disable runtime enforcement during lifecycle transitions.
    void disarm();

    /// Seed the last-safe command from current feedback (degrees).
    /// Call in on_activate() for each arm.
    void seed(size_t arm, const double feedback_deg[kJointsPerArm]);

    /// Check and filter a command (degrees, in-place).
    /// Commands pass through unchanged until the guard is armed.
    /// If the command violates the z-floor, it is replaced with the last safe
    /// command and false is returned.  Otherwise the safe snapshot is updated
    /// and true is returned.
    bool filter(size_t arm, double cmd_deg[kJointsPerArm],
                const rclcpp::Logger &logger);

    bool enabled() const { return enabled_; }
    bool armed() const { return armed_; }
    bool active() const { return enabled_ && armed_; }

private:
    bool enabled_{false};
    bool armed_{false};
    double z_threshold_{0.0};
    std::array<M4, kArmCount> mount_tf_{};
    M4 tool_tf_{};
    bool has_tool_{false};
    std::array<std::array<double, kJointsPerArm>, kArmCount> last_safe_deg_{};
    std::array<bool, kArmCount> has_safe_{{false, false}};
    std::array<int, kArmCount> violation_streak_{};

    bool checkArmSafe(size_t arm, const double cmd_deg[kJointsPerArm]) const;
};

}  // namespace marvin_system

#endif  // ROS2_CONTROL_MARVIN__HARDWARE__WORKSPACE_GUARD_HPP_
