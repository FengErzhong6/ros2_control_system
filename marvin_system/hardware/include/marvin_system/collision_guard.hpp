#ifndef ROS2_CONTROL_MARVIN__HARDWARE__COLLISION_GUARD_HPP_
#define ROS2_CONTROL_MARVIN__HARDWARE__COLLISION_GUARD_HPP_

#include <array>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"

namespace marvin_system {

class CollisionGuard {
public:
    static constexpr size_t kJointsPerArm = 7;
    static constexpr size_t kArmCount = 2;
    static constexpr size_t kGripperCount = 2;

    struct Evaluation {
        bool ready{false};
        bool safe{true};
        double max_safe_alpha{1.0};
        int blocking_sample{-1};
    };

    CollisionGuard();
    ~CollisionGuard();

    bool configure(
        const std::unordered_map<std::string, std::string> &params,
        const rclcpp::Node::SharedPtr &node,
        const rclcpp::Logger &logger);

    void arm();
    void disarm();

    bool enabled() const;
    bool armed() const;
    bool active() const;
    bool ready() const;

    double check_rate_hz() const;
    double min_command_delta_deg() const;
    double near_distance_m() const;
    double hard_collision_distance_m() const;
    double escape_min_distance_improvement_m() const;
    int interpolation_steps() const;
    int binary_search_steps() const;

    const std::string &status_message() const;

    Evaluation evaluate_motion(
        const std::array<std::array<double, kJointsPerArm>, kArmCount> &from_deg,
        const std::array<std::array<double, kJointsPerArm>, kArmCount> &to_deg,
        const std::array<double, kGripperCount> &gripper_percent,
        const std::array<bool, kGripperCount> &gripper_valid,
        const rclcpp::Logger &logger);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    bool enabled_{false};
    bool armed_{false};
    bool ready_{false};
    double check_rate_hz_{30.0};
    double min_command_delta_deg_{0.0};
    double near_distance_m_{0.10};
    double hard_collision_distance_m_{0.05};
    double escape_min_distance_improvement_m_{0.001};
    int interpolation_steps_{6};
    int binary_search_steps_{5};
    std::string status_message_{"Collision guard disabled."};
};

}  // namespace marvin_system

#endif  // ROS2_CONTROL_MARVIN__HARDWARE__COLLISION_GUARD_HPP_
