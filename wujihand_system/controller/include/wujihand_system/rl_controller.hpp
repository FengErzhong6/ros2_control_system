#ifndef ROS2_CONTROL_WUJIHAND__CONTROLLER__RL_CONTROLLER_HPP_
#define ROS2_CONTROL_WUJIHAND__CONTROLLER__RL_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace wujihand_system{
class RLPolicy;

class RLController : public controller_interface::ControllerInterface{
public:
    RLController();

    ~RLController() override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(
        const rclcpp::Time &time, const rclcpp::Duration &period
    ) override;

    controller_interface::CallbackReturn on_init() override;

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state
    ) override;

    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state
    ) override;

    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state
    ) override;

protected:
    // fixed sizes
    static constexpr size_t kNumJoints = 20;
    static constexpr size_t kObsDim = kNumJoints * 2;
    static constexpr size_t kActDim = kNumJoints;

    // interface names
    std::array<std::string, kNumJoints> joint_names_{};

    // full interface names
    std::array<std::string, kActDim> command_interface_types_{};
    std::array<std::string, kObsDim> state_interface_types_{}; // first kNumJoints: position, next kNumJoints: velocity

    // loaned interfaces (avoid name lookups in update)
    std::array<hardware_interface::LoanedCommandInterface *, kNumJoints> joint_position_command_interface_{};
    std::array<hardware_interface::LoanedStateInterface *, kNumJoints> joint_position_state_interface_{};
    std::array<hardware_interface::LoanedStateInterface *, kNumJoints> joint_velocity_state_interface_{};

    // RL policy
    std::unique_ptr<RLPolicy> policy_;
    double joint_vel_obs_scale_{1.0};
    std::array<double, kNumJoints> lower_bound_{};
    std::array<double, kNumJoints> upper_bound_{};
    std::array<double, kNumJoints> range_{};
    std::array<double, kNumJoints> inv_range_{};
    std::array<double, kNumJoints> sum_{};

    std::array<float, kObsDim> policy_obs_{};
    std::array<float, kActDim> policy_act_{};
};

} // namespace wujihand_system

#endif