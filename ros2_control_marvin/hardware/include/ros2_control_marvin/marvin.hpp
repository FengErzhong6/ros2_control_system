#ifndef ROS2_CONTROL_MARVIN__HARDWARE__MARVIN_HPP_
#define ROS2_CONTROL_MARVIN__HARDWARE__MARVIN_HPP_

#include <chrono>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Keep SDK headers out of this public header when possible.
// Include MarvinSDK headers in the .cpp implementation file.

namespace ros2_control_marvin{

class MarvinHardware : public hardware_interface::SystemInterface{
public:
	RCLCPP_SHARED_PTR_DEFINITIONS(MarvinHardware)

	MarvinHardware() = default;
	~MarvinHardware() override;

	hardware_interface::CallbackReturn on_init(
		const hardware_interface::HardwareComponentInterfaceParams &params
	) override;

	hardware_interface::CallbackReturn on_configure(
		const rclcpp_lifecycle::State &previous_state
	) override;

	hardware_interface::CallbackReturn on_activate(
		const rclcpp_lifecycle::State &previous_state
	) override;

	hardware_interface::CallbackReturn on_deactivate(
		const rclcpp_lifecycle::State &previous_state
	) override;

	hardware_interface::CallbackReturn on_cleanup(
		const rclcpp_lifecycle::State &previous_state
	) override;

	hardware_interface::return_type read(
		const rclcpp::Time &time, const rclcpp::Duration &period
	) override;

	hardware_interface::return_type write(
		const rclcpp::Time &time, const rclcpp::Duration &period
	) override;

private:
	// Parsed from URDF/ros2_control tags.
	size_t joint_count_{0};

	// State buffers (size == joint_count_). Populate in read().
	std::vector<double> hw_positions_;
	std::vector<double> hw_velocities_;
	std::vector<double> hw_efforts_;

	// Command buffers (size == joint_count_). Consume in write().
	std::vector<double> hw_position_commands_;

	// Joint limits (size == joint_count_). Parsed from <command_interface> params.
	std::vector<double> joint_min_;
	std::vector<double> joint_max_;

	// Interface availability (depends on URDF declared state/command interfaces).
	bool has_velocity_state_{false};
	bool has_effort_state_{false};

	// SDK mapping / safety parameters.
	int arm_idx_{0};
	int no_frame_timeout_ms_{800};
	int last_frame_serial_{-1};
	std::chrono::steady_clock::time_point last_frame_time_{};
	std::chrono::steady_clock::time_point last_error_log_time_{};
	bool activated_{false};
};

} // namespace ros2_control_marvin

#endif // ROS2_CONTROL_MARVIN__HARDWARE__MARVIN_HPP_