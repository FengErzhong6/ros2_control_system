#ifndef ROS2_CONTROL_WUJIHAND__HARDWARE__WUJIHAND_HPP_
#define ROS2_CONTROL_WUJIHAND__HARDWARE__WUJIHAND_HPP_
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "wujihandcpp/device/hand.hpp"
#include "wujihandcpp/device/controller.hpp"

namespace ros2_control_wujihand{
class WujiHandHardware : public hardware_interface::SystemInterface{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(WujiHandHardware)

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
    std::unique_ptr<wujihandcpp::device::Hand> hand_;
    std::unique_ptr<wujihandcpp::device::IController> controller_;
    bool activated_{false};
};

}   // namespace ros2_control_wujihand

#endif // ROS2_CONTROL_WUJIHAND__HARDWARE__WUJIHAND_HPP_