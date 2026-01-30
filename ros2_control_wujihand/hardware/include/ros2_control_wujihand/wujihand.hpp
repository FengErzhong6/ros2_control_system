#ifndef ROS2_CONTROL_WUJIHAND__HARDWARE__WUJIHAND_HPP_
#define ROS2_CONTROL_WUJIHAND__HARDWARE__WUJIHAND_HPP_

#include <atomic>
#include <array>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
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

    ~WujiHandHardware() override;

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
    static constexpr size_t kFingerCount = 5;
    static constexpr size_t kJointCount = 4;
    static constexpr size_t kTotalJoints = kFingerCount * kJointCount;

    double low_pass_cutoff_frequency_;

    enum class IoRequestType {
        Configure,
        Activate,
        Deactivate,
        Cleanup,
    };
    struct IoRequest {
        IoRequestType type;
        std::promise<hardware_interface::CallbackReturn> promise;
    };

    hardware_interface::CallbackReturn submit_io_request(IoRequestType type);
    void io_thread_main();

    std::unique_ptr<wujihandcpp::device::Hand> hand_;
    std::unique_ptr<wujihandcpp::device::IController> controller_;
    std::atomic<bool> activated_{false};

    std::atomic<bool> io_stop_{false};
    std::atomic<bool> io_error_{false};
    std::thread io_thread_;
    std::mutex io_mutex_;
    std::condition_variable io_cv_;
    std::optional<IoRequest> io_request_;

    struct Frame {
        std::array<double, kTotalJoints> data{};
        uint64_t frame_id{0};
    };

    // State direction: IO thread writes, RT thread reads.
    std::array<Frame, 2> state_frames_{};
    std::atomic<uint8_t> state_active_{0};
    uint64_t state_frame_id_{0};

    // Command direction: RT thread writes, IO thread reads.
    std::array<Frame, 2> cmd_frames_{};
    std::atomic<uint8_t> cmd_active_{0};
    std::atomic<uint64_t> cmd_frame_id_{0};
};

}   // namespace ros2_control_wujihand

#endif // ROS2_CONTROL_WUJIHAND__HARDWARE__WUJIHAND_HPP_