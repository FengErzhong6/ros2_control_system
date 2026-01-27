#include "ros2_control_wujihand/wujihand.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "wujihandcpp/data/hand.hpp"
#include "wujihandcpp/data/joint.hpp"
#include "wujihandcpp/filter/low_pass.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
// help functions to get joint and interface names
inline std::string joint_name_from_index(const size_t finger_idx, const size_t joint_idx)
{
    return "finger" + std::to_string(finger_idx) + "_joint" + std::to_string(joint_idx);
}

inline std::string position_if_name_from_index(const size_t finger_idx, const size_t joint_idx)
{
    return joint_name_from_index(finger_idx, joint_idx) + "/" + hardware_interface::HW_IF_POSITION;
}
}  // namespace

namespace ros2_control_wujihand{

WujiHandHardware::~WujiHandHardware()
{
    io_stop_.store(true, std::memory_order_relaxed);
    io_cv_.notify_all();
    if(io_thread_.joinable()){
        io_thread_.join();
    }
}

hardware_interface::CallbackReturn WujiHandHardware::submit_io_request(IoRequestType type)
{
    IoRequest req{type, std::promise<hardware_interface::CallbackReturn>{}};
    auto fut = req.promise.get_future();
    {
        std::lock_guard<std::mutex> lock(io_mutex_);
        if(io_request_.has_value()){
            RCLCPP_ERROR(get_logger(), "I/O thread request already pending.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        io_request_.emplace(std::move(req));
    }
    io_cv_.notify_all();
    return fut.get();
}

void WujiHandHardware::io_thread_main()
{
    using namespace std::chrono;

    // Default to 100 Hz if rw_rate is not set.
    const int rate_hz = (info_.rw_rate > 0) ? info_.rw_rate : 100;
    const auto period = nanoseconds(static_cast<long long>(1e9 / rate_hz));

    while(!io_stop_.load(std::memory_order_relaxed)){
        // Wait for either a request or the next periodic tick.
        std::optional<IoRequest> req;
        {
            std::unique_lock<std::mutex> lock(io_mutex_);
            io_cv_.wait_for(lock, period, [&] {
                return io_stop_.load(std::memory_order_relaxed) || io_request_.has_value();
            });

            if(io_stop_.load(std::memory_order_relaxed)){
                break;
            }

            if(io_request_.has_value()){
                req = std::move(io_request_);
                io_request_.reset();
            }
        }

        // Handle request first (synchronous lifecycle semantics).
        if(req.has_value()){
            hardware_interface::CallbackReturn result = hardware_interface::CallbackReturn::ERROR;
            try{
                switch(req->type){
                    case IoRequestType::Configure: {
                        io_error_.store(false, std::memory_order_relaxed);
                        controller_.reset();
                        hand_ = std::make_unique<wujihandcpp::device::Hand>();
                        result = hardware_interface::CallbackReturn::SUCCESS;
                        break;
                    }
                    case IoRequestType::Activate: {
                        io_error_.store(false, std::memory_order_relaxed);
                        if(!hand_){
                            RCLCPP_ERROR(get_logger(), "I/O Activate: hand_ is null.");
                            result = hardware_interface::CallbackReturn::ERROR;
                            break;
                        }

                        // enable all joints
                        hand_->write<wujihandcpp::data::joint::Enabled>(true);

                        // create the realtime controller
                        controller_ = hand_->realtime_controller<true>(wujihandcpp::filter::LowPass{5.0});
                        if(!controller_){
                            RCLCPP_ERROR(get_logger(), "I/O Activate: failed to get realtime controller.");
                            result = hardware_interface::CallbackReturn::ERROR;
                            break;
                        }

                        // Initialize caches to current position and hold.
                        const auto &actual_pos = controller_->get_joint_actual_position();
                        double hold_target[kFingerCount][kJointCount] = {};
                        for(size_t f = 0; f < kFingerCount; ++f){
                            for(size_t j = 0; j < kJointCount; ++j){
                                const double pos = actual_pos[f][j].load(std::memory_order_relaxed);
                                state_cache_[f][j].store(pos, std::memory_order_relaxed);
                                target_cache_[f][j].store(pos, std::memory_order_relaxed);
                                hold_target[f][j] = pos;
                            }
                        }
                        controller_->set_joint_target_position(hold_target);
                        result = hardware_interface::CallbackReturn::SUCCESS;
                        break;
                    }
                    case IoRequestType::Deactivate: {
                        io_error_.store(false, std::memory_order_relaxed);
                        if(hand_ && controller_){
                            const auto &actual_pos = controller_->get_joint_actual_position();
                            double hold_target[kFingerCount][kJointCount] = {};
                            for(size_t f = 0; f < kFingerCount; ++f){
                                for(size_t j = 0; j < kJointCount; ++j){
                                    const double pos = actual_pos[f][j].load(std::memory_order_relaxed);
                                    state_cache_[f][j].store(pos, std::memory_order_relaxed);
                                    target_cache_[f][j].store(pos, std::memory_order_relaxed);
                                    hold_target[f][j] = pos;
                                }
                            }
                            controller_->set_joint_target_position(hold_target);
                        }
                        if(hand_){
                            hand_->write<wujihandcpp::data::joint::Enabled>(false);
                        }
                        controller_.reset();
                        result = hardware_interface::CallbackReturn::SUCCESS;
                        break;
                    }
                    case IoRequestType::Cleanup: {
                        io_error_.store(false, std::memory_order_relaxed);
                        try{
                            if(hand_){
                                hand_->write<wujihandcpp::data::joint::Enabled>(false);
                            }
                        } catch(const std::exception &e){
                            // Best-effort; proceed to release resources.
                            RCLCPP_WARN(get_logger(), "I/O Cleanup: disable failed: %s", e.what());
                        }
                        controller_.reset();
                        hand_.reset();
                        result = hardware_interface::CallbackReturn::SUCCESS;
                        break;
                    }
                }
            } catch(const std::exception &e){
                RCLCPP_ERROR(get_logger(), "I/O thread request exception: %s", e.what());
                result = hardware_interface::CallbackReturn::ERROR;
            }

            req->promise.set_value(result);
            continue;
        }

        // Periodic I/O while activated.
        if(!activated_.load(std::memory_order_relaxed)){
            continue;
        }
        if(io_error_.load(std::memory_order_relaxed)){
            continue;
        }
        if(!hand_ || !controller_){
            continue;
        }

        try{
            const auto &actual_pos = controller_->get_joint_actual_position();
            for(size_t f = 0; f < kFingerCount; ++f){
                for(size_t j = 0; j < kJointCount; ++j){
                    state_cache_[f][j].store(
                        actual_pos[f][j].load(std::memory_order_relaxed),
                        std::memory_order_relaxed
                    );
                }
            }

            double target[kFingerCount][kJointCount] = {};
            for(size_t f = 0; f < kFingerCount; ++f){
                for(size_t j = 0; j < kJointCount; ++j){
                    target[f][j] = target_cache_[f][j].load(std::memory_order_relaxed);
                }
            }
            controller_->set_joint_target_position(target);
        } catch(const std::exception &e){
            io_error_.store(true, std::memory_order_relaxed);
            RCLCPP_ERROR(get_logger(), "I/O cycle exception: %s", e.what());
        }
    }
}

hardware_interface::CallbackReturn WujiHandHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params
){
    if(hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS){
        return hardware_interface::CallbackReturn::ERROR;
    }
    // If we need to read some parameters from the ros2_control URDF, please add the code here.

    // Log the info about the hw here
    RCLCPP_INFO(get_logger(), "wujihand hardware update is %dHz.", info_.rw_rate);

    // check joint configurations in URDF
    for(const auto &joint : info_.joints){
        // wujihand has exactly one state and command interface on each joint
        if(joint.command_interfaces.size() != 1){
            RCLCPP_FATAL(
                get_logger(),
                "Joint '%s' has %zu command interfaces found. 1 expected.",
                joint.name.c_str(),
                joint.command_interfaces.size()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        if(joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION){
            RCLCPP_FATAL(
                get_logger(),
                "Joint '%s' has '%s' command interface. '%s' expected.",
                joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        if(joint.state_interfaces.size() != 1){
            RCLCPP_FATAL(
                get_logger(),
                "Joint '%s' has %zu state interfaces found. 1 expected.",
                joint.name.c_str(),
                joint.state_interfaces.size()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        if(joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION){
            RCLCPP_FATAL(
                get_logger(),
                "Joint '%s' has '%s' state interface. '%s' expected.",
                joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    // Start the dedicated I/O thread once. It will be responsible for ALL SDK calls.
    if(!io_thread_.joinable()){
        io_stop_.store(false, std::memory_order_relaxed);
        io_thread_ = std::thread(&WujiHandHardware::io_thread_main, this);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WujiHandHardware::on_configure(
    const rclcpp_lifecycle::State &
){
    activated_.store(false, std::memory_order_relaxed);
    io_error_.store(false, std::memory_order_relaxed);
    return submit_io_request(IoRequestType::Configure);
}

hardware_interface::CallbackReturn WujiHandHardware::on_activate(
    const rclcpp_lifecycle::State &
){
    activated_.store(false, std::memory_order_relaxed);
    io_error_.store(false, std::memory_order_relaxed);

    const auto ret = submit_io_request(IoRequestType::Activate);
    if(ret != hardware_interface::CallbackReturn::SUCCESS){
        return ret;
    }

    // Align command/state interfaces to the cached initial positions.
    for(size_t f = 0; f < kFingerCount; ++f){
        for(size_t j = 0; j < kJointCount; ++j){
            const double pos = state_cache_[f][j].load(std::memory_order_relaxed);
            const auto if_name = position_if_name_from_index(f, j);
            set_state(if_name, pos);
            set_command(if_name, pos);
        }
    }

    activated_.store(true, std::memory_order_relaxed);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WujiHandHardware::on_deactivate(
    const rclcpp_lifecycle::State &
){
    activated_.store(false, std::memory_order_relaxed);
    io_error_.store(false, std::memory_order_relaxed);
    return submit_io_request(IoRequestType::Deactivate);
}

hardware_interface::CallbackReturn WujiHandHardware::on_cleanup(
    const rclcpp_lifecycle::State &
){
    activated_.store(false, std::memory_order_relaxed);
    io_error_.store(false, std::memory_order_relaxed);
    return submit_io_request(IoRequestType::Cleanup);
}

hardware_interface::return_type WujiHandHardware::read(
    const rclcpp::Time &, const rclcpp::Duration &
){
    if(!activated_.load(std::memory_order_relaxed)){
        return hardware_interface::return_type::OK;
    }
    if(io_error_.load(std::memory_order_relaxed)){
        return hardware_interface::return_type::ERROR;
    }

    for(size_t f = 0; f < kFingerCount; ++f){
        for(size_t j = 0; j < kJointCount; ++j){
            set_state(
                position_if_name_from_index(f, j),
                state_cache_[f][j].load(std::memory_order_relaxed)
            );
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type WujiHandHardware::write(
    const rclcpp::Time &, const rclcpp::Duration &
){
    if(!activated_.load(std::memory_order_relaxed)){
        return hardware_interface::return_type::OK;
    }
    if(io_error_.load(std::memory_order_relaxed)){
        return hardware_interface::return_type::ERROR;
    }

    for(size_t f = 0; f < kFingerCount; ++f){
        for(size_t j = 0; j < kJointCount; ++j){
            target_cache_[f][j].store(
                get_command<double>(position_if_name_from_index(f, j)),
                std::memory_order_relaxed
            );
        }
    }
    return hardware_interface::return_type::OK;
}

}// namespace ros2_control_wujihand

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    ros2_control_wujihand::WujiHandHardware, hardware_interface::SystemInterface
)
