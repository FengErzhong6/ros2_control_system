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

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WujiHandHardware::on_configure(
    const rclcpp_lifecycle::State &
){
    activated_ = false;
    controller_.reset();

    // we only need to use the default construction function for the wujihand now.
    try{
        hand_ = std::make_unique<wujihandcpp::device::Hand>();
    } catch(const wujihandcpp::device::TimeoutError &e){
        RCLCPP_ERROR(get_logger(), "Timeout when connecting to wujihand: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    } catch(const std::exception &e){
        RCLCPP_ERROR(get_logger(), "Error when connecting to wujihand: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WujiHandHardware::on_activate(
    const rclcpp_lifecycle::State &
){
    activated_ = false;
    if(!hand_){
        RCLCPP_ERROR(get_logger(), "Cannot activate wujihand hardware, hand_ is null.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    try{
        // enable all joints
        hand_->write<wujihandcpp::data::joint::Enabled>(true);

        // create the realtime controller
        controller_ = hand_->realtime_controller<true>(wujihandcpp::filter::LowPass{5.0});
        if(!controller_){
            RCLCPP_ERROR(get_logger(), "Failed to get realtime controller from wujihand.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // command and state should be equal when starting
        const auto &actual_pos = controller_->get_joint_actual_position();

        double initial_target[5][4] = {};
        for(size_t f = 0; f < 5; ++f){
            for(size_t j = 0; j < 4; ++j){
                const double pos = actual_pos[f][j].load(std::memory_order_relaxed);
                const auto if_name = position_if_name_from_index(f, j);
                set_state(if_name, pos);
                set_command(if_name, pos);
                initial_target[f][j] = pos;
            }
        }
        controller_->set_joint_target_position(initial_target);
    } catch(const std::exception &e){
        RCLCPP_ERROR(get_logger(), "Exception during on_activate: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    activated_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WujiHandHardware::on_deactivate(
    const rclcpp_lifecycle::State &
){
    activated_ = false;
    // Goal: keep current position, then disable joints.
    if(!hand_){
        RCLCPP_WARN(get_logger(), "on_deactivate(): hand_ is null, skipping disable.");
        controller_.reset();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    try{
        if(controller_){
            const auto &actual_pos = controller_->get_joint_actual_position();
            double hold_target[5][4] = {};
            for(size_t f = 0; f < 5; ++f){
                for(size_t j = 0; j < 4; ++j){
                    const double pos = actual_pos[f][j].load(std::memory_order_relaxed);
                    const auto if_name = position_if_name_from_index(f, j);
                    set_state(if_name, pos);
                    set_command(if_name, pos);
                    hold_target[f][j] = pos;
                }
            }
            controller_->set_joint_target_position(hold_target);
        }

        // Disable all joints
        hand_->write<wujihandcpp::data::joint::Enabled>(false);
    } catch(const std::exception &e){
        RCLCPP_ERROR(get_logger(), "Exception during on_deactivate: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    controller_.reset();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WujiHandHardware::on_cleanup(
    const rclcpp_lifecycle::State &
){
    const bool was_activated = activated_;
    activated_ = false;

    bool ok = true;
    try{
        // cleanup() is expected after deactivate(), but may be called directly on error paths.
        // Only do a best-effort disable if we still look active.
        if(hand_ && (was_activated || controller_)){
            hand_->write<wujihandcpp::data::joint::Enabled>(false);
        }
    } catch(const std::exception &e){
        RCLCPP_ERROR(get_logger(), "Exception during on_cleanup (disable): %s", e.what());
        ok = false;
    }

    controller_.reset();
    hand_.reset();

    return ok ? hardware_interface::CallbackReturn::SUCCESS
              : hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::return_type WujiHandHardware::read(
    const rclcpp::Time &, const rclcpp::Duration &
){
    // When not activated, it's normal to not access the hardware.
    if(!activated_){
        return hardware_interface::return_type::OK;
    }
    // When activated, missing handles are an error.
    if(!hand_ || !controller_){
        RCLCPP_ERROR(get_logger(), "read(): activated but hand_/controller_ is null.");
        return hardware_interface::return_type::ERROR;
    }

    try{
        const auto &actual_pos = controller_->get_joint_actual_position();
        for(size_t f = 0; f < 5; ++f){
            for(size_t j = 0; j < 4; ++j){
                set_state(
                    position_if_name_from_index(f, j),
                    actual_pos[f][j].load(std::memory_order_relaxed)
                );
            }
        }
    } catch(const std::exception &e){
        RCLCPP_ERROR(get_logger(), "Exception during read(): %s", e.what());
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type WujiHandHardware::write(
    const rclcpp::Time &, const rclcpp::Duration &
){
    // When not activated, it's normal to not access the hardware.
    if(!activated_){
        return hardware_interface::return_type::OK;
    }
    // When activated, missing handles are an error.
    if(!hand_ || !controller_){
        RCLCPP_ERROR(get_logger(), "write(): activated but hand_/controller_ is null.");
        return hardware_interface::return_type::ERROR;
    }

    try{
        double target[5][4] = {};
        for(size_t f = 0; f < 5; ++f){
            for(size_t j = 0; j < 4; ++j){
                target[f][j] = get_command<double>(position_if_name_from_index(f, j));
            }
        }
        controller_->set_joint_target_position(target);
    } catch(const std::exception &e){
        RCLCPP_ERROR(get_logger(), "Exception during write(): %s", e.what());
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

}// namespace ros2_control_wujihand

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    ros2_control_wujihand::WujiHandHardware, hardware_interface::SystemInterface
)
