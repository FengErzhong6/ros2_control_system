#include "ros2_control_wujihand/rl_controller.hpp"

#include <stddef.h>

#include <algorithm>
#include <array>
#include <iterator>
#include <limits>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace ros2_control_wujihand{

namespace {

template <typename LoanedInterfaceT>
LoanedInterfaceT *find_loaned_interface(std::vector<LoanedInterfaceT> &interfaces, const std::string &full_name)
{
    for(auto &iface : interfaces){
        if(iface.get_name() == full_name){
            return &iface;
        }
    }
    return nullptr;
}

} // namespace

RLController::RLController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn RLController::on_init()
{
    const auto logger = get_node()->get_logger();

    try{
        // Explicit (fixed-length) parameter lists.
        // - joints: 20 joint names
        // - command_interfaces: 20 command interface types (one per joint)
        // - state_interfaces: 40 state interface types (first 20: position, next 20: velocity)
        auto_declare<std::vector<std::string>>("joints", {});
        auto_declare<std::vector<std::string>>("command_interfaces", {});
        auto_declare<std::vector<std::string>>("state_interfaces", {});
    } catch(const std::exception &e){
        RCLCPP_ERROR(logger, "Failed to declare parameters: %s", e.what());
        return CallbackReturn::ERROR;
    }

    // Declare-only: do not validate parameters here.
    // All parameter reading + validation happens in on_configure().
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RLController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
    conf.names.reserve(kActDim);
    for(size_t i = 0; i < kActDim; ++i){
        conf.names.push_back(joint_names_[i] + "/" + command_interface_types_[i]);
    }
    return conf;
}

controller_interface::InterfaceConfiguration RLController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
    conf.names.reserve(kObsDim);
    for(size_t i = 0; i < kObsDim; ++i){
        const size_t joint_index = (i < kNumJoints) ? i : (i - kNumJoints);
        conf.names.push_back(joint_names_[joint_index] + "/" + state_interface_types_[i]);
    }
    return conf;
}

controller_interface::CallbackReturn RLController::on_configure(const rclcpp_lifecycle::State &)
{
    const auto logger = get_node()->get_logger();

    // Read parameters.
    std::vector<std::string> joints_param;
    std::vector<std::string> command_ifaces_param;
    std::vector<std::string> state_ifaces_param;

    if(!get_node()->get_parameter("joints", joints_param)){
        RCLCPP_ERROR(logger, "Missing required parameter 'joints'.");
        return CallbackReturn::ERROR;
    }
    if(!get_node()->get_parameter("command_interfaces", command_ifaces_param)){
        RCLCPP_ERROR(logger, "Missing required parameter 'command_interfaces'.");
        return CallbackReturn::ERROR;
    }
    if(!get_node()->get_parameter("state_interfaces", state_ifaces_param)){
        RCLCPP_ERROR(logger, "Missing required parameter 'state_interfaces'.");
        return CallbackReturn::ERROR;
    }

    // Require explicit fixed-length lists.
    if(joints_param.size() != kNumJoints){
        RCLCPP_ERROR(logger, "Parameter 'joints' must have exactly %zu entries (got %zu).", kNumJoints, joints_param.size());
        return CallbackReturn::ERROR;
    }
    if(command_ifaces_param.size() != kActDim){
        RCLCPP_ERROR(
            logger,
            "Parameter 'command_interfaces' must have exactly %zu entries (got %zu).",
            kActDim,
            command_ifaces_param.size()
        );
        return CallbackReturn::ERROR;
    }
    if(state_ifaces_param.size() != kObsDim){
        RCLCPP_ERROR(
            logger,
            "Parameter 'state_interfaces' must have exactly %zu entries (got %zu).",
            kObsDim,
            state_ifaces_param.size()
        );
        return CallbackReturn::ERROR;
    }

    std::copy_n(joints_param.begin(), kNumJoints, joint_names_.begin());
    std::copy_n(command_ifaces_param.begin(), kActDim, command_interface_types_.begin());
    std::copy_n(state_ifaces_param.begin(), kObsDim, state_interface_types_.begin());

    // Validate joint names (non-empty + unique).
    std::unordered_set<std::string> seen;
    seen.reserve(kNumJoints);
    for(size_t i = 0; i < kNumJoints; ++i){
        if(joint_names_[i].empty()){
            RCLCPP_ERROR(logger, "Parameter 'joints[%zu]' is empty.", i);
            return CallbackReturn::ERROR;
        }
        if(!seen.insert(joint_names_[i]).second){
            RCLCPP_ERROR(logger, "Duplicate joint name in 'joints': '%s'.", joint_names_[i].c_str());
            return CallbackReturn::ERROR;
        }
    }

    // Validate interface types.
    for(size_t i = 0; i < kActDim; ++i){
        const auto &type = command_interface_types_[i];
        if(type != "position"){
            RCLCPP_ERROR(logger, "Unsupported command interface type '%s' at index %zu (only 'position' is supported).", type.c_str(), i);
            return CallbackReturn::ERROR;
        }
    }

    // Enforce fixed ordering: first 20 position, next 20 velocity.
    for(size_t i = 0; i < kNumJoints; ++i){
        if(state_interface_types_[i] != "position"){
            RCLCPP_ERROR(logger, "state_interfaces[%zu] must be 'position' (got '%s').", i, state_interface_types_[i].c_str());
            return CallbackReturn::ERROR;
        }
    }
    for(size_t i = kNumJoints; i < kObsDim; ++i){
        if(state_interface_types_[i] != "velocity"){
            RCLCPP_ERROR(logger, "state_interfaces[%zu] must be 'velocity' (got '%s').", i, state_interface_types_[i].c_str());
            return CallbackReturn::ERROR;
        }
    }

    // Reset cached interface pointers.
    for(size_t i = 0; i < kNumJoints; ++i){
        joint_position_command_interface_[i] = nullptr;
        joint_position_state_interface_[i] = nullptr;
        joint_velocity_state_interface_[i] = nullptr;
    }

    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RLController::on_activate(const rclcpp_lifecycle::State &)
{
    const auto logger = get_node()->get_logger();

    // Bind per-joint command interfaces.
    for(size_t i = 0; i < kNumJoints; ++i){
        const auto &type = command_interface_types_[i];
        if(type != "position"){
            RCLCPP_ERROR(logger, "Unsupported command interface type '%s' at index %zu (only 'position' is supported).", type.c_str(), i);
            return CallbackReturn::ERROR;
        }
        const std::string full_name = joint_names_[i] + "/" + type;
        auto *iface = find_loaned_interface(command_interfaces_, full_name);
        if(iface == nullptr){
            RCLCPP_ERROR(logger, "Missing command interface '%s'.", full_name.c_str());
            return CallbackReturn::ERROR;
        }
        joint_position_command_interface_[i] = iface;
    }

    // Bind per-joint state interfaces (required by the fast update path).
    for(size_t i = 0; i < kNumJoints; ++i){
        {
            const std::string full_name = joint_names_[i] + "/position";
            auto *iface = find_loaned_interface(state_interfaces_, full_name);
            if(iface == nullptr){
                RCLCPP_ERROR(logger, "Missing state interface '%s'.", full_name.c_str());
                return CallbackReturn::ERROR;
            }
            joint_position_state_interface_[i] = iface;
        }
        {
            const std::string full_name = joint_names_[i] + "/velocity";
            auto *iface = find_loaned_interface(state_interfaces_, full_name);
            if(iface == nullptr){
                RCLCPP_ERROR(logger, "Missing state interface '%s'.", full_name.c_str());
                return CallbackReturn::ERROR;
            }
            joint_velocity_state_interface_[i] = iface;
        }
    }

    // Initialize commands to current positions if possible (avoid jumps).
    for(size_t i = 0; i < kNumJoints; ++i){
        const auto pos_opt = joint_position_state_interface_[i]->get_optional<double>();
        if(pos_opt.has_value()){
            (void)joint_position_command_interface_[i]->set_value(pos_opt.value());
        }
    }

    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RLController::on_deactivate(const rclcpp_lifecycle::State &)
{
    for(size_t i = 0; i < kNumJoints; ++i){
        joint_position_command_interface_[i] = nullptr;
        joint_position_state_interface_[i] = nullptr;
        joint_velocity_state_interface_[i] = nullptr;
    }
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type RLController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
    // Fast path: fixed ordering (first 20 position, next 20 velocity).
    std::array<double, kObsDim> obs{};

    for(size_t i = 0; i < kNumJoints; ++i){
        const auto v = joint_position_state_interface_[i]->get_optional<double>();
        obs[i] = v.value_or(std::numeric_limits<double>::quiet_NaN());
    }
    for(size_t i = 0; i < kNumJoints; ++i){
        const auto v = joint_velocity_state_interface_[i]->get_optional<double>();
        obs[i + kNumJoints] = v.value_or(std::numeric_limits<double>::quiet_NaN());
    }

    // Temporary behavior: command all joints to zero position.
    for(size_t i = 0; i < kNumJoints; ++i){
        (void)joint_position_command_interface_[i]->set_value(0.0);
    }

    (void)obs;
    return controller_interface::return_type::OK;
}

} // namespace ros2_control_wujihand

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ros2_control_wujihand::RLController, controller_interface::ControllerInterface)