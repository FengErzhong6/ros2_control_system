#include <algorithm>
#include <array>
#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit/utils/moveit_error_code.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace {

constexpr size_t kJointsPerArm = 7;

const std::array<const char*, kJointsPerArm> kLeftJointNames{
    {"Joint1_L", "Joint2_L", "Joint3_L", "Joint4_L", "Joint5_L", "Joint6_L", "Joint7_L"}};
const std::array<const char*, kJointsPerArm> kRightJointNames{
    {"Joint1_R", "Joint2_R", "Joint3_R", "Joint4_R", "Joint5_R", "Joint6_R", "Joint7_R"}};

template <typename T>
T get_param_or_declare(rclcpp::Node *node,
                       const std::string &name,
                       const T &default_value)
{
    if (node->has_parameter(name)) {
        return node->get_parameter(name).get_value<T>();
    }
    return node->declare_parameter<T>(name, default_value);
}

std::vector<double> get_double_array_param(
    const rclcpp::Node *node,
    const std::string &name)
{
    if (!node->has_parameter(name)) {
        return {};
    }
    const auto value = node->get_parameter(name).as_double_array();
    return std::vector<double>(value.begin(), value.end());
}

std::string to_bool_string(bool value)
{
    return value ? "ON" : "OFF";
}

}  // namespace

namespace marvin_system {

class MarvinMotionServer : public rclcpp::Node {
public:
    explicit MarvinMotionServer(rclcpp::NodeOptions options)
        : rclcpp::Node(
              "marvin_motion_server",
              options.automatically_declare_parameters_from_overrides(true))
    {
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        backend_ = get_param_or_declare<std::string>(this, "backend", "legacy");
        go_home_service_name_ = get_param_or_declare<std::string>(
            this, "go_home_service_name", "/marvin_motion/go_home");
        legacy_go_home_service_ = get_param_or_declare<std::string>(
            this, "legacy_go_home_service", "/tracker_teleop_controller/go_home");
        allow_legacy_go_home_fallback_ = get_param_or_declare<bool>(
            this, "allow_legacy_go_home_fallback", false);
        planning_group_ = get_param_or_declare<std::string>(
            this, "planning_group", "dual_arm");
        home_pose_id_ = get_param_or_declare<std::string>(
            this, "home_pose_id", "home");
        planning_time_sec_ = get_param_or_declare<double>(
            this, "planning_time_sec", 5.0);
        move_group_wait_sec_ = get_param_or_declare<double>(
            this, "move_group_wait_sec", 10.0);
        num_planning_attempts_ = get_param_or_declare<int64_t>(
            this, "num_planning_attempts", 3);
        max_velocity_scaling_ = get_param_or_declare<double>(
            this, "max_velocity_scaling", 0.2);
        max_acceleration_scaling_ = get_param_or_declare<double>(
            this, "max_acceleration_scaling", 0.2);
        execute_trajectory_ = get_param_or_declare<bool>(
            this, "execute_trajectory", true);
        switch_to_trajectory_controller_ = get_param_or_declare<bool>(
            this, "switch_to_trajectory_controller", false);
        reactivate_controller_after_moveit_ = get_param_or_declare<bool>(
            this, "reactivate_controller_after_moveit", false);
        controller_switch_timeout_sec_ = get_param_or_declare<double>(
            this, "controller_switch_timeout_sec", 5.0);
        planning_pipeline_id_ = get_param_or_declare<std::string>(
            this, "planning_pipeline_id", "ompl");
        planner_id_ = get_param_or_declare<std::string>(
            this, "planner_id", "RRTConnect");
        scene_frame_id_ = get_param_or_declare<std::string>(
            this, "scene_frame_id", "world");
        controller_manager_switch_service_ = get_param_or_declare<std::string>(
            this, "controller_manager_switch_service", "/controller_manager/switch_controller");
        trajectory_controller_name_ = get_param_or_declare<std::string>(
            this, "trajectory_controller_name", "dual_arm_trajectory_controller");
        primary_controller_name_ = get_param_or_declare<std::string>(
            this, "primary_controller_name", "");

        legacy_go_home_client_ = this->create_client<std_srvs::srv::Trigger>(
            legacy_go_home_service_,
            rclcpp::ServicesQoS(),
            callback_group_);
        switch_controller_client_ =
            this->create_client<controller_manager_msgs::srv::SwitchController>(
                controller_manager_switch_service_,
                rclcpp::ServicesQoS(),
                callback_group_);
        go_home_service_ = this->create_service<std_srvs::srv::Trigger>(
            go_home_service_name_,
            std::bind(
                &MarvinMotionServer::handle_go_home,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            rclcpp::ServicesQoS(),
            callback_group_);

        const auto home_left = get_named_pose_values("left");
        const auto home_right = get_named_pose_values("right");
        RCLCPP_INFO(
            get_logger(),
            "Motion layer ready. backend=%s service=%s legacy_fallback=%s "
            "planning_group=%s execute=%s switch_ctrl=%s home_left=%zu home_right=%zu scene_objects=%zu",
            backend_.c_str(),
            go_home_service_name_.c_str(),
            to_bool_string(allow_legacy_go_home_fallback_).c_str(),
            planning_group_.c_str(),
            to_bool_string(execute_trajectory_).c_str(),
            to_bool_string(switch_to_trajectory_controller_).c_str(),
            home_left.size(),
            home_right.size(),
            count_scene_objects());
    }

private:
    std::vector<double> get_named_pose_values(const std::string &side) const
    {
        return get_double_array_param(this, "named_poses." + home_pose_id_ + "." + side);
    }

    size_t count_scene_objects() const
    {
        const auto params = this->list_parameters({"scene"}, 4).names;
        std::set<std::string> object_names;
        for (const auto &name : params) {
            const auto first_dot = name.find('.');
            const auto second_dot = name.find('.', first_dot + 1);
            if (first_dot == std::string::npos || second_dot == std::string::npos) {
                continue;
            }
            object_names.insert(name.substr(first_dot + 1, second_dot - first_dot - 1));
        }
        return object_names.size();
    }

    bool ensure_moveit_interfaces(std::string &error_message)
    {
        std::lock_guard<std::mutex> lock(moveit_mutex_);
        if (move_group_ && planning_scene_interface_) {
            return true;
        }

        try {
            planning_scene_interface_ =
                std::make_unique<moveit::planning_interface::PlanningSceneInterface>();

            moveit::planning_interface::MoveGroupInterface::Options options(planning_group_);
            move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
                std::static_pointer_cast<rclcpp::Node>(shared_from_this()),
                options,
                std::shared_ptr<tf2_ros::Buffer>(),
                rclcpp::Duration::from_seconds(move_group_wait_sec_));
            move_group_->setPlanningTime(planning_time_sec_);
            move_group_->setNumPlanningAttempts(
                static_cast<unsigned int>(std::max<int64_t>(1, num_planning_attempts_)));
            move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_);
            move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_);
            if (!planning_pipeline_id_.empty()) {
                move_group_->setPlanningPipelineId(planning_pipeline_id_);
            }
            if (!planner_id_.empty()) {
                move_group_->setPlannerId(planner_id_);
            }
            move_group_->startStateMonitor();

            RCLCPP_INFO(
                get_logger(),
                "MoveIt interface connected. group=%s planning_frame=%s end_effector=%s",
                move_group_->getName().c_str(),
                move_group_->getPlanningFrame().c_str(),
                move_group_->getEndEffectorLink().c_str());
        } catch (const std::exception &ex) {
            move_group_.reset();
            planning_scene_interface_.reset();
            error_message = std::string("Failed to initialize MoveIt interfaces: ") + ex.what();
            return false;
        }
        return true;
    }

    bool apply_scene(std::string &error_message)
    {
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        std::set<std::string> object_names;
        const auto params = this->list_parameters({"scene"}, 4).names;
        for (const auto &name : params) {
            const auto parts = split_param_name(name);
            if (parts.size() >= 3 && parts[0] == "scene") {
                object_names.insert(parts[1]);
            }
        }

        for (const auto &object_name : object_names) {
            const auto enabled = get_param_or_declare<bool>(
                this, "scene." + object_name + ".enabled", false);
            if (!enabled) {
                continue;
            }

            const auto size = get_double_array_param(
                this, "scene." + object_name + ".size");
            const auto position = get_double_array_param(
                this, "scene." + object_name + ".pose.position");
            const auto orientation = get_double_array_param(
                this, "scene." + object_name + ".pose.orientation");

            if (size.size() != 3 || position.size() != 3 || orientation.size() != 4) {
                error_message = "Invalid scene object parameters for '" + object_name + "'";
                return false;
            }

            moveit_msgs::msg::CollisionObject object;
            object.id = object_name;
            object.header.frame_id = scene_frame_id_;
            object.operation = moveit_msgs::msg::CollisionObject::ADD;

            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
            primitive.dimensions = {size[0], size[1], size[2]};

            geometry_msgs::msg::Pose pose;
            pose.position.x = position[0];
            pose.position.y = position[1];
            pose.position.z = position[2];
            pose.orientation.x = orientation[0];
            pose.orientation.y = orientation[1];
            pose.orientation.z = orientation[2];
            pose.orientation.w = orientation[3];

            object.primitives.push_back(primitive);
            object.primitive_poses.push_back(pose);
            collision_objects.push_back(object);
        }

        if (collision_objects.empty()) {
            return true;
        }

        if (!planning_scene_interface_->applyCollisionObjects(collision_objects)) {
            error_message = "Failed to apply collision objects to planning scene.";
            return false;
        }

        RCLCPP_INFO(
            get_logger(),
            "Applied %zu collision object(s) to planning scene.",
            collision_objects.size());
        return true;
    }

    static std::vector<std::string> split_param_name(const std::string &name)
    {
        std::vector<std::string> parts;
        size_t start = 0;
        while (start < name.size()) {
            const auto end = name.find('.', start);
            if (end == std::string::npos) {
                parts.push_back(name.substr(start));
                break;
            }
            parts.push_back(name.substr(start, end - start));
            start = end + 1;
        }
        return parts;
    }

    bool build_home_target(std::map<std::string, double> &target, std::string &error_message)
    {
        const auto left = get_named_pose_values("left");
        const auto right = get_named_pose_values("right");
        if (left.size() != kJointsPerArm || right.size() != kJointsPerArm) {
            error_message = "Named pose '" + home_pose_id_ + "' is incomplete.";
            return false;
        }

        for (size_t i = 0; i < kJointsPerArm; ++i) {
            target[kLeftJointNames[i]] = left[i];
            target[kRightJointNames[i]] = right[i];
        }
        return true;
    }

    bool switch_controllers(
        const std::vector<std::string> &activate,
        const std::vector<std::string> &deactivate,
        int32_t strictness,
        std::string &error_message)
    {
        if (!switch_controller_client_->wait_for_service(
                std::chrono::duration<double>(controller_switch_timeout_sec_))) {
            error_message =
                "Controller switch service unavailable: " + controller_manager_switch_service_;
            return false;
        }

        auto request =
            std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->activate_controllers = activate;
        request->deactivate_controllers = deactivate;
        request->strictness = strictness;
        request->activate_asap = false;
        request->timeout.sec = static_cast<int32_t>(controller_switch_timeout_sec_);
        request->timeout.nanosec = static_cast<uint32_t>(
            std::max(0.0, controller_switch_timeout_sec_ -
                              static_cast<double>(request->timeout.sec)) *
            1.0e9);

        auto future = switch_controller_client_->async_send_request(request);
        const auto status = future.wait_for(
            std::chrono::duration<double>(controller_switch_timeout_sec_ + 1.0));
        if (status != std::future_status::ready) {
            error_message = "Controller switch timed out.";
            return false;
        }

        const auto result = future.get();
        if (!result->ok) {
            if (strictness == controller_manager_msgs::srv::SwitchController::Request::FORCE_AUTO) {
                RCLCPP_WARN(
                    get_logger(),
                    "Controller switch reported non-OK during FORCE_AUTO mode, continuing: %s",
                    result->message.c_str());
                return true;
            }
            error_message = "Controller switch failed: " + result->message;
            return false;
        }
        return true;
    }

    void handle_legacy_go_home(std_srvs::srv::Trigger::Response &response)
    {
        if (!allow_legacy_go_home_fallback_) {
            response.success = false;
            response.message =
                "Legacy go_home fallback is disabled. Select the MoveIt backend for safe planning.";
            RCLCPP_ERROR(get_logger(), "%s", response.message.c_str());
            return;
        }

        if (!legacy_go_home_client_->wait_for_service(std::chrono::seconds(2))) {
            response.success = false;
            response.message = "Legacy go_home service unavailable: " + legacy_go_home_service_;
            RCLCPP_ERROR(get_logger(), "%s", response.message.c_str());
            return;
        }

        RCLCPP_WARN(
            get_logger(),
            "Using legacy go_home fallback via %s. This path is transitional and not MoveIt-backed.",
            legacy_go_home_service_.c_str());
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = legacy_go_home_client_->async_send_request(request);
        const auto status = future.wait_for(std::chrono::seconds(10));
        if (status != std::future_status::ready) {
            response.success = false;
            response.message = "Legacy go_home fallback timed out.";
            RCLCPP_ERROR(get_logger(), "%s", response.message.c_str());
            return;
        }

        const auto result = future.get();
        response.success = result->success;
        response.message = result->message;
    }

    void handle_moveit_go_home(std_srvs::srv::Trigger::Response &response)
    {
        std::string error_message;
        if (!ensure_moveit_interfaces(error_message)) {
            response.success = false;
            response.message = error_message;
            RCLCPP_ERROR(get_logger(), "%s", response.message.c_str());
            return;
        }

        if (!apply_scene(error_message)) {
            response.success = false;
            response.message = error_message;
            RCLCPP_ERROR(get_logger(), "%s", response.message.c_str());
            return;
        }

        std::map<std::string, double> target;
        if (!build_home_target(target, error_message)) {
            response.success = false;
            response.message = error_message;
            RCLCPP_ERROR(get_logger(), "%s", response.message.c_str());
            return;
        }

        bool switched_to_trajectory_controller = false;
        auto restore_primary_controller = [&]() {
            if (!switched_to_trajectory_controller || !reactivate_controller_after_moveit_) {
                return;
            }
            std::string restore_error;
            if (primary_controller_name_.empty()) {
                return;
            }
            if (!switch_controllers(
                    {primary_controller_name_},
                    {},
                    controller_manager_msgs::srv::SwitchController::Request::FORCE_AUTO,
                    restore_error)) {
                RCLCPP_ERROR(get_logger(), "Failed to restore primary controller: %s",
                             restore_error.c_str());
            }
        };

        if (switch_to_trajectory_controller_) {
            if (!switch_controllers(
                    {trajectory_controller_name_},
                    {},
                    controller_manager_msgs::srv::SwitchController::Request::FORCE_AUTO,
                    error_message)) {
                response.success = false;
                response.message = error_message;
                RCLCPP_ERROR(get_logger(), "%s", response.message.c_str());
                return;
            }
            switched_to_trajectory_controller = true;
            RCLCPP_INFO(
                get_logger(),
                "Switched to trajectory controller '%s' for MoveIt execution.",
                trajectory_controller_name_.c_str());
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::core::MoveItErrorCode result;
        {
            std::lock_guard<std::mutex> lock(moveit_mutex_);
            move_group_->setStartStateToCurrentState();
            if (!move_group_->setJointValueTarget(target)) {
                restore_primary_controller();
                response.success = false;
                response.message = "Failed to set MoveIt joint target for home pose.";
                RCLCPP_ERROR(get_logger(), "%s", response.message.c_str());
                return;
            }

            result = move_group_->plan(plan);
            if (result != moveit::core::MoveItErrorCode::SUCCESS) {
                restore_primary_controller();
                response.success = false;
                response.message =
                    "MoveIt planning failed: " + moveit::core::errorCodeToString(result);
                RCLCPP_ERROR(get_logger(), "%s", response.message.c_str());
                return;
            }

            if (!execute_trajectory_) {
                restore_primary_controller();
                response.success = true;
                response.message = "MoveIt planning succeeded (execution disabled).";
                RCLCPP_INFO(get_logger(), "%s", response.message.c_str());
                return;
            }

            result = move_group_->execute(plan);
            if (result != moveit::core::MoveItErrorCode::SUCCESS) {
                restore_primary_controller();
                response.success = false;
                response.message =
                    "MoveIt execution failed: " + moveit::core::errorCodeToString(result);
                RCLCPP_ERROR(get_logger(), "%s", response.message.c_str());
                return;
            }
        }

        restore_primary_controller();
        response.success = true;
        response.message = "MoveIt go_home plan executed successfully.";
        RCLCPP_INFO(get_logger(), "%s", response.message.c_str());
    }

    void handle_go_home(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (backend_ == "legacy") {
            handle_legacy_go_home(*response);
            return;
        }

        if (backend_ != "moveit") {
            response->success = false;
            response->message = "Unsupported motion backend: " + backend_;
            RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
            return;
        }

        handle_moveit_go_home(*response);
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr go_home_service_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr legacy_go_home_client_;

    std::string backend_;
    std::string go_home_service_name_;
    std::string legacy_go_home_service_;
    bool allow_legacy_go_home_fallback_{false};
    std::string planning_group_;
    std::string home_pose_id_;
    double planning_time_sec_{5.0};
    double move_group_wait_sec_{10.0};
    int64_t num_planning_attempts_{3};
    double max_velocity_scaling_{0.2};
    double max_acceleration_scaling_{0.2};
    bool execute_trajectory_{true};
    bool switch_to_trajectory_controller_{false};
    bool reactivate_controller_after_moveit_{false};
    double controller_switch_timeout_sec_{5.0};
    std::string planning_pipeline_id_;
    std::string planner_id_;
    std::string scene_frame_id_;
    std::string controller_manager_switch_service_;
    std::string trajectory_controller_name_;
    std::string primary_controller_name_;

    std::mutex moveit_mutex_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
};

}  // namespace marvin_system

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<marvin_system::MarvinMotionServer>(rclcpp::NodeOptions{});
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    try {
        executor.spin();
    } catch (const std::exception &ex) {
        RCLCPP_FATAL(node->get_logger(), "Motion server exception: %s", ex.what());
    }
    executor.remove_node(node);
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}
