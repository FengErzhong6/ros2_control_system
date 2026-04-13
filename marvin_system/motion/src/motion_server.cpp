#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "marvin_system/workspace_guard.hpp"
#include "marvin_system/srv/get_motion_mode.hpp"
#include "marvin_system/srv/get_motion_status.hpp"
#include "marvin_system/srv/set_motion_mode.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit/utils/moveit_error_code.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace {

constexpr size_t kJointsPerArm = 7;
constexpr size_t kTotalTrackedJoints = kJointsPerArm * 2;
constexpr size_t kTrackedGripperCount = 2;

constexpr char kModeSafeHold[] = "SAFE_HOLD";
constexpr char kModeTeleop[] = "TELEOP";
constexpr char kModeMotion[] = "MOTION";
constexpr char kModeFault[] = "FAULT";

constexpr char kTeleopStateDisarmed[] = "DISARMED";
constexpr char kTeleopStateArmed[] = "ARMED";
constexpr char kTeleopStateEnabled[] = "ENABLED";
constexpr char kTeleopStateUnknown[] = "UNKNOWN";

constexpr char kControllerStateMissing[] = "missing";
constexpr char kControllerStateUnconfigured[] = "unconfigured";
constexpr char kControllerStateInactive[] = "inactive";
constexpr char kControllerStateActive[] = "active";

const std::array<const char *, kJointsPerArm> kLeftJointNames{
    {"Joint1_L", "Joint2_L", "Joint3_L", "Joint4_L", "Joint5_L", "Joint6_L", "Joint7_L"}};
const std::array<const char *, kJointsPerArm> kRightJointNames{
    {"Joint1_R", "Joint2_R", "Joint3_R", "Joint4_R", "Joint5_R", "Joint6_R", "Joint7_R"}};
const std::array<const char *, kTrackedGripperCount> kGripperJointNames{
    {"gripper_L", "gripper_R"}};
const std::array<const char *, kTrackedGripperCount> kGripperAttachFrames{
    {"ee_L", "ee_R"}};
const std::array<const char *, kTrackedGripperCount> kGripperCollisionObjectIds{
    {"gripper_L_collision", "gripper_R_collision"}};
const std::array<int32_t, kTrackedGripperCount> kGripperMarkerIds{{0, 1}};
const std::array<double, kJointsPerArm> kJointLowerLimits{
    {-3.1067, -2.0944, -3.1067, -2.5307, -3.1067, -1.0472, -1.5708}};
const std::array<double, kJointsPerArm> kJointUpperLimits{
    {3.1067, 2.0944, 3.1067, 1.0472, 3.1067, 1.0472, 1.5708}};
const std::array<std::array<const char *, 5>, kTrackedGripperCount> kGripperTouchLinks{{
    {{"Link7_L", "ee_L", "gripper_L_link", "left_finger_L_link", "right_finger_L_link"}},
    {{"Link7_R", "ee_R", "gripper_R_link", "left_finger_R_link", "right_finger_R_link"}},
}};

constexpr char kGripperCollisionMarkerTopic[] = "/gripper_collision_markers";
constexpr double kGripperHeightInterceptM = 0.15;
constexpr double kGripperHeightSlopeM = -0.03;
constexpr double kGripperRadiusInterceptM = 0.0425;
constexpr double kGripperRadiusSlopeM = 0.0575;
constexpr double kGripperCollisionSyncEpsilon = 1.0e-3;
constexpr std::chrono::milliseconds kGripperCollisionSyncPeriod(100);
constexpr std::chrono::seconds kMoveitBootstrapRetryPeriod(1);

template <typename T>
T get_param_or_declare(
    rclcpp::Node *node,
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

std::vector<std::string> get_string_array_param(
    const rclcpp::Node *node,
    const std::string &name)
{
    if (!node->has_parameter(name)) {
        return {};
    }
    const auto value = node->get_parameter(name).as_string_array();
    return std::vector<std::string>(value.begin(), value.end());
}

std::string to_bool_string(bool value)
{
    return value ? "ON" : "OFF";
}

std::string normalize_token(std::string value)
{
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), [](unsigned char ch) {
                    return !std::isspace(ch);
                }));
    value.erase(std::find_if(value.rbegin(), value.rend(), [](unsigned char ch) {
                    return !std::isspace(ch);
                }).base(),
                value.end());
    for (char &ch : value) {
        if (ch == '-' || std::isspace(static_cast<unsigned char>(ch))) {
            ch = '_';
            continue;
        }
        ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    }
    return value;
}

std::string normalize_mode_name(const std::string &raw_mode)
{
    const std::string normalized = normalize_token(raw_mode);
    if (normalized == "SAFE_HOLD" || normalized == "SAFEHOLD") {
        return kModeSafeHold;
    }
    if (normalized == "TELEOP") {
        return kModeTeleop;
    }
    if (normalized == "MOTION") {
        return kModeMotion;
    }
    if (normalized == "FAULT") {
        return kModeFault;
    }
    return "";
}

std::string parse_teleop_state_keyword(const std::string &raw_state)
{
    const auto delimiter = raw_state.find('|');
    const auto token = delimiter == std::string::npos ? raw_state : raw_state.substr(0, delimiter);
    const std::string normalized = normalize_token(token);
    if (normalized.empty()) {
        return kTeleopStateUnknown;
    }
    return normalized;
}

bool teleop_state_is_armed(const std::string &state)
{
    return state == kTeleopStateArmed || state == kTeleopStateEnabled;
}

bool teleop_state_is_enabled(const std::string &state)
{
    return state == kTeleopStateEnabled;
}

double clamp_gripper_percent(double value)
{
    return std::clamp(value, 0.0, 1.0);
}

double gripper_height_m(double percent)
{
    return kGripperHeightInterceptM + kGripperHeightSlopeM * percent;
}

double gripper_radius_m(double percent)
{
    return kGripperRadiusInterceptM + kGripperRadiusSlopeM * percent;
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
        set_mode_service_name_ = get_param_or_declare<std::string>(
            this, "set_mode_service_name", "/marvin_motion/set_mode");
        get_mode_service_name_ = get_param_or_declare<std::string>(
            this, "get_mode_service_name", "/marvin_motion/get_mode");
        get_status_service_name_ = get_param_or_declare<std::string>(
            this, "get_status_service_name", "/marvin_motion/get_status");
        set_enabled_service_name_ = get_param_or_declare<std::string>(
            this, "set_enabled_service_name", "/marvin_motion/set_enabled");

        legacy_go_home_service_ = get_param_or_declare<std::string>(
            this, "legacy_go_home_service", "/tracker_teleop_controller/go_home");
        tracker_set_armed_service_ = get_param_or_declare<std::string>(
            this, "tracker_set_armed_service", "/tracker_teleop_controller/set_armed");
        tracker_set_enabled_service_ = get_param_or_declare<std::string>(
            this, "tracker_set_enabled_service", "/tracker_teleop_controller/set_enabled");
        teleop_state_topic_ = get_param_or_declare<std::string>(
            this, "teleop_state_topic", "/tracker_teleop_controller/teleop_state");

        allow_legacy_go_home_fallback_ = get_param_or_declare<bool>(
            this, "allow_legacy_go_home_fallback", false);
        planning_group_ = get_param_or_declare<std::string>(
            this, "planning_group", "dual_arm");
        home_pose_id_ = get_param_or_declare<std::string>(
            this, "home_pose_id", "home");
        initial_mode_ = normalize_mode_name(get_param_or_declare<std::string>(
            this, "initial_mode", kModeSafeHold));
        if (initial_mode_.empty()) {
            initial_mode_ = kModeSafeHold;
        }
        go_home_return_mode_ = normalize_mode_name(get_param_or_declare<std::string>(
            this, "go_home_return_mode", kModeSafeHold));
        if (go_home_return_mode_.empty()) {
            go_home_return_mode_ = kModeSafeHold;
        }
        planning_time_sec_ = get_param_or_declare<double>(
            this, "planning_time_sec", 5.0);
        move_group_wait_sec_ = get_param_or_declare<double>(
            this, "move_group_wait_sec", 10.0);
        teleop_service_timeout_sec_ = get_param_or_declare<double>(
            this, "teleop_service_timeout_sec", 5.0);
        legacy_go_home_timeout_sec_ = get_param_or_declare<double>(
            this, "legacy_go_home_timeout_sec", 10.0);
        legacy_go_home_settle_timeout_sec_ = get_param_or_declare<double>(
            this, "legacy_go_home_settle_timeout_sec", 20.0);
        legacy_home_tolerance_rad_ = get_param_or_declare<double>(
            this, "legacy_home_tolerance_rad", 0.5 * M_PI / 180.0);
        num_planning_attempts_ = get_param_or_declare<int64_t>(
            this, "num_planning_attempts", 3);
        max_velocity_scaling_ = get_param_or_declare<double>(
            this, "max_velocity_scaling", 0.2);
        max_acceleration_scaling_ = get_param_or_declare<double>(
            this, "max_acceleration_scaling", 0.2);
        execute_trajectory_ = get_param_or_declare<bool>(
            this, "execute_trajectory", true);
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
        controller_manager_list_service_ = get_param_or_declare<std::string>(
            this, "controller_manager_list_service", "/controller_manager/list_controllers");
        trajectory_controller_name_ = get_param_or_declare<std::string>(
            this, "trajectory_controller_name", "dual_arm_trajectory_controller");
        primary_controller_name_ = get_param_or_declare<std::string>(
            this, "primary_controller_name", "tracker_teleop_controller");
        workspace_guard_service_name_ = get_param_or_declare<std::string>(
            this, "workspace_guard_service_name", "");
        use_mock_hardware_ = get_param_or_declare<bool>(this, "use_mock_hardware", false);
        go_home_pose_sequence_ = get_string_array_param(this, "go_home_pose_sequence");
        if (go_home_pose_sequence_.empty()) {
            go_home_pose_sequence_ = {home_pose_id_};
        }
        go_home_pose_sequence_.erase(
            std::remove_if(
                go_home_pose_sequence_.begin(),
                go_home_pose_sequence_.end(),
                [](const std::string &pose_id) { return pose_id.empty(); }),
            go_home_pose_sequence_.end());
        if (go_home_pose_sequence_.empty()) {
            go_home_pose_sequence_.push_back(home_pose_id_);
        }
        recovery_enabled_ = get_param_or_declare<bool>(this, "recovery_enabled", true);
        recovery_command_topic_ = get_param_or_declare<std::string>(
            this, "recovery_command_topic", "");
        recovery_command_joint_names_ = get_string_array_param(
            this, "recovery_command_joint_names");
        if (recovery_command_joint_names_.empty()) {
            for (const auto *name : kLeftJointNames) {
                recovery_command_joint_names_.push_back(name);
            }
            for (const auto *name : kRightJointNames) {
                recovery_command_joint_names_.push_back(name);
            }
        }
        recovery_feedback_timeout_sec_ = get_param_or_declare<double>(
            this, "recovery_feedback_timeout_sec", 2.0);
        recovery_settle_timeout_sec_ = get_param_or_declare<double>(
            this, "recovery_settle_timeout_sec", 8.0);
        recovery_position_tolerance_rad_ = get_param_or_declare<double>(
            this, "recovery_position_tolerance_rad", 0.05);
        recovery_joint2_step_rad_ = get_param_or_declare<double>(
            this, "recovery_joint2_step_rad", 2.0 * M_PI / 180.0);
        recovery_joint4_step_rad_ = get_param_or_declare<double>(
            this, "recovery_joint4_step_rad", 2.0 * M_PI / 180.0);
        recovery_search_layers_ = get_param_or_declare<int64_t>(
            this, "recovery_search_layers", 6);
        recovery_interpolation_samples_ = get_param_or_declare<int64_t>(
            this, "recovery_interpolation_samples", 10);
        recovery_max_iterations_ = get_param_or_declare<int64_t>(
            this, "recovery_max_iterations", 6);
        recovery_min_improvement_m_ = get_param_or_declare<double>(
            this, "recovery_min_improvement_m", 0.002);
        recovery_max_drop_m_ = get_param_or_declare<double>(
            this, "recovery_max_drop_m", 0.0005);
        workspace_z_min_param_ = get_param_or_declare<std::string>(
            this, "workspace_z_min", "");
        workspace_safety_margin_param_ = get_param_or_declare<std::string>(
            this, "workspace_safety_margin", "0.06");
        mount_xyz_l_param_ = get_param_or_declare<std::string>(
            this, "mount_xyz_L", "0 0.037 0.3618964");
        mount_rpy_l_param_ = get_param_or_declare<std::string>(
            this, "mount_rpy_L", "-1.5707963 0 0");
        mount_xyz_r_param_ = get_param_or_declare<std::string>(
            this, "mount_xyz_R", "0 -0.037 0.3618964");
        mount_rpy_r_param_ = get_param_or_declare<std::string>(
            this, "mount_rpy_R", "1.5707963 0 0");
        tool_offset_param_ = get_param_or_declare<std::string>(
            this, "tool_offset", "");
        teleop_services_configured_ = !tracker_set_armed_service_.empty() &&
                                      !tracker_set_enabled_service_.empty();
        teleop_state_feedback_configured_ = !teleop_state_topic_.empty();
        current_mode_ = initial_mode_;

        legacy_go_home_client_ = this->create_client<std_srvs::srv::Trigger>(
            legacy_go_home_service_,
            rclcpp::ServicesQoS(),
            callback_group_);
        if (teleop_services_configured_) {
            tracker_set_armed_client_ = this->create_client<std_srvs::srv::SetBool>(
                tracker_set_armed_service_,
                rclcpp::ServicesQoS(),
                callback_group_);
            tracker_set_enabled_client_ = this->create_client<std_srvs::srv::SetBool>(
                tracker_set_enabled_service_,
                rclcpp::ServicesQoS(),
                callback_group_);
            latest_teleop_state_ = kTeleopStateDisarmed;
        }
        switch_controller_client_ =
            this->create_client<controller_manager_msgs::srv::SwitchController>(
                controller_manager_switch_service_,
                rclcpp::ServicesQoS(),
                callback_group_);
        list_controllers_client_ =
            this->create_client<controller_manager_msgs::srv::ListControllers>(
                controller_manager_list_service_,
                rclcpp::ServicesQoS(),
                callback_group_);
        if (!workspace_guard_service_name_.empty()) {
            workspace_guard_client_ = this->create_client<std_srvs::srv::SetBool>(
                workspace_guard_service_name_,
                rclcpp::ServicesQoS(),
                callback_group_);
        }
        if (!recovery_command_topic_.empty()) {
            recovery_command_publisher_ =
                this->create_publisher<std_msgs::msg::Float64MultiArray>(
                    recovery_command_topic_, rclcpp::SystemDefaultsQoS());
        }
        if (use_mock_hardware_) {
            gripper_collision_marker_publisher_ =
                this->create_publisher<visualization_msgs::msg::MarkerArray>(
                    kGripperCollisionMarkerTopic, rclcpp::QoS(10).reliable());
        }

        if (teleop_state_feedback_configured_) {
            auto teleop_state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            teleop_state_subscription_ = this->create_subscription<std_msgs::msg::String>(
                teleop_state_topic_,
                teleop_state_qos,
                std::bind(&MarvinMotionServer::handle_teleop_state, this, std::placeholders::_1));
        }
        joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            rclcpp::SensorDataQoS(),
            std::bind(&MarvinMotionServer::handle_joint_state, this, std::placeholders::_1));
        gripper_collision_timer_ = this->create_wall_timer(
            kGripperCollisionSyncPeriod,
            std::bind(&MarvinMotionServer::handle_gripper_collision_timer, this),
            callback_group_);

        if (!workspace_z_min_param_.empty()) {
            std::unordered_map<std::string, std::string> workspace_params;
            workspace_params["workspace_z_min"] = workspace_z_min_param_;
            workspace_params["workspace_safety_margin"] = workspace_safety_margin_param_;
            workspace_params["mount_xyz_L"] = mount_xyz_l_param_;
            workspace_params["mount_rpy_L"] = mount_rpy_l_param_;
            workspace_params["mount_xyz_R"] = mount_xyz_r_param_;
            workspace_params["mount_rpy_R"] = mount_rpy_r_param_;
            if (!tool_offset_param_.empty()) {
                workspace_params["tool_offset"] = tool_offset_param_;
            }
            recovery_workspace_guard_configured_ =
                recovery_workspace_guard_.configure(workspace_params, get_logger());
        }

        go_home_service_ = this->create_service<std_srvs::srv::Trigger>(
            go_home_service_name_,
            std::bind(
                &MarvinMotionServer::handle_go_home,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            rclcpp::ServicesQoS(),
            callback_group_);
        set_mode_service_ = this->create_service<srv::SetMotionMode>(
            set_mode_service_name_,
            std::bind(
                &MarvinMotionServer::handle_set_mode,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            rclcpp::ServicesQoS(),
            callback_group_);
        get_mode_service_ = this->create_service<srv::GetMotionMode>(
            get_mode_service_name_,
            std::bind(
                &MarvinMotionServer::handle_get_mode,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            rclcpp::ServicesQoS(),
            callback_group_);
        get_status_service_ = this->create_service<srv::GetMotionStatus>(
            get_status_service_name_,
            std::bind(
                &MarvinMotionServer::handle_get_status,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            rclcpp::ServicesQoS(),
            callback_group_);
        set_enabled_service_ = this->create_service<std_srvs::srv::SetBool>(
            set_enabled_service_name_,
            std::bind(
                &MarvinMotionServer::handle_set_enabled,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            rclcpp::ServicesQoS(),
            callback_group_);

        RCLCPP_INFO(
            get_logger(),
            "Motion layer ready. backend=%s go_home=%s set_mode=%s set_enabled=%s "
            "legacy_fallback=%s return_mode=%s planning_group=%s execute=%s "
            "initial_mode=%s teleop_services=%s primary_controller=%s trajectory_controller=%s "
            "workspace_guard_service=%s go_home_sequence=%zu recovery=%s recovery_topic=%s scene_objects=%zu",
            backend_.c_str(),
            go_home_service_name_.c_str(),
            set_mode_service_name_.c_str(),
            set_enabled_service_name_.c_str(),
            to_bool_string(allow_legacy_go_home_fallback_).c_str(),
            go_home_return_mode_.c_str(),
            planning_group_.c_str(),
            to_bool_string(execute_trajectory_).c_str(),
            initial_mode_.c_str(),
            to_bool_string(teleop_services_configured_).c_str(),
            primary_controller_name_.c_str(),
            trajectory_controller_name_.c_str(),
            workspace_guard_service_name_.c_str(),
            go_home_pose_sequence_.size(),
            to_bool_string(recovery_enabled_ && recovery_workspace_guard_configured_).c_str(),
            recovery_command_topic_.c_str(),
            count_scene_objects());
    }

    ~MarvinMotionServer() override
    {
        shutdown_moveit_interfaces();
    }

    void shutdown_moveit_interfaces()
    {
        std::lock_guard<std::mutex> lock(moveit_mutex_);
        move_group_.reset();
        planning_scene_interface_.reset();
        static_scene_applied_.store(false, std::memory_order_relaxed);
        gripper_collision_objects_active_.fill(false);
        last_synced_gripper_percent_.fill(-1.0);
    }

private:
    struct ControllerStates {
        bool primary_exists{false};
        bool primary_active{false};
        std::string primary_state{kControllerStateUnconfigured};
        bool trajectory_exists{false};
        bool trajectory_active{false};
        std::string trajectory_state{kControllerStateUnconfigured};
    };

    struct MotionStatusSnapshot {
        bool success{false};
        std::string mode{kModeFault};
        std::string teleop_state{kTeleopStateUnknown};
        bool teleop_armed{false};
        bool teleop_enabled{false};
        std::string primary_controller_state{kControllerStateUnconfigured};
        std::string trajectory_controller_state{kControllerStateUnconfigured};
        bool motion_busy{false};
        bool controller_interlock_ok{true};
        std::string message;
    };

    struct RecoveryCandidate {
        std::array<std::array<double, kJointsPerArm>, 2> joints_deg{};
        marvin_system::WorkspaceGuard::Evaluation evaluation{};
        double movement_cost{0.0};
        bool valid{false};
    };

    std::vector<double> get_named_pose_values(const std::string &pose_id, const std::string &side) const
    {
        return get_double_array_param(this, "named_poses." + pose_id + "." + side);
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

    void handle_teleop_state(const std_msgs::msg::String &msg)
    {
        const std::string keyword = parse_teleop_state_keyword(msg.data);
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_teleop_state_ = keyword;
    }

    void handle_joint_state(const sensor_msgs::msg::JointState &msg)
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
            const auto &name = msg.name[i];
            recovery_aux_joint_positions_rad_[name] = msg.position[i];
            for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
                if (name == kLeftJointNames[joint]) {
                    joint_positions_rad_[joint] = msg.position[i];
                    joint_position_valid_[joint] = true;
                    break;
                }
                if (name == kRightJointNames[joint]) {
                    const size_t index = kJointsPerArm + joint;
                    joint_positions_rad_[index] = msg.position[i];
                    joint_position_valid_[index] = true;
                    break;
                }
            }
        }
    }

    bool wait_for_joint_feedback(std::string &error_message) const
    {
        const auto deadline =
            std::chrono::steady_clock::now() +
            std::chrono::duration<double>(std::max(0.1, recovery_feedback_timeout_sec_));
        while (std::chrono::steady_clock::now() < deadline) {
            {
                std::lock_guard<std::mutex> lock(joint_state_mutex_);
                if (std::all_of(
                        joint_position_valid_.begin(),
                        joint_position_valid_.end(),
                        [](bool valid) { return valid; })) {
                    return true;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        error_message = "Timed out waiting for /joint_states feedback.";
        return false;
    }

    bool current_joint_positions_deg(
        std::array<std::array<double, kJointsPerArm>, 2> &joint_positions_deg,
        std::string &error_message) const
    {
        if (!wait_for_joint_feedback(error_message)) {
            return false;
        }

        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
            joint_positions_deg[0][joint] = joint_positions_rad_[joint] * 180.0 / M_PI;
            joint_positions_deg[1][joint] =
                joint_positions_rad_[kJointsPerArm + joint] * 180.0 / M_PI;
        }
        return true;
    }

    static double movement_cost(
        const std::array<std::array<double, kJointsPerArm>, 2> &from_deg,
        const std::array<std::array<double, kJointsPerArm>, 2> &to_deg)
    {
        double cost = 0.0;
        for (size_t arm = 0; arm < 2; ++arm) {
            for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
                cost += std::abs(to_deg[arm][joint] - from_deg[arm][joint]);
            }
        }
        return cost;
    }

    bool candidate_path_is_safe_enough(
        const std::array<std::array<double, kJointsPerArm>, 2> &current_deg,
        const std::array<std::array<double, kJointsPerArm>, 2> &candidate_deg,
        double current_min_z,
        marvin_system::WorkspaceGuard::Evaluation &candidate_evaluation) const
    {
        double previous_min_z = current_min_z;
        const auto samples = std::max<int64_t>(1, recovery_interpolation_samples_);
        for (int64_t sample = 1; sample <= samples; ++sample) {
            const double alpha = static_cast<double>(sample) / static_cast<double>(samples);
            std::array<std::array<double, kJointsPerArm>, 2> interpolated{};
            for (size_t arm = 0; arm < 2; ++arm) {
                for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
                    interpolated[arm][joint] =
                        current_deg[arm][joint] +
                        (candidate_deg[arm][joint] - current_deg[arm][joint]) * alpha;
                }
            }

            const auto evaluation = recovery_workspace_guard_.evaluate(interpolated);
            if (evaluation.min_z < current_min_z - recovery_max_drop_m_) {
                return false;
            }
            if (evaluation.min_z < previous_min_z - recovery_max_drop_m_) {
                return false;
            }
            previous_min_z = evaluation.min_z;
            if (sample == samples) {
                candidate_evaluation = evaluation;
            }
        }
        return true;
    }

    RecoveryCandidate find_best_recovery_candidate(
        const std::array<std::array<double, kJointsPerArm>, 2> &current_deg,
        bool force_escape_safe_state) const
    {
        RecoveryCandidate best;
        const auto current_evaluation = recovery_workspace_guard_.evaluate(current_deg);
        if (current_evaluation.safe && !force_escape_safe_state) {
            best.valid = true;
            best.joints_deg = current_deg;
            best.evaluation = current_evaluation;
            return best;
        }

        std::array<bool, 2> arm_requires_recovery{};
        bool any_arm_requires_recovery = false;
        for (size_t arm = 0; arm < 2; ++arm) {
            const auto arm_evaluation =
                recovery_workspace_guard_.evaluate_arm(arm, current_deg[arm].data());
            arm_requires_recovery[arm] = !arm_evaluation.safe;
            any_arm_requires_recovery |= arm_requires_recovery[arm];
        }
        if (!any_arm_requires_recovery && force_escape_safe_state) {
            size_t preferred_arm = 0;
            auto preferred_eval =
                recovery_workspace_guard_.evaluate_arm(preferred_arm, current_deg[preferred_arm].data());
            for (size_t arm = 1; arm < 2; ++arm) {
                const auto candidate_eval =
                    recovery_workspace_guard_.evaluate_arm(arm, current_deg[arm].data());
                if (candidate_eval.min_z < preferred_eval.min_z) {
                    preferred_arm = arm;
                    preferred_eval = candidate_eval;
                }
            }
            arm_requires_recovery[preferred_arm] = true;
            any_arm_requires_recovery = true;
        }
        if (!any_arm_requires_recovery) {
            return best;
        }

        for (size_t arm = 0; arm < 2; ++arm) {
            if (!arm_requires_recovery[arm]) {
                continue;
            }
            const auto current_arm_evaluation =
                recovery_workspace_guard_.evaluate_arm(arm, current_deg[arm].data());
            for (int64_t layer2 = 0; layer2 <= recovery_search_layers_; ++layer2) {
                for (int64_t layer4 = 0; layer4 <= recovery_search_layers_; ++layer4) {
                    if (layer2 == 0 && layer4 == 0) {
                        continue;
                    }
                    for (const int sign2 : {-1, 1}) {
                        for (const int sign4 : {-1, 1}) {
                            std::array<std::array<double, kJointsPerArm>, 2> candidate = current_deg;
                            candidate[arm][1] = std::clamp(
                                current_deg[arm][1] +
                                    static_cast<double>(sign2) * static_cast<double>(layer2) *
                                        recovery_joint2_step_rad_ * 180.0 / M_PI,
                                kJointLowerLimits[1] * 180.0 / M_PI,
                                kJointUpperLimits[1] * 180.0 / M_PI);
                            candidate[arm][3] = std::clamp(
                                current_deg[arm][3] +
                                    static_cast<double>(sign4) * static_cast<double>(layer4) *
                                        recovery_joint4_step_rad_ * 180.0 / M_PI,
                                kJointLowerLimits[3] * 180.0 / M_PI,
                                kJointUpperLimits[3] * 180.0 / M_PI);
                            if (candidate == current_deg) {
                                continue;
                            }

                            marvin_system::WorkspaceGuard::Evaluation candidate_evaluation;
                            if (!candidate_path_is_safe_enough(
                                    current_deg,
                                    candidate,
                                    current_evaluation.min_z,
                                    candidate_evaluation)) {
                                continue;
                            }
                            const auto candidate_arm_evaluation =
                                recovery_workspace_guard_.evaluate_arm(arm, candidate[arm].data());
                            if (candidate_arm_evaluation.min_z <
                                current_arm_evaluation.min_z + recovery_min_improvement_m_) {
                                continue;
                            }

                            const double candidate_cost = movement_cost(current_deg, candidate);
                            if (!best.valid ||
                                candidate_arm_evaluation.min_z > best.evaluation.min_z +
                                                            recovery_min_improvement_m_ / 10.0 ||
                                (std::abs(candidate_arm_evaluation.min_z - best.evaluation.min_z) <=
                                     recovery_min_improvement_m_ / 10.0 &&
                                 candidate_cost < best.movement_cost)) {
                                best.valid = true;
                                best.joints_deg = candidate;
                                best.evaluation = candidate_arm_evaluation;
                                best.movement_cost = candidate_cost;
                            }
                        }
                    }
                }
            }
        }
        return best;
    }

    void publish_recovery_command(
        const std::array<std::array<double, kJointsPerArm>, 2> &joint_positions_deg)
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data.reserve(recovery_command_joint_names_.size());
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        for (const auto &joint_name : recovery_command_joint_names_) {
            bool handled = false;
            for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
                if (joint_name == kLeftJointNames[joint]) {
                    msg.data.push_back(joint_positions_deg[0][joint] * M_PI / 180.0);
                    handled = true;
                    break;
                }
                if (joint_name == kRightJointNames[joint]) {
                    msg.data.push_back(joint_positions_deg[1][joint] * M_PI / 180.0);
                    handled = true;
                    break;
                }
            }
            if (handled) {
                continue;
            }

            const auto it = recovery_aux_joint_positions_rad_.find(joint_name);
            msg.data.push_back(
                it == recovery_aux_joint_positions_rad_.end() ? 0.0 : it->second);
        }
        recovery_command_publisher_->publish(msg);
    }

    bool build_primary_command_seed_deg(
        std::array<std::array<double, kJointsPerArm>, 2> &joint_positions_deg,
        std::string &error_message) const
    {
        std::string feedback_error;
        if (current_joint_positions_deg(joint_positions_deg, feedback_error)) {
            return true;
        }

        const std::string fallback_pose_id =
            go_home_pose_sequence_.empty() ? home_pose_id_ : go_home_pose_sequence_.back();
        std::map<std::string, double> target;
        if (!build_named_target(fallback_pose_id, target, error_message)) {
            if (!feedback_error.empty()) {
                error_message =
                    "Failed to build primary command seed from /joint_states (" + feedback_error +
                    ") and fallback pose '" + fallback_pose_id + "' (" + error_message + ").";
            } else {
                error_message =
                    "Failed to build primary command seed from fallback pose '" +
                    fallback_pose_id + "': " + error_message;
            }
            return false;
        }

        for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
            joint_positions_deg[0][joint] = target.at(kLeftJointNames[joint]) * 180.0 / M_PI;
            joint_positions_deg[1][joint] = target.at(kRightJointNames[joint]) * 180.0 / M_PI;
        }

        RCLCPP_WARN(
            get_logger(),
            "Primary command seed fell back to named pose '%s' because /joint_states feedback "
            "was unavailable (%s).",
            fallback_pose_id.c_str(),
            feedback_error.c_str());
        return true;
    }

    bool reseed_primary_command_reference(const char *phase, std::string &error_message)
    {
        if (!recovery_command_publisher_) {
            return true;
        }

        std::array<std::array<double, kJointsPerArm>, 2> seed_deg{};
        if (!build_primary_command_seed_deg(seed_deg, error_message)) {
            return false;
        }

        // Publish more than once so the primary controller's command buffer is refreshed
        // both before and immediately after a controller-mode handoff.
        publish_recovery_command(seed_deg);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        publish_recovery_command(seed_deg);
        RCLCPP_INFO(
            get_logger(),
            "Reseeded primary controller reference from joint feedback (%s).",
            phase);
        return true;
    }

    bool wait_until_recovery_target_reached(
        const std::array<std::array<double, kJointsPerArm>, 2> &target_deg,
        std::string &error_message) const
    {
        const auto deadline =
            std::chrono::steady_clock::now() +
            std::chrono::duration<double>(std::max(0.1, recovery_settle_timeout_sec_));
        while (std::chrono::steady_clock::now() < deadline) {
            std::array<std::array<double, kJointsPerArm>, 2> current_deg{};
            if (!current_joint_positions_deg(current_deg, error_message)) {
                return false;
            }

            bool reached = true;
            for (size_t arm = 0; arm < 2 && reached; ++arm) {
                for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
                    const double error =
                        std::abs(target_deg[arm][joint] - current_deg[arm][joint]) * M_PI / 180.0;
                    if (error > recovery_position_tolerance_rad_) {
                        reached = false;
                        break;
                    }
                }
            }
            if (reached) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        error_message = "Recovery command did not converge before timeout.";
        return false;
    }

    bool execute_recovery(std::string &error_message, bool force_escape_safe_state = false)
    {
        if (!recovery_enabled_) {
            error_message = "Recovery is disabled.";
            return false;
        }
        if (!recovery_workspace_guard_configured_) {
            error_message = "Recovery workspace evaluator is not configured.";
            return false;
        }
        if (!recovery_command_publisher_) {
            error_message = "Recovery command publisher is not configured.";
            return false;
        }
        if (!transition_to_mode(kModeSafeHold, error_message)) {
            return false;
        }

        for (int64_t iteration = 0; iteration < std::max<int64_t>(1, recovery_max_iterations_);
             ++iteration) {
            std::array<std::array<double, kJointsPerArm>, 2> current_deg{};
            if (!current_joint_positions_deg(current_deg, error_message)) {
                return false;
            }

            const auto current_evaluation = recovery_workspace_guard_.evaluate(current_deg);
            if (current_evaluation.safe && !force_escape_safe_state) {
                RCLCPP_INFO(
                    get_logger(),
                    "Recovery finished before iteration %ld (min_z=%.4f).",
                    iteration,
                    current_evaluation.min_z);
                return true;
            }

            const auto candidate = find_best_recovery_candidate(
                current_deg, force_escape_safe_state);
            if (!candidate.valid) {
                error_message =
                    "Recovery could not find a safer Joint2/Joint4 candidate.";
                return false;
            }

            RCLCPP_WARN(
                get_logger(),
                "Recovery iteration %ld: min_z %.4f -> %.4f by adjusting Joint2/Joint4 on arm %zu only.",
                iteration + 1,
                current_evaluation.min_z,
                candidate.evaluation.min_z,
                candidate.evaluation.arm_index);

            publish_recovery_command(candidate.joints_deg);
            if (!wait_until_recovery_target_reached(candidate.joints_deg, error_message)) {
                return false;
            }
            force_escape_safe_state = false;
        }

        std::array<std::array<double, kJointsPerArm>, 2> final_deg{};
        if (!current_joint_positions_deg(final_deg, error_message)) {
            return false;
        }
        const auto final_evaluation = recovery_workspace_guard_.evaluate(final_deg);
        if (final_evaluation.safe) {
            return true;
        }

        error_message = "Recovery exhausted all iterations without restoring a safe z-floor state.";
        return false;
    }

    std::string cached_mode() const
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return current_mode_;
    }

    bool busy() const
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return motion_busy_;
    }

    void set_busy(bool value)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        motion_busy_ = value;
    }

    void update_cached_mode_and_state(
        const std::string &mode,
        const std::string &teleop_state)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_mode_ = mode;
        latest_teleop_state_ = teleop_state;
    }

    std::string infer_mode_from_snapshot(
        const ControllerStates &controllers,
        const std::string &teleop_state,
        bool motion_busy,
        const std::string &fallback_mode) const
    {
        if (motion_busy) {
            return kModeMotion;
        }
        if (controllers.primary_active && controllers.trajectory_active) {
            return kModeFault;
        }
        if (controllers.trajectory_active && !controllers.primary_active) {
            return kModeMotion;
        }
        if (controllers.primary_active) {
            if (!teleop_services_configured_) {
                return fallback_mode;
            }
            if (teleop_state == kTeleopStateDisarmed) {
                return kModeSafeHold;
            }
            if (teleop_state == kTeleopStateArmed || teleop_state == kTeleopStateEnabled) {
                return kModeTeleop;
            }
        }
        return fallback_mode;
    }

    bool wait_for_client_service(
        const std::string &service_name,
        const rclcpp::ClientBase::SharedPtr &client,
        double timeout_sec,
        std::string &error_message) const
    {
        if (!client) {
            error_message = service_name.empty()
                                ? "Required service client is not configured."
                                : "Required service client is not configured: " + service_name;
            return false;
        }
        if (client->service_is_ready()) {
            return true;
        }
        if (client->wait_for_service(std::chrono::duration<double>(timeout_sec))) {
            return true;
        }
        error_message = "Service unavailable: " + service_name;
        return false;
    }

    bool call_tracker_set_bool(
        const rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr &client,
        const std::string &service_name,
        bool value,
        std::string &error_message)
    {
        if (!wait_for_client_service(service_name, client, teleop_service_timeout_sec_, error_message)) {
            return false;
        }

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = value;
        auto future = client->async_send_request(request);
        const auto status = future.wait_for(std::chrono::duration<double>(teleop_service_timeout_sec_));
        if (status != std::future_status::ready) {
            error_message = "Service timed out: " + service_name;
            return false;
        }

        const auto response = future.get();
        if (!response->success) {
            error_message = service_name + " rejected request: " + response->message;
            return false;
        }
        return true;
    }

    bool call_teleop_set_armed(bool value, std::string &error_message)
    {
        if (!teleop_services_configured_) {
            return true;
        }
        if (!call_tracker_set_bool(
                tracker_set_armed_client_,
                tracker_set_armed_service_,
                value,
                error_message)) {
            return false;
        }

        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_teleop_state_ = value ? kTeleopStateArmed : kTeleopStateDisarmed;
        return true;
    }

    bool call_teleop_set_enabled(bool value, std::string &error_message)
    {
        if (!teleop_services_configured_) {
            return true;
        }
        if (!call_tracker_set_bool(
                tracker_set_enabled_client_,
                tracker_set_enabled_service_,
                value,
                error_message)) {
            return false;
        }

        std::lock_guard<std::mutex> lock(state_mutex_);
        if (value) {
            latest_teleop_state_ = kTeleopStateEnabled;
        } else if (latest_teleop_state_ == kTeleopStateEnabled) {
            latest_teleop_state_ = kTeleopStateArmed;
        }
        return true;
    }

    bool call_workspace_guard_set_enabled(bool value, std::string &error_message)
    {
        if (workspace_guard_service_name_.empty()) {
            return true;
        }
        if (!workspace_guard_client_) {
            error_message =
                "Workspace guard service client is not configured: " + workspace_guard_service_name_;
            return false;
        }
        return call_tracker_set_bool(
            workspace_guard_client_,
            workspace_guard_service_name_,
            value,
            error_message);
    }

    bool fetch_controller_states(
        ControllerStates &states,
        std::string &error_message)
    {
        if (!wait_for_client_service(
                controller_manager_list_service_,
                list_controllers_client_,
                controller_switch_timeout_sec_,
                error_message)) {
            return false;
        }

        auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
        auto future = list_controllers_client_->async_send_request(request);
        const auto status = future.wait_for(std::chrono::duration<double>(controller_switch_timeout_sec_));
        if (status != std::future_status::ready) {
            error_message = "Timed out waiting for controller list response.";
            return false;
        }

        const auto response = future.get();
        if (!response) {
            error_message = "Controller list request returned no response.";
            return false;
        }

        states = ControllerStates{};
        for (const auto &controller : response->controller) {
            if (!primary_controller_name_.empty() && controller.name == primary_controller_name_) {
                states.primary_exists = true;
                states.primary_state = controller.state;
                states.primary_active = controller.state == kControllerStateActive;
            }
            if (!trajectory_controller_name_.empty() &&
                controller.name == trajectory_controller_name_) {
                states.trajectory_exists = true;
                states.trajectory_state = controller.state;
                states.trajectory_active = controller.state == kControllerStateActive;
            }
        }

        if (!primary_controller_name_.empty() && !states.primary_exists) {
            states.primary_state = kControllerStateMissing;
        }
        if (!trajectory_controller_name_.empty() && !states.trajectory_exists) {
            states.trajectory_state = kControllerStateMissing;
        }
        return true;
    }

    bool switch_controllers(
        const std::vector<std::string> &activate,
        const std::vector<std::string> &deactivate,
        std::string &error_message)
    {
        if (activate.empty() && deactivate.empty()) {
            return true;
        }
        if (!wait_for_client_service(
                controller_manager_switch_service_,
                switch_controller_client_,
                controller_switch_timeout_sec_,
                error_message)) {
            return false;
        }

        auto request =
            std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->activate_controllers = activate;
        request->deactivate_controllers = deactivate;
        request->strictness =
            controller_manager_msgs::srv::SwitchController::Request::STRICT;
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
            error_message = "Controller switch failed: " + result->message;
            return false;
        }
        return true;
    }

    bool ensure_primary_controller_active(std::string &error_message)
    {
        if (primary_controller_name_.empty()) {
            error_message = "Primary controller name is not configured.";
            return false;
        }

        ControllerStates states;
        if (!fetch_controller_states(states, error_message)) {
            return false;
        }
        if (!states.primary_exists) {
            error_message = "Primary controller is not loaded: " + primary_controller_name_;
            return false;
        }
        if (states.primary_active && !states.trajectory_active) {
            return true;
        }

        std::vector<std::string> activate;
        std::vector<std::string> deactivate;
        if (!states.primary_active) {
            activate.push_back(primary_controller_name_);
        }
        if (states.trajectory_active) {
            deactivate.push_back(trajectory_controller_name_);
        }
        if (!switch_controllers(activate, deactivate, error_message)) {
            return false;
        }

        if (!fetch_controller_states(states, error_message)) {
            return false;
        }
        if (!states.primary_active || states.trajectory_active) {
            error_message =
                "Failed to activate primary controller and deactivate trajectory controller.";
            return false;
        }
        return true;
    }

    bool ensure_trajectory_controller_active(std::string &error_message)
    {
        if (trajectory_controller_name_.empty()) {
            error_message = "Trajectory controller name is not configured.";
            return false;
        }

        ControllerStates states;
        if (!fetch_controller_states(states, error_message)) {
            return false;
        }
        if (!states.trajectory_exists) {
            error_message =
                "Trajectory controller is not loaded: " + trajectory_controller_name_;
            return false;
        }
        if (states.trajectory_active && !states.primary_active) {
            return true;
        }

        std::vector<std::string> activate;
        std::vector<std::string> deactivate;
        if (!states.trajectory_active) {
            activate.push_back(trajectory_controller_name_);
        }
        if (states.primary_active) {
            deactivate.push_back(primary_controller_name_);
        }
        if (!switch_controllers(activate, deactivate, error_message)) {
            return false;
        }

        if (!fetch_controller_states(states, error_message)) {
            return false;
        }
        if (!states.trajectory_active || states.primary_active) {
            error_message =
                "Failed to activate trajectory controller and deactivate primary controller.";
            return false;
        }
        return true;
    }

    bool transition_to_mode(const std::string &target_mode, std::string &error_message)
    {
        const std::string normalized_mode = normalize_mode_name(target_mode);
        if (normalized_mode.empty()) {
            error_message = "Unsupported motion mode: " + target_mode;
            return false;
        }

        if (normalized_mode == kModeSafeHold) {
            if (!ensure_primary_controller_active(error_message)) {
                return false;
            }
            if (!call_teleop_set_enabled(false, error_message)) {
                return false;
            }
            if (!call_teleop_set_armed(false, error_message)) {
                return false;
            }
            update_cached_mode_and_state(
                kModeSafeHold,
                teleop_services_configured_ ? kTeleopStateDisarmed : kTeleopStateUnknown);
            return true;
        }

        if (normalized_mode == kModeTeleop) {
            if (!ensure_primary_controller_active(error_message)) {
                return false;
            }
            if (!call_teleop_set_enabled(false, error_message)) {
                return false;
            }
            if (!call_teleop_set_armed(true, error_message)) {
                return false;
            }
            update_cached_mode_and_state(
                kModeTeleop,
                teleop_services_configured_ ? kTeleopStateArmed : kTeleopStateUnknown);
            return true;
        }

        if (normalized_mode == kModeMotion) {
            ControllerStates states;
            if (!fetch_controller_states(states, error_message)) {
                return false;
            }
            if (states.primary_active) {
                if (!call_teleop_set_enabled(false, error_message)) {
                    return false;
                }
                if (!call_teleop_set_armed(false, error_message)) {
                    return false;
                }
            }
            if (!ensure_trajectory_controller_active(error_message)) {
                return false;
            }
            update_cached_mode_and_state(
                kModeMotion,
                teleop_services_configured_ ? kTeleopStateDisarmed : kTeleopStateUnknown);
            return true;
        }

        error_message = "FAULT mode cannot be requested directly.";
        return false;
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
        std::lock_guard<std::mutex> lock(moveit_mutex_);
        if (!planning_scene_interface_) {
            error_message = "Planning scene interface is not initialized.";
            return false;
        }

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

    std::optional<double> get_gripper_percent(const char *joint_name) const
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        const auto it = recovery_aux_joint_positions_rad_.find(joint_name);
        if (it == recovery_aux_joint_positions_rad_.end()) {
            return std::nullopt;
        }
        if (!std::isfinite(it->second)) {
            return std::nullopt;
        }
        return clamp_gripper_percent(it->second);
    }

    geometry_msgs::msg::Pose make_gripper_collision_pose(double height_m) const
    {
        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.z = 0.5 * height_m;
        return pose;
    }

    moveit_msgs::msg::AttachedCollisionObject make_gripper_collision_object(
        size_t index,
        double percent) const
    {
        const double height_m = gripper_height_m(percent);
        const double radius_m = gripper_radius_m(percent);

        moveit_msgs::msg::AttachedCollisionObject attached;
        attached.link_name = kGripperAttachFrames[index];
        attached.object.id = kGripperCollisionObjectIds[index];
        attached.object.header.frame_id = kGripperAttachFrames[index];
        attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        primitive.dimensions = {height_m, radius_m};

        attached.object.primitives.push_back(primitive);
        attached.object.primitive_poses.push_back(make_gripper_collision_pose(height_m));
        for (const auto *touch_link : kGripperTouchLinks[index]) {
            attached.touch_links.emplace_back(touch_link);
        }
        return attached;
    }

    void maybe_initialize_moveit_scene()
    {
        if (backend_ != "moveit") {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(moveit_mutex_);
            if (move_group_ &&
                planning_scene_interface_ &&
                static_scene_applied_.load(std::memory_order_relaxed)) {
                return;
            }
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - last_moveit_bootstrap_attempt_ < kMoveitBootstrapRetryPeriod) {
            return;
        }
        last_moveit_bootstrap_attempt_ = now;

        std::string error_message;
        if (!ensure_moveit_interfaces(error_message)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Dynamic gripper collision bootstrap skipped: %s",
                error_message.c_str());
            return;
        }
        if (!static_scene_applied_.load(std::memory_order_relaxed) &&
            !apply_scene(error_message)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Failed to apply static planning scene before gripper collision sync: %s",
                error_message.c_str());
            return;
        }
        static_scene_applied_.store(true, std::memory_order_relaxed);
    }

    void sync_gripper_collision_objects()
    {
        if (backend_ != "moveit") {
            return;
        }

        std::lock_guard<std::mutex> lock(moveit_mutex_);
        if (!planning_scene_interface_) {
            return;
        }

        for (size_t index = 0; index < kTrackedGripperCount; ++index) {
            const auto percent = get_gripper_percent(kGripperJointNames[index]);
            if (!percent.has_value()) {
                continue;
            }

            if (gripper_collision_objects_active_[index] &&
                std::abs(*percent - last_synced_gripper_percent_[index]) <
                    kGripperCollisionSyncEpsilon) {
                continue;
            }

            if (!planning_scene_interface_->applyAttachedCollisionObject(
                    make_gripper_collision_object(index, *percent))) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    5000,
                    "Failed to sync attached gripper collision object '%s'.",
                    kGripperCollisionObjectIds[index]);
                continue;
            }

            gripper_collision_objects_active_[index] = true;
            last_synced_gripper_percent_[index] = *percent;
        }
    }

    visualization_msgs::msg::Marker make_gripper_collision_marker(
        size_t index,
        double percent) const
    {
        const double height_m = gripper_height_m(percent);
        const double radius_m = gripper_radius_m(percent);

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = kGripperAttachFrames[index];
        marker.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        marker.ns = "gripper_collision";
        marker.id = kGripperMarkerIds[index];
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.frame_locked = true;
        marker.pose = make_gripper_collision_pose(height_m);
        marker.scale.x = radius_m * 2.0;
        marker.scale.y = radius_m * 2.0;
        marker.scale.z = height_m;
        marker.color.a = 0.28f;
        if (index == 0) {
            marker.color.r = 0.10f;
            marker.color.g = 0.80f;
            marker.color.b = 0.35f;
        } else {
            marker.color.r = 0.95f;
            marker.color.g = 0.45f;
            marker.color.b = 0.15f;
        }
        return marker;
    }

    void publish_gripper_collision_markers()
    {
        if (!use_mock_hardware_ || !gripper_collision_marker_publisher_) {
            return;
        }

        visualization_msgs::msg::MarkerArray marker_array;
        for (size_t index = 0; index < kTrackedGripperCount; ++index) {
            const auto percent = get_gripper_percent(kGripperJointNames[index]);
            if (percent.has_value()) {
                marker_array.markers.push_back(
                    make_gripper_collision_marker(index, *percent));
                gripper_collision_markers_active_[index] = true;
            }
        }

        if (!marker_array.markers.empty()) {
            gripper_collision_marker_publisher_->publish(marker_array);
        }
    }

    void handle_gripper_collision_timer()
    {
        publish_gripper_collision_markers();
        maybe_initialize_moveit_scene();
        sync_gripper_collision_objects();
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

    bool build_named_target(
        const std::string &pose_id,
        std::map<std::string, double> &target,
        std::string &error_message) const
    {
        const auto left = get_named_pose_values(pose_id, "left");
        const auto right = get_named_pose_values(pose_id, "right");
        if (left.size() != kJointsPerArm || right.size() != kJointsPerArm) {
            error_message = "Named pose '" + pose_id + "' is incomplete.";
            return false;
        }

        for (size_t i = 0; i < kJointsPerArm; ++i) {
            target[kLeftJointNames[i]] = left[i];
            target[kRightJointNames[i]] = right[i];
        }
        return true;
    }

    bool named_target_reached(
        const std::map<std::string, double> &target,
        double tolerance_rad,
        std::string &largest_error_joint,
        double &largest_error_rad) const
    {
        largest_error_joint.clear();
        largest_error_rad = 0.0;

        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
            const auto left_it = target.find(kLeftJointNames[joint]);
            if (left_it != target.end()) {
                if (!joint_position_valid_[joint]) {
                    largest_error_joint = kLeftJointNames[joint];
                    largest_error_rad = std::numeric_limits<double>::infinity();
                    return false;
                }
                const double error =
                    std::abs(joint_positions_rad_[joint] - left_it->second);
                if (error > largest_error_rad) {
                    largest_error_rad = error;
                    largest_error_joint = kLeftJointNames[joint];
                }
                if (error > tolerance_rad) {
                    return false;
                }
            }

            const auto right_it = target.find(kRightJointNames[joint]);
            if (right_it != target.end()) {
                const size_t index = kJointsPerArm + joint;
                if (!joint_position_valid_[index]) {
                    largest_error_joint = kRightJointNames[joint];
                    largest_error_rad = std::numeric_limits<double>::infinity();
                    return false;
                }
                const double error =
                    std::abs(joint_positions_rad_[index] - right_it->second);
                if (error > largest_error_rad) {
                    largest_error_rad = error;
                    largest_error_joint = kRightJointNames[joint];
                }
                if (error > tolerance_rad) {
                    return false;
                }
            }
        }

        return true;
    }

    bool wait_for_named_pose_reached(
        const std::string &pose_id,
        double timeout_sec,
        double tolerance_rad,
        std::string &error_message)
    {
        std::map<std::string, double> target;
        if (!build_named_target(pose_id, target, error_message)) {
            return false;
        }
        if (!wait_for_joint_feedback(error_message)) {
            return false;
        }

        const auto deadline =
            std::chrono::steady_clock::now() +
            std::chrono::duration<double>(std::max(0.5, timeout_sec));
        std::string largest_error_joint;
        double largest_error_rad = 0.0;
        while (std::chrono::steady_clock::now() < deadline) {
            if (named_target_reached(
                    target,
                    tolerance_rad,
                    largest_error_joint,
                    largest_error_rad)) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        error_message =
            "Legacy go_home did not reach pose '" + pose_id + "' within " +
            std::to_string(timeout_sec) + "s";
        if (!largest_error_joint.empty() && std::isfinite(largest_error_rad)) {
            error_message +=
                " (largest error: " + largest_error_joint + "=" +
                std::to_string(largest_error_rad) + " rad)";
        }
        error_message += ".";
        return false;
    }

    bool plan_and_execute_named_target(
        const std::string &pose_id,
        std::string &error_message)
    {
        std::map<std::string, double> target;
        if (!build_named_target(pose_id, target, error_message)) {
            return false;
        }

        sync_gripper_collision_objects();

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::core::MoveItErrorCode result;
        {
            std::lock_guard<std::mutex> lock(moveit_mutex_);
            move_group_->setStartStateToCurrentState();
            if (!move_group_->setJointValueTarget(target)) {
                error_message = "Failed to set MoveIt joint target for pose '" + pose_id + "'.";
                return false;
            }

            result = move_group_->plan(plan);
            if (result != moveit::core::MoveItErrorCode::SUCCESS) {
                error_message =
                    "MoveIt planning failed for pose '" + pose_id + "': " +
                    moveit::core::errorCodeToString(result);
                return false;
            }

            if (!execute_trajectory_) {
                return true;
            }

            result = move_group_->execute(plan);
            if (result != moveit::core::MoveItErrorCode::SUCCESS) {
                error_message =
                    "MoveIt execution failed for pose '" + pose_id + "': " +
                    moveit::core::errorCodeToString(result);
                return false;
            }
        }

        RCLCPP_INFO(get_logger(), "MoveIt pose '%s' executed successfully.", pose_id.c_str());
        return true;
    }

    bool handle_legacy_go_home(std::string &error_message)
    {
        if (!allow_legacy_go_home_fallback_) {
            error_message =
                "Legacy go_home fallback is disabled. Select the MoveIt backend for safe planning.";
            return false;
        }

        if (!wait_for_client_service(
                legacy_go_home_service_,
                legacy_go_home_client_,
                2.0,
                error_message)) {
            return false;
        }

        RCLCPP_WARN(
            get_logger(),
            "Using legacy go_home fallback via %s. This path is transitional and not MoveIt-backed.",
            legacy_go_home_service_.c_str());
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = legacy_go_home_client_->async_send_request(request);
        const auto status = future.wait_for(
            std::chrono::duration<double>(legacy_go_home_timeout_sec_));
        if (status != std::future_status::ready) {
            error_message = "Legacy go_home fallback timed out.";
            return false;
        }

        const auto result = future.get();
        if (!result->success) {
            error_message = result->message;
            return false;
        }
        return true;
    }

    bool execute_moveit_go_home(std::string &error_message)
    {
        if (!ensure_moveit_interfaces(error_message)) {
            return false;
        }
        if (!apply_scene(error_message)) {
            return false;
        }
        for (const auto &pose_id : go_home_pose_sequence_) {
            if (!plan_and_execute_named_target(pose_id, error_message)) {
                return false;
            }
        }
        return true;
    }

    bool collect_status(MotionStatusSnapshot &status)
    {
        ControllerStates controllers;
        std::string error_message;
        const bool controllers_ok = fetch_controller_states(controllers, error_message);

        std::string teleop_state;
        std::string fallback_mode;
        bool motion_busy = false;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            teleop_state = latest_teleop_state_;
            fallback_mode = current_mode_;
            motion_busy = motion_busy_;
        }
        if (teleop_state.empty()) {
            teleop_state = kTeleopStateUnknown;
        }

        status.teleop_state = teleop_state;
        status.teleop_armed = teleop_state_is_armed(teleop_state);
        status.teleop_enabled = teleop_state_is_enabled(teleop_state);
        status.motion_busy = motion_busy;
        status.mode = fallback_mode;

        if (!controllers_ok) {
            status.success = false;
            status.primary_controller_state = primary_controller_name_.empty()
                                                  ? kControllerStateUnconfigured
                                                  : kControllerStateMissing;
            status.trajectory_controller_state = trajectory_controller_name_.empty()
                                                     ? kControllerStateUnconfigured
                                                     : kControllerStateMissing;
            status.message = error_message;
            return false;
        }

        status.primary_controller_state = controllers.primary_state;
        status.trajectory_controller_state = controllers.trajectory_state;
        status.controller_interlock_ok =
            !(controllers.primary_active && controllers.trajectory_active);
        status.mode = infer_mode_from_snapshot(
            controllers, teleop_state, motion_busy, fallback_mode);

        if (!status.controller_interlock_ok) {
            status.success = false;
            status.message =
                "Controller interlock violated: primary and trajectory controllers are both active.";
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_mode_ = kModeFault;
            return false;
        }

        status.success = true;
        status.message = "Motion status OK.";
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_mode_ = status.mode;
        }
        return true;
    }

    void handle_set_mode(
        const std::shared_ptr<srv::SetMotionMode::Request> request,
        std::shared_ptr<srv::SetMotionMode::Response> response)
    {
        const std::string target_mode = normalize_mode_name(request->mode);
        if (target_mode.empty()) {
            response->success = false;
            response->message = "Unsupported motion mode: " + request->mode;
            response->active_mode = cached_mode();
            return;
        }
        if (busy()) {
            response->success = false;
            response->message = "Motion command rejected because another motion is in progress.";
            response->active_mode = cached_mode();
            return;
        }

        std::string error_message;
        response->success = transition_to_mode(target_mode, error_message);
        response->active_mode = cached_mode();
        if (response->success) {
            response->message = "Motion mode set to " + target_mode + ".";
        } else {
            response->message = error_message;
        }
    }

    void handle_get_mode(
        const std::shared_ptr<srv::GetMotionMode::Request> /*request*/,
        std::shared_ptr<srv::GetMotionMode::Response> response)
    {
        MotionStatusSnapshot status;
        response->success = collect_status(status);
        response->mode = status.mode;
        response->message = status.message;
    }

    void handle_get_status(
        const std::shared_ptr<srv::GetMotionStatus::Request> /*request*/,
        std::shared_ptr<srv::GetMotionStatus::Response> response)
    {
        MotionStatusSnapshot status;
        response->success = collect_status(status);
        response->mode = status.mode;
        response->teleop_state = status.teleop_state;
        response->teleop_armed = status.teleop_armed;
        response->teleop_enabled = status.teleop_enabled;
        response->primary_controller_state = status.primary_controller_state;
        response->trajectory_controller_state = status.trajectory_controller_state;
        response->motion_busy = status.motion_busy;
        response->controller_interlock_ok = status.controller_interlock_ok;
        response->message = status.message;
    }

    void handle_set_enabled(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (busy()) {
            response->success = false;
            response->message =
                "Enable request rejected because another motion is in progress.";
            return;
        }
        if (!teleop_services_configured_) {
            response->success = false;
            response->message =
                "set_enabled is unsupported because this motion configuration has no teleop services.";
            return;
        }

        std::string error_message;
        if (request->data) {
            if (!transition_to_mode(kModeTeleop, error_message)) {
                response->success = false;
                response->message = error_message;
                return;
            }
            if (!call_teleop_set_enabled(true, error_message)) {
                response->success = false;
                response->message = error_message;
                return;
            }
            response->success = true;
            response->message = "Tracker teleop enabled via motion layer.";
            return;
        }

        if (!call_teleop_set_enabled(false, error_message)) {
            MotionStatusSnapshot status;
            collect_status(status);
            if (status.mode == kModeMotion) {
                response->success = true;
                response->message = "Teleop already disabled while motion mode is active.";
                return;
            }
            response->success = false;
            response->message = error_message;
            return;
        }

        response->success = true;
        response->message = "Tracker teleop disabled via motion layer.";
    }

    void handle_go_home(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (busy()) {
            response->success = false;
            response->message = "GoHome rejected because another motion is in progress.";
            return;
        }

        set_busy(true);

        const auto finish = [this](const std::string &mode) {
            set_busy(false);
            if (mode == kModeFault) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                current_mode_ = kModeFault;
            }
        };

        std::string error_message;
        bool go_home_ok = false;
        bool workspace_guard_disabled = false;
        bool used_legacy_fallback_after_moveit_failure = false;
        bool wait_for_legacy_home_completion = false;
        if (backend_ == "legacy") {
            if (transition_to_mode(kModeTeleop, error_message)) {
                go_home_ok = handle_legacy_go_home(error_message);
                wait_for_legacy_home_completion = go_home_ok;
            }
        } else if (backend_ == "moveit") {
            if (!call_workspace_guard_set_enabled(false, error_message)) {
                finish(kModeSafeHold);
                response->success = false;
                response->message = error_message;
                return;
            }
            workspace_guard_disabled = true;
            if (recovery_enabled_ && recovery_workspace_guard_configured_ &&
                recovery_command_publisher_) {
                std::array<std::array<double, kJointsPerArm>, 2> current_deg{};
                if (!current_joint_positions_deg(current_deg, error_message)) {
                    finish(kModeSafeHold);
                    response->success = false;
                    response->message = error_message;
                    return;
                }

                const auto current_evaluation = recovery_workspace_guard_.evaluate(current_deg);
                if (!current_evaluation.safe) {
                    RCLCPP_WARN(
                        get_logger(),
                        "GoHome requested from unsafe state (min_z=%.4f). Starting recovery.",
                        current_evaluation.min_z);
                    if (!execute_recovery(error_message)) {
                        finish(kModeSafeHold);
                        response->success = false;
                        response->message = error_message;
                        return;
                    }
                }
            }

            if (transition_to_mode(kModeMotion, error_message)) {
                go_home_ok = execute_moveit_go_home(error_message);
                if (!go_home_ok &&
                    (error_message.rfind("MoveIt planning failed", 0) == 0 ||
                     error_message.rfind("MoveIt execution failed", 0) == 0) &&
                    recovery_enabled_ && recovery_workspace_guard_configured_ &&
                    recovery_command_publisher_) {
                    RCLCPP_WARN(
                        get_logger(),
                        "Direct MoveIt home command failed from current state (%s). "
                        "Attempting escape recovery before retrying home.",
                        error_message.c_str());

                    std::string recovery_mode_error;
                    if (!transition_to_mode(kModeSafeHold, recovery_mode_error)) {
                        error_message +=
                            " Failed to switch back to SAFE_HOLD for recovery: " +
                            recovery_mode_error;
                    } else if (!execute_recovery(error_message, true)) {
                        // error_message already filled by execute_recovery
                    } else if (!transition_to_mode(kModeMotion, recovery_mode_error)) {
                        error_message +=
                            " Recovery succeeded but failed to re-enter MOTION mode: " +
                            recovery_mode_error;
                    } else {
                        go_home_ok = execute_moveit_go_home(error_message);
                    }
                }

                if (!go_home_ok &&
                    allow_legacy_go_home_fallback_ &&
                    (error_message.rfind("MoveIt planning failed", 0) == 0 ||
                     error_message.rfind("MoveIt execution failed", 0) == 0)) {
                    RCLCPP_WARN(
                        get_logger(),
                        "MoveIt home failed (%s). Falling back to legacy tracker go_home.",
                        error_message.c_str());
                    std::string fallback_mode_error;
                    if (!transition_to_mode(kModeTeleop, fallback_mode_error)) {
                        error_message +=
                            " Legacy fallback unavailable because TELEOP mode restore failed: " +
                            fallback_mode_error;
                    } else {
                        std::string legacy_error;
                        if (handle_legacy_go_home(legacy_error)) {
                            go_home_ok = true;
                            used_legacy_fallback_after_moveit_failure = true;
                            wait_for_legacy_home_completion = true;
                            error_message.clear();
                        } else {
                            error_message += " Legacy fallback failed: " + legacy_error;
                        }
                    }
                }
            }
        } else {
            error_message = "Unsupported motion backend: " + backend_;
        }

        if (go_home_ok && wait_for_legacy_home_completion) {
            std::string completion_error;
            if (!wait_for_named_pose_reached(
                    home_pose_id_,
                    legacy_go_home_settle_timeout_sec_,
                    legacy_home_tolerance_rad_,
                    completion_error)) {
                go_home_ok = false;
                error_message = completion_error;
            }
        }

        const bool returning_to_primary_controller =
            go_home_return_mode_ == kModeSafeHold || go_home_return_mode_ == kModeTeleop;
        if (go_home_ok && returning_to_primary_controller) {
            std::string reseed_error;
            if (!reseed_primary_command_reference("before mode restore", reseed_error)) {
                RCLCPP_WARN(
                    get_logger(),
                    "Failed to reseed primary controller reference before restoring mode: %s",
                    reseed_error.c_str());
            }
        }

        std::string restore_error;
        const bool restored_mode = transition_to_mode(go_home_return_mode_, restore_error);

        if (go_home_ok && restored_mode && returning_to_primary_controller) {
            std::string reseed_error;
            if (!reseed_primary_command_reference("after mode restore", reseed_error)) {
                RCLCPP_WARN(
                    get_logger(),
                    "Failed to reseed primary controller reference after restoring mode: %s",
                    reseed_error.c_str());
            }
        }

        std::string workspace_guard_restore_error;
        const bool restored_workspace_guard =
            !workspace_guard_disabled ||
            call_workspace_guard_set_enabled(true, workspace_guard_restore_error);

        if (!restored_mode || !restored_workspace_guard) {
            finish(kModeFault);
            response->success = false;
            if (!restored_mode) {
                if (error_message.empty()) {
                    response->message =
                        "GoHome completed motion step but failed to restore mode: " + restore_error;
                } else {
                    response->message =
                        error_message + " Restore step also failed: " + restore_error;
                }
            } else {
                response->message =
                    error_message.empty()
                        ? "GoHome restored motion mode but failed to re-enable workspace guard: " +
                              workspace_guard_restore_error
                        : error_message + " Workspace guard re-enable also failed: " +
                              workspace_guard_restore_error;
            }
            return;
        }

        if (!go_home_ok) {
            finish(kModeSafeHold);
            response->success = false;
            response->message = error_message;
            return;
        }

        finish(go_home_return_mode_);
        response->success = true;
        if (backend_ == "moveit") {
            if (used_legacy_fallback_after_moveit_failure) {
                response->message =
                    "MoveIt go_home fallback completed via tracker legacy home.";
                return;
            }
            response->message = "MoveIt go_home plan executed successfully.";
            return;
        }
        response->message = "Legacy go_home completed successfully.";
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr go_home_service_;
    rclcpp::Service<srv::SetMotionMode>::SharedPtr set_mode_service_;
    rclcpp::Service<srv::GetMotionMode>::SharedPtr get_mode_service_;
    rclcpp::Service<srv::GetMotionStatus>::SharedPtr get_status_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_enabled_service_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr legacy_go_home_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr tracker_set_armed_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr tracker_set_enabled_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr workspace_guard_client_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
        switch_controller_client_;
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr
        list_controllers_client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr teleop_state_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr recovery_command_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        gripper_collision_marker_publisher_;
    rclcpp::TimerBase::SharedPtr gripper_collision_timer_;

    std::string backend_;
    std::string go_home_service_name_;
    std::string set_mode_service_name_;
    std::string get_mode_service_name_;
    std::string get_status_service_name_;
    std::string set_enabled_service_name_;
    std::string legacy_go_home_service_;
    std::string tracker_set_armed_service_;
    std::string tracker_set_enabled_service_;
    std::string teleop_state_topic_;
    bool allow_legacy_go_home_fallback_{false};
    std::string planning_group_;
    std::string home_pose_id_;
    std::string initial_mode_{kModeSafeHold};
    std::string go_home_return_mode_{kModeSafeHold};
    double planning_time_sec_{5.0};
    double move_group_wait_sec_{10.0};
    double teleop_service_timeout_sec_{5.0};
    double legacy_go_home_timeout_sec_{10.0};
    double legacy_go_home_settle_timeout_sec_{20.0};
    double legacy_home_tolerance_rad_{0.5 * M_PI / 180.0};
    int64_t num_planning_attempts_{3};
    double max_velocity_scaling_{0.2};
    double max_acceleration_scaling_{0.2};
    bool execute_trajectory_{true};
    double controller_switch_timeout_sec_{5.0};
    std::string planning_pipeline_id_;
    std::string planner_id_;
    std::string scene_frame_id_;
    std::string controller_manager_switch_service_;
    std::string controller_manager_list_service_;
    std::string trajectory_controller_name_;
    std::string primary_controller_name_;
    std::string workspace_guard_service_name_;
    bool use_mock_hardware_{false};
    std::vector<std::string> go_home_pose_sequence_;
    bool recovery_enabled_{true};
    std::string recovery_command_topic_;
    std::vector<std::string> recovery_command_joint_names_;
    double recovery_feedback_timeout_sec_{2.0};
    double recovery_settle_timeout_sec_{8.0};
    double recovery_position_tolerance_rad_{0.05};
    double recovery_joint2_step_rad_{2.0 * M_PI / 180.0};
    double recovery_joint4_step_rad_{2.0 * M_PI / 180.0};
    int64_t recovery_search_layers_{6};
    int64_t recovery_interpolation_samples_{10};
    int64_t recovery_max_iterations_{6};
    double recovery_min_improvement_m_{0.002};
    double recovery_max_drop_m_{0.0005};
    std::string workspace_z_min_param_;
    std::string workspace_safety_margin_param_;
    std::string mount_xyz_l_param_;
    std::string mount_rpy_l_param_;
    std::string mount_xyz_r_param_;
    std::string mount_rpy_r_param_;
    std::string tool_offset_param_;
    bool teleop_services_configured_{true};
    bool teleop_state_feedback_configured_{true};

    mutable std::mutex state_mutex_;
    std::string current_mode_{kModeSafeHold};
    std::string latest_teleop_state_{kTeleopStateUnknown};
    bool motion_busy_{false};
    mutable std::mutex joint_state_mutex_;
    std::array<double, kTotalTrackedJoints> joint_positions_rad_{};
    std::array<bool, kTotalTrackedJoints> joint_position_valid_{};
    std::unordered_map<std::string, double> recovery_aux_joint_positions_rad_;
    marvin_system::WorkspaceGuard recovery_workspace_guard_;
    bool recovery_workspace_guard_configured_{false};

    std::mutex moveit_mutex_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::atomic<bool> static_scene_applied_{false};
    std::array<double, kTrackedGripperCount> last_synced_gripper_percent_{{-1.0, -1.0}};
    std::array<bool, kTrackedGripperCount> gripper_collision_objects_active_{{false, false}};
    std::array<bool, kTrackedGripperCount> gripper_collision_markers_active_{{false, false}};
    std::chrono::steady_clock::time_point last_moveit_bootstrap_attempt_{};
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
    node->shutdown_moveit_interfaces();
    executor.remove_node(node);
    node.reset();
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}
