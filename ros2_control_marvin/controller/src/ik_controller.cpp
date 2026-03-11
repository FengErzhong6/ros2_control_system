#include "ros2_control_marvin/ik_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace ros2_control_marvin {

namespace {

template <typename LoanedInterfaceT>
LoanedInterfaceT *find_loaned_interface(
    std::vector<LoanedInterfaceT> &interfaces, const std::string &full_name)
{
    for (auto &iface : interfaces) {
        if (iface.get_name() == full_name) {
            return &iface;
        }
    }
    return nullptr;
}

}  // namespace

IKController::IKController() : controller_interface::ControllerInterface() {}

IKController::~IKController() = default;

// ---------------------------------------------------------------------------
// on_init
// ---------------------------------------------------------------------------
controller_interface::CallbackReturn IKController::on_init()
{
    const auto logger = get_node()->get_logger();
    try {
        auto_declare<std::vector<std::string>>("joints", {});
        auto_declare<std::string>("kine_config_path", "");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(logger, "Failed to declare parameters: %s", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Interface configuration
// ---------------------------------------------------------------------------
controller_interface::InterfaceConfiguration
IKController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
    conf.names.reserve(joint_names_.size());
    for (const auto &jn : joint_names_) {
        conf.names.push_back(jn + "/position");
    }
    return conf;
}

controller_interface::InterfaceConfiguration
IKController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
    conf.names.reserve(joint_names_.size());
    for (const auto &jn : joint_names_) {
        conf.names.push_back(jn + "/position");
    }
    return conf;
}

// ---------------------------------------------------------------------------
// Kinematics initialisation (Marvin SDK)
// ---------------------------------------------------------------------------
bool IKController::initKinematics()
{
    const auto logger = get_node()->get_logger();

    FX_LOG_SWITCH(0);

    FX_BOOL ret = LOADMvCfg(
        const_cast<FX_CHAR *>(kine_config_path_.c_str()),
        kine_type_, kine_grv_, kine_dh_, kine_pnva_, kine_bd_,
        kine_mass_, kine_mcp_, kine_inertia_);

    if (!ret) {
        RCLCPP_ERROR(logger, "LOADMvCfg failed for '%s'", kine_config_path_.c_str());
        return false;
    }

    RCLCPP_INFO(logger, "Loaded kinematic config: type_L=%ld, type_R=%ld",
                static_cast<long>(kine_type_[0]), static_cast<long>(kine_type_[1]));

    for (int arm = 0; arm < static_cast<int>(kArmCount); ++arm) {
        if (!FX_Robot_Init_Type(arm, kine_type_[arm])) {
            RCLCPP_ERROR(logger, "FX_Robot_Init_Type failed for arm %d", arm);
            return false;
        }
        if (!FX_Robot_Init_Kine(arm, kine_dh_[arm])) {
            RCLCPP_ERROR(logger, "FX_Robot_Init_Kine failed for arm %d", arm);
            return false;
        }
        if (!FX_Robot_Init_Lmt(arm, kine_pnva_[arm], kine_bd_[arm])) {
            RCLCPP_ERROR(logger, "FX_Robot_Init_Lmt failed for arm %d", arm);
            return false;
        }
        RCLCPP_INFO(logger, "Kinematics initialized for arm %d", arm);
    }

    return true;
}

// ---------------------------------------------------------------------------
// Pose → 4×4 homogeneous matrix
// ---------------------------------------------------------------------------
void IKController::poseToMatrix4(
    const geometry_msgs::msg::PoseStamped &pose, Matrix4 mat)
{
    std::memset(mat, 0, sizeof(Matrix4));

    const auto &p = pose.pose.position;
    const auto &q = pose.pose.orientation;

    const double qx = q.x, qy = q.y, qz = q.z, qw = q.w;
    const double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    const double s = (n > 1e-12) ? (2.0 / (n * n)) : 0.0;

    const double wx = s * qw * qx, wy = s * qw * qy, wz = s * qw * qz;
    const double xx = s * qx * qx, xy = s * qx * qy, xz = s * qx * qz;
    const double yy = s * qy * qy, yz = s * qy * qz, zz = s * qz * qz;

    mat[0][0] = 1.0 - (yy + zz);
    mat[0][1] = xy - wz;
    mat[0][2] = xz + wy;
    mat[0][3] = p.x * kM2Mm;  // m → mm

    mat[1][0] = xy + wz;
    mat[1][1] = 1.0 - (xx + zz);
    mat[1][2] = yz - wx;
    mat[1][3] = p.y * kM2Mm;

    mat[2][0] = xz - wy;
    mat[2][1] = yz + wx;
    mat[2][2] = 1.0 - (xx + yy);
    mat[2][3] = p.z * kM2Mm;

    mat[3][0] = 0.0;
    mat[3][1] = 0.0;
    mat[3][2] = 0.0;
    mat[3][3] = 1.0;
}

// ---------------------------------------------------------------------------
// Quaternion validity check
// ---------------------------------------------------------------------------
bool IKController::isQuaternionValid(const geometry_msgs::msg::Quaternion &q) const
{
    if (!std::isfinite(q.x) || !std::isfinite(q.y) ||
        !std::isfinite(q.z) || !std::isfinite(q.w)) {
        return false;
    }
    const double norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::abs(norm_sq - 1.0) < 0.01;
}

// ---------------------------------------------------------------------------
// IKResult → string (compile-time table, no heap allocation)
// ---------------------------------------------------------------------------
const char *IKController::ikResultToString(IKResult result)
{
    switch (result) {
        case IKResult::kSuccess:             return "IK solved successfully";
        case IKResult::kNoTarget:            return "No target";
        case IKResult::kInvalidQuaternion:   return "Invalid quaternion (non-finite or not unit-length)";
        case IKResult::kOutOfRange:          return "Target out of reachable workspace";
        case IKResult::kJointLimitExceeded:  return "IK solution exceeds joint limits";
        case IKResult::kSingularity:         return "IK solution near singularity";
        default:                             return "Unknown IK error";
    }
}

// ---------------------------------------------------------------------------
// Publish IK status feedback (only on state change)
// ---------------------------------------------------------------------------
void IKController::publishIKStatus(size_t arm_index, IKResult result)
{
    if (arm_index >= kArmCount || !pub_ik_status_[arm_index]) {
        return;
    }
    if (result == last_ik_result_[arm_index]) {
        return;
    }
    last_ik_result_[arm_index] = result;

    std_msgs::msg::String status_msg;
    const bool success = (result == IKResult::kSuccess);
    status_msg.data = std::string(success ? "[OK] " : "[FAIL] ") + ikResultToString(result);
    pub_ik_status_[arm_index]->publish(status_msg);
}

// ---------------------------------------------------------------------------
// 4×4 homogeneous matrix → PoseStamped (mm → m, rotation matrix → quaternion)
// ---------------------------------------------------------------------------
void IKController::matrix4ToPose(const Matrix4 mat,
                                  geometry_msgs::msg::PoseStamped &pose) const
{
    pose.header.frame_id = "base_link";
    pose.header.stamp = get_node()->get_clock()->now();

    pose.pose.position.x = mat[0][3] * kMm2M;
    pose.pose.position.y = mat[1][3] * kMm2M;
    pose.pose.position.z = mat[2][3] * kMm2M;

    // Robust rotation-matrix → quaternion (Shepperd's method)
    const double trace = mat[0][0] + mat[1][1] + mat[2][2];
    double qw, qx, qy, qz;

    if (trace > 0.0) {
        double s = 0.5 / std::sqrt(trace + 1.0);
        qw = 0.25 / s;
        qx = (mat[2][1] - mat[1][2]) * s;
        qy = (mat[0][2] - mat[2][0]) * s;
        qz = (mat[1][0] - mat[0][1]) * s;
    } else if (mat[0][0] > mat[1][1] && mat[0][0] > mat[2][2]) {
        double s = 2.0 * std::sqrt(1.0 + mat[0][0] - mat[1][1] - mat[2][2]);
        qw = (mat[2][1] - mat[1][2]) / s;
        qx = 0.25 * s;
        qy = (mat[0][1] + mat[1][0]) / s;
        qz = (mat[0][2] + mat[2][0]) / s;
    } else if (mat[1][1] > mat[2][2]) {
        double s = 2.0 * std::sqrt(1.0 + mat[1][1] - mat[0][0] - mat[2][2]);
        qw = (mat[0][2] - mat[2][0]) / s;
        qx = (mat[0][1] + mat[1][0]) / s;
        qy = 0.25 * s;
        qz = (mat[1][2] + mat[2][1]) / s;
    } else {
        double s = 2.0 * std::sqrt(1.0 + mat[2][2] - mat[0][0] - mat[1][1]);
        qw = (mat[1][0] - mat[0][1]) / s;
        qx = (mat[0][2] + mat[2][0]) / s;
        qy = (mat[1][2] + mat[2][1]) / s;
        qz = 0.25 * s;
    }

    double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    pose.pose.orientation.x = qx / norm;
    pose.pose.orientation.y = qy / norm;
    pose.pose.orientation.z = qz / norm;
    pose.pose.orientation.w = qw / norm;
}

// ---------------------------------------------------------------------------
// Compute FK for both arms from last_joint_deg_ and publish current poses
// ---------------------------------------------------------------------------
void IKController::computeAndPublishFK()
{
    const auto logger = get_node()->get_logger();
    static const char *arm_labels[] = {"LEFT", "RIGHT"};

    for (size_t arm = 0; arm < kArmCount; ++arm) {
        FX_DOUBLE joints[kJointsPerArm];
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            joints[j] = last_joint_deg_[arm][j];
        }

        Matrix4 tcp;
        if (!FX_Robot_Kine_FK(static_cast<FX_INT32L>(arm), joints, tcp)) {
            RCLCPP_WARN(logger, "FK failed for arm %s", arm_labels[arm]);
            continue;
        }

        geometry_msgs::msg::PoseStamped pose;
        matrix4ToPose(tcp, pose);
        pub_current_pose_[arm]->publish(pose);

        RCLCPP_INFO(logger,
            "Arm %s FK pose: pos=(%.4f, %.4f, %.4f) m  quat=(%.4f, %.4f, %.4f, %.4f)",
            arm_labels[arm],
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
            pose.pose.orientation.x, pose.pose.orientation.y,
            pose.pose.orientation.z, pose.pose.orientation.w);
    }
}

// ---------------------------------------------------------------------------
// Inverse kinematics solve for a single arm
// ---------------------------------------------------------------------------
IKController::IKResult IKController::solveIK(
    size_t arm_index,
    const geometry_msgs::msg::PoseStamped &target_pose,
    double (&result_joints_rad)[kJointsPerArm])
{
    const auto logger = get_node()->get_logger();

    if (!isQuaternionValid(target_pose.pose.orientation)) {
        RCLCPP_WARN_THROTTLE(logger, *get_node()->get_clock(), 1000,
                             "Arm %zu: invalid quaternion (x=%.4f y=%.4f z=%.4f w=%.4f)",
                             arm_index,
                             target_pose.pose.orientation.x,
                             target_pose.pose.orientation.y,
                             target_pose.pose.orientation.z,
                             target_pose.pose.orientation.w);
        return IKResult::kInvalidQuaternion;
    }

    auto &ik = ik_params_[arm_index];
    std::memset(&ik, 0, sizeof(FX_InvKineSolvePara));

    poseToMatrix4(target_pose, ik.m_Input_IK_TargetTCP);

    for (size_t j = 0; j < kJointsPerArm; ++j) {
        ik.m_Input_IK_RefJoint[j] = last_joint_deg_[arm_index][j];
    }

    ik.m_Input_IK_ZSPType = FX_PILOT_NSP_TYPES_NEAR_REF;

    FX_INT32L serial = static_cast<FX_INT32L>(arm_index);
    if (!FX_Robot_Kine_IK(serial, &ik)) {
        RCLCPP_WARN_THROTTLE(logger, *get_node()->get_clock(), 1000,
                             "Arm %zu: IK failed (out_of_range=%d)",
                             arm_index, static_cast<int>(ik.m_Output_IsOutRange));
        return IKResult::kOutOfRange;
    }

    if (ik.m_Output_IsJntExd) {
        RCLCPP_WARN_THROTTLE(logger, *get_node()->get_clock(), 1000,
                             "Arm %zu: joint limits exceeded (max=%.2f deg)",
                             arm_index, ik.m_Output_JntExdABS);
        return IKResult::kJointLimitExceeded;
    }

    for (size_t j = 0; j < kJointsPerArm; ++j) {
        if (ik.m_Output_IsDeg[j]) {
            RCLCPP_WARN_THROTTLE(logger, *get_node()->get_clock(), 1000,
                                 "Arm %zu: near singularity at J%zu", arm_index, j + 1);
            return IKResult::kSingularity;
        }
    }

    for (size_t j = 0; j < kJointsPerArm; ++j) {
        result_joints_rad[j] = ik.m_Output_RetJoint[j] * kDeg2Rad;
    }

    return IKResult::kSuccess;
}

// ---------------------------------------------------------------------------
// on_configure
// ---------------------------------------------------------------------------
controller_interface::CallbackReturn
IKController::on_configure(const rclcpp_lifecycle::State &)
{
    const auto logger = get_node()->get_logger();

    std::vector<std::string> joints_param;
    if (!get_node()->get_parameter("joints", joints_param)) {
        RCLCPP_ERROR(logger, "Missing required parameter 'joints'.");
        return CallbackReturn::ERROR;
    }
    if (joints_param.size() != kTotalJoints) {
        RCLCPP_ERROR(logger, "Parameter 'joints' must have exactly %zu entries (got %zu).",
                     kTotalJoints, joints_param.size());
        return CallbackReturn::ERROR;
    }
    joint_names_ = joints_param;

    if (!get_node()->get_parameter("kine_config_path", kine_config_path_)) {
        RCLCPP_ERROR(logger, "Missing required parameter 'kine_config_path'.");
        return CallbackReturn::ERROR;
    }
    if (kine_config_path_.empty()) {
        RCLCPP_ERROR(logger, "Parameter 'kine_config_path' is empty.");
        return CallbackReturn::ERROR;
    }

    if (!initKinematics()) {
        RCLCPP_ERROR(logger, "Kinematics initialization failed.");
        return CallbackReturn::ERROR;
    }
    kine_initialized_ = true;

    // Initialize last joint angles to zero
    for (auto &arm : last_joint_deg_) {
        arm.fill(0.0);
    }

    // Reset interface pointers
    cmd_interfaces_.fill(nullptr);
    state_interfaces_pos_.fill(nullptr);

    // IK status publishers
    pub_ik_status_[0] = get_node()->create_publisher<std_msgs::msg::String>(
        "~/ik_status_left", rclcpp::SystemDefaultsQoS());
    pub_ik_status_[1] = get_node()->create_publisher<std_msgs::msg::String>(
        "~/ik_status_right", rclcpp::SystemDefaultsQoS());

    // FK current-pose publishers (transient_local so late joiners get the last value)
    auto fk_qos = rclcpp::QoS(1).transient_local();
    pub_current_pose_[0] = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/current_pose_left", fk_qos);
    pub_current_pose_[1] = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/current_pose_right", fk_qos);

    // Subscribers for target poses
    sub_target_left_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/target_pose_left", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            std::lock_guard<std::mutex> lk(arm_targets_[0].mtx);
            arm_targets_[0].pose = *msg;
            arm_targets_[0].has_target = true;
        });

    sub_target_right_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/target_pose_right", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            std::lock_guard<std::mutex> lk(arm_targets_[1].mtx);
            arm_targets_[1].pose = *msg;
            arm_targets_[1].has_target = true;
        });

    RCLCPP_INFO(logger, "IKController configured: subscribing to ~/target_pose_left and ~/target_pose_right");
    return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_activate
// ---------------------------------------------------------------------------
controller_interface::CallbackReturn
IKController::on_activate(const rclcpp_lifecycle::State &)
{
    const auto logger = get_node()->get_logger();

    for (size_t i = 0; i < kTotalJoints; ++i) {
        const std::string cmd_name = joint_names_[i] + "/position";

        auto *cmd_iface = find_loaned_interface(command_interfaces_, cmd_name);
        if (!cmd_iface) {
            RCLCPP_ERROR(logger, "Missing command interface '%s'.", cmd_name.c_str());
            return CallbackReturn::ERROR;
        }
        cmd_interfaces_[i] = cmd_iface;

        auto *state_iface = find_loaned_interface(state_interfaces_, cmd_name);
        if (!state_iface) {
            RCLCPP_ERROR(logger, "Missing state interface '%s'.", cmd_name.c_str());
            return CallbackReturn::ERROR;
        }
        state_interfaces_pos_[i] = state_iface;
    }

    // Seed last_joint_deg_ and commands from current state (avoid jumps)
    for (size_t i = 0; i < kTotalJoints; ++i) {
        const size_t arm = i / kJointsPerArm;
        const size_t j = i % kJointsPerArm;

        const auto pos_opt = state_interfaces_pos_[i]->get_optional<double>();
        const double pos_rad = pos_opt.has_value() ? pos_opt.value() : 0.0;

        last_joint_deg_[arm][j] = pos_rad * kRad2Deg;
        (void)cmd_interfaces_[i]->set_value(pos_rad);
    }

    // Clear pending targets
    for (auto &t : arm_targets_) {
        std::lock_guard<std::mutex> lk(t.mtx);
        t.has_target = false;
    }

    // Compute FK from current joint state and publish (initial end-effector pose)
    computeAndPublishFK();

    RCLCPP_INFO(logger, "IKController activated.");
    return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_deactivate
// ---------------------------------------------------------------------------
controller_interface::CallbackReturn
IKController::on_deactivate(const rclcpp_lifecycle::State &)
{
    cmd_interfaces_.fill(nullptr);
    state_interfaces_pos_.fill(nullptr);
    return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// update  (called at controller_manager update_rate)
// ---------------------------------------------------------------------------
controller_interface::return_type
IKController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
    if (!kine_initialized_) {
        return controller_interface::return_type::OK;
    }

    for (size_t arm = 0; arm < kArmCount; ++arm) {
        geometry_msgs::msg::PoseStamped target;
        bool has_target = false;

        {
            std::lock_guard<std::mutex> lk(arm_targets_[arm].mtx);
            has_target = arm_targets_[arm].has_target;
            if (has_target) {
                target = arm_targets_[arm].pose;
                arm_targets_[arm].has_target = false;  // consume the target
            }
        }

        if (!has_target) {
            continue;
        }

        double result_rad[kJointsPerArm];
        IKResult result = solveIK(arm, target, result_rad);

        publishIKStatus(arm, result);  // only publishes on state change

        if (result != IKResult::kSuccess) {
            continue;
        }

        const size_t offset = arm * kJointsPerArm;
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            (void)cmd_interfaces_[offset + j]->set_value(result_rad[j]);
            last_joint_deg_[arm][j] = result_rad[j] * kRad2Deg;
        }
    }

    return controller_interface::return_type::OK;
}

}  // namespace ros2_control_marvin

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ros2_control_marvin::IKController,
                       controller_interface::ControllerInterface)
