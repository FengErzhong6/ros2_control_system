#include "internal.hpp"

#include <exception>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace marvin_system {

TrackerTeleopController::TrackerTeleopController()
    : controller_interface::ControllerInterface() {}

TrackerTeleopController::~TrackerTeleopController() = default;

controller_interface::CallbackReturn TrackerTeleopController::on_init()
{
    try {
        auto_declare<std::vector<std::string>>("joints", {});
        auto_declare<std::string>("kine_config_path", "");

        auto_declare<std::string>("tracker_frames.torso", "tracker_torso");
        auto_declare<std::string>("tracker_frames.left_hand", "tracker_left_hand");
        auto_declare<std::string>("tracker_frames.right_hand", "tracker_right_hand");
        auto_declare<std::string>("tracker_frames.left_upper_arm", "tracker_left_upper_arm");
        auto_declare<std::string>("tracker_frames.right_upper_arm", "tracker_right_upper_arm");

        auto_declare<double>("position_scale", 1.0);
        auto_declare<bool>("enable_orientation", true);
        auto_declare<bool>("enable_ik_reference_logs", false);
        auto_declare<std::string>("base_frame", "base_link");
        auto_declare<double>("j4_bound", 0.0);
        auto_declare<double>("dh_d1", 0.0);
        auto_declare<std::vector<std::string>>("viz_base_frames", {"Base_L", "Base_R"});
        auto_declare<double>("tracking_ik.fk_accept_tol", 1e-3);
        auto_declare<double>("tracking_ik.fine_psi_range_deg", 2.0);
        auto_declare<double>("tracking_ik.fine_psi_step_deg", 0.1);
        auto_declare<double>("tracking_ik.fast_psi_range_deg", 12.0);
        auto_declare<double>("tracking_ik.fast_psi_step_deg", 0.5);
        auto_declare<double>("tracking_ik.expand_psi_range_deg", 63.0);
        auto_declare<double>("tracking_ik.expand_psi_step_deg", 2.0);
        auto_declare<double>("tracking_ik.score.desired_dir_weight", 0.03);
        auto_declare<double>("tracking_ik.score.continuity_dir_weight", 0.03);
        auto_declare<double>("tracking_ik.score.magnitude_weight", 0.05);
        auto_declare<double>("tracking_ik.score.psi_delta_weight", 0.02);
        auto_declare<double>("tracking_ik.score.branch_switch_penalty", 20.0);

        auto_declare<double>("smoothing_alpha", 0.3);
        auto_declare<double>("max_joint_velocity", 2.0);
        auto_declare<double>("base_x_scale", 1.0);
        auto_declare<double>("tracker_timeout_sec", 0.1);
        auto_declare<std::vector<double>>("home_joint_positions.left", {});
        auto_declare<std::vector<double>>("home_joint_positions.right", {});
        auto_declare<double>("home_tolerance_deg", 0.5);
        for (const auto &side : {"left", "right"}) {
            for (const auto &group :
                 {"shoulder_T_chest", "wrist_T_ee", "arm_human_T_arm_robot"}) {
                std::string prefix = std::string(group) + "." + side;
                auto_declare<std::vector<double>>(prefix + ".position", {0.0, 0.0, 0.0});
                auto_declare<std::vector<double>>(prefix + ".orientation", {0.0, 0.0, 0.0, 1.0});
            }
            auto_declare<std::vector<double>>(
                std::string("elbow_direction_correction.") + side, {0.0, 0.0, 0.0});
            auto_declare<double>(
                std::string("startup_ref_dir_sign.") + side, side == std::string("right") ? -1.0 : 1.0);
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
TrackerTeleopController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
    conf.names.reserve(joint_names_.size());
    for (const auto &jn : joint_names_) {
        conf.names.push_back(jn + "/position");
    }
    return conf;
}

controller_interface::InterfaceConfiguration
TrackerTeleopController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
    conf.names.reserve(joint_names_.size());
    for (const auto &jn : joint_names_) {
        conf.names.push_back(jn + "/position");
    }
    return conf;
}

controller_interface::CallbackReturn
TrackerTeleopController::on_configure(const rclcpp_lifecycle::State &)
{
    if (!readJointNames() || !initializeKinematics() || !loadControllerParameters()) {
        return CallbackReturn::ERROR;
    }

    resetRuntimeState();
    createRosInterfaces();

    const auto logger = get_node()->get_logger();
    RCLCPP_INFO(logger, "TrackerTeleopController configured (scale=%.2f, orientation=%s)",
                position_scale_, enable_orientation_ ? "ON" : "OFF");
    publishTeleopState("configured: teleop locked until armed");
    return CallbackReturn::SUCCESS;
}

bool TrackerTeleopController::bindJointInterfaces()
{
    const auto logger = get_node()->get_logger();

    for (size_t index = 0; index < kTotalJoints; ++index) {
        const std::string interface_name = joint_names_[index] + "/position";

        auto *cmd = find_loaned_interface(command_interfaces_, interface_name);
        if (!cmd) {
            RCLCPP_ERROR(logger, "Missing command interface '%s'.", interface_name.c_str());
            return false;
        }
        cmd_interfaces_[index] = cmd;

        auto *state = find_loaned_interface(state_interfaces_, interface_name);
        if (!state) {
            RCLCPP_ERROR(logger, "Missing state interface '%s'.", interface_name.c_str());
            return false;
        }
        state_interfaces_pos_[index] = state;
    }

    return true;
}

void TrackerTeleopController::initializeJointTargetsFromState()
{
    for (size_t index = 0; index < kTotalJoints; ++index) {
        const size_t arm = index / kJointsPerArm;
        const size_t joint = index % kJointsPerArm;
        auto &runtime = arm_state_[arm];
        const auto pos_opt = state_interfaces_pos_[index]->get_optional<double>();
        const double pos_rad = pos_opt.has_value() ? pos_opt.value() : 0.0;

        runtime.last_joint_deg[joint] = pos_rad * kRad2Deg;
        runtime.smoothed_joints_rad[joint] = pos_rad;
        runtime.target_joints_rad[joint] = pos_rad;
        (void)cmd_interfaces_[index]->set_value(pos_rad);
    }

    for (size_t arm = 0; arm < kArmCount; ++arm) {
        auto &runtime = arm_state_[arm];
        runtime.last_selected_ref_dir = {
            {kDefaultShoulderVElbow[0], kDefaultShoulderVElbow[1], kDefaultShoulderVElbow[2]}};
        normalizeVector(runtime.last_selected_ref_dir);
        runtime.last_selected_branch = -1;
        (void)seedTrackingStateFromCurrentJoints(arm);
        runtime.has_valid_target = true;
    }
}

bool TrackerTeleopController::seedTrackingStateFromCurrentJoints(size_t arm)
{
    if (arm >= kArmCount) {
        return false;
    }

    const auto logger = get_node()->get_logger();
    auto &runtime = arm_state_[arm];
    std::array<double, 3> seeded_ref_dir = runtime.last_selected_ref_dir;
    if (!extractSolvedUpperArmDir(
            static_cast<FX_INT32L>(arm), runtime.smoothed_joints_rad, seeded_ref_dir)) {
        RCLCPP_WARN(
            logger,
            "Failed to extract current upper-arm direction for %s arm; "
            "keeping default startup ref_dir and branch.",
            kSideTags[arm]);
        return false;
    }

    runtime.last_selected_ref_dir = seeded_ref_dir;

    FX_DOUBLE joints_deg[kJointsPerArm];
    Matrix4 tcp_mat{};
    for (size_t j = 0; j < kJointsPerArm; ++j) {
        joints_deg[j] = runtime.last_joint_deg[j];
    }

    if (!FX_Robot_Kine_FK(static_cast<FX_INT32L>(arm), joints_deg, tcp_mat)) {
        RCLCPP_WARN(
            logger,
            "Failed to compute FK for %s arm startup pose; "
            "seeded ref_dir only, branch remains unknown.",
            kSideTags[arm]);
        return false;
    }

    geometry_msgs::msg::PoseStamped base_T_ee;
    base_T_ee.header.frame_id = base_frame_;
    base_T_ee.header.stamp = get_node()->get_clock()->now();
    matrix4ToPose(tcp_mat, base_T_ee);

    tracking_ik::Request request;
    tracking_ik::Result result{};
    tracking_ik::SetDefaultRequest(&request);
    poseToMatrix4(base_T_ee, request.target_tcp);
    request.desired_upper_arm_dir[0] = seeded_ref_dir[0];
    request.desired_upper_arm_dir[1] = seeded_ref_dir[1];
    request.desired_upper_arm_dir[2] = seeded_ref_dir[2];
    request.prev_selected_ref_dir[0] = seeded_ref_dir[0];
    request.prev_selected_ref_dir[1] = seeded_ref_dir[1];
    request.prev_selected_ref_dir[2] = seeded_ref_dir[2];
    request.prev_selected_branch = -1;
    for (size_t j = 0; j < kJointsPerArm; ++j) {
        request.ref_joint_deg[j] = runtime.last_joint_deg[j];
    }

    request.fk_accept_tol = tracking_ik_config_.fk_accept_tol;
    request.fine_psi_range_deg = tracking_ik_config_.fine_psi_range_deg;
    request.fine_psi_step_deg = tracking_ik_config_.fine_psi_step_deg;
    request.fast_psi_range_deg = tracking_ik_config_.fast_psi_range_deg;
    request.fast_psi_step_deg = tracking_ik_config_.fast_psi_step_deg;
    request.expand_psi_range_deg = tracking_ik_config_.expand_psi_range_deg;
    request.expand_psi_step_deg = tracking_ik_config_.expand_psi_step_deg;
    request.score_params.desired_dir_weight =
        tracking_ik_config_.desired_dir_weight;
    request.score_params.continuity_dir_weight =
        tracking_ik_config_.continuity_dir_weight;
    request.score_params.magnitude_weight =
        tracking_ik_config_.magnitude_weight;
    request.score_params.psi_delta_weight =
        tracking_ik_config_.psi_delta_weight;
    request.score_params.branch_switch_penalty =
        tracking_ik_config_.branch_switch_penalty;

    if (!tracking_ik::Solve(&tracking_ik_geometry_, &request, &result)) {
        RCLCPP_WARN(
            logger,
            "Failed to seed startup branch for %s arm from current pose; "
            "using current upper-arm dir only.",
            kSideTags[arm]);
        return false;
    }

    if (!result.success) {
        RCLCPP_WARN(
            logger,
            "trackingIK could not classify %s arm startup pose; "
            "using current upper-arm dir only.",
            kSideTags[arm]);
        return false;
    }

    runtime.last_selected_ref_dir = {
        {result.selected_ref_dir[0], result.selected_ref_dir[1], result.selected_ref_dir[2]}};
    if (startup_ref_dir_sign_[arm] < 0.0) {
        for (double &component : runtime.last_selected_ref_dir) {
            component = -component;
        }
    }
    normalizeVector(runtime.last_selected_ref_dir);
    runtime.last_selected_branch = result.selected_branch;

    RCLCPP_INFO(
        logger,
        "Seeded %s arm startup tracking state from hardware pose: "
        "branch=%ld ref_dir=[%.3f, %.3f, %.3f] psi=%.1f deg sign=%+.0f",
        kSideTags[arm],
        static_cast<long>(runtime.last_selected_branch),
        runtime.last_selected_ref_dir[0],
        runtime.last_selected_ref_dir[1],
        runtime.last_selected_ref_dir[2],
        result.selected_psi_deg,
        startup_ref_dir_sign_[arm]);
    return true;
}

controller_interface::CallbackReturn
TrackerTeleopController::on_activate(const rclcpp_lifecycle::State &)
{
    if (!bindJointInterfaces()) {
        return CallbackReturn::ERROR;
    }

    initializeJointTargetsFromState();
    resetTrackerState();
    resetTeleopRuntime(TeleopState::kDisarmed);
    setTeleopState(TeleopState::kDisarmed, "controller activated: teleop locked");

    computeAndPublishFK();

    const auto logger = get_node()->get_logger();
    RCLCPP_INFO(
        logger,
        "TrackerTeleopController activated (absolute mapping mode, startup state=%s).",
        teleopStateToString(getTeleopState()));
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TrackerTeleopController::on_deactivate(const rclcpp_lifecycle::State &)
{
    resetTeleopRuntime(TeleopState::kDisarmed);
    cmd_interfaces_.fill(nullptr);
    state_interfaces_pos_.fill(nullptr);
    return CallbackReturn::SUCCESS;
}

/*
 * 位姿链（与 TF 一致，均为 ROS 系）：
 *   vr_T_tracker → corrections.yaml → ros_T_world_tracker (tracker_publisher)
 *   chest_T_wrist = (ros_T_chest)^{-1} * ros_T_wrist  [TF: torso_T_hand]
 *   shoulder_T_wrist  = shoulder_T_chest * chest_T_wrist
 *   shoulder_T_ee     = shoulder_T_wrist * wrist_T_ee
 *   shoulder_R_arm_robot = shoulder_R_chest * chest_R_arm * arm_human_R_arm_robot
 *   shoulder_v_elbow = Y 列 of shoulder_R_arm_robot → IK 零空间
 *
 * NOTE: marvin SDK 的 IK 输入为 base_T_ee（base 坐标系下的末端位姿）。
 *       shoulder_T_ee 需加 [0, 0, +d1] 偏置转换为 base_T_ee 后传入 IK。
 *       肘向量原点在 shoulder，直接传入 IK 即可。
 *
 * Runtime behavior:
 *   - TF lookups run in a non-RT timer (pollTfCallback, 200 Hz).
 *   - update() only consumes cached TF, updates IK targets, and applies smoothing.
 *   - Diagnostics, marker publication, and IK status publication run in a
 *     separate non-RT timer, outside the main control loop.
 *   - When tracker data times out, the controller holds the current joint command
 *     instead of continuing toward the last IK target.
 */
controller_interface::return_type
TrackerTeleopController::update(const rclcpp::Time &, const rclcpp::Duration &period)
{
    if (!kine_initialized_) {
        return controller_interface::return_type::OK;
    }

    const double dt = period.seconds();
    const auto now = get_node()->get_clock()->now();
    (void)updateTfSnapshot();
    if (go_home_requested_.load(std::memory_order_relaxed) ||
        go_home_active_.load(std::memory_order_relaxed)) {
        processGoHome(dt);
        return controller_interface::return_type::OK;
    }

    const bool teleop_enabled = isTeleopEnabled(now);
    const bool force_reacquire = teleop_enabled &&
        force_tracker_reacquire_.exchange(false, std::memory_order_relaxed);

    if (!teleop_enabled) {
        holdAllArms(dt);
        return controller_interface::return_type::OK;
    }

    for (size_t arm = 0; arm < kArmCount; ++arm) {
        processArmUpdate(arm, tf_snapshot_[arm], now, dt, force_reacquire);
    }

    return controller_interface::return_type::OK;
}

}  // namespace marvin_system

PLUGINLIB_EXPORT_CLASS(marvin_system::TrackerTeleopController,
                       controller_interface::ControllerInterface)
