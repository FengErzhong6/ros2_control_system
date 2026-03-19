#include "ros2_control_marvin/tracker_teleop_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>

#include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Matrix3x3.h"

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
        auto_declare<std::string>("base_frame", "base_link");
        auto_declare<std::vector<double>>("default_elbow_direction",
                                          {0.0, 0.0, -1.0});
        auto_declare<double>("zsp_angle", 0.0);
        auto_declare<double>("j4_bound", 0.0);

        auto_declare<double>("smoothing_alpha", 0.3);
        auto_declare<double>("max_joint_velocity", 2.0);
        auto_declare<double>("base_x_scale", 1.0);
        auto_declare<bool>("ik_fallback_near_ref", true);
        auto_declare<bool>("ik_clamp_joint_limits", true);

        for (const auto &side : {"left", "right"}) {
            for (const auto &group :
                 {"base_T_chest", "wrist_T_ee", "arm_human_T_arm_robot"}) {
                std::string prefix = std::string(group) + "." + side;
                auto_declare<std::vector<double>>(prefix + ".position", {0.0, 0.0, 0.0});
                auto_declare<std::vector<double>>(prefix + ".orientation", {0.0, 0.0, 0.0, 1.0});
            }
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

bool TrackerTeleopController::lookupTf(
    const std::string &target_frame,
    const std::string &source_frame,
    geometry_msgs::msg::TransformStamped &out_target_T_source)
{
    try {
        out_target_T_source = tf_buffer_->lookupTransform(
            target_frame, source_frame, tf2::TimePointZero);
        return true;
    } catch (const tf2::TransformException &) {
        return false;
    }
}

void TrackerTeleopController::pollTfCallback()
{
    const std::string *frame_hand[] = {&frame_left_hand_, &frame_right_hand_};
    const std::string *frame_arm[] = {&frame_left_upper_arm_, &frame_right_upper_arm_};

    std::array<CachedTrackerData, kArmCount> fresh;
    for (int arm = 0; arm < static_cast<int>(kArmCount); ++arm) {
        fresh[arm].hand_valid = lookupTf(
            frame_torso_, *frame_hand[arm], fresh[arm].chest_T_hand);
        fresh[arm].arm_valid = lookupTf(
            frame_torso_, *frame_arm[arm], fresh[arm].chest_T_arm);
    }

    std::lock_guard<std::mutex> lk(tf_cache_mutex_);
    tf_cache_ = fresh;
}

TrackerTeleopController::IKResult TrackerTeleopController::solveIK(
    size_t arm,
    const geometry_msgs::msg::PoseStamped &base_T_ee,
    const double (&base_v_elbow)[3],
    double (&out_q_joints_rad)[kJointsPerArm])
{
    const auto logger = get_node()->get_logger();

    if (!isQuaternionValid(base_T_ee.pose.orientation)) {
        RCLCPP_WARN_THROTTLE(logger, *get_node()->get_clock(), 1000,
                             "Arm %zu: invalid quaternion", arm);
        return IKResult::kInvalidQuaternion;
    }

    auto &ik = ik_params_[arm];
    FX_INT32L serial = static_cast<FX_INT32L>(arm);

    auto setup_common = [&]() {
        std::memset(&ik, 0, sizeof(FX_InvKineSolvePara));
        poseToMatrix4(base_T_ee, ik.m_Input_IK_TargetTCP);
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            ik.m_Input_IK_RefJoint[j] = last_joint_deg_[arm][j];
        }
    };

    auto evaluate_ik = [&]() -> IKResult {
        if (!FX_Robot_Kine_IK(serial, &ik)) {
            return IKResult::kOutOfRange;
        }
        if (ik.m_Output_IsJntExd) {
            return IKResult::kJointLimitExceeded;
        }
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            if (ik.m_Output_IsDeg[j]) {
                return IKResult::kSingularity;
            }
        }
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            out_q_joints_rad[j] = ik.m_Output_RetJoint[j] * kDeg2Rad;
        }
        return IKResult::kSuccess;
    };

    auto classify_j4 = [this](double j4) -> const char * {
        return (j4 < j4_bound_) ? "Case A (type -1)" : "Case B (type +1)";
    };

    // Attempt 1: NEAR_DIR — use tracker elbow direction to constrain null-space
    setup_common();
    ik.m_Input_IK_ZSPType = FX_PILOT_NSP_TYPES_NEAR_DIR;
    ik.m_Input_IK_ZSPPara[0] = base_v_elbow[0];
    ik.m_Input_IK_ZSPPara[1] = base_v_elbow[1];
    ik.m_Input_IK_ZSPPara[2] = base_v_elbow[2];
    ik.m_Input_ZSP_Angle = zsp_angle_;

    IKResult dir_result = evaluate_ik();

    if (!first_ik_logged_[arm]) {
        RCLCPP_INFO(logger,
            "Arm %zu IK input: ee_T_base pos=[%.4f, %.4f, %.4f] m  "
            "quat=[%.4f, %.4f, %.4f, %.4f]  "
            "elbow_ref_vec=[%.3f, %.3f, %.3f]",
            arm,
            base_T_ee.pose.position.x, base_T_ee.pose.position.y,
            base_T_ee.pose.position.z,
            base_T_ee.pose.orientation.x, base_T_ee.pose.orientation.y,
            base_T_ee.pose.orientation.z, base_T_ee.pose.orientation.w,
            base_v_elbow[0], base_v_elbow[1], base_v_elbow[2]);
        RCLCPP_INFO(logger,
            "Arm %zu: NEAR_DIR result=%s  refJoint=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
            arm, ikResultToString(dir_result),
            last_joint_deg_[arm][0], last_joint_deg_[arm][1], last_joint_deg_[arm][2],
            last_joint_deg_[arm][3], last_joint_deg_[arm][4], last_joint_deg_[arm][5],
            last_joint_deg_[arm][6]);
        if (dir_result == IKResult::kSuccess) {
            double j4 = ik.m_Output_RetJoint[3];
            RCLCPP_INFO(logger,
                "Arm %zu: NEAR_DIR solved [%s] (J4=%.2f deg, bound=%.2f deg)  "
                "retJoint=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
                arm, classify_j4(j4), j4, j4_bound_,
                ik.m_Output_RetJoint[0], ik.m_Output_RetJoint[1], ik.m_Output_RetJoint[2],
                ik.m_Output_RetJoint[3], ik.m_Output_RetJoint[4], ik.m_Output_RetJoint[5],
                ik.m_Output_RetJoint[6]);
        }
        first_ik_logged_[arm] = true;
    }

    if (dir_result == IKResult::kSuccess) {
        return IKResult::kSuccess;
    }

    // Attempt 2 (optional): NEAR_REF — relax elbow constraint, pick solution
    // nearest to current joint state
    IKResult ref_result = IKResult::kNoTarget;
    if (ik_fallback_near_ref_) {
        RCLCPP_WARN_THROTTLE(logger, *get_node()->get_clock(), 1000,
            "Arm %zu: NEAR_DIR failed (%s), falling back to NEAR_REF  "
            "target pos=[%.4f, %.4f, %.4f] quat=[%.4f, %.4f, %.4f, %.4f]  "
            "elbow_ref=[%.3f, %.3f, %.3f]  refJoint=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
            arm, ikResultToString(dir_result),
            base_T_ee.pose.position.x, base_T_ee.pose.position.y,
            base_T_ee.pose.position.z,
            base_T_ee.pose.orientation.x, base_T_ee.pose.orientation.y,
            base_T_ee.pose.orientation.z, base_T_ee.pose.orientation.w,
            base_v_elbow[0], base_v_elbow[1], base_v_elbow[2],
            last_joint_deg_[arm][0], last_joint_deg_[arm][1], last_joint_deg_[arm][2],
            last_joint_deg_[arm][3], last_joint_deg_[arm][4], last_joint_deg_[arm][5],
            last_joint_deg_[arm][6]);

        setup_common();
        ik.m_Input_IK_ZSPType = FX_PILOT_NSP_TYPES_NEAR_REF;

        ref_result = evaluate_ik();
        if (ref_result == IKResult::kSuccess) {
            double j4 = ik.m_Output_RetJoint[3];
            RCLCPP_WARN_THROTTLE(logger, *get_node()->get_clock(), 1000,
                "Arm %zu: NEAR_REF fallback solved [%s] (J4=%.2f deg, bound=%.2f deg)  "
                "retJoint=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
                arm, classify_j4(j4), j4, j4_bound_,
                ik.m_Output_RetJoint[0], ik.m_Output_RetJoint[1], ik.m_Output_RetJoint[2],
                ik.m_Output_RetJoint[3], ik.m_Output_RetJoint[4], ik.m_Output_RetJoint[5],
                ik.m_Output_RetJoint[6]);
            return IKResult::kSuccess;
        }
        RCLCPP_WARN_THROTTLE(logger, *get_node()->get_clock(), 1000,
            "Arm %zu: NEAR_REF also failed (%s)", arm, ikResultToString(ref_result));
    }

    // Attempt 3 (optional): clamp to joint limits if the best attempt had a
    // solution that only exceeded limits.
    // Use whichever attempt last produced a kJointLimitExceeded result;
    // its data is still in the ik struct.
    IKResult clamp_candidate = ik_fallback_near_ref_ ? ref_result : dir_result;
    if (ik_clamp_joint_limits_ &&
        clamp_candidate == IKResult::kJointLimitExceeded) {
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            double val = ik.m_Output_RetJoint[j];
            double lo  = ik.m_Output_RunLmtN[j];
            double hi  = ik.m_Output_RunLmtP[j];
            out_q_joints_rad[j] = std::clamp(val, lo, hi) * kDeg2Rad;
        }
        double j4_clamped = std::clamp(ik.m_Output_RetJoint[3],
                                       ik.m_Output_RunLmtN[3],
                                       ik.m_Output_RunLmtP[3]);
        RCLCPP_WARN_THROTTLE(logger, *get_node()->get_clock(), 2000,
            "Arm %zu: clamped to joint limits [%s] (J4=%.2f deg, bound=%.2f deg, "
            "exceedance=%.1f deg)",
            arm, classify_j4(j4_clamped), j4_clamped, j4_bound_,
            ik.m_Output_JntExdABS);
        return IKResult::kJointLimitClamped;
    }

    RCLCPP_WARN_THROTTLE(logger, *get_node()->get_clock(), 1000,
        "Arm %zu: IK all attempts failed — NEAR_DIR: %s%s%s  "
        "target pos=[%.4f, %.4f, %.4f] quat=[%.4f, %.4f, %.4f, %.4f]  "
        "elbow_ref=[%.3f, %.3f, %.3f]  refJoint=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
        arm, ikResultToString(dir_result),
        ik_fallback_near_ref_ ? ", NEAR_REF: " : "",
        ik_fallback_near_ref_ ? ikResultToString(ref_result) : "",
        base_T_ee.pose.position.x, base_T_ee.pose.position.y,
        base_T_ee.pose.position.z,
        base_T_ee.pose.orientation.x, base_T_ee.pose.orientation.y,
        base_T_ee.pose.orientation.z, base_T_ee.pose.orientation.w,
        base_v_elbow[0], base_v_elbow[1], base_v_elbow[2],
        last_joint_deg_[arm][0], last_joint_deg_[arm][1], last_joint_deg_[arm][2],
        last_joint_deg_[arm][3], last_joint_deg_[arm][4], last_joint_deg_[arm][5],
        last_joint_deg_[arm][6]);
    return dir_result;
}

const char *TrackerTeleopController::ikResultToString(IKResult result)
{
    switch (result) {
        case IKResult::kSuccess:             return "IK solved successfully";
        case IKResult::kNoTarget:            return "No target";
        case IKResult::kJointLimitClamped:   return "IK solved (clamped to joint limits)";
        case IKResult::kInvalidQuaternion:   return "Invalid quaternion";
        case IKResult::kOutOfRange:          return "Target out of reachable workspace";
        case IKResult::kJointLimitExceeded:  return "IK solution exceeds joint limits";
        case IKResult::kSingularity:         return "IK solution near singularity";
        default:                             return "Unknown IK error";
    }
}

void TrackerTeleopController::publishIKStatus(size_t arm, IKResult result)
{
    if (arm >= kArmCount || !pub_ik_status_[arm]) return;
    if (result == last_ik_result_[arm]) return;
    last_ik_result_[arm] = result;

    std_msgs::msg::String msg;
    const bool ok = (result == IKResult::kSuccess);
    msg.data = std::string(ok ? "[OK] " : "[FAIL] ") + ikResultToString(result);
    pub_ik_status_[arm]->publish(msg);
}

void TrackerTeleopController::computeAndPublishFK()
{
    const auto logger = get_node()->get_logger();
    static const char *arm_labels[] = {"LEFT", "RIGHT"};

    for (size_t arm = 0; arm < kArmCount; ++arm) {
        FX_DOUBLE joints[kJointsPerArm];
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            joints[j] = last_joint_deg_[arm][j];
        }

        Matrix4 tcp_mat;
        if (!FX_Robot_Kine_FK(static_cast<FX_INT32L>(arm), joints, tcp_mat)) {
            RCLCPP_WARN(logger, "FK failed for arm %s", arm_labels[arm]);
            continue;
        }

        geometry_msgs::msg::PoseStamped base_T_tcp;
        base_T_tcp.header.frame_id = base_frame_;
        base_T_tcp.header.stamp = get_node()->get_clock()->now();
        matrix4ToPose(tcp_mat, base_T_tcp);
        pub_current_pose_[arm]->publish(base_T_tcp);

        RCLCPP_INFO(logger,
            "Arm %s FK base_T_tcp: pos=(%.4f, %.4f, %.4f) m  quat=(%.4f, %.4f, %.4f, %.4f)",
            arm_labels[arm],
            base_T_tcp.pose.position.x, base_T_tcp.pose.position.y, base_T_tcp.pose.position.z,
            base_T_tcp.pose.orientation.x, base_T_tcp.pose.orientation.y,
            base_T_tcp.pose.orientation.z, base_T_tcp.pose.orientation.w);
    }
}

static tf2::Transform readRigidTransform(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
    const std::string &prefix)
{
    std::vector<double> pos_vec, ori_vec;
    node->get_parameter(prefix + ".position", pos_vec);
    node->get_parameter(prefix + ".orientation", ori_vec);

    tf2::Vector3 t(0, 0, 0);
    tf2::Quaternion q(0, 0, 0, 1);

    if (pos_vec.size() >= 3) {
        t.setValue(pos_vec[0], pos_vec[1], pos_vec[2]);
    }
    if (ori_vec.size() >= 4) {
        q.setValue(ori_vec[0], ori_vec[1], ori_vec[2], ori_vec[3]);
    }
    return tf2::Transform(q, t);
}

controller_interface::CallbackReturn
TrackerTeleopController::on_configure(const rclcpp_lifecycle::State &)
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

    if (!get_node()->get_parameter("kine_config_path", kine_config_path_) ||
        kine_config_path_.empty()) {
        RCLCPP_ERROR(logger, "Missing or empty parameter 'kine_config_path'.");
        return CallbackReturn::ERROR;
    }
    if (!initMarvinKinematics(kine_config_path_, kine_data_, logger)) {
        RCLCPP_ERROR(logger, "Kinematics initialization failed.");
        return CallbackReturn::ERROR;
    }
    kine_initialized_ = true;

    get_node()->get_parameter("position_scale", position_scale_);
    get_node()->get_parameter("enable_orientation", enable_orientation_);
    get_node()->get_parameter("base_frame", base_frame_);
    get_node()->get_parameter("zsp_angle", zsp_angle_);
    get_node()->get_parameter("j4_bound", j4_bound_);
    get_node()->get_parameter("smoothing_alpha", smoothing_alpha_);
    get_node()->get_parameter("max_joint_velocity", max_joint_velocity_);
    get_node()->get_parameter("base_x_scale", base_x_scale_);
    get_node()->get_parameter("ik_fallback_near_ref", ik_fallback_near_ref_);
    get_node()->get_parameter("ik_clamp_joint_limits", ik_clamp_joint_limits_);

    smoothing_alpha_ = std::clamp(smoothing_alpha_, 0.01, 1.0);

    std::vector<double> elbow_dir_param;
    if (get_node()->get_parameter("default_elbow_direction", elbow_dir_param) &&
        elbow_dir_param.size() >= 3) {
        base_v_elbow_default_ = {{elbow_dir_param[0], elbow_dir_param[1], elbow_dir_param[2]}};
    }

    static const char *side_labels[] = {"left", "right"};
    static const char *side_tags[]   = {"LEFT", "RIGHT"};
    struct {
        const char *name;
        std::array<tf2::Transform, kArmCount> &arr;
    } param_groups[] = {
        {"base_T_chest", base_T_chest_},
        {"wrist_T_ee", wrist_T_ee_},
        {"arm_human_T_arm_robot", arm_human_T_arm_robot_},
    };
    for (auto &[name, arr] : param_groups) {
        for (int i = 0; i < static_cast<int>(kArmCount); ++i) {
            arr[i] = readRigidTransform(get_node(),
                                        std::string(name) + "." + side_labels[i]);
            const auto &t = arr[i].getOrigin();
            const auto &q = arr[i].getRotation();
            RCLCPP_INFO(logger,
                        "%s %s: t=[%.3f, %.3f, %.3f] quat=[%.4f, %.4f, %.4f, %.4f]",
                        name, side_tags[i],
                        t.x(), t.y(), t.z(),
                        q.x(), q.y(), q.z(), q.w());
        }
    }

    RCLCPP_INFO(logger, "Default base_v_elbow: [%.2f, %.2f, %.2f]",
                base_v_elbow_default_[0], base_v_elbow_default_[1], base_v_elbow_default_[2]);
    RCLCPP_INFO(logger, "ZSP angle: %.2f deg, J4 bound: %.2f deg", zsp_angle_, j4_bound_);
    RCLCPP_INFO(logger, "Smoothing: alpha=%.3f, max_vel=%.2f rad/s",
                smoothing_alpha_, max_joint_velocity_);
    RCLCPP_INFO(logger, "Base X scale: %.3f", base_x_scale_);
    RCLCPP_INFO(logger, "IK fallback NEAR_REF: %s, clamp joint limits: %s",
                ik_fallback_near_ref_ ? "ON" : "OFF",
                ik_clamp_joint_limits_ ? "ON" : "OFF");

    for (auto &arm : last_joint_deg_) arm.fill(0.0);
    for (auto &arm : smoothed_joints_rad_) arm.fill(0.0);
    cmd_interfaces_.fill(nullptr);
    state_interfaces_pos_.fill(nullptr);

    pub_ik_status_[0] = get_node()->create_publisher<std_msgs::msg::String>(
        "~/ik_status_left", rclcpp::SystemDefaultsQoS());
    pub_ik_status_[1] = get_node()->create_publisher<std_msgs::msg::String>(
        "~/ik_status_right", rclcpp::SystemDefaultsQoS());

    auto fk_qos = rclcpp::QoS(1).transient_local();
    pub_current_pose_[0] = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/current_pose_left", fk_qos);
    pub_current_pose_[1] = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/current_pose_right", fk_qos);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_node()->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
        *tf_buffer_, get_node(), false);

    tf_poll_timer_ = get_node()->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&TrackerTeleopController::pollTfCallback, this));

    get_node()->get_parameter("tracker_frames.torso", frame_torso_);
    get_node()->get_parameter("tracker_frames.left_hand", frame_left_hand_);
    get_node()->get_parameter("tracker_frames.right_hand", frame_right_hand_);
    get_node()->get_parameter("tracker_frames.left_upper_arm", frame_left_upper_arm_);
    get_node()->get_parameter("tracker_frames.right_upper_arm", frame_right_upper_arm_);

    RCLCPP_INFO(logger, "TF frames: torso=%s, L_hand=%s, R_hand=%s, L_arm=%s, R_arm=%s",
                frame_torso_.c_str(), frame_left_hand_.c_str(), frame_right_hand_.c_str(),
                frame_left_upper_arm_.c_str(), frame_right_upper_arm_.c_str());

    RCLCPP_INFO(logger, "TrackerTeleopController configured (scale=%.2f, orientation=%s)",
                position_scale_, enable_orientation_ ? "ON" : "OFF");
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TrackerTeleopController::on_activate(const rclcpp_lifecycle::State &)
{
    const auto logger = get_node()->get_logger();

    for (size_t i = 0; i < kTotalJoints; ++i) {
        const std::string name = joint_names_[i] + "/position";

        auto *cmd = find_loaned_interface(command_interfaces_, name);
        if (!cmd) {
            RCLCPP_ERROR(logger, "Missing command interface '%s'.", name.c_str());
            return CallbackReturn::ERROR;
        }
        cmd_interfaces_[i] = cmd;

        auto *state = find_loaned_interface(state_interfaces_, name);
        if (!state) {
            RCLCPP_ERROR(logger, "Missing state interface '%s'.", name.c_str());
            return CallbackReturn::ERROR;
        }
        state_interfaces_pos_[i] = state;
    }

    for (size_t i = 0; i < kTotalJoints; ++i) {
        const size_t arm = i / kJointsPerArm;
        const size_t j = i % kJointsPerArm;
        const auto pos_opt = state_interfaces_pos_[i]->get_optional<double>();
        const double pos_rad = pos_opt.has_value() ? pos_opt.value() : 0.0;
        last_joint_deg_[arm][j] = pos_rad * kRad2Deg;
        smoothed_joints_rad_[arm][j] = pos_rad;
        target_joints_rad_[arm][j] = pos_rad;
        (void)cmd_interfaces_[i]->set_value(pos_rad);
    }

    has_valid_target_.fill(false);
    first_ik_logged_.fill(false);
    last_hand_tf_stamp_.fill(rclcpp::Time(0, 0, RCL_ROS_TIME));

    computeAndPublishFK();

    RCLCPP_INFO(logger, "TrackerTeleopController activated (absolute mapping mode).");
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TrackerTeleopController::on_deactivate(const rclcpp_lifecycle::State &)
{
    cmd_interfaces_.fill(nullptr);
    state_interfaces_pos_.fill(nullptr);
    return CallbackReturn::SUCCESS;
}

/*
 * 位姿链（与 TF 一致，均为 ROS 系）：
 *   vr_T_tracker → corrections.yaml → ros_T_world_tracker (tracker_publisher)
 *   chest_T_wrist = (ros_T_chest)^{-1} * ros_T_wrist  [TF: torso_T_hand]
 *   base_T_wrist  = base_T_chest * chest_T_wrist
 *   base_T_ee     = base_T_wrist * wrist_T_ee
 *   base_R_arm_robot = base_R_chest * chest_R_arm * arm_human_R_arm_robot
 *   base_v_elbow = Y 列 of base_R_arm_robot → IK 零空间
 *
 * RT safety:
 *   - TF lookups run in a non-RT timer (pollTfCallback, 200 Hz).
 *   - update() only does try_lock (never blocks) to grab cached TF.
 *   - Smoothing runs every cycle (1 kHz) so motion stays fluid even
 *     when IK fails or no new tracker data arrives.
 */
controller_interface::return_type
TrackerTeleopController::update(const rclcpp::Time &, const rclcpp::Duration &period)
{
    if (!kine_initialized_) {
        return controller_interface::return_type::OK;
    }

    const auto logger = get_node()->get_logger();
    const double dt = period.seconds();

    if (tf_cache_mutex_.try_lock()) {
        tf_snapshot_ = tf_cache_;
        tf_cache_mutex_.unlock();
    }

    for (int arm = 0; arm < static_cast<int>(kArmCount); ++arm) {

        const auto &snap = tf_snapshot_[arm];

        if (snap.hand_valid) {
            rclcpp::Time stamp(snap.chest_T_hand.header.stamp);

            if (stamp != last_hand_tf_stamp_[arm]) {
                last_hand_tf_stamp_[arm] = stamp;

                const tf2::Transform &base_T_chest = base_T_chest_[arm];
                const auto &tf_wrist = snap.chest_T_hand.transform;

                tf2::Transform chest_T_wrist(
                    tf2::Quaternion(tf_wrist.rotation.x, tf_wrist.rotation.y,
                                   tf_wrist.rotation.z, tf_wrist.rotation.w),
                    tf2::Vector3(tf_wrist.translation.x * position_scale_,
                                tf_wrist.translation.y * position_scale_,
                                tf_wrist.translation.z * position_scale_));

                if (!enable_orientation_) {
                    chest_T_wrist.setRotation(tf2::Quaternion::getIdentity());
                }

                tf2::Transform base_T_wrist = base_T_chest * chest_T_wrist;
                tf2::Transform base_T_ee_tf = base_T_wrist * wrist_T_ee_[arm];

                // Scale x-offset relative to the chest anchor in base frame
                tf2::Vector3 ee_pos = base_T_ee_tf.getOrigin();
                double offset_x = ee_pos.x() - base_T_chest.getOrigin().x();
                ee_pos.setX(base_T_chest.getOrigin().x() + offset_x * base_x_scale_);
                base_T_ee_tf.setOrigin(ee_pos);

                geometry_msgs::msg::PoseStamped base_T_ee;
                base_T_ee.header.frame_id = base_frame_;
                base_T_ee.pose.position.x = base_T_ee_tf.getOrigin().x();
                base_T_ee.pose.position.y = base_T_ee_tf.getOrigin().y();
                base_T_ee.pose.position.z = base_T_ee_tf.getOrigin().z();
                const tf2::Quaternion &ee_q = base_T_ee_tf.getRotation();
                base_T_ee.pose.orientation.x = ee_q.x();
                base_T_ee.pose.orientation.y = ee_q.y();
                base_T_ee.pose.orientation.z = ee_q.z();
                base_T_ee.pose.orientation.w = ee_q.w();

                double base_v_elbow[3] = {
                    base_v_elbow_default_[0],
                    base_v_elbow_default_[1],
                    base_v_elbow_default_[2]};

                if (snap.arm_valid) {
                    const auto &tf_arm = snap.chest_T_arm.transform;
                    tf2::Quaternion chest_R_arm(
                        tf_arm.rotation.x, tf_arm.rotation.y,
                        tf_arm.rotation.z, tf_arm.rotation.w);
                    tf2::Quaternion base_R_arm_robot =
                        base_T_chest.getRotation() * chest_R_arm *
                        arm_human_T_arm_robot_[arm].getRotation();

                    tf2::Matrix3x3 base_M_arm_robot(base_R_arm_robot);
                    tf2::Vector3 y_col = base_M_arm_robot.getColumn(1);
                    base_v_elbow[0] = y_col.x();
                    base_v_elbow[1] = y_col.y();
                    base_v_elbow[2] = y_col.z();

                    if (!first_ik_logged_[arm]) {
                        RCLCPP_INFO(logger,
                            "Arm %d elbow dir: base_v_elbow=[%.3f, %.3f, %.3f]  "
                            "chest_R_arm quat=[%.3f, %.3f, %.3f, %.3f]",
                            arm,
                            base_v_elbow[0], base_v_elbow[1], base_v_elbow[2],
                            tf_arm.rotation.x, tf_arm.rotation.y,
                            tf_arm.rotation.z, tf_arm.rotation.w);
                    }
                }

                if (!first_ik_logged_[arm]) {
                    RCLCPP_INFO(logger,
                        "Arm %d IK target: base_T_ee pos=[%.4f, %.4f, %.4f]  "
                        "quat=[%.4f, %.4f, %.4f, %.4f]  "
                        "base_v_elbow=[%.3f, %.3f, %.3f]",
                        arm,
                        base_T_ee.pose.position.x, base_T_ee.pose.position.y,
                        base_T_ee.pose.position.z,
                        base_T_ee.pose.orientation.x, base_T_ee.pose.orientation.y,
                        base_T_ee.pose.orientation.z, base_T_ee.pose.orientation.w,
                        base_v_elbow[0], base_v_elbow[1], base_v_elbow[2]);
                }

                double out_q_joints_rad[kJointsPerArm];
                IKResult ik_result = solveIK(arm, base_T_ee, base_v_elbow,
                                             out_q_joints_rad);
                publishIKStatus(arm, ik_result);

                if (ik_result == IKResult::kSuccess ||
                    ik_result == IKResult::kJointLimitClamped) {
                    for (size_t j = 0; j < kJointsPerArm; ++j) {
                        target_joints_rad_[arm][j] = out_q_joints_rad[j];
                    }
                    has_valid_target_[arm] = true;
                }
            }
        }

        if (!has_valid_target_[arm]) {
            continue;
        }

        const size_t offset = arm * kJointsPerArm;
        const double max_delta = max_joint_velocity_ * dt;

        for (size_t j = 0; j < kJointsPerArm; ++j) {
            double filtered = smoothing_alpha_ * target_joints_rad_[arm][j] +
                              (1.0 - smoothing_alpha_) * smoothed_joints_rad_[arm][j];

            double delta = filtered - smoothed_joints_rad_[arm][j];
            if (std::abs(delta) > max_delta) {
                filtered = smoothed_joints_rad_[arm][j] +
                           std::copysign(max_delta, delta);
            }

            smoothed_joints_rad_[arm][j] = filtered;
            (void)cmd_interfaces_[offset + j]->set_value(filtered);
            last_joint_deg_[arm][j] = filtered * kRad2Deg;
        }
    }

    return controller_interface::return_type::OK;
}

}  // namespace ros2_control_marvin

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ros2_control_marvin::TrackerTeleopController,
                       controller_interface::ControllerInterface)
