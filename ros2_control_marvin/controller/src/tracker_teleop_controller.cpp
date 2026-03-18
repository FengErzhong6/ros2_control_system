#include "ros2_control_marvin/tracker_teleop_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>

#include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "tf2/exceptions.h"

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

void quatMultiply(const geometry_msgs::msg::Quaternion &a,
                  const geometry_msgs::msg::Quaternion &b,
                  geometry_msgs::msg::Quaternion &out_R)
{
    out_R.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    out_R.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    out_R.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    out_R.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
}

void quatToR(const geometry_msgs::msg::Quaternion &q, double (&out_R)[3][3])
{
    out_R[0][0] = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    out_R[0][1] = 2.0 * (q.x * q.y - q.z * q.w);
    out_R[0][2] = 2.0 * (q.x * q.z + q.y * q.w);
    out_R[1][0] = 2.0 * (q.x * q.y + q.z * q.w);
    out_R[1][1] = 1.0 - 2.0 * (q.x * q.x + q.z * q.z);
    out_R[1][2] = 2.0 * (q.y * q.z - q.x * q.w);
    out_R[2][0] = 2.0 * (q.x * q.z - q.y * q.w);
    out_R[2][1] = 2.0 * (q.y * q.z + q.x * q.w);
    out_R[2][2] = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
}

void quatRotateVector(const geometry_msgs::msg::Quaternion &R_q,
                      const double (&v)[3],
                      double (&out_v)[3])
{
    double t0 = 2.0 * (R_q.y * v[2] - R_q.z * v[1]);
    double t1 = 2.0 * (R_q.z * v[0] - R_q.x * v[2]);
    double t2 = 2.0 * (R_q.x * v[1] - R_q.y * v[0]);
    out_v[0] = v[0] + R_q.w * t0 + (R_q.y * t2 - R_q.z * t1);
    out_v[1] = v[1] + R_q.w * t1 + (R_q.z * t0 - R_q.x * t2);
    out_v[2] = v[2] + R_q.w * t2 + (R_q.x * t1 - R_q.y * t0);
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

        auto_declare<double>("smoothing_alpha", 0.3);
        auto_declare<double>("max_joint_velocity", 2.0);

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
    std::memset(&ik, 0, sizeof(FX_InvKineSolvePara));

    poseToMatrix4(base_T_ee, ik.m_Input_IK_TargetTCP);

    for (size_t j = 0; j < kJointsPerArm; ++j) {
        ik.m_Input_IK_RefJoint[j] = last_joint_deg_[arm][j];
    }

    ik.m_Input_IK_ZSPType = FX_PILOT_NSP_TYPES_NEAR_DIR;
    ik.m_Input_IK_ZSPPara[0] = base_v_elbow[0];
    ik.m_Input_IK_ZSPPara[1] = base_v_elbow[1];
    ik.m_Input_IK_ZSPPara[2] = base_v_elbow[2];
    ik.m_Input_ZSP_Angle = zsp_angle_;

    FX_INT32L serial = static_cast<FX_INT32L>(arm);
    if (!FX_Robot_Kine_IK(serial, &ik)) {
        RCLCPP_WARN_THROTTLE(logger, *get_node()->get_clock(), 1000,
                             "Arm %zu: IK failed (out_of_range=%d)",
                             arm, static_cast<int>(ik.m_Output_IsOutRange));
        return IKResult::kOutOfRange;
    }

    if (ik.m_Output_IsJntExd) {
        RCLCPP_WARN_THROTTLE(logger, *get_node()->get_clock(), 1000,
                             "Arm %zu: joint limits exceeded (max=%.2f deg)",
                             arm, ik.m_Output_JntExdABS);
        return IKResult::kJointLimitExceeded;
    }

    for (size_t j = 0; j < kJointsPerArm; ++j) {
        if (ik.m_Output_IsDeg[j]) {
            RCLCPP_WARN_THROTTLE(logger, *get_node()->get_clock(), 1000,
                                 "Arm %zu: near singularity at J%zu", arm, j + 1);
            return IKResult::kSingularity;
        }
    }

    for (size_t j = 0; j < kJointsPerArm; ++j) {
        out_q_joints_rad[j] = ik.m_Output_RetJoint[j] * kDeg2Rad;
    }

    return IKResult::kSuccess;
}

const char *TrackerTeleopController::ikResultToString(IKResult result)
{
    switch (result) {
        case IKResult::kSuccess:             return "IK solved successfully";
        case IKResult::kNoTarget:            return "No target";
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

static void readRigidTransform(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
    const std::string &prefix,
    TrackerTeleopController::RigidTransform &out_T)
{
    std::vector<double> pos_vec, ori_vec;
    node->get_parameter(prefix + ".position", pos_vec);
    node->get_parameter(prefix + ".orientation", ori_vec);
    if (pos_vec.size() >= 3) {
        out_T.t[0] = pos_vec[0];
        out_T.t[1] = pos_vec[1];
        out_T.t[2] = pos_vec[2];
    }
    if (ori_vec.size() >= 4) {
        out_T.q.x = ori_vec[0];
        out_T.q.y = ori_vec[1];
        out_T.q.z = ori_vec[2];
        out_T.q.w = ori_vec[3];
    }
    quatToR(out_T.q, out_T.R);
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
    get_node()->get_parameter("smoothing_alpha", smoothing_alpha_);
    get_node()->get_parameter("max_joint_velocity", max_joint_velocity_);

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
        std::array<RigidTransform, kArmCount> &arr;
    } param_groups[] = {
        {"base_T_chest", base_T_chest_},
        {"wrist_T_ee", wrist_T_ee_},
        {"arm_human_T_arm_robot", arm_human_T_arm_robot_},
    };
    for (auto &[name, arr] : param_groups) {
        for (int i = 0; i < static_cast<int>(kArmCount); ++i) {
            readRigidTransform(get_node(),
                               std::string(name) + "." + side_labels[i], arr[i]);
            RCLCPP_INFO(logger,
                        "%s %s: t=[%.3f, %.3f, %.3f] quat=[%.4f, %.4f, %.4f, %.4f]",
                        name, side_tags[i],
                        arr[i].t[0], arr[i].t[1], arr[i].t[2],
                        arr[i].q.x, arr[i].q.y, arr[i].q.z, arr[i].q.w);
        }
    }

    RCLCPP_INFO(logger, "Default base_v_elbow: [%.2f, %.2f, %.2f]",
                base_v_elbow_default_[0], base_v_elbow_default_[1], base_v_elbow_default_[2]);
    RCLCPP_INFO(logger, "ZSP angle: %.2f deg", zsp_angle_);
    RCLCPP_INFO(logger, "Smoothing: alpha=%.3f, max_vel=%.2f rad/s",
                smoothing_alpha_, max_joint_velocity_);

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
        (void)cmd_interfaces_[i]->set_value(pos_rad);
    }

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
 */
controller_interface::return_type
TrackerTeleopController::update(const rclcpp::Time &, const rclcpp::Duration &period)
{
    if (!kine_initialized_) {
        return controller_interface::return_type::OK;
    }

    const double dt = period.seconds();
    const std::string *frame_hand[] = {&frame_left_hand_, &frame_right_hand_};
    const std::string *frame_arm[] = {&frame_left_upper_arm_, &frame_right_upper_arm_};

    for (int arm = 0; arm < static_cast<int>(kArmCount); ++arm) {
        geometry_msgs::msg::TransformStamped chest_T_wrist;
        if (!lookupTf(frame_torso_, *frame_hand[arm], chest_T_wrist)) {
            continue;
        }

        rclcpp::Time stamp(chest_T_wrist.header.stamp);
        if (stamp == last_hand_tf_stamp_[arm]) {
            continue;
        }
        last_hand_tf_stamp_[arm] = stamp;

        const RigidTransform &base_T_chest = base_T_chest_[arm];
        const auto &chest_t_wrist = chest_T_wrist.transform.translation;
        const auto &chest_R_wrist = chest_T_wrist.transform.rotation;

        double chest_t_wrist_scaled[3] = {
            chest_t_wrist.x * position_scale_,
            chest_t_wrist.y * position_scale_,
            chest_t_wrist.z * position_scale_,
        };

        geometry_msgs::msg::PoseStamped base_T_ee;
        base_T_ee.header.frame_id = base_frame_;
        base_T_ee.pose.position.x = base_T_chest.t[0]
            + base_T_chest.R[0][0] * chest_t_wrist_scaled[0]
            + base_T_chest.R[0][1] * chest_t_wrist_scaled[1]
            + base_T_chest.R[0][2] * chest_t_wrist_scaled[2];
        base_T_ee.pose.position.y = base_T_chest.t[1]
            + base_T_chest.R[1][0] * chest_t_wrist_scaled[0]
            + base_T_chest.R[1][1] * chest_t_wrist_scaled[1]
            + base_T_chest.R[1][2] * chest_t_wrist_scaled[2];
        base_T_ee.pose.position.z = base_T_chest.t[2]
            + base_T_chest.R[2][0] * chest_t_wrist_scaled[0]
            + base_T_chest.R[2][1] * chest_t_wrist_scaled[1]
            + base_T_chest.R[2][2] * chest_t_wrist_scaled[2];

        geometry_msgs::msg::Quaternion base_R_wrist;
        if (enable_orientation_) {
            quatMultiply(base_T_chest.q, chest_R_wrist, base_R_wrist);
        } else {
            base_R_wrist = base_T_chest.q;
        }

        const RigidTransform &wrist_T_ee = wrist_T_ee_[arm];
        double wrist_T_ee_t_in_base[3];
        quatRotateVector(base_R_wrist, wrist_T_ee.t, wrist_T_ee_t_in_base);
        base_T_ee.pose.position.x += wrist_T_ee_t_in_base[0];
        base_T_ee.pose.position.y += wrist_T_ee_t_in_base[1];
        base_T_ee.pose.position.z += wrist_T_ee_t_in_base[2];

        geometry_msgs::msg::Quaternion base_R_ee;
        quatMultiply(base_R_wrist, wrist_T_ee.q, base_R_ee);
        base_T_ee.pose.orientation = base_R_ee;

        double base_v_elbow[3] = {
            base_v_elbow_default_[0], base_v_elbow_default_[1], base_v_elbow_default_[2]};

        geometry_msgs::msg::TransformStamped chest_T_arm;
        if (lookupTf(frame_torso_, *frame_arm[arm], chest_T_arm)) {
            geometry_msgs::msg::Quaternion base_R_chest_arm;
            quatMultiply(base_T_chest.q, chest_T_arm.transform.rotation, base_R_chest_arm);
            geometry_msgs::msg::Quaternion base_R_arm_robot;
            quatMultiply(base_R_chest_arm, arm_human_T_arm_robot_[arm].q, base_R_arm_robot);

            base_v_elbow[0] = 2.0 * (base_R_arm_robot.x * base_R_arm_robot.y
                                     + base_R_arm_robot.w * base_R_arm_robot.z);
            base_v_elbow[1] = 1.0 - 2.0 * (base_R_arm_robot.x * base_R_arm_robot.x
                                           + base_R_arm_robot.z * base_R_arm_robot.z);
            base_v_elbow[2] = 2.0 * (base_R_arm_robot.y * base_R_arm_robot.z
                                     - base_R_arm_robot.w * base_R_arm_robot.x);
        }

        double out_q_joints_rad[kJointsPerArm];
        IKResult ik_result = solveIK(arm, base_T_ee, base_v_elbow, out_q_joints_rad);
        publishIKStatus(arm, ik_result);

        if (ik_result != IKResult::kSuccess) {
            continue;
        }

        const size_t offset = arm * kJointsPerArm;
        const double max_delta = max_joint_velocity_ * dt;

        for (size_t j = 0; j < kJointsPerArm; ++j) {
            double filtered = smoothing_alpha_ * out_q_joints_rad[j] +
                              (1.0 - smoothing_alpha_) * smoothed_joints_rad_[arm][j];

            double delta = filtered - smoothed_joints_rad_[arm][j];
            if (std::abs(delta) > max_delta) {
                filtered = smoothed_joints_rad_[arm][j] + std::copysign(max_delta, delta);
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
