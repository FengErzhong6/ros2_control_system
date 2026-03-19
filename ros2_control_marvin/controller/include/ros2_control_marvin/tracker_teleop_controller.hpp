#ifndef ROS2_CONTROL_MARVIN__CONTROLLER__TRACKER_TELEOP_CONTROLLER_HPP_
#define ROS2_CONTROL_MARVIN__CONTROLLER__TRACKER_TELEOP_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "ros2_control_marvin/marvin_kine_utils.hpp"

namespace ros2_control_marvin {

class TrackerTeleopController : public controller_interface::ControllerInterface {
public:
    TrackerTeleopController();
    ~TrackerTeleopController() override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

private:
    static constexpr int kLeft = 0;
    static constexpr int kRight = 1;

    std::vector<std::string> joint_names_;

    std::array<hardware_interface::LoanedCommandInterface *, kTotalJoints> cmd_interfaces_{};
    std::array<hardware_interface::LoanedStateInterface *, kTotalJoints> state_interfaces_pos_{};

    // Kinematics
    bool kine_initialized_{false};
    std::string kine_config_path_;
    MarvinKineData kine_data_;
    std::array<FX_InvKineSolvePara, kArmCount> ik_params_{};
    std::array<std::array<double, kJointsPerArm>, kArmCount> last_joint_deg_{};

    // TF2 for tracker pose lookup
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // TF cache: populated by non-RT timer, consumed lock-free by RT update()
    struct CachedTrackerData {
        geometry_msgs::msg::TransformStamped chest_T_hand;
        geometry_msgs::msg::TransformStamped chest_T_arm;
        bool hand_valid{false};
        bool arm_valid{false};
    };
    rclcpp::TimerBase::SharedPtr tf_poll_timer_;
    std::mutex tf_cache_mutex_;
    std::array<CachedTrackerData, kArmCount> tf_cache_;
    std::array<CachedTrackerData, kArmCount> tf_snapshot_;

    // Tracker TF frame names
    std::string frame_torso_;
    std::string frame_left_hand_;
    std::string frame_right_hand_;
    std::string frame_left_upper_arm_;
    std::string frame_right_upper_arm_;

    // IK status publishers
    std::array<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr, kArmCount> pub_ik_status_;
    std::array<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr, kArmCount>
        pub_current_pose_;

    // Parameters
    double position_scale_{1.0};
    double base_x_scale_{1.0};
    bool enable_orientation_{true};
    bool ik_fallback_near_ref_{true};
    bool ik_clamp_joint_limits_{true};
    std::string base_frame_{"base_link"};
    std::array<double, 3> base_v_elbow_default_{{0.0, 0.0, -1.0}};
    double zsp_angle_{0.0};
    double j4_bound_{0.0};

    /** chest → base (标定人机胸系到臂基). */
    std::array<tf2::Transform, kArmCount> base_T_chest_;
    /** wrist_human → ee (人手腕系 → TCP). */
    std::array<tf2::Transform, kArmCount> wrist_T_ee_;
    /** arm_human → arm_robot (人上臂系 → 肘向参考系，仅用旋转). */
    std::array<tf2::Transform, kArmCount> arm_human_T_arm_robot_;

    // Smoothing parameters (low-pass + velocity clamping)
    double smoothing_alpha_{0.3};
    double max_joint_velocity_{2.0};  // rad/s
    std::array<std::array<double, kJointsPerArm>, kArmCount> smoothed_joints_rad_{};

    // IK target: last successful solution (smoothing always converges to this)
    std::array<std::array<double, kJointsPerArm>, kArmCount> target_joints_rad_{};
    std::array<bool, kArmCount> has_valid_target_{{false, false}};
    std::array<bool, kArmCount> first_ik_logged_{{false, false}};

    // TF timestamp tracking: skip IK when tracker data hasn't changed
    std::array<rclcpp::Time, kArmCount> last_hand_tf_stamp_{
        {rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Time(0, 0, RCL_ROS_TIME)}};

    // IK result tracking
    enum class IKResult : int8_t {
        kSuccess = 0,
        kNoTarget = 1,
        kJointLimitClamped = 2,
        kInvalidQuaternion = -1,
        kOutOfRange = -2,
        kJointLimitExceeded = -3,
        kSingularity = -4,
    };
    std::array<IKResult, kArmCount> last_ik_result_{
        {IKResult::kNoTarget, IKResult::kNoTarget}};

    // Core methods
    void pollTfCallback();
    bool lookupTf(const std::string &target_frame,
                  const std::string &source_frame,
                  geometry_msgs::msg::TransformStamped &out_chest_T_source);
    IKResult solveIK(size_t arm, const geometry_msgs::msg::PoseStamped &base_T_ee,
                     const double (&base_v_elbow)[3],
                     double (&out_q_joints_rad)[kJointsPerArm]);
    void computeAndPublishFK();
    void publishIKStatus(size_t arm, IKResult result);
    static const char *ikResultToString(IKResult result);
};

}  // namespace ros2_control_marvin

#endif  // ROS2_CONTROL_MARVIN__CONTROLLER__TRACKER_TELEOP_CONTROLLER_HPP_
