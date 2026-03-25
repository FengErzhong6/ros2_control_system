#ifndef ROS2_CONTROL_MARVIN__CONTROLLER__IK_CONTROLLER_HPP_
#define ROS2_CONTROL_MARVIN__CONTROLLER__IK_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_msgs/msg/string.hpp"

#include "FxRobot.h"

namespace marvin_system {

class IKController : public controller_interface::ControllerInterface {
public:
    IKController();
    ~IKController() override;

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
    static constexpr size_t kJointsPerArm = 7;
    static constexpr size_t kArmCount = 2;
    static constexpr size_t kTotalJoints = kJointsPerArm * kArmCount;
    static constexpr double kDeg2Rad = 0.01745329251994329576923690768489;
    static constexpr double kRad2Deg = 57.295779513082320876798154814105;
    static constexpr double kMm2M = 0.001;
    static constexpr double kM2Mm = 1000.0;

    std::vector<std::string> joint_names_;

    // Loaned interface pointers (avoid name lookups in update)
    std::array<hardware_interface::LoanedCommandInterface *, kTotalJoints> cmd_interfaces_{};
    std::array<hardware_interface::LoanedStateInterface *, kTotalJoints> state_interfaces_pos_{};

    // Marvin kinematics SDK data
    bool kine_initialized_{false};
    std::string kine_config_path_;
    FX_INT32L kine_type_[2]{};
    FX_DOUBLE kine_grv_[2][3]{};
    FX_DOUBLE kine_dh_[2][8][4]{};
    FX_DOUBLE kine_pnva_[2][7][4]{};
    FX_DOUBLE kine_bd_[2][4][3]{};
    FX_DOUBLE kine_mass_[2][7]{};
    FX_DOUBLE kine_mcp_[2][7][3]{};
    FX_DOUBLE kine_inertia_[2][7][6]{};

    // Target pose subscription (one per arm)
    struct ArmTarget {
        std::mutex mtx;
        bool has_target{false};
        geometry_msgs::msg::PoseStamped pose;
    };
    std::array<ArmTarget, kArmCount> arm_targets_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_target_left_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_target_right_;

    // IK status feedback publishers (one per arm)
    std::array<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr, kArmCount> pub_ik_status_;

    // FK-derived current pose publishers (transient_local, published on activate)
    std::array<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr, kArmCount>
        pub_current_pose_;

    // IK solve parameters
    std::array<FX_InvKineSolvePara, kArmCount> ik_params_{};

    // Last commanded joint angles (degrees, for IK reference)
    std::array<std::array<double, kJointsPerArm>, kArmCount> last_joint_deg_{};

    enum class IKResult : int8_t {
        kSuccess = 0,
        kNoTarget = 1,
        kInvalidQuaternion = -1,
        kOutOfRange = -2,
        kJointLimitExceeded = -3,
        kSingularity = -4,
    };

    // Per-arm status tracking (only publish on change)
    std::array<IKResult, kArmCount> last_ik_result_{
        {IKResult::kNoTarget, IKResult::kNoTarget}};

    bool initKinematics();
    IKResult solveIK(size_t arm_index, const geometry_msgs::msg::PoseStamped &target_pose,
                     double (&result_joints_rad)[kJointsPerArm]);
    void poseToMatrix4(const geometry_msgs::msg::PoseStamped &pose, Matrix4 mat);
    void matrix4ToPose(const Matrix4 mat, geometry_msgs::msg::PoseStamped &pose) const;
    void computeAndPublishFK();
    bool isQuaternionValid(const geometry_msgs::msg::Quaternion &q) const;
    void publishIKStatus(size_t arm_index, IKResult result);
    static const char *ikResultToString(IKResult result);
};

}  // namespace marvin_system

#endif  // ROS2_CONTROL_MARVIN__CONTROLLER__IK_CONTROLLER_HPP_
