#ifndef ROS2_CONTROL_MARVIN__CONTROLLER__TRACKER_TELEOP_CONTROLLER_HPP_
#define ROS2_CONTROL_MARVIN__CONTROLLER__TRACKER_TELEOP_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
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
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "TrackingIk.h"
#include "marvin_system/marvin_kine_utils.hpp"

namespace marvin_system {

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
    tracking_ik::Geometry tracking_ik_geometry_{};
    bool tracking_ik_geometry_loaded_{false};
    std::array<std::array<double, kJointsPerArm>, kArmCount> last_joint_deg_{};
    std::array<std::array<double, 3>, kArmCount> last_selected_ref_dir_{
        {{{0.0, 0.0, -1.0}}, {{0.0, 0.0, -1.0}}}};
    std::array<int64_t, kArmCount> last_selected_branch_{{-1, -1}};

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
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_viz_markers_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    std::mutex diagnostics_mutex_;

    // Parameters
    double position_scale_{1.0};
    double base_x_scale_{1.0};
    bool enable_orientation_{true};
    std::string base_frame_{"base_link"};
    std::array<double, 3> shoulder_v_elbow_default_{{0.0, 0.0, -1.0}};
    double j4_bound_{0.0};
    double dh_d1_{0.0};
    rclcpp::Duration tracker_timeout_{0, 100000000};
    std::array<std::string, kArmCount> viz_base_frames_{{"Base_L", "Base_R"}};
    struct TrackingIkConfig {
        double fk_accept_tol{1e-3};
        double fast_psi_range_deg{12.0};
        double fast_psi_step_deg{1.0};
        double expand_psi_range_deg{36.0};
        double expand_psi_step_deg{3.0};
        double desired_dir_weight{0.03};
        double continuity_dir_weight{0.03};
        double magnitude_weight{0.05};
        double psi_delta_weight{0.02};
        double branch_switch_penalty{20.0};
    } tracking_ik_config_{};

    /** chest → shoulder (标定人机胸系到肩关节系). */
    std::array<tf2::Transform, kArmCount> shoulder_T_chest_;
    /** wrist_human → ee (人手腕系 → TCP). */
    std::array<tf2::Transform, kArmCount> wrist_T_ee_;
    /** arm_human → arm_robot (人上臂系 → 肘向参考系，仅用旋转). */
    std::array<tf2::Transform, kArmCount> arm_human_T_arm_robot_;
    /** 肘向量方向修正旋转 (RPY→quaternion)，左右手分别配置. */
    std::array<tf2::Quaternion, kArmCount> elbow_dir_correction_{
        {tf2::Quaternion::getIdentity(), tf2::Quaternion::getIdentity()}};

    // Smoothing parameters (low-pass + velocity clamping)
    double smoothing_alpha_{0.3};
    double max_joint_velocity_{2.0};  // rad/s
    std::array<std::array<double, kJointsPerArm>, kArmCount> smoothed_joints_rad_{};

    // IK target: last successful solution (smoothing always converges to this)
    std::array<std::array<double, kJointsPerArm>, kArmCount> target_joints_rad_{};
    std::array<bool, kArmCount> has_valid_target_{{false, false}};
    std::array<std::array<double, kJointsPerArm>, kArmCount> home_joints_rad_{};
    bool has_home_joints_{false};
    double home_tolerance_rad_{0.5 * kDeg2Rad};

    // TF timestamp tracking: recompute IK when either hand or upper-arm input changes
    std::array<rclcpp::Time, kArmCount> last_hand_tf_stamp_{
        {rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Time(0, 0, RCL_ROS_TIME)}};
    std::array<rclcpp::Time, kArmCount> last_arm_tf_stamp_{
        {rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Time(0, 0, RCL_ROS_TIME)}};
    std::array<bool, kArmCount> last_hand_tf_valid_{{false, false}};
    std::array<bool, kArmCount> last_arm_tf_valid_{{false, false}};
    std::array<bool, kArmCount> last_hand_tf_fresh_{{false, false}};
    std::array<bool, kArmCount> last_arm_tf_fresh_{{false, false}};
    std::array<bool, kArmCount> tracker_fresh_{{false, false}};

    // IK result tracking
    enum class IKResult : int8_t {
        kSuccess = 0,
        kNoTarget = 1,
        kJointLimitClamped = 2,
        kInvalidQuaternion = -1,
        kOutOfRange = -2,
        kJointLimitExceeded = -3,
        kSingularity = -4,
        kSolveFailed = -5,
    };
    std::array<IKResult, kArmCount> last_ik_result_{
        {IKResult::kNoTarget, IKResult::kNoTarget}};
    std::array<bool, kArmCount> marker_visible_{{false, false}};

    enum class TeleopState : int8_t {
        kDisarmed = 0,
        kArmed = 1,
        kEnabled = 2,
    };
    std::atomic<int> teleop_state_{static_cast<int>(TeleopState::kDisarmed)};
    std::atomic<bool> force_tracker_reacquire_{false};
    std::atomic<bool> go_home_requested_{false};
    std::atomic<bool> go_home_active_{false};
    std::atomic<int> active_home_joint_index_{-1};
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_armed_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_enabled_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_go_home_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_teleop_state_;

    struct ArmDiagnostics {
        ArmDiagnostics() = default;
        bool pending{false};
        bool tracker_fresh{false};
        bool hand_valid{false};
        bool arm_valid{false};
        bool has_base_T_ee{false};
        geometry_msgs::msg::PoseStamped base_T_ee;
        std::array<double, 3> shoulder_v_elbow{{0.0, 0.0, -1.0}};
        IKResult dir_result{IKResult::kNoTarget};
        IKResult ref_result{IKResult::kNoTarget};
        IKResult final_result{IKResult::kNoTarget};
        bool used_near_ref{false};
        bool clamped{false};
        bool has_solution{false};
        std::array<double, kJointsPerArm> q_joints_rad{};
        double solution_j4_deg{0.0};
        bool solver_reachable{false};
        bool used_expanded_search{false};
        int64_t candidate_count{0};
        int64_t psi_eval_count{0};
        int64_t selected_branch{-1};
        double selected_psi_deg{0.0};
        double best_fk_residual_l1{0.0};
        double best_ref_score_l1{0.0};
        double best_desired_dir_score_deg{0.0};
        double best_continuity_dir_score_deg{0.0};
        bool solved_upper_arm_dir_valid{false};
        std::array<double, 3> solved_upper_arm_dir{{0.0, 0.0, 0.0}};
        double solved_upper_arm_dir_angle_deg{0.0};
    };
    struct PendingDiagnosticsSlot {
        std::atomic<uint64_t> sequence{0};
        std::atomic<bool> pending{false};
        ArmDiagnostics snapshot;
    };
    std::array<PendingDiagnosticsSlot, kArmCount> pending_diagnostics_;

    struct TrackerInputState {
        rclcpp::Time hand_stamp{0, 0, RCL_ROS_TIME};
        rclcpp::Time arm_stamp{0, 0, RCL_ROS_TIME};
        bool hand_changed{false};
        bool arm_changed{false};
        bool tracker_input_changed{false};
        bool hand_fresh{false};
        bool arm_fresh{false};
    };

    // Configuration helpers
    bool readJointNames();
    bool initializeKinematics();
    bool loadControllerParameters();
    void loadTransformParameters();
    void loadElbowCorrectionParameters();
    bool loadHomeJointParameters(const std::vector<double> &home_left,
                                 const std::vector<double> &home_right,
                                 double home_tolerance_deg);
    void loadTrackerFrameParameters();
    void logConfigurationSummary(double home_tolerance_deg) const;
    void resetRuntimeState();
    void createRosInterfaces();

    // Activation/runtime helpers
    bool bindJointInterfaces();
    void initializeJointTargetsFromState();
    bool seedTrackingStateFromCurrentJoints(size_t arm);
    void resetTrackerState();
    void resetTeleopRuntime(TeleopState teleop_state);
    void holdAllArms(double dt);

    // Core methods
    void pollTfCallback();
    bool lookupTf(const std::string &target_frame,
                  const std::string &source_frame,
                  geometry_msgs::msg::TransformStamped &out_chest_T_source);
    IKResult solveIK(size_t arm, const geometry_msgs::msg::PoseStamped &base_T_ee,
                     const double (&shoulder_v_elbow)[3],
                     double (&out_q_joints_rad)[kJointsPerArm],
                     ArmDiagnostics *diag);
    void computeAndPublishFK();
    void publishIKStatus(size_t arm, IKResult result);
    void queueDiagnostics(size_t arm, const ArmDiagnostics &diag);
    void diagnosticsTimerCallback();
    void publishVizMarkers(size_t arm, const ArmDiagnostics &diag);
    void clearVizMarkers(size_t arm);
    bool updateTfSnapshot();
    TrackerInputState evaluateTrackerInputState(
        size_t arm, const CachedTrackerData &snap, const rclcpp::Time &now);
    void holdCurrentPosition(size_t arm);
    void applySmoothedCommand(size_t arm, double dt);
    void fillArmTargetFromTracker(
        size_t arm, const CachedTrackerData &snap,
        geometry_msgs::msg::PoseStamped &base_T_ee,
        std::array<double, 3> &shoulder_v_elbow) const;
    void handleStaleTracker(size_t arm, const CachedTrackerData &snap, bool tracker_input_changed);
    void handleFreshTrackerUpdate(size_t arm, const CachedTrackerData &snap);
    void processArmUpdate(size_t arm, const CachedTrackerData &snap,
                          const rclcpp::Time &now, double dt, bool force_reacquire);
    std::string buildIkLogChain(const ArmDiagnostics &diag) const;
    void logArmDiagnostics(size_t arm, const ArmDiagnostics &diag);
    void handleSetArmed(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void handleSetEnabled(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void handleGoHome(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    TeleopState getTeleopState() const;
    void setTeleopState(TeleopState new_state, const std::string &reason);
    bool isTeleopEnabled(const rclcpp::Time &now);
    void publishTeleopState(const std::string &reason);
    void processGoHome(double dt);
    void startGoHomeSequence();
    bool isHomeJointReached(size_t arm, size_t joint) const;
    static const char *teleopStateToString(TeleopState state);
    static const char *ikResultToString(IKResult result);
};

}  // namespace marvin_system

#endif  // ROS2_CONTROL_MARVIN__CONTROLLER__TRACKER_TELEOP_CONTROLLER_HPP_
