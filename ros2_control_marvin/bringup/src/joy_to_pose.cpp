#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"

class JoyToPose : public rclcpp::Node {
public:
    JoyToPose()
        : Node("joy_to_pose")
    {
        linear_speed_ = declare_parameter("linear_speed", 0.1);
        angular_speed_ = declare_parameter("angular_speed", 0.5);
        button_angular_speed_ = declare_parameter("button_angular_speed", 0.3);
        deadzone_ = declare_parameter("deadzone", 0.15);
        publish_rate_ = declare_parameter("publish_rate", 50.0);

        auto left_topic = declare_parameter(
            "target_pose_left_topic",
            std::string("/ik_controller/target_pose_left"));
        auto right_topic = declare_parameter(
            "target_pose_right_topic",
            std::string("/ik_controller/target_pose_right"));
        auto fk_left_topic = declare_parameter(
            "current_pose_left_topic",
            std::string("/ik_controller/current_pose_left"));
        auto fk_right_topic = declare_parameter(
            "current_pose_right_topic",
            std::string("/ik_controller/current_pose_right"));

        pub_pose_[kLeft] =
            create_publisher<geometry_msgs::msg::PoseStamped>(left_topic, 10);
        pub_pose_[kRight] =
            create_publisher<geometry_msgs::msg::PoseStamped>(right_topic, 10);

        // Subscribe to FK current poses (transient_local to receive latched value)
        auto fk_qos = rclcpp::QoS(1).transient_local();
        sub_fk_pose_[kLeft] =
            create_subscription<geometry_msgs::msg::PoseStamped>(
                fk_left_topic, fk_qos,
                [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    onFKPose(kLeft, msg);
                });
        sub_fk_pose_[kRight] =
            create_subscription<geometry_msgs::msg::PoseStamped>(
                fk_right_topic, fk_qos,
                [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    onFKPose(kRight, msg);
                });

        // Subscribe to IK status for drift prevention
        auto ik_status_left_topic = declare_parameter(
            "ik_status_left_topic",
            std::string("/ik_controller/ik_status_left"));
        auto ik_status_right_topic = declare_parameter(
            "ik_status_right_topic",
            std::string("/ik_controller/ik_status_right"));
        sub_ik_status_[kLeft] = create_subscription<std_msgs::msg::String>(
            ik_status_left_topic, 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                onIKStatus(kLeft, msg);
            });
        sub_ik_status_[kRight] = create_subscription<std_msgs::msg::String>(
            ik_status_right_topic, 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                onIKStatus(kRight, msg);
            });

        sub_joy_ = create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            [this](const sensor_msgs::msg::Joy::SharedPtr msg) { onJoy(msg); });

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate_),
            [this]() { update(); });

        RCLCPP_INFO(get_logger(),
            "JoyToPose ready: linear=%.3f m/s, angular=%.3f rad/s, "
            "btn_angular=%.3f rad/s, deadzone=%.2f, rate=%.0f Hz",
            linear_speed_, angular_speed_, button_angular_speed_,
            deadzone_, publish_rate_);
        RCLCPP_INFO(get_logger(),
            "Waiting for FK initial poses from IK controller...");
    }

private:
    static constexpr int kLeft = 0;
    static constexpr int kRight = 1;

    // Xbox-compatible axis / button indices
    static constexpr int kAxisLeftX  = 0;
    static constexpr int kAxisLeftY  = 1;
    static constexpr int kAxisLT     = 2;
    static constexpr int kAxisRightX = 3;
    static constexpr int kAxisRightY = 4;
    static constexpr int kAxisRT     = 5;
    static constexpr int kBtnA  = 0;
    static constexpr int kBtnB  = 1;
    static constexpr int kBtnLB = 4;
    static constexpr int kBtnRB = 5;

    double linear_speed_;
    double angular_speed_;
    double button_angular_speed_;
    double deadzone_;
    double publish_rate_;

    int active_arm_{kLeft};
    std::array<geometry_msgs::msg::PoseStamped, 2> poses_;
    std::array<geometry_msgs::msg::PoseStamped, 2> last_good_pose_;
    std::array<bool, 2> pose_initialized_{{false, false}};
    std::array<bool, 2> ik_ok_{{true, true}};
    sensor_msgs::msg::Joy latest_joy_;
    bool joy_received_{false};
    bool prev_btn_a_{false};
    bool prev_btn_b_{false};

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
    std::array<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr, 2>
        sub_fk_pose_;
    std::array<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr, 2>
        sub_ik_status_;
    std::array<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr, 2>
        pub_pose_;
    rclcpp::TimerBase::SharedPtr timer_;

    double getAxis(int index) const
    {
        if (index >= 0 &&
            index < static_cast<int>(latest_joy_.axes.size()))
            return latest_joy_.axes[index];
        return 0.0;
    }

    bool getButton(int index) const
    {
        if (index >= 0 &&
            index < static_cast<int>(latest_joy_.buttons.size()))
            return latest_joy_.buttons[index] != 0;
        return false;
    }

    /// Remap stick value through deadzone: raw ∈ [-1,1] → out ∈ [-1,1], 0 inside deadzone.
    static double applyDeadzone(double value, double dz)
    {
        if (std::abs(value) < dz) return 0.0;
        double sign = (value > 0.0) ? 1.0 : -1.0;
        return sign * (std::abs(value) - dz) / (1.0 - dz);
    }

    /// Convert trigger axis (+1 released … -1 pressed) to [0, 1] with deadzone.
    /// Treats raw == 0 as released (handles uninitialized triggers).
    static double triggerToUnit(double raw, double dz)
    {
        if (raw == 0.0) return 0.0;
        double amount = std::clamp((1.0 - raw) / 2.0, 0.0, 1.0);
        return (amount > dz) ? (amount - dz) / (1.0 - dz) : 0.0;
    }

    /// Apply small axis-angle rotation (rx, ry, rz in radians) to quaternion.
    static void applyRotation(geometry_msgs::msg::Quaternion &q,
                              double rx, double ry, double rz)
    {
        double angle = std::sqrt(rx * rx + ry * ry + rz * rz);
        if (angle < 1e-12) return;

        double ha = angle / 2.0;
        double s  = std::sin(ha) / angle;
        double dw = std::cos(ha), dx = rx * s, dy = ry * s, dz = rz * s;

        // q_new = dq · q_old  (rotation expressed in base frame)
        double nw = dw * q.w - dx * q.x - dy * q.y - dz * q.z;
        double nx = dw * q.x + dx * q.w + dy * q.z - dz * q.y;
        double ny = dw * q.y - dx * q.z + dy * q.w + dz * q.x;
        double nz = dw * q.z + dx * q.y - dy * q.x + dz * q.w;

        double norm = std::sqrt(nw * nw + nx * nx + ny * ny + nz * nz);
        q.w = nw / norm;
        q.x = nx / norm;
        q.y = ny / norm;
        q.z = nz / norm;
    }

    // ── callbacks ────────────────────────────────────────────────────────

    void onFKPose(int arm, const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (pose_initialized_[arm]) return;
        poses_[arm] = *msg;
        last_good_pose_[arm] = *msg;
        pose_initialized_[arm] = true;

        const char *label = (arm == kLeft) ? "LEFT" : "RIGHT";
        RCLCPP_INFO(get_logger(),
            "%s arm initial pose received: pos=(%.4f, %.4f, %.4f) "
            "quat=(%.4f, %.4f, %.4f, %.4f)",
            label,
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
            msg->pose.orientation.x, msg->pose.orientation.y,
            msg->pose.orientation.z, msg->pose.orientation.w);

        if (pose_initialized_[kLeft] && pose_initialized_[kRight]) {
            RCLCPP_INFO(get_logger(),
                "Both arms initialized. Active arm: LEFT  "
                "(press A = left, B = right)");
        }
    }

    void onIKStatus(int arm, const std_msgs::msg::String::SharedPtr msg)
    {
        if (!pose_initialized_[arm]) return;

        bool ok = (msg->data.rfind("[OK]", 0) == 0);
        if (ok && !ik_ok_[arm]) {
            last_good_pose_[arm] = poses_[arm];
            RCLCPP_INFO(get_logger(), "%s arm: IK recovered",
                        (arm == kLeft) ? "LEFT" : "RIGHT");
        }
        if (!ok && ik_ok_[arm]) {
            RCLCPP_WARN(get_logger(), "%s arm: %s — pose clamped at boundary",
                        (arm == kLeft) ? "LEFT" : "RIGHT",
                        msg->data.c_str());
        }
        ik_ok_[arm] = ok;
        if (!ok) {
            poses_[arm] = last_good_pose_[arm];
        }
    }

    void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        latest_joy_ = *msg;
        joy_received_ = true;

        // Rising-edge detection for arm switching
        bool a = getButton(kBtnA);
        bool b = getButton(kBtnB);
        if (a && !prev_btn_a_) {
            active_arm_ = kLeft;
            RCLCPP_INFO(get_logger(), "Switched to LEFT arm");
        }
        if (b && !prev_btn_b_) {
            active_arm_ = kRight;
            RCLCPP_INFO(get_logger(), "Switched to RIGHT arm");
        }
        prev_btn_a_ = a;
        prev_btn_b_ = b;
    }

    void update()
    {
        if (!joy_received_ || !pose_initialized_[active_arm_]) return;

        // When IK is failing: revert to boundary and re-try from there
        // When IK is OK: track current pose so revert target stays fresh
        if (!ik_ok_[active_arm_]) {
            poses_[active_arm_] = last_good_pose_[active_arm_];
        } else {
            last_good_pose_[active_arm_] = poses_[active_arm_];
        }

        const double dt = 1.0 / publish_rate_;

        // ── translation (left stick + triggers) ──
        double lx = applyDeadzone(getAxis(kAxisLeftX), deadzone_);
        double ly = applyDeadzone(getAxis(kAxisLeftY), deadzone_);
        double lt = triggerToUnit(getAxis(kAxisLT), deadzone_);
        double rt = triggerToUnit(getAxis(kAxisRT), deadzone_);

        // ── rotation (right stick + bumpers) ──
        double rx_stick = applyDeadzone(getAxis(kAxisRightX), deadzone_);
        double ry_stick = applyDeadzone(getAxis(kAxisRightY), deadzone_);
        bool lb = getButton(kBtnLB);
        bool rb = getButton(kBtnRB);

        bool has_input = (std::abs(lx) > 0 || std::abs(ly) > 0 ||
                          lt > 0 || rt > 0 ||
                          std::abs(rx_stick) > 0 || std::abs(ry_stick) > 0 ||
                          lb || rb);
        if (!has_input) return;

        auto &pose = poses_[active_arm_];

        // Left stick Y ↑ → X+ (前进)    Left stick X ← → Y+ (左移)
        // RT → Z+ (上升)                 LT → Z- (下降)
        pose.pose.position.x += ly * linear_speed_ * dt;
        pose.pose.position.y += lx * linear_speed_ * dt;
        pose.pose.position.z += (rt - lt) * linear_speed_ * dt;

        // Right stick Y ↑ → Pitch 抬头   Right stick X → → Yaw 右转
        // RB → Roll 顺时针               LB → Roll 逆时针
        double d_rx = 0.0, d_ry = 0.0, d_rz = 0.0;
        if (lb) d_rx -= button_angular_speed_ * dt;
        if (rb) d_rx += button_angular_speed_ * dt;
        d_ry += ry_stick * angular_speed_ * dt;
        d_rz += rx_stick * angular_speed_ * dt;

        applyRotation(pose.pose.orientation, d_rx, d_ry, d_rz);

        pose.header.stamp = now();
        pub_pose_[active_arm_]->publish(pose);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToPose>());
    rclcpp::shutdown();
    return 0;
}
