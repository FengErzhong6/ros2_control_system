#include <cmath>
#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

static const std::vector<std::string> DEFAULT_JOINTS = {
    "Joint1_L", "Joint2_L", "Joint3_L", "Joint4_L",
    "Joint5_L", "Joint6_L", "Joint7_L",
    "Joint1_R", "Joint2_R", "Joint3_R", "Joint4_R",
    "Joint5_R", "Joint6_R", "Joint7_R",
};

class GuiJointStateToForwardCommand : public rclcpp::Node {
public:
    GuiJointStateToForwardCommand()
        : Node("gui_joint_state_to_forward_command")
    {
        joint_order_ = declare_parameter<std::vector<std::string>>(
            "joint_names", DEFAULT_JOINTS);

        const auto n = joint_order_.size();
        last_cmd_.assign(n, 0.0);
        real_pos_.assign(n, 0.0);
        prev_gui_.assign(n, 0.0);
        user_override_.assign(n, false);

        auto input_topic = declare_parameter("input_topic", std::string("gui_joint_states"));
        auto output_topic = declare_parameter(
            "output_topic", std::string("/forward_position_controller/commands"));

        pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(output_topic, 10);

        seed_watchdog_ = create_wall_timer(
            std::chrono::seconds(1),
            [this, output_topic]() {
                if (!seeded_) {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 3000,
                        "Still waiting for hardware joint_states before forwarding GUI commands to %s.",
                        output_topic.c_str());
                }
            });

        real_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                on_real_js(msg);
            });

        gui_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            input_topic, 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                on_gui_js(msg);
            });

        RCLCPP_INFO(get_logger(), "Bridging %s -> %s (%zu joints)",
                    input_topic.c_str(), output_topic.c_str(), n);
    }

private:
    void extract_positions(const sensor_msgs::msg::JointState::SharedPtr &msg,
                           std::vector<double> &out)
    {
        std::unordered_map<std::string, size_t> name_to_idx;
        for (size_t i = 0; i < msg->name.size(); ++i)
            name_to_idx[msg->name[i]] = i;
        for (size_t i = 0; i < joint_order_.size(); ++i) {
            auto it = name_to_idx.find(joint_order_[i]);
            if (it != name_to_idx.end() && it->second < msg->position.size())
                out[i] = msg->position[it->second];
        }
    }

    void publish_cmd(const std::vector<double> &cmd)
    {
        const auto subscriber_count = pub_->get_subscription_count();
        if (subscriber_count == 0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 3000,
                "Publishing GUI command, but %s has no subscribers yet.",
                pub_->get_topic_name());
        } else if (!output_ready_logged_) {
            output_ready_logged_ = true;
            RCLCPP_INFO(
                get_logger(), "Forward command output %s is connected to %zu subscriber(s).",
                pub_->get_topic_name(), subscriber_count);
        }

        std_msgs::msg::Float64MultiArray msg;
        msg.data = cmd;
        pub_->publish(msg);
    }

    void on_real_js(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        extract_positions(msg, real_pos_);
        if (seeded_) return;
        seeded_ = true;
        last_cmd_ = real_pos_;
        publish_cmd(last_cmd_);
        RCLCPP_INFO(get_logger(),
            "Seeded initial positions from hardware feedback on /joint_states.");
    }

    /// Only publish when the operator actually moves a slider away from the
    /// current measured joint angle. Pure GUI refreshes sourced from real
    /// joint_states should not generate new commands, otherwise the arm can
    /// end up chasing its own feedback and jittering.
    void on_gui_js(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!seeded_) return;

        std::vector<double> gui_vals(joint_order_.size(), 0.0);
        extract_positions(msg, gui_vals);

        if (!have_prev_gui_) {
            prev_gui_ = gui_vals;
            have_prev_gui_ = true;
            return;
        }

        constexpr double kGuiChangeThreshold = 1e-4;
        constexpr double kUserIntentThreshold = 0.01;
        bool should_publish = false;

        for (size_t i = 0; i < joint_order_.size(); ++i) {
            const bool gui_changed =
                std::abs(gui_vals[i] - prev_gui_[i]) > kGuiChangeThreshold;
            const bool away_from_real =
                std::abs(gui_vals[i] - real_pos_[i]) > kUserIntentThreshold;

            if (gui_changed && away_from_real) {
                user_override_[i] = true;
            }

            if (!user_override_[i]) {
                continue;
            }

            if (std::abs(gui_vals[i] - last_cmd_[i]) > kGuiChangeThreshold) {
                last_cmd_[i] = gui_vals[i];
                should_publish = true;
                if (!first_user_command_logged_) {
                    first_user_command_logged_ = true;
                    RCLCPP_INFO(
                        get_logger(), "Accepted first GUI override: %s -> %.4f rad.",
                        joint_order_[i].c_str(), gui_vals[i]);
                }
            }

            if (std::abs(gui_vals[i] - real_pos_[i]) <= kGuiChangeThreshold) {
                user_override_[i] = false;
            }
        }

        prev_gui_ = gui_vals;
        if (should_publish) {
            publish_cmd(last_cmd_);
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr real_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gui_sub_;
    rclcpp::TimerBase::SharedPtr seed_watchdog_;

    std::vector<std::string> joint_order_;
    std::vector<double> last_cmd_;
    std::vector<double> real_pos_;
    std::vector<double> prev_gui_;
    std::vector<bool> user_override_;
    bool seeded_{false};
    bool have_prev_gui_{false};
    bool output_ready_logged_{false};
    bool first_user_command_logged_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GuiJointStateToForwardCommand>());
    rclcpp::shutdown();
    return 0;
}
