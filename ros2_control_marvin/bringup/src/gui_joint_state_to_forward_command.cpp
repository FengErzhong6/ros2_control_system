#include <cmath>
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
        gui_initial_.assign(n, 0.0);

        auto input_topic = declare_parameter("input_topic", std::string("gui_joint_states"));
        auto output_topic = declare_parameter(
            "output_topic", std::string("/forward_position_controller/commands"));

        pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(output_topic, 10);

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
        std_msgs::msg::Float64MultiArray msg;
        msg.data = cmd;
        pub_->publish(msg);
    }

    void on_real_js(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (seeded_) return;
        extract_positions(msg, real_pos_);
        seeded_ = true;
        last_cmd_ = real_pos_;
        publish_cmd(last_cmd_);
        RCLCPP_INFO(get_logger(),
            "Seeded initial positions from hardware feedback.");
    }

    /// Only forward GUI values for joints the user has actually moved
    /// (i.e., slider value differs from the initial "Centering" value).
    /// For untouched joints, keep the real hardware position.
    void on_gui_js(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!seeded_) return;

        std::vector<double> gui_vals(joint_order_.size(), 0.0);
        extract_positions(msg, gui_vals);

        if (!gui_baseline_set_) {
            gui_initial_ = gui_vals;
            gui_baseline_set_ = true;
        }

        constexpr double kSliderMovedThreshold = 0.001;
        for (size_t i = 0; i < joint_order_.size(); ++i) {
            bool user_moved =
                std::abs(gui_vals[i] - gui_initial_[i]) > kSliderMovedThreshold;
            last_cmd_[i] = user_moved ? gui_vals[i] : real_pos_[i];
        }
        publish_cmd(last_cmd_);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr real_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gui_sub_;

    std::vector<std::string> joint_order_;
    std::vector<double> last_cmd_;
    std::vector<double> real_pos_;
    std::vector<double> gui_initial_;
    bool seeded_{false};
    bool gui_baseline_set_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GuiJointStateToForwardCommand>());
    rclcpp::shutdown();
    return 0;
}
