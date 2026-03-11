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
        last_cmd_.assign(joint_order_.size(), 0.0);

        auto input_topic = declare_parameter("input_topic", std::string("gui_joint_states"));
        auto output_topic = declare_parameter(
            "output_topic", std::string("/forward_position_controller/commands"));

        pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(output_topic, 10);
        sub_ = create_subscription<sensor_msgs::msg::JointState>(
            input_topic, 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_js(msg); });

        // Publish initial all-zeros so the forward controller has a defined target.
        timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {
            if (initial_published_) return;
            publish_cmd(last_cmd_);
            initial_published_ = true;
        });

        RCLCPP_INFO(get_logger(), "Bridging %s -> %s (%zu joints)",
                    input_topic.c_str(), output_topic.c_str(), joint_order_.size());
    }

private:
    void publish_cmd(const std::vector<double> &cmd)
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = cmd;
        pub_->publish(msg);
    }

    void on_js(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::unordered_map<std::string, size_t> name_to_idx;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            name_to_idx[msg->name[i]] = i;
        }

        std::vector<double> cmd(joint_order_.size(), 0.0);
        for (size_t i = 0; i < joint_order_.size(); ++i) {
            auto it = name_to_idx.find(joint_order_[i]);
            if (it != name_to_idx.end() && it->second < msg->position.size()) {
                cmd[i] = msg->position[it->second];
            } else {
                cmd[i] = last_cmd_[i];
            }
        }

        last_cmd_ = cmd;
        publish_cmd(cmd);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> joint_order_;
    std::vector<double> last_cmd_;
    bool initial_published_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GuiJointStateToForwardCommand>());
    rclcpp::shutdown();
    return 0;
}
