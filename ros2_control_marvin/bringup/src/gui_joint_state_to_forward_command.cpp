#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace {

static constexpr std::array<const char *, 14> kJointOrder = {{
  "Joint1_L",
  "Joint2_L",
  "Joint3_L",
  "Joint4_L",
  "Joint5_L",
  "Joint6_L",
  "Joint7_L",
  "Joint1_R",
  "Joint2_R",
  "Joint3_R",
  "Joint4_R",
  "Joint5_R",
  "Joint6_R",
  "Joint7_R",
}};

std::unordered_map<std::string, double> extract_positions(const sensor_msgs::msg::JointState &msg)
{
  std::unordered_map<std::string, double> out;
  out.reserve(msg.name.size());

  std::unordered_map<std::string, size_t> name_to_index;
  name_to_index.reserve(msg.name.size());
  for (size_t i = 0; i < msg.name.size(); ++i) {
    name_to_index.emplace(msg.name[i], i);
  }

  for (const char *name : kJointOrder) {
    const auto it = name_to_index.find(name);
    if (it == name_to_index.end()) {
      continue;
    }
    const size_t idx = it->second;
    if (idx >= msg.position.size()) {
      continue;
    }
    out.emplace(name, static_cast<double>(msg.position[idx]));
  }

  return out;
}

}  // namespace

class GuiJointStateToForwardCommand : public rclcpp::Node
{
public:
  GuiJointStateToForwardCommand() : rclcpp::Node("gui_joint_state_to_forward_command")
  {
    gui_topic_ = this->declare_parameter<std::string>("gui_topic", "gui_joint_states");
    real_topic_ = this->declare_parameter<std::string>("real_topic", "/joint_states");
    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "/forward_position_controller/commands");

    pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(output_topic_, 10);

    sub_gui_ = this->create_subscription<sensor_msgs::msg::JointState>(
      gui_topic_, 10,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) { this->on_gui(*msg); });

    sub_real_ = this->create_subscription<sensor_msgs::msg::JointState>(
      real_topic_, 10,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) { this->on_real(*msg); });

    last_cmd_.assign(kJointOrder.size(), 0.0);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), [this] { this->publish_initial_once(); });

    RCLCPP_INFO(
      this->get_logger(), "Bridging GUI '%s' + REAL '%s' -> '%s'",
      gui_topic_.c_str(), real_topic_.c_str(), output_topic_.c_str());
  }

private:
  void maybe_init()
  {
    if (initialized_) {
      return;
    }
    if (!q_real0_.has_value() || !q_gui0_.has_value()) {
      return;
    }

    // Initial hold = current real pose.
    std::vector<double> cmd;
    cmd.reserve(kJointOrder.size());
    for (const char *name : kJointOrder) {
      const auto it = q_real0_->find(name);
      cmd.push_back(it != q_real0_->end() ? it->second : 0.0);
    }

    last_cmd_ = cmd;
    publish_cmd(cmd);
    initialized_ = true;

    RCLCPP_INFO(this->get_logger(), "Initialized offset mapping; initial hold command published.");
  }

  void publish_initial_once()
  {
    if (!initialized_) {
      maybe_init();
      return;
    }
    if (initial_published_) {
      return;
    }
    publish_cmd(last_cmd_);
    initial_published_ = true;
  }

  void publish_cmd(const std::vector<double> &cmd)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = cmd;
    pub_->publish(msg);
  }

  void on_real(const sensor_msgs::msg::JointState &msg)
  {
    if (q_real0_.has_value()) {
      return;
    }
    auto q = extract_positions(msg);
    if (!q.empty()) {
      q_real0_ = std::move(q);
      RCLCPP_INFO(this->get_logger(), "Captured initial real joint pose.");
      maybe_init();
    }
  }

  void on_gui(const sensor_msgs::msg::JointState &msg)
  {
    if (!q_gui0_.has_value()) {
      auto q = extract_positions(msg);
      if (!q.empty()) {
        q_gui0_ = std::move(q);
        RCLCPP_INFO(this->get_logger(), "Captured initial GUI joint pose.");
        maybe_init();
      }
      return;
    }

    if (!initialized_ || !q_real0_.has_value() || !q_gui0_.has_value()) {
      return;
    }

    const auto q_gui = extract_positions(msg);

    std::vector<double> cmd;
    cmd.reserve(kJointOrder.size());
    for (const char *name : kJointOrder) {
      const double q0_real = [&] {
        const auto it = q_real0_->find(name);
        return it != q_real0_->end() ? it->second : 0.0;
      }();
      const double q0_gui = [&] {
        const auto it = q_gui0_->find(name);
        return it != q_gui0_->end() ? it->second : 0.0;
      }();
      const double q_now_gui = [&] {
        const auto it = q_gui.find(name);
        return it != q_gui.end() ? it->second : q0_gui;
      }();

      double v = q0_real + (q_now_gui - q0_gui);
      if (!std::isfinite(v)) {
        v = q0_real;
      }
      cmd.push_back(v);
    }

    last_cmd_ = cmd;
    publish_cmd(cmd);
  }

private:
  std::string gui_topic_;
  std::string real_topic_;
  std::string output_topic_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_gui_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_real_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<std::unordered_map<std::string, double>> q_real0_;
  std::optional<std::unordered_map<std::string, double>> q_gui0_;

  bool initialized_{false};
  bool initial_published_{false};
  std::vector<double> last_cmd_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GuiJointStateToForwardCommand>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
