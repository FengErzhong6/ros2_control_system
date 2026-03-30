#include "internal.hpp"

#include <string>

#include "rclcpp/logging.hpp"

namespace marvin_system {

void TrackerTeleopController::startGoHomeSequence()
{
    if (!has_home_joints_) {
        return;
    }

    for (size_t arm = 0; arm < kArmCount; ++arm) {
        auto &runtime = arm_state_[arm];
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            runtime.target_joints_rad[j] = runtime.smoothed_joints_rad[j];
        }
        runtime.has_valid_target = true;
        runtime.target_joints_rad[kJointsPerArm - 1] =
            runtime.home_joints_rad[kJointsPerArm - 1];
    }

    active_home_joint_index_.store(
        static_cast<int>(kJointsPerArm) - 1, std::memory_order_relaxed);
    go_home_active_.store(true, std::memory_order_relaxed);

    RCLCPP_INFO(
        get_node()->get_logger(),
        "Go-home sequence started (Joint%zu -> Joint1).",
        kJointsPerArm);
}

bool TrackerTeleopController::isHomeJointReached(size_t arm, size_t joint) const
{
    const size_t idx = arm * kJointsPerArm + joint;
    const auto &runtime = arm_state_[arm];
    if (!state_interfaces_pos_[idx]) {
        return false;
    }

    const auto pos_opt = state_interfaces_pos_[idx]->get_optional<double>();
    const double pos_rad = pos_opt.has_value() ?
        pos_opt.value() : runtime.smoothed_joints_rad[joint];
    return std::abs(pos_rad - runtime.home_joints_rad[joint]) <= home_tolerance_rad_;
}

void TrackerTeleopController::processGoHome(double dt)
{
    if (go_home_requested_.exchange(false, std::memory_order_relaxed)) {
        startGoHomeSequence();
    }

    if (!go_home_active_.load(std::memory_order_relaxed)) {
        return;
    }

    const int joint_index = active_home_joint_index_.load(std::memory_order_relaxed);
    if (joint_index < 0) {
        go_home_active_.store(false, std::memory_order_relaxed);
        return;
    }

    const size_t jt = static_cast<size_t>(joint_index);
    for (size_t arm = 0; arm < kArmCount; ++arm) {
        auto &runtime = arm_state_[arm];
        runtime.target_joints_rad[jt] = runtime.home_joints_rad[jt];
        runtime.has_valid_target = true;
        applySmoothedCommand(arm, dt);
    }

    bool reached = true;
    for (size_t arm = 0; arm < kArmCount; ++arm) {
        reached = reached && isHomeJointReached(arm, jt);
    }
    if (!reached) {
        return;
    }

    const int next_joint_index = joint_index - 1;
    active_home_joint_index_.store(next_joint_index, std::memory_order_relaxed);

    if (next_joint_index >= 0) {
        const size_t next_jt = static_cast<size_t>(next_joint_index);
        for (size_t arm = 0; arm < kArmCount; ++arm) {
            auto &runtime = arm_state_[arm];
            runtime.target_joints_rad[next_jt] = runtime.home_joints_rad[next_jt];
        }
        RCLCPP_INFO(get_node()->get_logger(), "Go-home reached Joint%zu, continuing to Joint%zu.",
                    jt + 1, next_jt + 1);
        return;
    }

    go_home_active_.store(false, std::memory_order_relaxed);
    RCLCPP_INFO(get_node()->get_logger(), "Go-home sequence completed.");
}

void TrackerTeleopController::handleSetArmed(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data) {
        if (getTeleopState() == TeleopState::kDisarmed) {
            setTeleopState(TeleopState::kArmed, "service arm");
        }
        response->success = true;
        response->message = "Tracker teleop armed; holding current position.";
        return;
    }

    go_home_requested_.store(false, std::memory_order_relaxed);
    go_home_active_.store(false, std::memory_order_relaxed);
    active_home_joint_index_.store(-1, std::memory_order_relaxed);
    setTeleopState(TeleopState::kDisarmed, "service disarm");
    response->success = true;
    response->message = "Tracker teleop disarmed; holding current position.";
}

void TrackerTeleopController::handleSetEnabled(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data) {
        if (getTeleopState() == TeleopState::kDisarmed) {
            response->success = false;
            response->message = "Teleop is disarmed; arm before enabling.";
            return;
        }
        go_home_requested_.store(false, std::memory_order_relaxed);
        go_home_active_.store(false, std::memory_order_relaxed);
        active_home_joint_index_.store(-1, std::memory_order_relaxed);
        setTeleopState(TeleopState::kEnabled, "service enable");
        response->success = true;
        response->message = startup_sync_config_.enabled ?
            "Tracker teleop enabled; holding current pose until tracker aligns." :
            "Tracker teleop enabled.";
        return;
    }

    if (getTeleopState() == TeleopState::kEnabled) {
        setTeleopState(TeleopState::kArmed, "service disable");
    }
    response->success = true;
    response->message = "Tracker teleop disabled; holding current position.";
}

void TrackerTeleopController::handleGoHome(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    if (!has_home_joints_) {
        response->success = false;
        response->message = "Home joint positions are not configured.";
        return;
    }

    setTeleopState(TeleopState::kArmed, "service go_home");
    go_home_requested_.store(true, std::memory_order_relaxed);
    response->success = true;
    response->message = "Go-home accepted; moving to default pose.";
}

TrackerTeleopController::TeleopState TrackerTeleopController::getTeleopState() const
{
    return static_cast<TeleopState>(teleop_state_.load(std::memory_order_relaxed));
}

void TrackerTeleopController::setTeleopState(
    TeleopState new_state, const std::string &reason)
{
    const TeleopState old_state = getTeleopState();
    teleop_state_.store(static_cast<int>(new_state), std::memory_order_relaxed);
    if (new_state == TeleopState::kEnabled) {
        force_tracker_reacquire_.store(true, std::memory_order_relaxed);
    }

    if (old_state == new_state) {
        return;
    }

    RCLCPP_WARN(
        get_node()->get_logger(),
        "Tracker teleop state: %s -> %s (%s)",
        teleopStateToString(old_state),
        teleopStateToString(new_state),
        reason.c_str());
    publishTeleopState(reason);
}

bool TrackerTeleopController::isTeleopEnabled(const rclcpp::Time &now)
{
    (void)now;
    return getTeleopState() == TeleopState::kEnabled;
}

void TrackerTeleopController::publishTeleopState(const std::string &reason)
{
    if (!pub_teleop_state_) {
        return;
    }

    std_msgs::msg::String msg;
    msg.data = std::string(teleopStateToString(getTeleopState())) + " | " + reason;
    pub_teleop_state_->publish(msg);
}

const char *TrackerTeleopController::teleopStateToString(TeleopState state)
{
    switch (state) {
        case TeleopState::kDisarmed: return "DISARMED";
        case TeleopState::kArmed: return "ARMED";
        case TeleopState::kEnabled: return "ENABLED";
        default: return "UNKNOWN";
    }
}

}  // namespace marvin_system
