#include "internal.hpp"

#include <iomanip>
#include <sstream>
#include <string>

#include "rclcpp/logging.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace marvin_system {

const char *TrackerTeleopController::ikResultToString(IKResult result)
{
    switch (result) {
        case IKResult::kSuccess:             return "IK solved successfully";
        case IKResult::kNoTarget:            return "No target";
        case IKResult::kJointLimitClamped:   return "IK solved (clamped to joint limits)";
        case IKResult::kInvalidQuaternion:   return "Invalid quaternion";
        case IKResult::kOutOfRange:          return "Target out of reachable workspace";
        case IKResult::kJointLimitExceeded:  return "IK solution exceeds joint limits";
        case IKResult::kSingularity:         return "IK solution near singularity";
        case IKResult::kSolveFailed:         return "Tracking IK solver failed";
        default:                             return "Unknown IK error";
    }
}

void TrackerTeleopController::publishIKStatus(size_t arm, IKResult result)
{
    if (arm >= kArmCount || !pub_ik_status_[arm]) return;
    auto &runtime = arm_state_[arm];
    if (result == runtime.last_ik_result) return;
    runtime.last_ik_result = result;

    std_msgs::msg::String msg;
    const bool ok = (result == IKResult::kSuccess ||
                     result == IKResult::kJointLimitClamped);
    msg.data = std::string(ok ? "[OK] " : "[FAIL] ") + ikResultToString(result);
    pub_ik_status_[arm]->publish(msg);
}

void TrackerTeleopController::queueDiagnostics(size_t arm, const ArmDiagnostics &diag)
{
    if (arm >= kArmCount) {
        return;
    }

    auto &slot = arm_state_[arm].pending_diagnostics;
    slot.sequence.fetch_add(1, std::memory_order_acq_rel);
    slot.snapshot = diag;
    slot.snapshot.pending = false;
    slot.pending.store(true, std::memory_order_release);
    slot.sequence.fetch_add(1, std::memory_order_release);
}

std::string TrackerTeleopController::buildIkLogChain(const ArmDiagnostics &diag) const
{
    auto classify_j4 = [this](double j4_deg) -> const char * {
        return (j4_deg < j4_bound_) ? "A" : "B";
    };

    std::ostringstream chain;
    chain << "TRACKING_IK ";
    if (diag.final_result == IKResult::kSuccess) {
        chain << "OK(" << classify_j4(diag.solution_j4_deg) << ")";
    } else {
        chain << "FAIL(" << ikResultToString(diag.final_result) << ")";
    }
    chain << " reachable=" << (diag.solver_reachable ? "Y" : "N");
    chain << " expand=" << (diag.used_expanded_search ? "Y" : "N");
    chain << " cands=" << diag.candidate_count;
    chain << " psi_eval=" << diag.psi_eval_count;
    chain << " branch=" << diag.selected_branch;
    chain << " psi=" << diag.selected_psi_deg;
    if (diag.has_solution) {
        chain << " fk_l1=" << diag.best_fk_residual_l1;
        chain << " ref_l1=" << diag.best_ref_score_l1;
        chain << " desired_score=" << diag.best_desired_dir_score;
        chain << " continuity_score=" << diag.best_continuity_dir_score;
        if (diag.solved_upper_arm_dir_valid) {
            chain << " nsp_dir=" << diag.solved_upper_arm_dir_angle_deg << "deg";
        } else {
            chain << " nsp_dir=NA";
        }
    }
    return chain.str();
}

bool TrackerTeleopController::readCurrentJointPositions(
    size_t arm, std::array<double, kJointsPerArm> &joints_rad) const
{
    if (arm >= kArmCount) {
        return false;
    }

    const size_t offset = arm * kJointsPerArm;
    bool has_state = true;
    for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
        const size_t idx = offset + joint;
        if (!state_interfaces_pos_[idx]) {
            has_state = false;
            break;
        }

        const auto pos_opt = state_interfaces_pos_[idx]->get_optional<double>();
        if (!pos_opt.has_value()) {
            has_state = false;
            break;
        }
        joints_rad[joint] = pos_opt.value();
    }

    return has_state;
}

bool TrackerTeleopController::computeCurrentUpperArmDir(
    size_t arm, std::array<double, 3> &upper_arm_dir) const
{
    std::array<double, kJointsPerArm> joints_rad{};
    if (!readCurrentJointPositions(arm, joints_rad)) {
        return false;
    }

    return extractSolvedUpperArmDir(
        static_cast<FX_INT32L>(arm), joints_rad, upper_arm_dir);
}

void TrackerTeleopController::logArmDiagnostics(size_t arm, const ArmDiagnostics &diag)
{
    const auto logger = get_node()->get_logger();
    const std::string chain = buildIkLogChain(diag);
    std::array<double, 3> current_upper_arm_dir{};
    const bool current_upper_arm_dir_valid =
        computeCurrentUpperArmDir(arm, current_upper_arm_dir);
    const double current_upper_arm_dir_angle_deg =
        current_upper_arm_dir_valid ?
            angleBetweenVectorsDeg(diag.shoulder_v_elbow, current_upper_arm_dir) :
            0.0;
    const std::string tracker_y_str = [&diag]() {
        if (!diag.tracker_y_axis_valid) {
            return std::string("NA");
        }
        std::ostringstream oss;
        oss << "[" << std::fixed << std::setprecision(3)
            << diag.tracker_y_axis[0] << ", "
            << diag.tracker_y_axis[1] << ", "
            << diag.tracker_y_axis[2] << "]";
        return oss.str();
    }();
    const auto format_vec3 = [](const std::array<double, 3> &vec) {
        std::ostringstream oss;
        oss << "[" << std::fixed << std::setprecision(3)
            << vec[0] << ", " << vec[1] << ", " << vec[2] << "]";
        return oss.str();
    };
    const std::string prev_ref_str = format_vec3(diag.request_prev_selected_ref_dir);
    const std::string selected_ref_str = format_vec3(diag.selected_ref_dir);

    if (diag.has_solution) {
        if (diag.solved_upper_arm_dir_valid) {
            if (current_upper_arm_dir_valid) {
                RCLCPP_INFO(
                    logger,
                    "Arm %zu IK | shoulder_T_ee=[%.4f, %.4f, %.4f] quat=[%.4f, %.4f, %.4f, %.4f] "
                    "| tracker_y=%s | desired_upper_arm_dir=[%.3f, %.3f, %.3f] "
                    "| prev_ref_dir=%s prev_branch=%ld | selected_ref_dir=%s "
                    "| actual_nsp_y=[%.3f, %.3f, %.3f] "
                    "| solved_nsp_y=[%.3f, %.3f, %.3f] | actual_nsp_dir=%.4fdeg | %s "
                    "| q=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f] deg",
                    arm,
                    diag.base_T_ee.pose.position.x, diag.base_T_ee.pose.position.y,
                    diag.base_T_ee.pose.position.z - dh_d1_,
                    diag.base_T_ee.pose.orientation.x, diag.base_T_ee.pose.orientation.y,
                    diag.base_T_ee.pose.orientation.z, diag.base_T_ee.pose.orientation.w,
                    tracker_y_str.c_str(),
                    diag.shoulder_v_elbow[0], diag.shoulder_v_elbow[1], diag.shoulder_v_elbow[2],
                    prev_ref_str.c_str(),
                    static_cast<long>(diag.request_prev_selected_branch),
                    selected_ref_str.c_str(),
                    current_upper_arm_dir[0], current_upper_arm_dir[1], current_upper_arm_dir[2],
                    diag.solved_upper_arm_dir[0], diag.solved_upper_arm_dir[1], diag.solved_upper_arm_dir[2],
                    current_upper_arm_dir_angle_deg,
                    chain.c_str(),
                    diag.q_joints_rad[0] * kRad2Deg, diag.q_joints_rad[1] * kRad2Deg,
                    diag.q_joints_rad[2] * kRad2Deg, diag.q_joints_rad[3] * kRad2Deg,
                    diag.q_joints_rad[4] * kRad2Deg, diag.q_joints_rad[5] * kRad2Deg,
                    diag.q_joints_rad[6] * kRad2Deg);
                return;
            }
            RCLCPP_INFO(
                logger,
                "Arm %zu IK | shoulder_T_ee=[%.4f, %.4f, %.4f] quat=[%.4f, %.4f, %.4f, %.4f] "
                "| tracker_y=%s | desired_upper_arm_dir=[%.3f, %.3f, %.3f] "
                "| prev_ref_dir=%s prev_branch=%ld | selected_ref_dir=%s "
                "| actual_nsp_y=NA | solved_nsp_y=[%.3f, %.3f, %.3f] | %s "
                "| q=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f] deg",
                arm,
                diag.base_T_ee.pose.position.x, diag.base_T_ee.pose.position.y,
                diag.base_T_ee.pose.position.z - dh_d1_,
                diag.base_T_ee.pose.orientation.x, diag.base_T_ee.pose.orientation.y,
                diag.base_T_ee.pose.orientation.z, diag.base_T_ee.pose.orientation.w,
                tracker_y_str.c_str(),
                diag.shoulder_v_elbow[0], diag.shoulder_v_elbow[1], diag.shoulder_v_elbow[2],
                prev_ref_str.c_str(),
                static_cast<long>(diag.request_prev_selected_branch),
                selected_ref_str.c_str(),
                diag.solved_upper_arm_dir[0], diag.solved_upper_arm_dir[1], diag.solved_upper_arm_dir[2],
                chain.c_str(),
                diag.q_joints_rad[0] * kRad2Deg, diag.q_joints_rad[1] * kRad2Deg,
                diag.q_joints_rad[2] * kRad2Deg, diag.q_joints_rad[3] * kRad2Deg,
                diag.q_joints_rad[4] * kRad2Deg, diag.q_joints_rad[5] * kRad2Deg,
                diag.q_joints_rad[6] * kRad2Deg);
            return;
        }
        if (current_upper_arm_dir_valid) {
            RCLCPP_INFO(
                logger,
                "Arm %zu IK | shoulder_T_ee=[%.4f, %.4f, %.4f] quat=[%.4f, %.4f, %.4f, %.4f] "
                "| tracker_y=%s | desired_upper_arm_dir=[%.3f, %.3f, %.3f] "
                "| prev_ref_dir=%s prev_branch=%ld | selected_ref_dir=%s "
                "| actual_nsp_y=[%.3f, %.3f, %.3f] "
                "| actual_nsp_dir=%.4fdeg | %s "
                "| q=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f] deg",
                arm,
                diag.base_T_ee.pose.position.x, diag.base_T_ee.pose.position.y,
                diag.base_T_ee.pose.position.z - dh_d1_,
                diag.base_T_ee.pose.orientation.x, diag.base_T_ee.pose.orientation.y,
                diag.base_T_ee.pose.orientation.z, diag.base_T_ee.pose.orientation.w,
                tracker_y_str.c_str(),
                diag.shoulder_v_elbow[0], diag.shoulder_v_elbow[1], diag.shoulder_v_elbow[2],
                prev_ref_str.c_str(),
                static_cast<long>(diag.request_prev_selected_branch),
                selected_ref_str.c_str(),
                current_upper_arm_dir[0], current_upper_arm_dir[1], current_upper_arm_dir[2],
                current_upper_arm_dir_angle_deg,
                chain.c_str(),
                diag.q_joints_rad[0] * kRad2Deg, diag.q_joints_rad[1] * kRad2Deg,
                diag.q_joints_rad[2] * kRad2Deg, diag.q_joints_rad[3] * kRad2Deg,
                diag.q_joints_rad[4] * kRad2Deg, diag.q_joints_rad[5] * kRad2Deg,
                diag.q_joints_rad[6] * kRad2Deg);
            return;
        }
        RCLCPP_INFO(
            logger,
            "Arm %zu IK | shoulder_T_ee=[%.4f, %.4f, %.4f] quat=[%.4f, %.4f, %.4f, %.4f] "
            "| tracker_y=%s | desired_upper_arm_dir=[%.3f, %.3f, %.3f] "
            "| prev_ref_dir=%s prev_branch=%ld | selected_ref_dir=%s "
            "| actual_nsp_y=NA | %s "
            "| q=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f] deg",
            arm,
            diag.base_T_ee.pose.position.x, diag.base_T_ee.pose.position.y,
            diag.base_T_ee.pose.position.z - dh_d1_,
            diag.base_T_ee.pose.orientation.x, diag.base_T_ee.pose.orientation.y,
            diag.base_T_ee.pose.orientation.z, diag.base_T_ee.pose.orientation.w,
            tracker_y_str.c_str(),
            diag.shoulder_v_elbow[0], diag.shoulder_v_elbow[1], diag.shoulder_v_elbow[2],
            prev_ref_str.c_str(),
            static_cast<long>(diag.request_prev_selected_branch),
            selected_ref_str.c_str(),
            chain.c_str(),
            diag.q_joints_rad[0] * kRad2Deg, diag.q_joints_rad[1] * kRad2Deg,
            diag.q_joints_rad[2] * kRad2Deg, diag.q_joints_rad[3] * kRad2Deg,
            diag.q_joints_rad[4] * kRad2Deg, diag.q_joints_rad[5] * kRad2Deg,
            diag.q_joints_rad[6] * kRad2Deg);
        return;
    }

    if (current_upper_arm_dir_valid) {
        RCLCPP_WARN(
            logger,
            "Arm %zu IK | shoulder_T_ee=[%.4f, %.4f, %.4f] quat=[%.4f, %.4f, %.4f, %.4f] "
            "| tracker_y=%s | desired_upper_arm_dir=[%.3f, %.3f, %.3f] "
            "| prev_ref_dir=%s prev_branch=%ld | selected_ref_dir=%s "
            "| actual_nsp_y=[%.3f, %.3f, %.3f] "
            "| actual_nsp_dir=%.4fdeg | %s",
            arm,
            diag.base_T_ee.pose.position.x, diag.base_T_ee.pose.position.y,
            diag.base_T_ee.pose.position.z - dh_d1_,
            diag.base_T_ee.pose.orientation.x, diag.base_T_ee.pose.orientation.y,
            diag.base_T_ee.pose.orientation.z, diag.base_T_ee.pose.orientation.w,
            tracker_y_str.c_str(),
            diag.shoulder_v_elbow[0], diag.shoulder_v_elbow[1], diag.shoulder_v_elbow[2],
            prev_ref_str.c_str(),
            static_cast<long>(diag.request_prev_selected_branch),
            selected_ref_str.c_str(),
            current_upper_arm_dir[0], current_upper_arm_dir[1], current_upper_arm_dir[2],
            current_upper_arm_dir_angle_deg,
            chain.c_str());
        return;
    }
    RCLCPP_WARN(
        logger,
        "Arm %zu IK | shoulder_T_ee=[%.4f, %.4f, %.4f] quat=[%.4f, %.4f, %.4f, %.4f] "
        "| tracker_y=%s | desired_upper_arm_dir=[%.3f, %.3f, %.3f] "
        "| prev_ref_dir=%s prev_branch=%ld | selected_ref_dir=%s "
        "| actual_nsp_y=NA | %s",
        arm,
        diag.base_T_ee.pose.position.x, diag.base_T_ee.pose.position.y,
        diag.base_T_ee.pose.position.z - dh_d1_,
        diag.base_T_ee.pose.orientation.x, diag.base_T_ee.pose.orientation.y,
        diag.base_T_ee.pose.orientation.z, diag.base_T_ee.pose.orientation.w,
        tracker_y_str.c_str(),
        diag.shoulder_v_elbow[0], diag.shoulder_v_elbow[1], diag.shoulder_v_elbow[2],
        prev_ref_str.c_str(),
        static_cast<long>(diag.request_prev_selected_branch),
        selected_ref_str.c_str(),
        chain.c_str());
}

void TrackerTeleopController::publishVizMarkers(size_t arm, const ArmDiagnostics &diag)
{
    if (!pub_viz_markers_ || !diag.has_base_T_ee) {
        return;
    }

    visualization_msgs::msg::MarkerArray viz_markers;
    auto now_stamp = get_node()->get_clock()->now();
    const std::string &viz_frame = viz_base_frames_[arm];
    const double ox = 0.0, oy = 0.0, oz = dh_d1_;

    visualization_msgs::msg::Marker ee_arrow;
    ee_arrow.header.frame_id = viz_frame;
    ee_arrow.header.stamp = now_stamp;
    ee_arrow.ns = "ee_position";
    ee_arrow.id = static_cast<int>(arm);
    ee_arrow.type = visualization_msgs::msg::Marker::ARROW;
    ee_arrow.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Point p0, p1;
    p0.x = ox; p0.y = oy; p0.z = oz;
    p1.x = diag.base_T_ee.pose.position.x;
    p1.y = diag.base_T_ee.pose.position.y;
    p1.z = diag.base_T_ee.pose.position.z;
    ee_arrow.points.push_back(p0);
    ee_arrow.points.push_back(p1);
    ee_arrow.scale.x = 0.008;
    ee_arrow.scale.y = 0.015;
    ee_arrow.scale.z = 0.02;
    ee_arrow.color.r = (arm == 0) ? 1.0f : 0.2f;
    ee_arrow.color.g = (arm == 0) ? 0.2f : 0.5f;
    ee_arrow.color.b = (arm == 0) ? 0.2f : 1.0f;
    ee_arrow.color.a = 1.0f;
    viz_markers.markers.push_back(ee_arrow);

    auto append_delete_marker = [&](const char *ns) {
        visualization_msgs::msg::Marker marker;
        marker.header = ee_arrow.header;
        marker.ns = ns;
        marker.id = static_cast<int>(arm);
        marker.action = visualization_msgs::msg::Marker::DELETE;
        viz_markers.markers.push_back(marker);
    };

    constexpr double kElbowVizScale = 0.3;
    if (diag.arm_valid) {
        visualization_msgs::msg::Marker elbow_arrow;
        elbow_arrow.header = ee_arrow.header;
        elbow_arrow.ns = "elbow_direction";
        elbow_arrow.id = static_cast<int>(arm);
        elbow_arrow.type = visualization_msgs::msg::Marker::ARROW;
        elbow_arrow.action = visualization_msgs::msg::Marker::ADD;
        geometry_msgs::msg::Point e0, e1;
        e0.x = ox; e0.y = oy; e0.z = oz;
        e1.x = ox + diag.shoulder_v_elbow[0] * kElbowVizScale;
        e1.y = oy + diag.shoulder_v_elbow[1] * kElbowVizScale;
        e1.z = oz + diag.shoulder_v_elbow[2] * kElbowVizScale;
        elbow_arrow.points.push_back(e0);
        elbow_arrow.points.push_back(e1);
        elbow_arrow.scale = ee_arrow.scale;
        elbow_arrow.color.r = (arm == 0) ? 1.0f : 0.0f;
        elbow_arrow.color.g = (arm == 0) ? 0.8f : 0.8f;
        elbow_arrow.color.b = (arm == 0) ? 0.0f : 1.0f;
        elbow_arrow.color.a = 1.0f;
        viz_markers.markers.push_back(elbow_arrow);

        if (diag.solved_upper_arm_dir_valid) {
            visualization_msgs::msg::Marker solved_elbow_arrow;
            solved_elbow_arrow.header = ee_arrow.header;
            solved_elbow_arrow.ns = "solved_elbow_direction";
            solved_elbow_arrow.id = static_cast<int>(arm);
            solved_elbow_arrow.type = visualization_msgs::msg::Marker::ARROW;
            solved_elbow_arrow.action = visualization_msgs::msg::Marker::ADD;
            geometry_msgs::msg::Point s0, s1;
            s0.x = ox; s0.y = oy; s0.z = oz;
            s1.x = ox + diag.solved_upper_arm_dir[0] * kElbowVizScale;
            s1.y = oy + diag.solved_upper_arm_dir[1] * kElbowVizScale;
            s1.z = oz + diag.solved_upper_arm_dir[2] * kElbowVizScale;
            solved_elbow_arrow.points.push_back(s0);
            solved_elbow_arrow.points.push_back(s1);
            solved_elbow_arrow.scale = ee_arrow.scale;
            solved_elbow_arrow.color.r = 0.1f;
            solved_elbow_arrow.color.g = 1.0f;
            solved_elbow_arrow.color.b = 0.2f;
            solved_elbow_arrow.color.a = 1.0f;
            viz_markers.markers.push_back(solved_elbow_arrow);
        } else {
            append_delete_marker("solved_elbow_direction");
        }
    } else {
        append_delete_marker("elbow_direction");
        append_delete_marker("solved_elbow_direction");
    }

    pub_viz_markers_->publish(viz_markers);
    arm_state_[arm].marker_visible = true;
}

void TrackerTeleopController::clearVizMarkers(size_t arm)
{
    if (!pub_viz_markers_ || !arm_state_[arm].marker_visible) {
        return;
    }

    visualization_msgs::msg::MarkerArray clear_markers;
    for (const char *ns : {"ee_position", "elbow_direction", "solved_elbow_direction"}) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = viz_base_frames_[arm];
        marker.header.stamp = get_node()->get_clock()->now();
        marker.ns = ns;
        marker.id = static_cast<int>(arm);
        marker.action = visualization_msgs::msg::Marker::DELETE;
        clear_markers.markers.push_back(marker);
    }
    pub_viz_markers_->publish(clear_markers);
    arm_state_[arm].marker_visible = false;
}

void TrackerTeleopController::diagnosticsTimerCallback()
{
    const auto logger = get_node()->get_logger();
    std::array<ArmDiagnostics, kArmCount> diagnostics;
    std::array<bool, kArmCount> has_pending{{false, false}};

    for (size_t arm = 0; arm < kArmCount; ++arm) {
        auto &slot = arm_state_[arm].pending_diagnostics;
        if (!slot.pending.exchange(false, std::memory_order_acq_rel)) {
            continue;
        }

        while (true) {
            const uint64_t seq_begin = slot.sequence.load(std::memory_order_acquire);
            if ((seq_begin & 1U) != 0U) {
                continue;
            }

            diagnostics[arm] = slot.snapshot;

            const uint64_t seq_end = slot.sequence.load(std::memory_order_acquire);
            if (seq_begin == seq_end) {
                has_pending[arm] = true;
                break;
            }
        }
    }

    for (size_t arm = 0; arm < kArmCount; ++arm) {
        if (!has_pending[arm]) {
            continue;
        }

        const auto &diag = diagnostics[arm];

        publishIKStatus(arm, diag.final_result);

        if (!diag.tracker_fresh) {
            clearVizMarkers(arm);
            RCLCPP_WARN(logger,
                        "Arm %zu tracker data unavailable or stale; holding current joint command.",
                        arm);
            continue;
        }

        if (diag.has_base_T_ee) {
            publishVizMarkers(arm, diag);
        }

        logArmDiagnostics(arm, diag);
    }
}

void TrackerTeleopController::computeAndPublishFK()
{
    const auto logger = get_node()->get_logger();
    static const char *arm_labels[] = {"LEFT", "RIGHT"};

    for (size_t arm = 0; arm < kArmCount; ++arm) {
        FX_DOUBLE joints[kJointsPerArm];
        const auto &runtime = arm_state_[arm];
        for (size_t j = 0; j < kJointsPerArm; ++j) {
            joints[j] = runtime.last_joint_deg[j];
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

}  // namespace marvin_system
