#include <cmath>
#include <string>
#include <vector>
#include <map>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "openvr.h"

struct TrackerConfig {
    std::string serial;
    std::string frame_id;
    std::string role;
};

struct RoleCorrection {
    double offset[3] = {0.0, 0.0, 0.0};
    double rot[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
};

// ── helpers ──────────────────────────────────────────────────────────────

static void quaternionToRotationMatrix(double qx, double qy, double qz, double qw,
                                       double (&rot)[3][3])
{
    rot[0][0] = 1.0 - 2.0 * (qy * qy + qz * qz);
    rot[0][1] = 2.0 * (qx * qy - qz * qw);
    rot[0][2] = 2.0 * (qx * qz + qy * qw);
    rot[1][0] = 2.0 * (qx * qy + qz * qw);
    rot[1][1] = 1.0 - 2.0 * (qx * qx + qz * qz);
    rot[1][2] = 2.0 * (qy * qz - qx * qw);
    rot[2][0] = 2.0 * (qx * qz - qy * qw);
    rot[2][1] = 2.0 * (qy * qz + qx * qw);
    rot[2][2] = 1.0 - 2.0 * (qx * qx + qy * qy);
}

static void applyRotationCorrection(double (&rot)[3][3], const double (&corr)[3][3])
{
    double tmp[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            tmp[i][j] = 0.0;
            for (int k = 0; k < 3; ++k)
                tmp[i][j] += rot[i][k] * corr[k][j];
        }
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            rot[i][j] = tmp[i][j];
}

static void rotationToQuaternion(const double (&rot)[3][3],
                                 double &qx, double &qy, double &qz, double &qw)
{
    double trace = rot[0][0] + rot[1][1] + rot[2][2];
    if (trace > 0) {
        double s = 0.5 / std::sqrt(trace + 1.0);
        qw = 0.25 / s;
        qx = (rot[2][1] - rot[1][2]) * s;
        qy = (rot[0][2] - rot[2][0]) * s;
        qz = (rot[1][0] - rot[0][1]) * s;
    } else if (rot[0][0] > rot[1][1] && rot[0][0] > rot[2][2]) {
        double s = 2.0 * std::sqrt(1.0 + rot[0][0] - rot[1][1] - rot[2][2]);
        qw = (rot[2][1] - rot[1][2]) / s;
        qx = 0.25 * s;
        qy = (rot[0][1] + rot[1][0]) / s;
        qz = (rot[0][2] + rot[2][0]) / s;
    } else if (rot[1][1] > rot[2][2]) {
        double s = 2.0 * std::sqrt(1.0 + rot[1][1] - rot[0][0] - rot[2][2]);
        qw = (rot[0][2] - rot[2][0]) / s;
        qx = (rot[0][1] + rot[1][0]) / s;
        qy = 0.25 * s;
        qz = (rot[1][2] + rot[2][1]) / s;
    } else {
        double s = 2.0 * std::sqrt(1.0 + rot[2][2] - rot[0][0] - rot[1][1]);
        qw = (rot[1][0] - rot[0][1]) / s;
        qx = (rot[0][2] + rot[2][0]) / s;
        qy = (rot[1][2] + rot[2][1]) / s;
        qz = 0.25 * s;
    }
}

// SteamVR 3×4 HmdMatrix34_t → TF TransformStamped.
//
// Pipeline:
//   1. vr_T_tracker 原始 OpenVR 位姿
//   2. vr 系下 tracker 局部偏置 + 姿态校正（trackers.yaml corrections）
//   3. ros_T_vr 映射 → world_ros_T_tracker 写入 TF
//
// SteamVR: X=right, Y=up, Z=backward   (right-handed)
// ROS:     X=forward, Y=left, Z=up     (right-handed, REP-103)
// 链: world_vr_T_tracker → 腕部偏置/姿态校正 → world_ros_T_tracker (等价于 ros_R_vr * …)
static geometry_msgs::msg::TransformStamped matrixToTransform(
    const vr::HmdMatrix34_t &m,
    const std::string &parent_frame, const std::string &child_frame,
    rclcpp::Time stamp, bool apply_ros_T_vr,
    const RoleCorrection &correction)
{
    geometry_msgs::msg::TransformStamped world_ros_T_tracker;
    world_ros_T_tracker.header.stamp = stamp;
    world_ros_T_tracker.header.frame_id = parent_frame;
    world_ros_T_tracker.child_frame_id = child_frame;

    // Step 1: vr_T_tracker（OpenVR standing 系下设备位姿）
    double vr_R_tracker[3][3];
    double vr_t_tracker[3] = {m.m[0][3], m.m[1][3], m.m[2][3]};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            vr_R_tracker[i][j] = m.m[i][j];

    // Step 2: tracker 局部偏置 vr_p_wrist = vr_p_tracker + vr_R_tracker * p_offset（corrections.yaml）
    const auto &off = correction.offset;
    vr_t_tracker[0] += vr_R_tracker[0][0] * off[0] + vr_R_tracker[0][1] * off[1]
                        + vr_R_tracker[0][2] * off[2];
    vr_t_tracker[1] += vr_R_tracker[1][0] * off[0] + vr_R_tracker[1][1] * off[1]
                        + vr_R_tracker[1][2] * off[2];
    vr_t_tracker[2] += vr_R_tracker[2][0] * off[0] + vr_R_tracker[2][1] * off[1]
                        + vr_R_tracker[2][2] * off[2];

    // Step 3: VR 系下右乘姿态校正 vr_R_tracker ← vr_R_tracker * R_corr
    applyRotationCorrection(vr_R_tracker, correction.rot);

    // Step 4: ros_T_vr：SteamVR → ROS（REP-103）
    double ros_t_tracker[3];
    double ros_R_tracker[3][3];

    if (apply_ros_T_vr) {
        ros_t_tracker[0] = -vr_t_tracker[2];
        ros_t_tracker[1] = -vr_t_tracker[0];
        ros_t_tracker[2] =  vr_t_tracker[1];
        ros_R_tracker[0][0] =  vr_R_tracker[2][2];
        ros_R_tracker[0][1] =  vr_R_tracker[2][0];
        ros_R_tracker[0][2] = -vr_R_tracker[2][1];
        ros_R_tracker[1][0] =  vr_R_tracker[0][2];
        ros_R_tracker[1][1] =  vr_R_tracker[0][0];
        ros_R_tracker[1][2] = -vr_R_tracker[0][1];
        ros_R_tracker[2][0] = -vr_R_tracker[1][2];
        ros_R_tracker[2][1] = -vr_R_tracker[1][0];
        ros_R_tracker[2][2] =  vr_R_tracker[1][1];
    } else {
        ros_t_tracker[0] = vr_t_tracker[0];
        ros_t_tracker[1] = vr_t_tracker[1];
        ros_t_tracker[2] = vr_t_tracker[2];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                ros_R_tracker[i][j] = vr_R_tracker[i][j];
    }

    world_ros_T_tracker.transform.translation.x = ros_t_tracker[0];
    world_ros_T_tracker.transform.translation.y = ros_t_tracker[1];
    world_ros_T_tracker.transform.translation.z = ros_t_tracker[2];

    double qx, qy, qz, qw;
    rotationToQuaternion(ros_R_tracker, qx, qy, qz, qw);
    world_ros_T_tracker.transform.rotation.x = qx;
    world_ros_T_tracker.transform.rotation.y = qy;
    world_ros_T_tracker.transform.rotation.z = qz;
    world_ros_T_tracker.transform.rotation.w = qw;
    return world_ros_T_tracker;
}

// ── node ─────────────────────────────────────────────────────────────────

class TrackerPublisher : public rclcpp::Node
{
public:
    TrackerPublisher()
        : Node("tracker_publisher")
    {
        coordinate_transform_ = declare_parameter("coordinate_transform", true);
        double rate = declare_parameter("publish_rate", 100.0);
        parent_frame_ = declare_parameter("parent_frame", "world");

        auto tracker_serials = declare_parameter<std::vector<std::string>>(
            "trackers.serials", std::vector<std::string>{});
        auto tracker_frame_ids = declare_parameter<std::vector<std::string>>(
            "trackers.frame_ids", std::vector<std::string>{});
        auto tracker_roles = declare_parameter<std::vector<std::string>>(
            "trackers.roles", std::vector<std::string>{});

        if (!tracker_serials.empty() &&
            tracker_serials.size() == tracker_frame_ids.size()) {
            for (size_t i = 0; i < tracker_serials.size(); i++) {
                TrackerConfig cfg;
                cfg.serial = tracker_serials[i];
                cfg.frame_id = tracker_frame_ids[i];
                cfg.role = (i < tracker_roles.size()) ? tracker_roles[i] : "";
                tracker_configs_.push_back(cfg);
                RCLCPP_INFO(get_logger(),
                            "Config tracker: serial=%s frame=%s role=%s",
                            cfg.serial.c_str(), cfg.frame_id.c_str(),
                            cfg.role.c_str());
            }
        }

        loadCorrections(tracker_roles);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        vr::EVRInitError err = vr::VRInitError_None;
        vr_system_ = vr::VR_Init(&err, vr::VRApplication_Other);
        if (err != vr::VRInitError_None) {
            RCLCPP_FATAL(get_logger(), "OpenVR init failed: %s",
                         vr::VR_GetVRInitErrorAsEnglishDescription(err));
            throw std::runtime_error("OpenVR init failed");
        }
        RCLCPP_INFO(get_logger(), "OpenVR initialized, publishing TF to parent_frame='%s'",
                    parent_frame_.c_str());

        scanDevices();

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate),
            std::bind(&TrackerPublisher::update, this));

        scan_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&TrackerPublisher::scanDevices, this));
    }

    ~TrackerPublisher() override
    {
        vr::VR_Shutdown();
        RCLCPP_INFO(get_logger(), "OpenVR shutdown");
    }

private:
    struct TrackerState {
        uint32_t device_index;
        std::string serial;
        std::string frame_id;
        std::string role;
    };

    void loadCorrections(const std::vector<std::string> &roles)
    {
        // Collect unique role names
        std::vector<std::string> unique_roles;
        for (const auto &r : roles) {
            if (!r.empty() && role_corrections_.find(r) == role_corrections_.end()) {
                unique_roles.push_back(r);
                role_corrections_[r] = RoleCorrection{};
            }
        }

        for (const auto &role : unique_roles) {
            auto &corr = role_corrections_[role];

            std::string pos_key = "corrections." + role + ".position";
            std::string ori_key = "corrections." + role + ".orientation";

            auto offset_vec = declare_parameter<std::vector<double>>(
                pos_key, {0.0, 0.0, 0.0});
            auto rotation_vec = declare_parameter<std::vector<double>>(
                ori_key, {0.0, 0.0, 0.0, 1.0});

            if (offset_vec.size() >= 3) {
                corr.offset[0] = offset_vec[0];
                corr.offset[1] = offset_vec[1];
                corr.offset[2] = offset_vec[2];
            }

            if (rotation_vec.size() >= 4) {
                quaternionToRotationMatrix(
                    rotation_vec[0], rotation_vec[1],
                    rotation_vec[2], rotation_vec[3],
                    corr.rot);
            }

            RCLCPP_INFO(get_logger(),
                "Correction [%s]: offset=[%.4f, %.4f, %.4f]  "
                "rotation=[%.4f, %.4f, %.4f, %.4f]",
                role.c_str(),
                corr.offset[0], corr.offset[1], corr.offset[2],
                rotation_vec.size() >= 4 ? rotation_vec[0] : 0.0,
                rotation_vec.size() >= 4 ? rotation_vec[1] : 0.0,
                rotation_vec.size() >= 4 ? rotation_vec[2] : 0.0,
                rotation_vec.size() >= 4 ? rotation_vec[3] : 1.0);
        }
    }

    void scanDevices()
    {
        for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
            auto cls = vr_system_->GetTrackedDeviceClass(i);
            if (cls == vr::TrackedDeviceClass_Invalid)
                continue;

            char serial_buf[256] = {};
            vr_system_->GetStringTrackedDeviceProperty(
                i, vr::Prop_SerialNumber_String, serial_buf, sizeof(serial_buf));
            std::string serial(serial_buf);

            char model_buf[256] = {};
            vr_system_->GetStringTrackedDeviceProperty(
                i, vr::Prop_ModelNumber_String, model_buf, sizeof(model_buf));

            const char *class_name = "Unknown";
            switch (cls) {
                case vr::TrackedDeviceClass_HMD:              class_name = "HMD"; break;
                case vr::TrackedDeviceClass_Controller:       class_name = "Controller"; break;
                case vr::TrackedDeviceClass_GenericTracker:    class_name = "Tracker"; break;
                case vr::TrackedDeviceClass_TrackingReference: class_name = "BaseStation"; break;
                default: break;
            }

            if (known_serials_.count(serial))
                continue;
            known_serials_[serial] = i;

            RCLCPP_INFO(get_logger(), "Device[%u] type=%s serial=%s model=%s",
                        i, class_name, serial.c_str(), model_buf);

            if (cls != vr::TrackedDeviceClass_GenericTracker)
                continue;

            TrackerState state;
            state.device_index = i;
            state.serial = serial;

            bool matched = false;
            for (auto &cfg : tracker_configs_) {
                if (cfg.serial == serial) {
                    state.frame_id = cfg.frame_id;
                    state.role = cfg.role;
                    matched = true;
                    RCLCPP_INFO(get_logger(), "Tracker matched config: %s -> TF '%s' (role=%s)",
                                serial.c_str(), cfg.frame_id.c_str(), cfg.role.c_str());
                    break;
                }
            }

            if (!matched) {
                std::string safe_serial = serial;
                for (auto &c : safe_serial)
                    if (c == '-') c = '_';
                state.frame_id = "tracker_" + safe_serial;
                RCLCPP_WARN(get_logger(),
                    "Tracker %s not in config, auto-assigned TF frame: %s",
                    serial.c_str(), state.frame_id.c_str());
            }

            active_trackers_.push_back(state);
        }

        if (active_trackers_.empty()) {
            RCLCPP_WARN(get_logger(), "No trackers found yet. Waiting for devices...");
        }
    }

    void update()
    {
        vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
        vr_system_->GetDeviceToAbsoluteTrackingPose(
            vr::TrackingUniverseStanding, 0, poses, vr::k_unMaxTrackedDeviceCount);

        auto stamp = now();
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        transforms.reserve(active_trackers_.size());

        for (auto &tracker : active_trackers_) {
            auto idx = tracker.device_index;
            if (!poses[idx].bPoseIsValid)
                continue;

            auto it = role_corrections_.find(tracker.role);
            const auto &correction = (it != role_corrections_.end())
                                         ? it->second
                                         : identity_correction_;

            auto tf = matrixToTransform(
                poses[idx].mDeviceToAbsoluteTracking,
                parent_frame_, tracker.frame_id,
                stamp, coordinate_transform_,
                correction);

            transforms.push_back(tf);

            RCLCPP_DEBUG(get_logger(),
                "[%s] pos=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)",
                tracker.serial.c_str(),
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z,
                tf.transform.rotation.x, tf.transform.rotation.y,
                tf.transform.rotation.z, tf.transform.rotation.w);
        }

        if (!transforms.empty()) {
            tf_broadcaster_->sendTransform(transforms);
        }
    }

    vr::IVRSystem *vr_system_ = nullptr;
    bool coordinate_transform_;
    std::string parent_frame_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::vector<TrackerConfig> tracker_configs_;
    std::vector<TrackerState> active_trackers_;
    std::map<std::string, uint32_t> known_serials_;
    std::map<std::string, RoleCorrection> role_corrections_;
    RoleCorrection identity_correction_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr scan_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackerPublisher>());
    rclcpp::shutdown();
    return 0;
}
