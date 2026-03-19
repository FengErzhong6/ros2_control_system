#include <cmath>
#include <string>
#include <vector>
#include <map>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "openvr.h"

struct TrackerConfig {
    std::string serial;
    std::string frame_id;
    std::string role;
};

struct RoleCorrection {
    tf2::Vector3 offset{0.0, 0.0, 0.0};
    tf2::Quaternion rotation{0.0, 0.0, 0.0, 1.0};
};

// SteamVR 3×4 HmdMatrix34_t → TF TransformStamped.
//
// Pipeline:
//   1. vr_T_tracker 原始 OpenVR 位姿
//   2. vr 系下 tracker 局部偏置 + 姿态校正（trackers.yaml corrections）
//   3. ros_T_vr 映射 → world_ros_T_tracker 写入 TF
//
// SteamVR: X=right, Y=up, Z=backward   (right-handed)
// ROS:     X=forward, Y=left, Z=up     (right-handed, REP-103)
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
    tf2::Matrix3x3 vr_R(m.m[0][0], m.m[0][1], m.m[0][2],
                         m.m[1][0], m.m[1][1], m.m[1][2],
                         m.m[2][0], m.m[2][1], m.m[2][2]);
    tf2::Vector3 vr_t(m.m[0][3], m.m[1][3], m.m[2][3]);

    // Step 2: tracker 局部偏置（corrections.yaml）
    //   vr_p_wrist = vr_p_tracker + vr_R_tracker * p_offset
    vr_t += vr_R * correction.offset;

    // Step 3: VR 系下右乘姿态校正 vr_R ← vr_R * R_corr
    tf2::Matrix3x3 R_corr(correction.rotation);
    vr_R *= R_corr;

    // Step 4: ros_T_vr：SteamVR → ROS（REP-103）
    tf2::Vector3 ros_t;
    tf2::Matrix3x3 ros_R;

    if (apply_ros_T_vr) {
        // ros_T_vr = [[-Z], [-X], [Y]]  for both position and rotation basis
        tf2::Matrix3x3 ros_T_vr(
             0,  0, -1,
            -1,  0,  0,
             0,  1,  0);
        ros_t = ros_T_vr * vr_t;
        ros_R = ros_T_vr * vr_R * ros_T_vr.transpose();
    } else {
        ros_t = vr_t;
        ros_R = vr_R;
    }

    world_ros_T_tracker.transform.translation.x = ros_t.x();
    world_ros_T_tracker.transform.translation.y = ros_t.y();
    world_ros_T_tracker.transform.translation.z = ros_t.z();

    tf2::Quaternion q;
    ros_R.getRotation(q);
    world_ros_T_tracker.transform.rotation.x = q.x();
    world_ros_T_tracker.transform.rotation.y = q.y();
    world_ros_T_tracker.transform.rotation.z = q.z();
    world_ros_T_tracker.transform.rotation.w = q.w();
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
                corr.offset.setValue(offset_vec[0], offset_vec[1], offset_vec[2]);
            }

            if (rotation_vec.size() >= 4) {
                corr.rotation.setValue(rotation_vec[0], rotation_vec[1],
                                      rotation_vec[2], rotation_vec[3]);
            }

            RCLCPP_INFO(get_logger(),
                "Correction [%s]: offset=[%.4f, %.4f, %.4f]  "
                "rotation=[%.4f, %.4f, %.4f, %.4f]",
                role.c_str(),
                corr.offset.x(), corr.offset.y(), corr.offset.z(),
                corr.rotation.x(), corr.rotation.y(),
                corr.rotation.z(), corr.rotation.w());
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
