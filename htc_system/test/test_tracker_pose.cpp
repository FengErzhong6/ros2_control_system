#include <cmath>
#include <string>
#include <vector>
#include <map>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "openvr.h"

struct TrackerConfig {
    std::string serial;
    std::string frame_id;
    std::string topic;
};

// SteamVR 3x4 HmdMatrix34_t → ROS PoseStamped
// SteamVR: X-right, Y-up,  Z-backward
// ROS:     X-forward, Y-left, Z-up
static geometry_msgs::msg::PoseStamped matrixToPose(
    const vr::HmdMatrix34_t &m, const std::string &frame_id,
    rclcpp::Time stamp, bool transform_coords)
{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;

    double rot[3][3];
    double tx, ty, tz;

    if (transform_coords) {
        // SteamVR → ROS coordinate remapping
        // SteamVR: X=right, Y=up, Z=backward (right-handed)
        // ROS:     X=forward, Y=left, Z=up   (right-handed, REP-103)
        //
        // Mapping: ROS_X = -SVR_Z, ROS_Y = -SVR_X, ROS_Z = SVR_Y
        // Position: t_ros = M * t_svr
        // Rotation: R_ros = M * R_svr * M^T   (similarity transform)
        //
        //     M = | 0  0 -1 |    M^T = | 0 -1  0 |
        //         |-1  0  0 |          | 0  0  1 |
        //         | 0  1  0 |          |-1  0  0 |
        tx = -m.m[2][3];
        ty = -m.m[0][3];
        tz =  m.m[1][3];
        rot[0][0] =  m.m[2][2]; rot[0][1] =  m.m[2][0]; rot[0][2] = -m.m[2][1];
        rot[1][0] =  m.m[0][2]; rot[1][1] =  m.m[0][0]; rot[1][2] = -m.m[0][1];
        rot[2][0] = -m.m[1][2]; rot[2][1] = -m.m[1][0]; rot[2][2] =  m.m[1][1];
    } else {
        tx = m.m[0][3];
        ty = m.m[1][3];
        tz = m.m[2][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                rot[i][j] = m.m[i][j];
    }

    msg.pose.position.x = tx;
    msg.pose.position.y = ty;
    msg.pose.position.z = tz;

    // Rotation matrix → quaternion (Shepperd's method)
    double trace = rot[0][0] + rot[1][1] + rot[2][2];
    double w, x, y, z;
    if (trace > 0) {
        double s = 0.5 / std::sqrt(trace + 1.0);
        w = 0.25 / s;
        x = (rot[2][1] - rot[1][2]) * s;
        y = (rot[0][2] - rot[2][0]) * s;
        z = (rot[1][0] - rot[0][1]) * s;
    } else if (rot[0][0] > rot[1][1] && rot[0][0] > rot[2][2]) {
        double s = 2.0 * std::sqrt(1.0 + rot[0][0] - rot[1][1] - rot[2][2]);
        w = (rot[2][1] - rot[1][2]) / s;
        x = 0.25 * s;
        y = (rot[0][1] + rot[1][0]) / s;
        z = (rot[0][2] + rot[2][0]) / s;
    } else if (rot[1][1] > rot[2][2]) {
        double s = 2.0 * std::sqrt(1.0 + rot[1][1] - rot[0][0] - rot[2][2]);
        w = (rot[0][2] - rot[2][0]) / s;
        x = (rot[0][1] + rot[1][0]) / s;
        y = 0.25 * s;
        z = (rot[1][2] + rot[2][1]) / s;
    } else {
        double s = 2.0 * std::sqrt(1.0 + rot[2][2] - rot[0][0] - rot[1][1]);
        w = (rot[1][0] - rot[0][1]) / s;
        x = (rot[0][2] + rot[2][0]) / s;
        y = (rot[1][2] + rot[2][1]) / s;
        z = 0.25 * s;
    }

    msg.pose.orientation.x = x;
    msg.pose.orientation.y = y;
    msg.pose.orientation.z = z;
    msg.pose.orientation.w = w;
    return msg;
}

class TestTrackerPose : public rclcpp::Node
{
public:
    TestTrackerPose()
        : Node("test_tracker_pose")
    {
        coordinate_transform_ = declare_parameter("coordinate_transform", true);
        double rate = declare_parameter("publish_rate", 100.0);

        // Read tracker list from YAML parameters
        auto tracker_serials = declare_parameter<std::vector<std::string>>(
            "trackers.serials", std::vector<std::string>{});
        auto tracker_frame_ids = declare_parameter<std::vector<std::string>>(
            "trackers.frame_ids", std::vector<std::string>{});
        auto tracker_topics = declare_parameter<std::vector<std::string>>(
            "trackers.topics", std::vector<std::string>{});

        if (!tracker_serials.empty() &&
            tracker_serials.size() == tracker_frame_ids.size() &&
            tracker_serials.size() == tracker_topics.size()) {
            for (size_t i = 0; i < tracker_serials.size(); i++) {
                TrackerConfig cfg;
                cfg.serial = tracker_serials[i];
                cfg.frame_id = tracker_frame_ids[i];
                cfg.topic = tracker_topics[i];
                tracker_configs_.push_back(cfg);
                RCLCPP_INFO(get_logger(), "Config tracker: serial=%s frame=%s topic=%s",
                            cfg.serial.c_str(), cfg.frame_id.c_str(), cfg.topic.c_str());
            }
        }

        // Initialize OpenVR
        vr::EVRInitError err = vr::VRInitError_None;
        vr_system_ = vr::VR_Init(&err, vr::VRApplication_Other);
        if (err != vr::VRInitError_None) {
            RCLCPP_FATAL(get_logger(), "OpenVR init failed: %s",
                         vr::VR_GetVRInitErrorAsEnglishDescription(err));
            throw std::runtime_error("OpenVR init failed");
        }
        RCLCPP_INFO(get_logger(), "OpenVR initialized successfully");

        scanDevices();

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate),
            std::bind(&TestTrackerPose::update, this));

        // Periodically re-scan for hot-plugged trackers
        scan_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&TestTrackerPose::scanDevices, this));
    }

    ~TestTrackerPose() override
    {
        vr::VR_Shutdown();
        RCLCPP_INFO(get_logger(), "OpenVR shutdown");
    }

private:
    struct TrackerState {
        uint32_t device_index;
        std::string serial;
        std::string frame_id;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
    };

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

            // Only track GenericTracker devices
            if (cls != vr::TrackedDeviceClass_GenericTracker)
                continue;

            TrackerState state;
            state.device_index = i;
            state.serial = serial;

            // Match against config or auto-assign
            bool matched = false;
            for (auto &cfg : tracker_configs_) {
                if (cfg.serial == serial) {
                    state.frame_id = cfg.frame_id;
                    state.publisher = create_publisher<geometry_msgs::msg::PoseStamped>(
                        cfg.topic, 10);
                    matched = true;
                    RCLCPP_INFO(get_logger(), "Tracker matched config: %s → %s",
                                serial.c_str(), cfg.topic.c_str());
                    break;
                }
            }

            if (!matched) {
                // Auto-generate topic and frame for unconfigured trackers
                std::string safe_serial = serial;
                for (auto &c : safe_serial)
                    if (c == '-') c = '_';
                state.frame_id = "tracker_" + safe_serial;
                std::string topic = "/htc/tracker_" + safe_serial + "/pose";
                state.publisher = create_publisher<geometry_msgs::msg::PoseStamped>(
                    topic, 10);
                RCLCPP_WARN(get_logger(),
                    "Tracker %s not in config, auto-assigned: topic=%s frame=%s",
                    serial.c_str(), topic.c_str(), state.frame_id.c_str());
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

        for (auto &tracker : active_trackers_) {
            auto idx = tracker.device_index;
            if (!poses[idx].bPoseIsValid)
                continue;

            auto msg = matrixToPose(
                poses[idx].mDeviceToAbsoluteTracking,
                tracker.frame_id, stamp, coordinate_transform_);

            tracker.publisher->publish(msg);

            RCLCPP_DEBUG(get_logger(),
                "[%s] pos=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)",
                tracker.serial.c_str(),
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w);
        }

        // 1Hz INFO print for quick inspection
        print_counter_++;
        if (print_counter_ >= static_cast<int>(print_interval_hz_)) {
            print_counter_ = 0;
            for (auto &tracker : active_trackers_) {
                auto idx = tracker.device_index;
                if (!poses[idx].bPoseIsValid) continue;
                auto msg = matrixToPose(
                    poses[idx].mDeviceToAbsoluteTracking,
                    tracker.frame_id, stamp, coordinate_transform_);
                RCLCPP_INFO(get_logger(),
                    "[%s] pos=(%.4f, %.4f, %.4f) m  quat=(%.4f, %.4f, %.4f, %.4f)",
                    tracker.serial.c_str(),
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                    msg.pose.orientation.x, msg.pose.orientation.y,
                    msg.pose.orientation.z, msg.pose.orientation.w);
            }
        }
    }

    vr::IVRSystem *vr_system_ = nullptr;
    bool coordinate_transform_;
    std::vector<TrackerConfig> tracker_configs_;
    std::vector<TrackerState> active_trackers_;
    std::map<std::string, uint32_t> known_serials_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr scan_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestTrackerPose>();
    RCLCPP_INFO(node->get_logger(),
        "=== Tracker Pose Test ===\n"
        "  Use 'ros2 topic echo /htc/tracker_xxx/pose' to view pose data\n"
        "  Set log level to DEBUG for per-frame output:\n"
        "    ros2 run htc_system test_tracker_pose --ros-args --log-level debug");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
