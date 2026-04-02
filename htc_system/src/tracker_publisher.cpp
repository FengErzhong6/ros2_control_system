#include <cmath>
#include <chrono>
#include <cstdlib>
#include <dlfcn.h>
#include <filesystem>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_prefix.hpp"
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

class OpenVrLibrary
{
public:
    using InitInternalFn = decltype(&vr::VR_InitInternal2);
    using ShutdownInternalFn = decltype(&vr::VR_ShutdownInternal);
    using GetGenericInterfaceFn = decltype(&vr::VR_GetGenericInterface);
    using GetErrorDescriptionFn =
        decltype(&vr::VR_GetVRInitErrorAsEnglishDescription);

    ~OpenVrLibrary()
    {
        shutdown();
    }

    void initialize(const std::string &override_path)
    {
        std::vector<std::filesystem::path> candidates =
            candidateLibraryPaths(override_path);
        if (candidates.empty()) {
            throw std::runtime_error("No OpenVR library path candidates available");
        }

        const int dlopen_flags =
            RTLD_NOW | RTLD_LOCAL
#ifdef RTLD_DEEPBIND
            | RTLD_DEEPBIND
#endif
            ;

        std::ostringstream attempts;
        bool first_attempt = true;
        for (const auto &candidate : candidates) {
            if (!first_attempt) {
                attempts << "; ";
            }
            first_attempt = false;

            if (!std::filesystem::exists(candidate)) {
                attempts << candidate << " (missing)";
                continue;
            }

            dlerror();
            handle_ = dlopen(candidate.c_str(), dlopen_flags);
            if (handle_ != nullptr) {
                library_path_ = candidate;
                break;
            }

            const char *error = dlerror();
            attempts << candidate << " (" << (error != nullptr ? error : "dlopen failed")
                     << ")";
        }

        if (handle_ == nullptr) {
            throw std::runtime_error(
                "Failed to load OpenVR runtime. Attempts: " + attempts.str());
        }

        init_internal_ = loadSymbol<InitInternalFn>("VR_InitInternal2");
        shutdown_internal_ = loadSymbol<ShutdownInternalFn>("VR_ShutdownInternal");
        get_generic_interface_ =
            loadSymbol<GetGenericInterfaceFn>("VR_GetGenericInterface");
        get_error_description_ = loadSymbol<GetErrorDescriptionFn>(
            "VR_GetVRInitErrorAsEnglishDescription");

        vr::EVRInitError err = vr::VRInitError_None;
        init_internal_(&err, vr::VRApplication_Other, nullptr);
        if (err != vr::VRInitError_None) {
            const std::string description = initErrorDescription(err);
            unloadHandle();
            throw std::runtime_error("OpenVR init failed: " + description);
        }
        initialized_ = true;

        err = vr::VRInitError_None;
        vr_system_ = static_cast<vr::IVRSystem *>(
            get_generic_interface_(vr::IVRSystem_Version, &err));
        if (err != vr::VRInitError_None || vr_system_ == nullptr) {
            const std::string description = initErrorDescription(err);
            shutdown();
            throw std::runtime_error(
                "Failed to acquire OpenVR IVRSystem interface: " + description);
        }
    }

    void shutdown()
    {
        if (initialized_ && shutdown_internal_ != nullptr) {
            shutdown_internal_();
        }
        initialized_ = false;
        shutdown_internal_ = nullptr;
        init_internal_ = nullptr;
        get_generic_interface_ = nullptr;
        get_error_description_ = nullptr;
        vr_system_ = nullptr;
        unloadHandle();
    }

    vr::IVRSystem *system() const
    {
        return vr_system_;
    }

    const std::filesystem::path &libraryPath() const
    {
        return library_path_;
    }

private:
    template <typename Fn>
    Fn loadSymbol(const char *name)
    {
        dlerror();
        void *symbol = dlsym(handle_, name);
        if (const char *error = dlerror(); error != nullptr) {
            throw std::runtime_error(
                std::string("Failed to resolve OpenVR symbol '") + name +
                "': " + error);
        }
        return reinterpret_cast<Fn>(symbol);
    }

    static std::vector<std::filesystem::path> candidateLibraryPaths(
        const std::string &override_path)
    {
        std::vector<std::filesystem::path> candidates;
        auto append = [&candidates](const std::filesystem::path &path) {
            if (path.empty()) {
                return;
            }
            for (const auto &existing : candidates) {
                if (existing == path) {
                    return;
                }
            }
            candidates.push_back(path);
        };

        if (!override_path.empty()) {
            append(override_path);
        }

        if (const char *env_path = std::getenv("HTC_SYSTEM_OPENVR_LIBRARY");
            env_path != nullptr && env_path[0] != '\0') {
            append(env_path);
        }

        std::error_code ec;
        const auto exe_path = std::filesystem::read_symlink("/proc/self/exe", ec);
        if (!ec) {
            append(exe_path.parent_path() / "libopenvr_api.so");
            append(exe_path.parent_path().parent_path() / "libopenvr_api.so");
        }

        try {
            const auto package_prefix =
                std::filesystem::path(ament_index_cpp::get_package_prefix("htc_system"));
            append(package_prefix / "lib" / "htc_system" / "libopenvr_api.so");
            append(package_prefix / "lib" / "libopenvr_api.so");
        } catch (const std::exception &) {
            // Allow direct executable runs outside an installed environment.
        }

        append(std::filesystem::path(HTC_SYSTEM_SOURCE_DIR) / "third_party" /
               "openvr" / "libopenvr_api.so");
        return candidates;
    }

    std::string initErrorDescription(vr::EVRInitError err) const
    {
        if (get_error_description_ == nullptr) {
            return "unknown";
        }
        const char *description = get_error_description_(err);
        return description != nullptr ? description : "unknown";
    }

    void unloadHandle()
    {
        if (handle_ != nullptr) {
            dlclose(handle_);
            handle_ = nullptr;
        }
        library_path_.clear();
    }

    void *handle_ = nullptr;
    InitInternalFn init_internal_ = nullptr;
    ShutdownInternalFn shutdown_internal_ = nullptr;
    GetGenericInterfaceFn get_generic_interface_ = nullptr;
    GetErrorDescriptionFn get_error_description_ = nullptr;
    vr::IVRSystem *vr_system_ = nullptr;
    std::filesystem::path library_path_;
    bool initialized_ = false;
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
        openvr_library_path_ =
            declare_parameter("openvr_library_path", std::string{});

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

        try {
            openvr_.initialize(openvr_library_path_);
        } catch (const std::exception &e) {
            RCLCPP_FATAL(
                get_logger(), "OpenVR runtime initialization failed: %s", e.what());
            throw;
        }
        vr_system_ = openvr_.system();
        const std::string openvr_library = openvr_.libraryPath().string();
        RCLCPP_INFO(
            get_logger(),
            "OpenVR initialized from %s, publishing TF to parent_frame='%s'",
            openvr_library.c_str(), parent_frame_.c_str());

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
        openvr_.shutdown();
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
    OpenVrLibrary openvr_;
    bool coordinate_transform_;
    std::string parent_frame_;
    std::string openvr_library_path_;
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
