#include "marvin_system/collision_guard.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <exception>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/collision_detection/collision_common.hpp"
#include "moveit/planning_scene/planning_scene.hpp"
#include "moveit/rdf_loader/rdf_loader.hpp"
#include "moveit/robot_model_loader/robot_model_loader.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

namespace {

constexpr double kDeg2Rad = M_PI / 180.0;

const std::array<const char *, marvin_system::CollisionGuard::kJointsPerArm> kLeftJointNames{
    {"Joint1_L", "Joint2_L", "Joint3_L", "Joint4_L", "Joint5_L", "Joint6_L", "Joint7_L"}};
const std::array<const char *, marvin_system::CollisionGuard::kJointsPerArm> kRightJointNames{
    {"Joint1_R", "Joint2_R", "Joint3_R", "Joint4_R", "Joint5_R", "Joint6_R", "Joint7_R"}};
const std::array<const char *, marvin_system::CollisionGuard::kGripperCount> kGripperJointNames{
    {"gripper_L", "gripper_R"}};
const std::array<const char *, marvin_system::CollisionGuard::kGripperCount> kGripperAttachFrames{
    {"ee_L", "ee_R"}};
const std::array<const char *, marvin_system::CollisionGuard::kGripperCount> kGripperObjectIds{
    {"collision_guard_gripper_L", "collision_guard_gripper_R"}};
const std::array<std::array<const char *, 8>, marvin_system::CollisionGuard::kGripperCount>
    kGripperTouchLinks{{
        {{"Link7_L", "ee_L", "gripper_L_link", "left_finger_L_link", "right_finger_L_link", "hand_mount_L", "left_palm_link", "left_finger1_link1"}},
        {{"Link7_R", "ee_R", "gripper_R_link", "left_finger_R_link", "right_finger_R_link", "hand_mount_R", "right_palm_link", "right_finger1_link1"}},
    }};

std::string param_str(
    const std::unordered_map<std::string, std::string> &params,
    const std::string &key,
    const std::string &default_value = {})
{
    const auto it = params.find(key);
    return it == params.end() ? default_value : it->second;
}

bool param_bool(
    const std::unordered_map<std::string, std::string> &params,
    const std::string &key,
    bool default_value)
{
    const auto it = params.find(key);
    if (it == params.end()) {
        return default_value;
    }
    std::string value = it->second;
    std::transform(
        value.begin(), value.end(), value.begin(), [](unsigned char ch) { return std::tolower(ch); });
    return value == "1" || value == "true" || value == "yes" || value == "on";
}

double param_double(
    const std::unordered_map<std::string, std::string> &params,
    const std::string &key,
    double default_value,
    double lower,
    double upper)
{
    const auto it = params.find(key);
    if (it == params.end()) {
        return default_value;
    }
    try {
        return std::clamp(std::stod(it->second), lower, upper);
    } catch (...) {
        return default_value;
    }
}

int param_int(
    const std::unordered_map<std::string, std::string> &params,
    const std::string &key,
    int default_value,
    int lower,
    int upper)
{
    const auto it = params.find(key);
    if (it == params.end()) {
        return default_value;
    }
    try {
        return std::clamp(std::stoi(it->second), lower, upper);
    } catch (...) {
        return default_value;
    }
}

template <size_t N>
bool parse_doubles(const std::string &text, std::array<double, N> &values)
{
    if constexpr (N == 3) {
        return std::sscanf(
                   text.c_str(),
                   "%lf %lf %lf",
                   &values[0],
                   &values[1],
                   &values[2]) == 3;
    } else if constexpr (N == 4) {
        return std::sscanf(
                   text.c_str(),
                   "%lf %lf %lf %lf",
                   &values[0],
                   &values[1],
                   &values[2],
                   &values[3]) == 4;
    } else {
        static_assert(N == 3 || N == 4, "Unsupported parse_doubles arity");
    }
}

std::string bool_to_xacro(bool value)
{
    return value ? "true" : "false";
}

double clamp_unit(double value)
{
    return std::clamp(value, 0.0, 1.0);
}

}  // namespace

namespace marvin_system {

struct CollisionGuard::Impl {
    std::string description_package{"marvin_system"};
    std::string description_file{"description/urdf/marvin_dual.urdf"};
    std::string semantic_package{"marvin_system"};
    std::string semantic_file{"description/srdf/marvin_dual.srdf"};
    std::string left_xyz{"0 0.037 0.5118964"};
    std::string left_rpy{"-1.5707963 0 0"};
    std::string right_xyz{"0 -0.037 0.5118964"};
    std::string right_rpy{"1.5707963 0 0"};
    bool use_gripper_left{true};
    bool use_gripper_right{true};
    bool table_enabled{true};
    std::array<double, 3> table_size{{1.2, 0.8, 0.05}};
    std::array<double, 3> table_position{{0.0, 0.0, -0.025}};
    std::array<double, 4> table_orientation{{0.0, 0.0, 0.0, 1.0}};
    double gripper_height_intercept_m{0.15};
    double gripper_height_slope_m{-0.03};
    double gripper_radius_intercept_m{0.0425};
    double gripper_radius_slope_m{0.0575};
    bool verbose{false};

    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    planning_scene::PlanningScenePtr planning_scene_;
    std::array<int, kJointsPerArm * kArmCount> arm_var_indices{};
    std::array<int, kGripperCount> gripper_var_indices{{-1, -1}};
    std::array<bool, kGripperCount> gripper_enabled{{false, false}};
};

CollisionGuard::CollisionGuard() = default;

CollisionGuard::~CollisionGuard() = default;

bool CollisionGuard::configure(
    const std::unordered_map<std::string, std::string> &params,
    const rclcpp::Node::SharedPtr &node,
    const rclcpp::Logger &logger)
{
    impl_ = std::make_unique<Impl>();
    enabled_ = param_bool(params, "collision_guard_enabled", false);
    armed_ = false;
    ready_ = false;
    status_message_ = enabled_ ? "Collision guard is initializing." : "Collision guard disabled.";

    check_rate_hz_ = param_double(params, "collision_guard_check_rate_hz", 30.0, 1.0, 500.0);
    min_command_delta_deg_ =
        param_double(params, "collision_guard_min_command_delta_deg", 0.0, 0.0, 30.0);
    escape_min_distance_improvement_m_ = param_double(
        params, "collision_guard_escape_min_distance_improvement_m", 0.001, 0.0, 0.1);
    interpolation_steps_ = param_int(params, "collision_guard_interpolation_steps", 6, 1, 100);
    binary_search_steps_ = param_int(params, "collision_guard_binary_search_steps", 5, 0, 32);

    if (!enabled_) {
        return false;
    }
    if (!node) {
        status_message_ = "Collision guard requested but no ROS node is available.";
        RCLCPP_ERROR(logger, "%s", status_message_.c_str());
        return false;
    }

    impl_->description_package =
        param_str(params, "collision_guard_description_package", impl_->description_package);
    impl_->description_file =
        param_str(params, "collision_guard_description_file", impl_->description_file);
    impl_->semantic_package =
        param_str(params, "collision_guard_semantic_package", impl_->semantic_package);
    impl_->semantic_file =
        param_str(params, "collision_guard_semantic_file", impl_->semantic_file);
    impl_->left_xyz = param_str(params, "mount_xyz_L", impl_->left_xyz);
    impl_->left_rpy = param_str(params, "mount_rpy_L", impl_->left_rpy);
    impl_->right_xyz = param_str(params, "mount_xyz_R", impl_->right_xyz);
    impl_->right_rpy = param_str(params, "mount_rpy_R", impl_->right_rpy);
    impl_->use_gripper_left = param_bool(params, "collision_guard_use_gripper_L", true);
    impl_->use_gripper_right = param_bool(params, "collision_guard_use_gripper_R", true);
    impl_->table_enabled = param_bool(params, "collision_guard_table_enabled", true);
    near_distance_m_ = param_double(
        params, "collision_guard_near_distance_m", 0.10, 0.0, 0.5);
    hard_collision_distance_m_ = param_double(
        params, "collision_guard_hard_collision_distance_m", 0.05, -0.05, 0.2);
    if (hard_collision_distance_m_ > near_distance_m_) {
        std::swap(hard_collision_distance_m_, near_distance_m_);
    }
    impl_->gripper_height_intercept_m = param_double(
        params, "collision_guard_gripper_height_intercept_m", 0.15, 0.001, 1.0);
    impl_->gripper_height_slope_m = param_double(
        params, "collision_guard_gripper_height_slope_m", -0.03, -1.0, 1.0);
    impl_->gripper_radius_intercept_m = param_double(
        params, "collision_guard_gripper_radius_intercept_m", 0.0425, 0.001, 1.0);
    impl_->gripper_radius_slope_m = param_double(
        params, "collision_guard_gripper_radius_slope_m", 0.0575, -1.0, 1.0);
    impl_->verbose = param_bool(params, "collision_guard_verbose", false);

    const auto table_size_text = param_str(params, "collision_guard_table_size", "1.2 0.8 0.05");
    const auto table_position_text =
        param_str(params, "collision_guard_table_position", "0 0 -0.025");
    const auto table_orientation_text =
        param_str(params, "collision_guard_table_orientation", "0 0 0 1");
    if (!parse_doubles(table_size_text, impl_->table_size)) {
        status_message_ = "Invalid collision_guard_table_size.";
        RCLCPP_ERROR(logger, "%s value='%s'", status_message_.c_str(), table_size_text.c_str());
        return false;
    }
    if (!parse_doubles(table_position_text, impl_->table_position)) {
        status_message_ = "Invalid collision_guard_table_position.";
        RCLCPP_ERROR(
            logger, "%s value='%s'", status_message_.c_str(), table_position_text.c_str());
        return false;
    }
    if (!parse_doubles(table_orientation_text, impl_->table_orientation)) {
        status_message_ = "Invalid collision_guard_table_orientation.";
        RCLCPP_ERROR(
            logger, "%s value='%s'", status_message_.c_str(), table_orientation_text.c_str());
        return false;
    }

    std::vector<std::string> xacro_args{
        "use_mock_hardware:=false",
        "left_xyz:=\"" + impl_->left_xyz + "\"",
        "left_rpy:=\"" + impl_->left_rpy + "\"",
        "right_xyz:=\"" + impl_->right_xyz + "\"",
        "right_rpy:=\"" + impl_->right_rpy + "\"",
        "use_gripper_L:=" + bool_to_xacro(impl_->use_gripper_left),
        "use_gripper_R:=" + bool_to_xacro(impl_->use_gripper_right),
        "collision_guard_enabled:=false",
    };

    std::string urdf_xml;
    std::string description_path;
    try {
        description_path =
            ament_index_cpp::get_package_share_directory(impl_->description_package) + "/" +
            impl_->description_file;
    } catch (const std::exception &ex) {
        status_message_ =
            "Failed to resolve collision guard description package '" + impl_->description_package +
            "': " + ex.what();
        RCLCPP_ERROR(logger, "%s", status_message_.c_str());
        return false;
    }

    if (!rdf_loader::RDFLoader::loadXacroFileToString(urdf_xml, description_path, xacro_args)) {
        status_message_ =
            "Failed to load collision guard URDF from " + impl_->description_package + "/" +
            impl_->description_file;
        RCLCPP_ERROR(logger, "%s", status_message_.c_str());
        return false;
    }

    std::string srdf_xml;
    if (!rdf_loader::RDFLoader::loadPkgFileToString(
            srdf_xml, impl_->semantic_package, impl_->semantic_file, {})) {
        status_message_ =
            "Failed to load collision guard SRDF from " + impl_->semantic_package + "/" +
            impl_->semantic_file;
        RCLCPP_ERROR(logger, "%s", status_message_.c_str());
        return false;
    }

    robot_model_loader::RobotModelLoader::Options options(urdf_xml, srdf_xml);
    options.load_kinematics_solvers = false;
    impl_->robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node, options);
    impl_->robot_model_ = impl_->robot_model_loader_->getModel();
    if (!impl_->robot_model_) {
        status_message_ = "Collision guard failed to build MoveIt RobotModel.";
        RCLCPP_ERROR(logger, "%s", status_message_.c_str());
        return false;
    }

    impl_->planning_scene_ = std::make_shared<planning_scene::PlanningScene>(impl_->robot_model_);
    impl_->planning_scene_->removeAllCollisionObjects();

    if (impl_->table_enabled) {
        moveit_msgs::msg::CollisionObject table;
        table.id = "collision_guard_table";
        table.header.frame_id = impl_->planning_scene_->getPlanningFrame();
        table.operation = moveit_msgs::msg::CollisionObject::ADD;

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions = {
            impl_->table_size[0],
            impl_->table_size[1],
            impl_->table_size[2],
        };

        geometry_msgs::msg::Pose pose;
        pose.position.x = impl_->table_position[0];
        pose.position.y = impl_->table_position[1];
        pose.position.z = impl_->table_position[2];
        pose.orientation.x = impl_->table_orientation[0];
        pose.orientation.y = impl_->table_orientation[1];
        pose.orientation.z = impl_->table_orientation[2];
        pose.orientation.w = impl_->table_orientation[3];

        table.primitives.push_back(primitive);
        table.primitive_poses.push_back(pose);
        if (!impl_->planning_scene_->processCollisionObjectMsg(table)) {
            status_message_ = "Collision guard failed to add table collision object.";
            RCLCPP_ERROR(logger, "%s", status_message_.c_str());
            return false;
        }
    }

    for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
        impl_->arm_var_indices[joint] =
            static_cast<int>(impl_->robot_model_->getVariableIndex(kLeftJointNames[joint]));
        impl_->arm_var_indices[kJointsPerArm + joint] =
            static_cast<int>(impl_->robot_model_->getVariableIndex(kRightJointNames[joint]));
    }

    impl_->gripper_enabled[0] =
        impl_->use_gripper_left && impl_->robot_model_->hasJointModel(kGripperJointNames[0]);
    impl_->gripper_enabled[1] =
        impl_->use_gripper_right && impl_->robot_model_->hasJointModel(kGripperJointNames[1]);
    for (size_t index = 0; index < kGripperCount; ++index) {
        if (!impl_->gripper_enabled[index]) {
            continue;
        }
        impl_->gripper_var_indices[index] =
            static_cast<int>(impl_->robot_model_->getVariableIndex(kGripperJointNames[index]));
    }

    ready_ = true;
    status_message_ =
        "Collision guard ready: async scene loaded for table and robot self-collision checks.";
    RCLCPP_INFO(
        logger,
        "Collision guard ready: rate=%.1f Hz, interpolation_steps=%d, binary_search_steps=%d, "
        "near_distance=%.4f m, hard_collision_distance=%.4f m, escape_improvement=%.4f m, "
        "table=%s.",
        check_rate_hz_,
        interpolation_steps_,
        binary_search_steps_,
        near_distance_m_,
        hard_collision_distance_m_,
        escape_min_distance_improvement_m_,
        impl_->table_enabled ? "on" : "off");
    return true;
}

void CollisionGuard::arm()
{
    if (!enabled_ || !ready_) {
        return;
    }
    armed_ = true;
}

void CollisionGuard::disarm()
{
    armed_ = false;
}

bool CollisionGuard::enabled() const
{
    return enabled_;
}

bool CollisionGuard::armed() const
{
    return armed_;
}

bool CollisionGuard::active() const
{
    return enabled_ && ready_ && armed_;
}

bool CollisionGuard::ready() const
{
    return ready_;
}

double CollisionGuard::check_rate_hz() const
{
    return check_rate_hz_;
}

double CollisionGuard::min_command_delta_deg() const
{
    return min_command_delta_deg_;
}

double CollisionGuard::near_distance_m() const
{
    return near_distance_m_;
}

double CollisionGuard::hard_collision_distance_m() const
{
    return hard_collision_distance_m_;
}

double CollisionGuard::escape_min_distance_improvement_m() const
{
    return escape_min_distance_improvement_m_;
}

int CollisionGuard::interpolation_steps() const
{
    return interpolation_steps_;
}

int CollisionGuard::binary_search_steps() const
{
    return binary_search_steps_;
}

const std::string &CollisionGuard::status_message() const
{
    return status_message_;
}

CollisionGuard::Evaluation CollisionGuard::evaluate_motion(
    const std::array<std::array<double, kJointsPerArm>, kArmCount> &from_deg,
    const std::array<std::array<double, kJointsPerArm>, kArmCount> &to_deg,
    const std::array<double, kGripperCount> &gripper_percent,
    const std::array<bool, kGripperCount> &gripper_valid,
    const rclcpp::Logger & /*logger*/)
{
    Evaluation evaluation;
    evaluation.ready = ready_;
    if (!active() || !impl_ || !impl_->planning_scene_) {
        return evaluation;
    }

    double max_delta_deg = 0.0;
    for (size_t arm = 0; arm < kArmCount; ++arm) {
        for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
            const double a = from_deg[arm][joint];
            const double b = to_deg[arm][joint];
            if (!std::isfinite(a) || !std::isfinite(b)) {
                evaluation.safe = false;
                evaluation.max_safe_alpha = 0.0;
                evaluation.blocking_sample = 0;
                return evaluation;
            }
            max_delta_deg = std::max(max_delta_deg, std::abs(b - a));
        }
    }
    if (max_delta_deg <= min_command_delta_deg_) {
        evaluation.safe = true;
        evaluation.max_safe_alpha = 1.0;
        return evaluation;
    }

    struct StateCheck {
        bool safe{true};
        double distance{std::numeric_limits<double>::max()};
        std::string pair_summary;
        bool environment_pair{false};
        bool affect_left{false};
        bool affect_right{false};
    };

    auto mark_affected_arms = [](const std::string &a, const std::string &b, StateCheck &check) {
        if (a.find("_L") != std::string::npos || b.find("_L") != std::string::npos) {
            check.affect_left = true;
        }
        if (a.find("_R") != std::string::npos || b.find("_R") != std::string::npos) {
            check.affect_right = true;
        }
        if (!check.affect_left && !check.affect_right) {
            check.affect_left = true;
            check.affect_right = true;
        }
    };

    auto evaluate_state_at = [&](double alpha) {
        auto &state = impl_->planning_scene_->getCurrentStateNonConst();
        for (size_t joint = 0; joint < kJointsPerArm; ++joint) {
            const double left_deg =
                from_deg[0][joint] + (to_deg[0][joint] - from_deg[0][joint]) * alpha;
            const double right_deg =
                from_deg[1][joint] + (to_deg[1][joint] - from_deg[1][joint]) * alpha;
            state.setVariablePosition(impl_->arm_var_indices[joint], left_deg * kDeg2Rad);
            state.setVariablePosition(
                impl_->arm_var_indices[kJointsPerArm + joint], right_deg * kDeg2Rad);
        }

        for (size_t index = 0; index < kGripperCount; ++index) {
            if (!impl_->gripper_enabled[index]) {
                continue;
            }
            const double percent = clamp_unit(gripper_percent[index]);
            state.setVariablePosition(impl_->gripper_var_indices[index], percent);
        }
        state.update();

        for (size_t index = 0; index < kGripperCount; ++index) {
            if (!impl_->gripper_enabled[index]) {
                continue;
            }

            moveit_msgs::msg::AttachedCollisionObject attached;
            attached.link_name = kGripperAttachFrames[index];
            attached.object.id = kGripperObjectIds[index];
            attached.object.header.frame_id = kGripperAttachFrames[index];
            attached.object.operation = gripper_valid[index]
                                            ? moveit_msgs::msg::CollisionObject::ADD
                                            : moveit_msgs::msg::CollisionObject::REMOVE;

            if (gripper_valid[index]) {
                const double percent = clamp_unit(gripper_percent[index]);
                const double height = std::max(
                    1.0e-4,
                    impl_->gripper_height_intercept_m +
                        impl_->gripper_height_slope_m * percent);
                const double radius = std::max(
                    1.0e-4,
                    impl_->gripper_radius_intercept_m +
                        impl_->gripper_radius_slope_m * percent);

                shape_msgs::msg::SolidPrimitive primitive;
                primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
                primitive.dimensions = {height, radius};

                geometry_msgs::msg::Pose pose;
                pose.orientation.w = 1.0;
                pose.position.z = 0.5 * height;

                attached.object.primitives.push_back(primitive);
                attached.object.primitive_poses.push_back(pose);
                for (const auto *touch_link : kGripperTouchLinks[index]) {
                    if (impl_->robot_model_ && impl_->robot_model_->hasLinkModel(touch_link)) {
                        attached.touch_links.emplace_back(touch_link);
                    }
                }
            }

            if (!impl_->planning_scene_->processAttachedCollisionObjectMsg(attached)) {
                return StateCheck{false, -std::numeric_limits<double>::max(), ""};
            }
        }

        collision_detection::CollisionRequest request;
        request.verbose = impl_->verbose;
        request.pad_environment_collisions = false;
        request.pad_self_collisions = false;
        request.distance = true;
        request.detailed_distance = true;
        collision_detection::CollisionResult env_result;
        impl_->planning_scene_->getCollisionEnv()->checkRobotCollision(
            request,
            env_result,
            state,
            impl_->planning_scene_->getAllowedCollisionMatrix());
        collision_detection::CollisionResult self_result;
        impl_->planning_scene_->checkSelfCollision(
            request,
            self_result,
            state,
            impl_->planning_scene_->getAllowedCollisionMatrix());
        StateCheck check;
        if (self_result.collision) {
            check.safe = false;
            check.distance = -std::numeric_limits<double>::max();
            if (!self_result.contacts.empty()) {
                const auto &pair = self_result.contacts.begin()->first;
                check.pair_summary = pair.first + " - " + pair.second + " (self)";
                mark_affected_arms(pair.first, pair.second, check);
            } else if (!self_result.distance_result.minimum_distance.link_names[0].empty() &&
                       !self_result.distance_result.minimum_distance.link_names[1].empty()) {
                check.pair_summary =
                    self_result.distance_result.minimum_distance.link_names[0] + " - " +
                    self_result.distance_result.minimum_distance.link_names[1] + " (self)";
                mark_affected_arms(
                    self_result.distance_result.minimum_distance.link_names[0],
                    self_result.distance_result.minimum_distance.link_names[1],
                    check);
            } else {
                check.pair_summary = "self collision";
                check.affect_left = true;
                check.affect_right = true;
            }
            return check;
        }

        check.safe = !env_result.collision;
        const auto &min_env = env_result.distance_result.minimum_distance;
        check.environment_pair = true;
        check.distance = std::isfinite(min_env.distance)
                             ? min_env.distance
                             : (env_result.collision ? -std::numeric_limits<double>::max()
                                                     : std::numeric_limits<double>::max());
        if (!min_env.link_names[0].empty() && !min_env.link_names[1].empty()) {
            check.pair_summary = min_env.link_names[0] + " - " + min_env.link_names[1];
            mark_affected_arms(min_env.link_names[0], min_env.link_names[1], check);
        } else if (!env_result.contacts.empty()) {
            const auto &pair = env_result.contacts.begin()->first;
            check.pair_summary = pair.first + " - " + pair.second;
            mark_affected_arms(pair.first, pair.second, check);
        }
        return check;
    };

    auto alpha_from_distance = [this](double distance) {
        if (distance <= hard_collision_distance_m_) {
            return 0.0;
        }
        if (distance >= near_distance_m_) {
            return 1.0;
        }
        const double denom = std::max(1.0e-9, near_distance_m_ - hard_collision_distance_m_);
        return std::clamp(
            (distance - hard_collision_distance_m_) / denom,
            0.0,
            1.0);
    };

    const int samples = std::max(1, interpolation_steps_);
    const StateCheck start_state = evaluate_state_at(0.0);
    evaluation.current_distance_m = start_state.distance;
    evaluation.best_distance_m = start_state.distance;
    evaluation.current_pair = start_state.pair_summary;
    evaluation.environment_pair = start_state.environment_pair;
    evaluation.affect_left = start_state.affect_left;
    evaluation.affect_right = start_state.affect_right;

    // Recovery mode: if the current state is already inside the near/collision band,
    // only allow prefixes that do not materially worsen clearance and prefer prefixes
    // that improve distance. This keeps recovery possible while staying conservative.
    if (start_state.distance < near_distance_m_) {
        if (start_state.distance <= hard_collision_distance_m_) {
            evaluation.max_safe_alpha = 0.0;
            evaluation.blocking_sample = 0;
            evaluation.safe = false;
            evaluation.best_distance_m = start_state.distance;
            return evaluation;
        }

        double best_alpha = 0.0;
        double best_distance = start_state.distance;
        for (int sample = 1; sample <= samples; ++sample) {
            const double alpha = static_cast<double>(sample) / static_cast<double>(samples);
            const StateCheck candidate = evaluate_state_at(alpha);

            if (candidate.distance <
                start_state.distance - escape_min_distance_improvement_m_) {
                break;
            }

            // When already inside the near band, do not "creep" around the
            // boundary. Only allow motion if a sampled prefix actually exits the
            // near band; otherwise hold and let explicit recovery / go_home take over.
            if (candidate.distance >= near_distance_m_ &&
                candidate.distance >
                    best_distance + escape_min_distance_improvement_m_) {
                best_distance = candidate.distance;
                best_alpha = alpha;
            }
        }

        evaluation.max_safe_alpha = best_alpha;
        evaluation.blocking_sample = 0;
        evaluation.safe = best_alpha >= 1.0 - 1.0e-6;
        evaluation.best_distance_m = best_distance;
        return evaluation;
    }

    double last_safe_alpha = 0.0;
    for (int sample = 1; sample <= samples; ++sample) {
        const double alpha = static_cast<double>(sample) / static_cast<double>(samples);
        const StateCheck candidate = evaluate_state_at(alpha);
        const double allowed_alpha = alpha_from_distance(candidate.distance);
        if (alpha <= allowed_alpha + 1.0e-6) {
            last_safe_alpha = alpha;
            continue;
        }

        evaluation.safe = false;
        evaluation.blocking_sample = sample;
        evaluation.max_safe_alpha = last_safe_alpha;
        evaluation.best_distance_m = std::max(start_state.distance, candidate.distance);
        evaluation.current_pair = candidate.pair_summary.empty() ? start_state.pair_summary : candidate.pair_summary;
        evaluation.environment_pair = candidate.environment_pair;
        evaluation.affect_left = candidate.affect_left;
        evaluation.affect_right = candidate.affect_right;

        if (binary_search_steps_ > 0) {
            double low = last_safe_alpha;
            double high = alpha;
            double low_distance = start_state.distance;
            for (int step = 0; step < binary_search_steps_; ++step) {
                const double mid = 0.5 * (low + high);
                const StateCheck mid_candidate = evaluate_state_at(mid);
                const double mid_allowed_alpha = alpha_from_distance(mid_candidate.distance);
                if (mid <= mid_allowed_alpha + 1.0e-6) {
                    low = mid;
                    low_distance = mid_candidate.distance;
                } else {
                    high = mid;
                }
            }
            evaluation.max_safe_alpha = low;
            evaluation.best_distance_m = low_distance;
        }
        return evaluation;
    }

    evaluation.safe = true;
    evaluation.max_safe_alpha = 1.0;
    evaluation.best_distance_m = start_state.distance;
    return evaluation;
}

}  // namespace marvin_system
