#include <array>
#include <unordered_map>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "marvin_system/workspace_guard.hpp"

namespace {

std::unordered_map<std::string, std::string> make_params()
{
    return {
        {"workspace_z_min", "0.2"},
        {"workspace_safety_margin", "0.0"},
        {"mount_xyz_L", "0 0.037 0.3618964"},
        {"mount_rpy_L", "-1.5707963 0 0"},
        {"mount_xyz_R", "0 -0.037 0.3618964"},
        {"mount_rpy_R", "1.5707963 0 0"},
        {"tool_offset", "0 -0.245 0"},
    };
}

void expect_cmd_eq(const std::array<double, 7> &expected,
                   const std::array<double, 7> &actual)
{
    for (size_t i = 0; i < expected.size(); ++i) {
        EXPECT_DOUBLE_EQ(expected[i], actual[i]) << "joint " << i;
    }
}

}  // namespace

TEST(WorkspaceGuardTest, EnforcesOnlyAfterArm)
{
    marvin_system::WorkspaceGuard guard;
    const auto logger = rclcpp::get_logger("workspace_guard_test");

    ASSERT_TRUE(guard.configure(make_params(), logger));
    EXPECT_TRUE(guard.enabled());
    EXPECT_FALSE(guard.armed());

    const std::array<double, 7> seed{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    const std::array<double, 7> violating{{60.0, -90.0, -90.0, -90.0, 0.0, 0.0, 0.0}};

    auto cmd = violating;
    EXPECT_TRUE(guard.filter(0, cmd.data(), logger));
    expect_cmd_eq(violating, cmd);

    guard.seed(0, seed.data());
    guard.arm();
    EXPECT_TRUE(guard.armed());

    cmd = violating;
    EXPECT_FALSE(guard.filter(0, cmd.data(), logger));
    expect_cmd_eq(seed, cmd);

    guard.disarm();
    EXPECT_FALSE(guard.armed());

    cmd = violating;
    EXPECT_TRUE(guard.filter(0, cmd.data(), logger));
    expect_cmd_eq(violating, cmd);
}
