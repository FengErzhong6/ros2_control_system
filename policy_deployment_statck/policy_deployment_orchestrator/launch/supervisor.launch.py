import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 禁用 FastDDS Shared Memory Transport，避免 RealSense 图像流耗尽 SHM 资源，
    # 导致 WujiHand ros2_control_node 无法初始化 DDS participant（进程无输出）。
    if not os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE"):
        try:
            os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = os.path.join(
                get_package_share_directory("policy_deployment_bringup"),
                "config", "fastdds_disable_shm.xml"
            )
        except Exception:
            pass
    if not os.environ.get("ROS_LOCALHOST_ONLY"):
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
    return LaunchDescription(
        [
            DeclareLaunchArgument("recipe_id", default_value="default_policy_deployment"),
            DeclareLaunchArgument("recipe_directory", default_value=""),
            DeclareLaunchArgument("policy_profiles_config", default_value=""),
            DeclareLaunchArgument("default_policy_profile_id", default_value=""),
            DeclareLaunchArgument("operator_id", default_value=""),
            DeclareLaunchArgument("site_name", default_value=""),
            DeclareLaunchArgument("connect_timeout_sec", default_value="20.0"),
            DeclareLaunchArgument("policy_step_timeout_sec", default_value="5.0"),
            DeclareLaunchArgument("observation_max_image_age_sec", default_value="1.0"),
            DeclareLaunchArgument("observation_max_joint_state_age_sec", default_value="1.0"),
            DeclareLaunchArgument("publish_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            DeclareLaunchArgument(
                "local_policy_openpi_src",
                default_value="/home/mmlab/codes/huangshzh/openpi_dexhand/src",
            ),
            DeclareLaunchArgument(
                "local_policy_openpi_client_src",
                default_value="/home/mmlab/codes/huangshzh/openpi_dexhand/packages/openpi-client/src",
            ),
            Node(
                package="policy_deployment_orchestrator",
                executable="supervisor",
                name="policy_deployment_supervisor",
                output="screen",
                parameters=[
                    {
                        "recipe_id": LaunchConfiguration("recipe_id"),
                        "recipe_directory": LaunchConfiguration("recipe_directory"),
                        "policy_profiles_config": LaunchConfiguration("policy_profiles_config"),
                        "default_policy_profile_id": LaunchConfiguration("default_policy_profile_id"),
                        "operator_id": LaunchConfiguration("operator_id"),
                        "site_name": LaunchConfiguration("site_name"),
                        "connect_timeout_sec": LaunchConfiguration("connect_timeout_sec"),
                        "policy_step_timeout_sec": LaunchConfiguration("policy_step_timeout_sec"),
                        "observation_max_image_age_sec": LaunchConfiguration("observation_max_image_age_sec"),
                        "observation_max_joint_state_age_sec": LaunchConfiguration(
                            "observation_max_joint_state_age_sec"
                        ),
                        "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                        "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
                        "local_policy_openpi_src": LaunchConfiguration("local_policy_openpi_src"),
                        "local_policy_openpi_client_src": LaunchConfiguration("local_policy_openpi_client_src"),
                    }
                ],
            ),
        ]
    )
