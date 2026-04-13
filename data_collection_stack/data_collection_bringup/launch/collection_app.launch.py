from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("data_collection_bringup")

    default_recipe_directory = PathJoinSubstitution([bringup_share, "config", "recipes"])
    default_startup_policy = PathJoinSubstitution(
        [bringup_share, "config", "policies", "startup.yaml"]
    )
    default_fault_policy = PathJoinSubstitution([bringup_share, "config", "policies", "fault.yaml"])
    default_ui_config = PathJoinSubstitution([bringup_share, "config", "session", "ui.yaml"])
    default_cameras_config = PathJoinSubstitution(
        [FindPackageShare("camera_system"), "bringup", "config", "cameras.yaml"]
    )
    default_trackers_config = PathJoinSubstitution(
        [FindPackageShare("htc_system"), "bringup", "config", "trackers.yaml"]
    )
    default_manus_config = PathJoinSubstitution(
        [FindPackageShare("manus_system"), "config", "manus_raw_publisher.yaml"]
    )

    supervisor_node = Node(
        package="data_collection_orchestrator",
        executable="supervisor",
        name="data_collection_supervisor",
        output="screen",
        parameters=[
            {
                "recipe_id": LaunchConfiguration("recipe_id"),
                "recipe_directory": LaunchConfiguration("recipe_directory"),
                "startup_policy_config": LaunchConfiguration("startup_policy_config"),
                "fault_policy_config": LaunchConfiguration("fault_policy_config"),
                "cameras_config": LaunchConfiguration("cameras_config"),
                "trackers_config": LaunchConfiguration("trackers_config"),
                "manus_config": LaunchConfiguration("manus_config"),
                "manus_user_name": LaunchConfiguration("manus_user_name"),
            }
        ],
    )
    ui_node = Node(
        package="data_collection_ui",
        executable="ui_stub",
        name="data_collection_ui_stub",
        output="screen",
        parameters=[
            {
                "recipe_id": LaunchConfiguration("recipe_id"),
                "recipe_directory": LaunchConfiguration("recipe_directory"),
                "cameras_config": LaunchConfiguration("cameras_config"),
                "ui_config": LaunchConfiguration("ui_config"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("use_ui")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "recipe_id",
                default_value="marvin_tracker_manus_camera_collection",
            ),
            DeclareLaunchArgument("use_ui", default_value="false"),
            DeclareLaunchArgument("recipe_directory", default_value=default_recipe_directory),
            DeclareLaunchArgument("cameras_config", default_value=default_cameras_config),
            DeclareLaunchArgument("trackers_config", default_value=default_trackers_config),
            DeclareLaunchArgument("manus_config", default_value=default_manus_config),
            DeclareLaunchArgument("manus_user_name", default_value=""),
            DeclareLaunchArgument("startup_policy_config", default_value=default_startup_policy),
            DeclareLaunchArgument("fault_policy_config", default_value=default_fault_policy),
            DeclareLaunchArgument("ui_config", default_value=default_ui_config),
            DeclareLaunchArgument("shutdown_on_ui_exit", default_value="true"),
            DeclareLaunchArgument("shutdown_on_supervisor_exit", default_value="false"),
            supervisor_node,
            ui_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=ui_node,
                    on_exit=[
                        EmitEvent(event=Shutdown(reason="data_collection_ui exited")),
                    ],
                ),
                condition=IfCondition(LaunchConfiguration("shutdown_on_ui_exit")),
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=supervisor_node,
                    on_exit=[
                        EmitEvent(event=Shutdown(reason="data_collection_supervisor exited")),
                    ],
                ),
                condition=IfCondition(LaunchConfiguration("shutdown_on_supervisor_exit")),
            ),
        ]
    )
