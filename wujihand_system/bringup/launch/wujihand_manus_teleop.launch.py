from launch import LaunchDescription
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    workspace_root = Path(get_package_share_directory("wujihand_system")).resolve().parents[3]
    default_python_venv_bin = str(workspace_root / ".venv_ros2" / "bin")

    use_pipeline = LaunchConfiguration("use_pipeline")
    start_manus = LaunchConfiguration("start_manus")
    start_wujihand = LaunchConfiguration("start_wujihand")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    activate_forward_controller = LaunchConfiguration("activate_forward_controller")
    manus_mock = LaunchConfiguration("manus_mock")
    enable_rviz = LaunchConfiguration("enable_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    left_mount_xyz = LaunchConfiguration("left_mount_xyz")
    left_mount_rpy = LaunchConfiguration("left_mount_rpy")
    right_mount_xyz = LaunchConfiguration("right_mount_xyz")
    right_mount_rpy = LaunchConfiguration("right_mount_rpy")
    python_venv_bin = LaunchConfiguration("python_venv_bin")
    teleop_config = LaunchConfiguration("teleop_config")
    manus_config = LaunchConfiguration("manus_config")
    manus_user_name = LaunchConfiguration("manus_user_name")
    identity_file = LaunchConfiguration("identity_file")

    teleop_default_config = PathJoinSubstitution(
        [FindPackageShare("wujihand_system"), "config", "manus_input.yaml"]
    )
    manus_default_config = PathJoinSubstitution(
        [FindPackageShare("manus_system"), "config", "manus_raw_publisher.yaml"]
    )
    identity_default_config = PathJoinSubstitution(
        [FindPackageShare("wujihand_system"), "config", "wujihand_identities.yaml"]
    )
    rviz_default_config = PathJoinSubstitution(
        [FindPackageShare("wujihand_system"), "rviz", "wujihand_dual_teleop.rviz"]
    )

    manus_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("manus_system"), "launch", "manus_raw_publisher.launch.py"])
        ),
        launch_arguments={
            "config_file": manus_config,
            "user_name": manus_user_name,
            "mock": manus_mock,
        }.items(),
        condition=IfCondition(start_manus),
    )

    wujihand_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("wujihand_system"), "launch", "wujihand_dual_control.launch.py"])
        ),
        launch_arguments={
            "use_mock_hardware": use_mock_hardware,
            "identity_file": identity_file,
            "activate_forward_controller": activate_forward_controller,
            "left_mount_xyz": left_mount_xyz,
            "left_mount_rpy": left_mount_rpy,
            "right_mount_xyz": right_mount_xyz,
            "right_mount_rpy": right_mount_rpy,
        }.items(),
        condition=IfCondition(start_wujihand),
    )

    pipeline_node = Node(
        package="wujihand_system",
        executable="wujihand_manus_pipeline.py",
        name="wujihand_manus_pipeline",
        output="screen",
        arguments=["-c", teleop_config],
        condition=IfCondition(use_pipeline),
    )

    adapter_node = Node(
        package="wujihand_system",
        executable="manus_input_adapter.py",
        name="manus_input_adapter",
        output="screen",
        arguments=["-c", teleop_config],
        condition=UnlessCondition(use_pipeline),
    )

    bridge_node = Node(
        package="wujihand_system",
        executable="wujihand_retarget_bridge.py",
        name="wujihand_retarget_bridge",
        output="screen",
        arguments=["-c", teleop_config],
        condition=UnlessCondition(use_pipeline),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(enable_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_pipeline", default_value="true"),
            DeclareLaunchArgument("start_manus", default_value="false"),
            DeclareLaunchArgument("start_wujihand", default_value="false"),
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            DeclareLaunchArgument("activate_forward_controller", default_value="true"),
            DeclareLaunchArgument("manus_mock", default_value="false"),
            DeclareLaunchArgument("enable_rviz", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=rviz_default_config),
            DeclareLaunchArgument("left_mount_xyz", default_value="0 0.25 0"),
            DeclareLaunchArgument("left_mount_rpy", default_value="0 0 0"),
            DeclareLaunchArgument("right_mount_xyz", default_value="0 -0.25 0"),
            DeclareLaunchArgument("right_mount_rpy", default_value="0 0 0"),
            DeclareLaunchArgument("python_venv_bin", default_value=default_python_venv_bin),
            SetEnvironmentVariable("PATH", [python_venv_bin, ":", EnvironmentVariable("PATH")]),
            DeclareLaunchArgument("teleop_config", default_value=teleop_default_config),
            DeclareLaunchArgument("manus_config", default_value=manus_default_config),
            DeclareLaunchArgument("manus_user_name", default_value="default"),
            DeclareLaunchArgument("identity_file", default_value=identity_default_config),
            manus_launch,
            wujihand_launch,
            pipeline_node,
            adapter_node,
            bridge_node,
            rviz_node,
        ]
    )
