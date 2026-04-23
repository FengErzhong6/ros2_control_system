from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


TRUTHY_VALUES = {"1", "true", "yes", "on"}


def as_bool(value: str) -> bool:
    return value.strip().lower() in TRUTHY_VALUES


def as_launch_bool(value: bool) -> str:
    return "true" if value else "false"


def generate_launch_description() -> LaunchDescription:
    workspace_root = Path(get_package_share_directory("wujihand_system")).resolve().parents[3]
    default_python_venv_bin = str(workspace_root / ".venv_ros2" / "bin")

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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "session_mode",
                default_value="manual",
                description="manual, real_teleop, mock_teleop, or hands_gui",
            ),
            DeclareLaunchArgument("use_pipeline", default_value="true"),
            DeclareLaunchArgument("start_manus", default_value="false"),
            DeclareLaunchArgument("start_wujihand", default_value="false"),
            DeclareLaunchArgument("start_left", default_value="true"),
            DeclareLaunchArgument("start_right", default_value="true"),
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            DeclareLaunchArgument("activate_forward_controller", default_value="true"),
            DeclareLaunchArgument("use_dual_gui", default_value="false"),
            DeclareLaunchArgument("manus_mock", default_value="false"),
            DeclareLaunchArgument("enable_rviz", default_value="false"),
            DeclareLaunchArgument("pipeline_namespace", default_value=""),
            DeclareLaunchArgument("pipeline_start_enabled", default_value="false"),
            DeclareLaunchArgument("left_namespace", default_value="left"),
            DeclareLaunchArgument("right_namespace", default_value="right"),
            DeclareLaunchArgument("rviz_config", default_value=rviz_default_config),
            DeclareLaunchArgument("left_mount_xyz", default_value="0 0.25 0"),
            DeclareLaunchArgument("left_mount_rpy", default_value="0 0 0"),
            DeclareLaunchArgument("right_mount_xyz", default_value="0 -0.25 0"),
            DeclareLaunchArgument("right_mount_rpy", default_value="0 0 0"),
            DeclareLaunchArgument("python_venv_bin", default_value=default_python_venv_bin),
            SetEnvironmentVariable(
                "PATH",
                [LaunchConfiguration("python_venv_bin"), ":", EnvironmentVariable("PATH")],
            ),
            DeclareLaunchArgument("teleop_config", default_value=teleop_default_config),
            DeclareLaunchArgument("manus_config", default_value=manus_default_config),
            DeclareLaunchArgument("manus_user_name", default_value="hsz"),
            DeclareLaunchArgument("identity_file", default_value=identity_default_config),
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context):
    session_mode = LaunchConfiguration("session_mode").perform(context).strip().lower()

    use_pipeline = as_bool(LaunchConfiguration("use_pipeline").perform(context))
    start_manus = as_bool(LaunchConfiguration("start_manus").perform(context))
    start_wujihand = as_bool(LaunchConfiguration("start_wujihand").perform(context))
    start_left = as_bool(LaunchConfiguration("start_left").perform(context))
    start_right = as_bool(LaunchConfiguration("start_right").perform(context))
    use_mock_hardware = as_bool(LaunchConfiguration("use_mock_hardware").perform(context))
    activate_forward_controller = as_bool(
        LaunchConfiguration("activate_forward_controller").perform(context)
    )
    use_dual_gui = as_bool(LaunchConfiguration("use_dual_gui").perform(context))
    manus_mock = as_bool(LaunchConfiguration("manus_mock").perform(context))
    enable_rviz = as_bool(LaunchConfiguration("enable_rviz").perform(context))

    if session_mode == "real_teleop":
        start_manus = True
        start_wujihand = True
        use_pipeline = True
        use_mock_hardware = False
        manus_mock = False
        activate_forward_controller = True
    elif session_mode == "mock_teleop":
        start_manus = True
        start_wujihand = True
        use_pipeline = True
        use_mock_hardware = True
        manus_mock = True
        activate_forward_controller = True
    elif session_mode == "hands_gui":
        start_manus = False
        start_wujihand = True
        use_pipeline = False
        use_dual_gui = True
        activate_forward_controller = True
    elif session_mode != "manual":
        raise RuntimeError(
            "session_mode must be one of: manual, real_teleop, mock_teleop, hands_gui"
        )

    summary = (
        "Resolved teleop launch flags: "
        f"session_mode={session_mode}, "
        f"start_manus={start_manus}, "
        f"start_wujihand={start_wujihand}, "
        f"use_pipeline={use_pipeline}, "
        f"start_left={start_left}, "
        f"start_right={start_right}, "
        f"use_mock_hardware={use_mock_hardware}, "
        f"manus_mock={manus_mock}, "
        f"use_dual_gui={use_dual_gui}, "
        f"activate_forward_controller={activate_forward_controller}, "
        f"enable_rviz={enable_rviz}"
    )

    teleop_config = LaunchConfiguration("teleop_config")
    manus_config = LaunchConfiguration("manus_config")
    manus_user_name = LaunchConfiguration("manus_user_name")
    identity_file = LaunchConfiguration("identity_file")
    rviz_config = LaunchConfiguration("rviz_config")
    pipeline_namespace = LaunchConfiguration("pipeline_namespace")
    pipeline_start_enabled = LaunchConfiguration("pipeline_start_enabled")
    left_mount_xyz = LaunchConfiguration("left_mount_xyz")
    left_mount_rpy = LaunchConfiguration("left_mount_rpy")
    right_mount_xyz = LaunchConfiguration("right_mount_xyz")
    right_mount_rpy = LaunchConfiguration("right_mount_rpy")
    left_namespace = LaunchConfiguration("left_namespace")
    right_namespace = LaunchConfiguration("right_namespace")

    actions = [LogInfo(msg=summary)]

    if start_manus:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("manus_system"), "launch", "manus_raw_publisher.launch.py"]
                    )
                ),
                launch_arguments={
                    "config_file": manus_config,
                    "user_name": manus_user_name,
                    "mock": as_launch_bool(manus_mock),
                }.items(),
            )
        )

    if start_wujihand:
        dual_launch_arguments = {
            "use_mock_hardware": as_launch_bool(use_mock_hardware),
            "identity_file": identity_file,
            "activate_forward_controller": as_launch_bool(activate_forward_controller),
            "start_left": as_launch_bool(start_left),
            "start_right": as_launch_bool(start_right),
            "left_mount_xyz": left_mount_xyz,
            "left_mount_rpy": left_mount_rpy,
            "right_mount_xyz": right_mount_xyz,
            "right_mount_rpy": right_mount_rpy,
            "left_namespace": left_namespace,
            "right_namespace": right_namespace,
        }
        if use_dual_gui:
            dual_launch_arguments["use_dual_gui"] = "true"

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("wujihand_system"), "launch", "wujihand_dual_control.launch.py"]
                    )
                ),
                launch_arguments=dual_launch_arguments.items(),
            )
        )

    if use_pipeline:
        actions.append(
            Node(
                package="wujihand_system",
                executable="wujihand_manus_pipeline.py",
                namespace=pipeline_namespace,
                name="wujihand_manus_pipeline",
                output="screen",
                arguments=["-c", teleop_config],
                parameters=[
                    {
                        "start_enabled": ParameterValue(
                            pipeline_start_enabled,
                            value_type=bool,
                        )
                    }
                ],
            )
        )
    else:
        actions.extend(
            [
                Node(
                    package="wujihand_system",
                    executable="manus_input_adapter.py",
                    name="manus_input_adapter",
                    output="screen",
                    arguments=["-c", teleop_config],
                ),
                Node(
                    package="wujihand_system",
                    executable="wujihand_retarget_bridge.py",
                    name="wujihand_retarget_bridge",
                    output="screen",
                    arguments=["-c", teleop_config],
                ),
            ]
        )

    if enable_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            )
        )

    return actions
