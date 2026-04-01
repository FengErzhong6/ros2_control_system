from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml(path_str: str) -> dict:
    path = Path(path_str)
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"Expected mapping at YAML root: {path}")
    return data


def _parse_bool(value: str) -> bool:
    lowered = value.strip().lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    raise RuntimeError(f"Expected true/false launch argument, got {value!r}")


def _parse_nonnegative_float(value: str, *, name: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(
            f"Launch argument '{name}' must be a non-negative float, got {value!r}"
        ) from exc

    if parsed < 0.0:
        raise RuntimeError(
            f"Launch argument '{name}' must be a non-negative float, got {value!r}"
        )

    return parsed


def _build_camera_params(target_camera_name: str, cameras_cfg: dict, defaults_cfg: dict) -> tuple[str, dict]:
    cameras = cameras_cfg.get("cameras", {})
    camera_cfg = cameras.get(target_camera_name)
    if camera_cfg is None:
        raise RuntimeError(f"Camera '{target_camera_name}' not found in cameras.yaml")

    if camera_cfg.get("driver") != "orbbec":
        raise RuntimeError(f"Camera '{target_camera_name}' is not configured as an Orbbec device")

    profile_name = camera_cfg.get("profile")
    profile_cfg = defaults_cfg.get("profiles", {}).get(profile_name)
    if profile_cfg is None:
        raise RuntimeError(
            f"Profile '{profile_name}' for camera '{target_camera_name}' not found in orbbec_defaults.yaml"
        )

    if not profile_cfg.get("color_enabled", True):
        raise RuntimeError(f"Camera '{target_camera_name}' has color_enabled=false, cannot launch image view")

    namespace = camera_cfg["namespace"]
    return namespace, {
        "camera_name": target_camera_name,
        "serial_number": camera_cfg["serial_number"],
        "frame_id": camera_cfg["frame_id"],
        "image_topic": "image_raw",
        "color_width": profile_cfg["color_width"],
        "color_height": profile_cfg["color_height"],
        "color_fps": profile_cfg["color_fps"],
        "color_encoding": profile_cfg["color_encoding"],
    }


def _launch_setup(context, *args, **kwargs):
    cameras_config_path = LaunchConfiguration("cameras_config").perform(context)
    defaults_config_path = LaunchConfiguration("orbbec_defaults_config").perform(context)
    target_camera_name = LaunchConfiguration("camera_name").perform(context)
    respawn = _parse_bool(LaunchConfiguration("respawn").perform(context))
    respawn_delay = _parse_nonnegative_float(
        LaunchConfiguration("respawn_delay").perform(context),
        name="respawn_delay",
    )

    cameras_cfg = _load_yaml(cameras_config_path)
    defaults_cfg = _load_yaml(defaults_config_path)

    namespace, camera_params = _build_camera_params(target_camera_name, cameras_cfg, defaults_cfg)

    return [
        Node(
            package="camera_system",
            executable="orbbec_camera_node",
            name="orbbec_camera_node",
            namespace=namespace,
            output="screen",
            parameters=[camera_params],
            respawn=respawn,
            respawn_delay=respawn_delay,
        ),
        Node(
            package="image_tools",
            executable="showimage",
            name=f"{namespace}_image_view",
            namespace="",
            output="screen",
            remappings=[("image", f"/{namespace}/image_raw")],
            parameters=[
                {
                    "reliability": "best_effort",
                    "history": "keep_last",
                    "depth": 1,
                    "window_name": target_camera_name,
                }
            ],
            condition=IfCondition(LaunchConfiguration("use_showimage")),
        ),
    ]


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("camera_system"))

    default_cameras_config = str(pkg_share / "bringup" / "config" / "cameras.yaml")
    default_orbbec_defaults = str(pkg_share / "bringup" / "config" / "orbbec_defaults.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_name",
            default_value="cam_left_wrist",
            description="Camera key in cameras.yaml for the Orbbec device to launch.",
        ),
        DeclareLaunchArgument(
            "cameras_config",
            default_value=default_cameras_config,
            description="Path to the camera inventory YAML file.",
        ),
        DeclareLaunchArgument(
            "orbbec_defaults_config",
            default_value=default_orbbec_defaults,
            description="Path to the Orbbec profile defaults YAML file.",
        ),
        DeclareLaunchArgument(
            "use_showimage",
            default_value="false",
            description="Whether to start image_tools/showimage for the Orbbec color image.",
        ),
        DeclareLaunchArgument(
            "respawn",
            default_value="true",
            description="Respawn the Orbbec driver node after unexpected exits.",
        ),
        DeclareLaunchArgument(
            "respawn_delay",
            default_value="2.0",
            description="Delay in seconds before the Orbbec driver node is restarted.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
