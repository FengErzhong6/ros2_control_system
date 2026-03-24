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


def _build_camera_params(camera_name: str, cameras_cfg: dict, defaults_cfg: dict) -> dict:
    cameras = cameras_cfg.get("cameras", {})
    camera_cfg = cameras.get(camera_name)
    if camera_cfg is None:
      raise RuntimeError(f"Camera '{camera_name}' not found in cameras.yaml")

    if camera_cfg.get("driver") != "orbbec":
      raise RuntimeError(f"Camera '{camera_name}' is not configured as an Orbbec device")

    profile_name = camera_cfg.get("profile")
    profile_cfg = defaults_cfg.get("profiles", {}).get(profile_name)
    if profile_cfg is None:
      raise RuntimeError(
          f"Profile '{profile_name}' for camera '{camera_name}' not found in orbbec_defaults.yaml")

    if not profile_cfg.get("color_enabled", True):
      raise RuntimeError(f"Camera '{camera_name}' has color_enabled=false, cannot launch wrist image view")

    return {
        "camera_name": camera_name,
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

    cameras_cfg = _load_yaml(cameras_config_path)
    defaults_cfg = _load_yaml(defaults_config_path)

    left_params = _build_camera_params("cam_left_wrist", cameras_cfg, defaults_cfg)
    right_params = _build_camera_params("cam_right_wrist", cameras_cfg, defaults_cfg)

    left_namespace = cameras_cfg["cameras"]["cam_left_wrist"]["namespace"]
    right_namespace = cameras_cfg["cameras"]["cam_right_wrist"]["namespace"]

    actions = [
        Node(
            package="camera_system",
            executable="orbbec_camera_node",
            name="orbbec_camera_node",
            namespace=left_namespace,
            output="screen",
            parameters=[left_params],
        ),
        Node(
            package="camera_system",
            executable="orbbec_camera_node",
            name="orbbec_camera_node",
            namespace=right_namespace,
            output="screen",
            parameters=[right_params],
        ),
        Node(
            package="image_tools",
            executable="showimage",
            name="left_wrist_image_view",
            namespace="",
            output="screen",
            remappings=[("image", f"/{left_namespace}/image_raw")],
            parameters=[
                {
                    "reliability": "best_effort",
                    "history": "keep_last",
                    "depth": 1,
                    "window_name": "cam_left_wrist",
                }
            ],
            condition=IfCondition(LaunchConfiguration("use_showimage")),
        ),
        Node(
            package="image_tools",
            executable="showimage",
            name="right_wrist_image_view",
            namespace="",
            output="screen",
            remappings=[("image", f"/{right_namespace}/image_raw")],
            parameters=[
                {
                    "reliability": "best_effort",
                    "history": "keep_last",
                    "depth": 1,
                    "window_name": "cam_right_wrist",
                }
            ],
            condition=IfCondition(LaunchConfiguration("use_showimage")),
        ),
    ]

    return actions


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("camera_system"))

    default_cameras_config = str(pkg_share / "bringup" / "config" / "cameras.yaml")
    default_orbbec_defaults = str(pkg_share / "bringup" / "config" / "orbbec_defaults.yaml")

    return LaunchDescription([
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
            description="Whether to start image_tools/showimage viewers for both wrist cameras.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
