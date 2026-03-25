from pathlib import Path

import yaml

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
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


def _build_camera_params(target_camera_name: str, cameras_cfg: dict, defaults_cfg: dict) -> tuple[str, dict]:
    cameras = cameras_cfg.get("cameras", {})
    camera_cfg = cameras.get(target_camera_name)
    if camera_cfg is None:
        raise RuntimeError(f"Camera '{target_camera_name}' not found in cameras.yaml")

    if camera_cfg.get("driver") != "realsense":
        raise RuntimeError(f"Camera '{target_camera_name}' is not configured as a RealSense device")

    profile_name = camera_cfg.get("profile")
    profile_cfg = defaults_cfg.get("profiles", {}).get(profile_name)
    if profile_cfg is None:
        raise RuntimeError(
            f"Profile '{profile_name}' for camera '{target_camera_name}' not found in realsense_defaults.yaml"
        )

    namespace = camera_cfg["namespace"]
    color_profile = (
        f"{profile_cfg['color_width']}x{profile_cfg['color_height']}x{profile_cfg['color_fps']}"
    )
    depth_profile = (
        f"{profile_cfg['depth_width']}x{profile_cfg['depth_height']}x{profile_cfg['depth_fps']}"
    )
    infra_profile = (
        f"{profile_cfg['infrared_width']}x{profile_cfg['infrared_height']}x{profile_cfg['infrared_fps']}"
    )

    params = {
        "camera_name": namespace,
        "serial_no": camera_cfg["serial_number"],
        "enable_color": profile_cfg.get("color_enabled", True),
        "rgb_camera.color_profile": color_profile,
        "rgb_camera.color_format": profile_cfg.get("color_format", "RGB8"),
        "enable_depth": profile_cfg.get("depth_enabled", False),
        "depth_module.depth_profile": depth_profile,
        "depth_module.depth_format": profile_cfg.get("depth_format", "Z16"),
        "enable_infra": profile_cfg.get("infrared_enabled", False),
        "enable_infra1": profile_cfg.get("infrared_enabled", False),
        "enable_infra2": False,
        "depth_module.infra_profile": infra_profile,
        "pointcloud.enable": profile_cfg.get("point_cloud_enabled", False),
        "align_depth.enable": profile_cfg.get("align_depth_enabled", False),
        "publish_tf": profile_cfg.get("publish_tf", True),
        "tf_publish_rate": profile_cfg.get("tf_publish_rate", 0.0),
        "base_frame_id": profile_cfg.get("base_frame_id", "link"),
    }
    return namespace, params


def _launch_setup(context, *args, **kwargs):
    cameras_config_path = LaunchConfiguration("cameras_config").perform(context)
    defaults_config_path = LaunchConfiguration("realsense_defaults_config").perform(context)
    target_camera_name = LaunchConfiguration("camera_name").perform(context)

    cameras_cfg = _load_yaml(cameras_config_path)
    defaults_cfg = _load_yaml(defaults_config_path)

    namespace, camera_params = _build_camera_params(target_camera_name, cameras_cfg, defaults_cfg)
    camera_topic_prefix = f"/{namespace}/{namespace}"

    return [
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name=namespace,
            namespace=namespace,
            output="screen",
            parameters=[camera_params],
        ),
        Node(
            package="image_tools",
            executable="showimage",
            name=f"{namespace}_image_view",
            namespace="",
            output="screen",
            remappings=[("image", f"{camera_topic_prefix}/color/image_raw")],
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
    try:
        pkg_share = Path(get_package_share_directory("camera_system"))
        default_cameras_config = str(pkg_share / "bringup" / "config" / "cameras.yaml")
        default_realsense_defaults = str(pkg_share / "bringup" / "config" / "realsense_defaults.yaml")
    except PackageNotFoundError:
        bringup_dir = Path(__file__).resolve().parents[1]
        default_cameras_config = str(bringup_dir / "config" / "cameras.yaml")
        default_realsense_defaults = str(bringup_dir / "config" / "realsense_defaults.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_name",
            default_value="cam_high",
            description="Camera key in cameras.yaml for the RealSense device to launch.",
        ),
        DeclareLaunchArgument(
            "cameras_config",
            default_value=default_cameras_config,
            description="Path to the camera inventory YAML file.",
        ),
        DeclareLaunchArgument(
            "realsense_defaults_config",
            default_value=default_realsense_defaults,
            description="Path to the RealSense profile defaults YAML file.",
        ),
        DeclareLaunchArgument(
            "use_showimage",
            default_value="true",
            description="Whether to start image_tools/showimage for the RealSense color image.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
