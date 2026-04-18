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


def _parse_optional_bool(value: str | None) -> bool | None:
    if value is None:
        return None
    stripped = value.strip()
    if stripped == "":
        return None
    return _parse_bool(stripped)


def _parse_optional_float(value: str | None, *, name: str) -> float | None:
    if value is None:
        return None
    stripped = value.strip()
    if stripped == "":
        return None
    try:
        return float(stripped)
    except ValueError as exc:
        raise RuntimeError(f"Launch argument '{name}' must be a float, got {value!r}") from exc


def _parse_optional_string(value: str | None) -> str | None:
    if value is None:
        return None
    stripped = value.strip()
    if stripped == "":
        return None
    return stripped


def _merge_driver_params(params: dict, overrides: object, *, source: str) -> None:
    if overrides is None:
        return
    if not isinstance(overrides, dict):
        raise RuntimeError(f"Expected '{source}' to be a mapping, got {type(overrides).__name__}")
    params.update(overrides)


def _build_camera_params(
    target_camera_name: str,
    cameras_cfg: dict,
    defaults_cfg: dict,
    launch_overrides: dict[str, object],
) -> tuple[str, dict, dict]:
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
        "rgb_camera.enable_auto_exposure": profile_cfg.get("rgb_enable_auto_exposure", True),
        "rgb_camera.auto_exposure_priority": profile_cfg.get(
            "rgb_auto_exposure_priority", False
        ),
        "enable_depth": profile_cfg.get("depth_enabled", False),
        "depth_module.depth_profile": depth_profile,
        "depth_module.depth_format": profile_cfg.get("depth_format", "Z16"),
        "enable_infra": profile_cfg.get("infrared_enabled", False),
        "enable_infra1": profile_cfg.get("infrared_enabled", False),
        "enable_infra2": False,
        "depth_module.infra_profile": infra_profile,
        "enable_gyro": profile_cfg.get("enable_gyro", False),
        "enable_accel": profile_cfg.get("enable_accel", False),
        "unite_imu_method": profile_cfg.get("unite_imu_method", 0),
        "enable_sync": profile_cfg.get("enable_sync", False),
        "pointcloud.enable": profile_cfg.get("point_cloud_enabled", False),
        "align_depth.enable": profile_cfg.get("align_depth_enabled", False),
        "publish_tf": profile_cfg.get("publish_tf", True),
        "tf_publish_rate": profile_cfg.get("tf_publish_rate", 0.0),
        "base_frame_id": profile_cfg.get("base_frame_id", "link"),
    }
    _merge_driver_params(
        params,
        profile_cfg.get("driver_params"),
        source=f"profile '{profile_name}' driver_params",
    )
    _merge_driver_params(
        params,
        camera_cfg.get("driver_params"),
        source=f"camera '{target_camera_name}' driver_params",
    )
    params.update({key: value for key, value in launch_overrides.items() if value is not None})
    mock_params = {
        "frame_id": camera_cfg["frame_id"],
        "width": profile_cfg["color_width"],
        "height": profile_cfg["color_height"],
        "encoding": profile_cfg.get("color_format", "RGB8").lower(),
    }
    return namespace, params, mock_params


def _build_preview_node(camera_cfg: dict, image_topic: str):
    preview_topic = camera_cfg.get("preview_topic")
    if not isinstance(preview_topic, str) or not preview_topic.strip():
        return None

    preview_topic = preview_topic.strip()
    if preview_topic == image_topic:
        return None

    preview_fps = float(camera_cfg.get("preview_fps", 20.0))
    if preview_fps <= 0.0:
        raise RuntimeError("preview_fps must be positive")

    parameters = {
        "input_topic": image_topic,
        "output_topic": preview_topic,
        "publish_rate": preview_fps,
        "skip_when_no_subscribers": True,
    }
    if "preview_width" in camera_cfg:
        preview_width = int(camera_cfg["preview_width"])
        if preview_width <= 0:
            raise RuntimeError("preview_width must be a positive integer")
        parameters["output_width"] = preview_width
    if "preview_height" in camera_cfg:
        preview_height = int(camera_cfg["preview_height"])
        if preview_height <= 0:
            raise RuntimeError("preview_height must be a positive integer")
        parameters["output_height"] = preview_height

    return Node(
        package="camera_system",
        executable="camera_preview_bridge",
        name=f"{camera_cfg['namespace']}_preview_bridge",
        namespace="",
        output="screen",
        parameters=[parameters],
    )


def _launch_setup(context, *args, **kwargs):
    cameras_config_path = LaunchConfiguration("cameras_config").perform(context)
    defaults_config_path = LaunchConfiguration("realsense_defaults_config").perform(context)
    target_camera_name = LaunchConfiguration("camera_name").perform(context)
    respawn = _parse_bool(LaunchConfiguration("respawn").perform(context))
    respawn_delay = _parse_nonnegative_float(
        LaunchConfiguration("respawn_delay").perform(context),
        name="respawn_delay",
    )
    launch_overrides = {
        "initial_reset": _parse_optional_bool(
            LaunchConfiguration("initial_reset").perform(context)
        ),
        "wait_for_device_timeout": _parse_optional_float(
            LaunchConfiguration("wait_for_device_timeout").perform(context),
            name="wait_for_device_timeout",
        ),
        "reconnect_timeout": _parse_optional_float(
            LaunchConfiguration("reconnect_timeout").perform(context),
            name="reconnect_timeout",
        ),
        "usb_port_id": _parse_optional_string(
            LaunchConfiguration("usb_port_id").perform(context)
        ),
    }

    cameras_cfg = _load_yaml(cameras_config_path)
    defaults_cfg = _load_yaml(defaults_config_path)

    namespace, camera_params, mock_params = _build_camera_params(
        target_camera_name,
        cameras_cfg,
        defaults_cfg,
        launch_overrides,
    )
    camera_cfg = cameras_cfg["cameras"][target_camera_name]
    camera_topic_prefix = f"/{namespace}/{namespace}"
    image_topic = f"{camera_topic_prefix}/color/image_raw"
    driver_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name=namespace,
        namespace=namespace,
        output="screen",
        parameters=[camera_params],
        respawn=respawn,
        respawn_delay=respawn_delay,
    )

    nodes = [driver_node]

    preview_node = _build_preview_node(camera_cfg=camera_cfg, image_topic=image_topic)
    if preview_node is not None:
        nodes.append(preview_node)

    nodes.append(
        Node(
            package="image_tools",
            executable="showimage",
            name=f"{namespace}_image_view",
            namespace="",
            output="screen",
            remappings=[("image", image_topic)],
            parameters=[
                {
                    "reliability": "best_effort",
                    "history": "keep_last",
                    "depth": 1,
                    "window_name": target_camera_name,
                }
            ],
            condition=IfCondition(LaunchConfiguration("use_showimage")),
        )
    )
    return nodes


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
        DeclareLaunchArgument(
            "respawn",
            default_value="true",
            description="Respawn the RealSense driver node after unexpected exits.",
        ),
        DeclareLaunchArgument(
            "respawn_delay",
            default_value="2.0",
            description="Delay in seconds before the RealSense driver node is restarted.",
        ),
        DeclareLaunchArgument(
            "initial_reset",
            default_value="",
            description=(
                "Optional RealSense driver override. Empty uses the profile/camera YAML setting; "
                "true/false forces startup reset behavior."
            ),
        ),
        DeclareLaunchArgument(
            "wait_for_device_timeout",
            default_value="",
            description=(
                "Optional RealSense driver override in seconds. Empty uses the profile/camera YAML "
                "setting; negative values mean wait indefinitely."
            ),
        ),
        DeclareLaunchArgument(
            "reconnect_timeout",
            default_value="",
            description=(
                "Optional RealSense driver override in seconds. Empty uses the profile/camera YAML "
                "setting."
            ),
        ),
        DeclareLaunchArgument(
            "usb_port_id",
            default_value="",
            description=(
                "Optional RealSense driver override for binding a specific USB port, for example "
                "4-1. Empty uses the profile/camera YAML setting."
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
