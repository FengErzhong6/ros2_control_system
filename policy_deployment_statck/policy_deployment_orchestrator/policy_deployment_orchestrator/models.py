from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class LaunchSpec:
    device_id: str
    adapter: str
    package: str
    launch_file: str
    arguments: dict[str, Any] = field(default_factory=dict)
    required: bool = True
    depends_on: tuple[str, ...] = ()
    startup_timeout_sec: float = 30.0
    ready_stability_window_sec: float = 0.0
    ready_relaunch_attempts: int = 0
    ready_relaunch_backoff_sec: float = 1.0
    ready_topics: tuple[str, ...] = ()
    ready_services: tuple[str, ...] = ()
    ready_controllers: dict[str, tuple[str, ...]] = field(default_factory=dict)
    ready_motion_status: bool = False
    ready_motion_status_timeout_sec: float = 12.0
    summary: str = ""


@dataclass(frozen=True)
class RecipeSpec:
    recipe_id: str
    path: Path
    launches: tuple[LaunchSpec, ...] = ()
    ready_timeout_sec: float = 45.0
    summary: str = ""


@dataclass(frozen=True)
class ImageInputSpec:
    name: str
    topic: str
    required: bool = True
    square_crop_anchor: str | None = None
    resize_size: tuple[int, int] | None = None


@dataclass(frozen=True)
class ActionSliceSpec:
    name: str
    start: int
    length: int
    mode: str = "absolute"


@dataclass(frozen=True)
class JointGroupSpec:
    name: str
    topic: str
    joint_names: tuple[str, ...]


@dataclass(frozen=True)
class CommandTargetSpec:
    name: str
    topic: str
    joint_names: tuple[str, ...]
    home: tuple[float, ...] = ()


@dataclass(frozen=True)
class PolicyProfileSpec:
    profile_id: str
    title: str
    server_host: str
    server_port: int
    default_prompt: str
    action_dim: int
    action_horizon: int
    action_space: str
    open_loop_horizon: int
    control_rate_hz: float
    image_inputs: tuple[ImageInputSpec, ...]
    right_arm_state: JointGroupSpec
    right_hand_state: JointGroupSpec
    marvin_command: CommandTargetSpec
    right_hand_command: CommandTargetSpec
    action_slices: tuple[ActionSliceSpec, ...]
    frozen_groups: tuple[str, ...] = ()
    summary: str = ""
    use_local_inference: bool = False
    local_policy_config_name: str = ""
    local_policy_checkpoint_dir: str = ""

    def slice_for(self, name: str) -> ActionSliceSpec:
        for action_slice in self.action_slices:
            if action_slice.name == name:
                return action_slice
        raise KeyError(f"Policy profile {self.profile_id!r} has no action slice {name!r}")


@dataclass(frozen=True)
class SupervisorConfig:
    recipe_id: str
    recipe_directory: Path | None
    policy_profiles_config: Path | None
    default_policy_profile_id: str
    operator_id: str = ""
    site_name: str = ""
    connect_timeout_sec: float = 20.0
    policy_step_timeout_sec: float = 5.0
    observation_max_image_age_sec: float = 1.0
    observation_max_joint_state_age_sec: float = 1.0
    publish_rate_hz: float = 10.0
    use_mock_hardware: bool = False
    local_policy_openpi_src: str = ""
    local_policy_openpi_client_src: str = ""
