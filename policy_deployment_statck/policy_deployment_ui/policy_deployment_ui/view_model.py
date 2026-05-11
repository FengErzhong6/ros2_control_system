from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class PolicyProfileEntry:
    profile_id: str
    title: str
    server_host: str
    server_port: int
    default_prompt: str
    summary: str = ""


@dataclass(frozen=True)
class PreviewStreamConfig:
    profile_id: str
    camera_id: str
    title: str
    preview_topic: str


@dataclass(frozen=True)
class DeploymentStateSnapshot:
    system_state: str = "IDLE"
    rollout_state: str = "IDLE"
    recipe_id: str = ""
    active_profile_id: str = ""
    active_prompt: str = ""
    summary: str = ""
    chunk_index: int = 0
    step_index: int = 0
    last_infer_ms: float = 0.0
    allowed_commands: tuple[str, ...] = ()
    device_count: int = 0
    fault_count: int = 0


@dataclass
class UiViewModel:
    title: str = "Policy Deployment Console"
    recipe_id: str = ""
    profile_id: str = ""
    prompt: str = ""
    max_steps: int = 0
    open_loop_horizon: int = 0
    policy_profiles_config: str = ""
    processed_preview_topic_prefix: str = "/policy_preview"
    state: DeploymentStateSnapshot = field(default_factory=DeploymentStateSnapshot)
    profiles: list[PolicyProfileEntry] = field(default_factory=list)
