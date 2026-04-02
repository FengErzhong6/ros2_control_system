from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional


@dataclass(frozen=True)
class SupervisorConfig:
    recipe_id: str
    recipe_directory: Optional[Path]
    startup_policy_config: Optional[Path]
    fault_policy_config: Optional[Path]


@dataclass(frozen=True)
class DeviceSpec:
    device_id: str
    adapter: str
    required: bool = True
    depends_on: list[str] = field(default_factory=list)
    config: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class RecipeSpec:
    recipe_id: str
    path: Path
    devices: list[DeviceSpec] = field(default_factory=list)
    record_topics: list[str] = field(default_factory=list)


@dataclass
class ActiveSession:
    session_id: str
    recipe_id: str
    operator_id: str
    session_tag: str
