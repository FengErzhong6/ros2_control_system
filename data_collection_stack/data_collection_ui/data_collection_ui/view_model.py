from dataclasses import dataclass, field

from .hotkeys import DEFAULT_HOTKEYS, HotkeyBinding


@dataclass(frozen=True)
class CameraStreamConfig:
    camera_id: str
    title: str
    preview_topic: str
    capture_topic: str | None = None
    preview_fps_limit: float | None = None
    capture_fps_target: float | None = None


@dataclass(frozen=True)
class UiRuntimeConfig:
    title: str = "Data Collection Console"
    refresh_hz: int = 20
    camera_grid_columns: int = 2
    enable_hotkeys: bool = True
    camera_streams: tuple[CameraStreamConfig, ...] = ()
    recipe_id: str = ""


@dataclass
class UiViewModel:
    config: UiRuntimeConfig = field(default_factory=UiRuntimeConfig)
    hotkeys: list[HotkeyBinding] = field(default_factory=lambda: list(DEFAULT_HOTKEYS))

    @property
    def title(self) -> str:
        return self.config.title
