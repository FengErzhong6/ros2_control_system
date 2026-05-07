from dataclasses import dataclass


@dataclass(frozen=True)
class HotkeyBinding:
    key: str
    command: str


DEFAULT_HOTKEYS = [
    HotkeyBinding(key="F5", command="StartSystem"),
    HotkeyBinding(key="Shift+F5", command="ShutdownSystem"),
    HotkeyBinding(key="Ctrl+T", command="StartTeleoperation"),
    HotkeyBinding(key="Ctrl+Shift+T", command="StopTeleoperation"),
    HotkeyBinding(key="Ctrl+Space", command="StartSession"),
    HotkeyBinding(key="S", command="StopSession"),
    HotkeyBinding(key="H", command="GoHome"),
    HotkeyBinding(key="Esc", command="AcknowledgeFault"),
]
