class SystemStates:
    IDLE = "IDLE"
    PREFLIGHT = "PREFLIGHT"
    STARTING = "STARTING"
    READY = "READY"
    ARMED = "ARMED"
    RECORDING = "RECORDING"
    PAUSED = "PAUSED"
    STOPPING = "STOPPING"
    FAULT = "FAULT"


class Commands:
    START_SYSTEM = "StartSystem"
    SHUTDOWN_SYSTEM = "ShutdownSystem"
    START_SESSION = "StartSession"
    STOP_SESSION = "StopSession"
    GO_HOME = "GoHome"
    ACKNOWLEDGE_FAULT = "AcknowledgeFault"


_ALLOWED_COMMANDS = {
    SystemStates.IDLE: [Commands.START_SYSTEM],
    SystemStates.READY: [
        Commands.SHUTDOWN_SYSTEM,
        Commands.GO_HOME,
        Commands.START_SESSION,
    ],
    SystemStates.ARMED: [
        Commands.SHUTDOWN_SYSTEM,
        Commands.GO_HOME,
        Commands.START_SESSION,
    ],
    SystemStates.RECORDING: [
        Commands.STOP_SESSION,
    ],
    SystemStates.PAUSED: [
        Commands.START_SESSION,
        Commands.STOP_SESSION,
        Commands.SHUTDOWN_SYSTEM,
    ],
    SystemStates.FAULT: [
        Commands.ACKNOWLEDGE_FAULT,
        Commands.SHUTDOWN_SYSTEM,
    ],
}


def allowed_commands_for(state: str) -> list[str]:
    return list(_ALLOWED_COMMANDS.get(state, []))


def is_command_allowed(state: str, command: str) -> bool:
    return command in _ALLOWED_COMMANDS.get(state, [])
