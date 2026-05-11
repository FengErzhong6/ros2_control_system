class SystemStates:
    IDLE = "IDLE"
    STARTING = "STARTING"
    READY = "READY"
    EXECUTING = "EXECUTING"
    HOMING = "HOMING"
    STOPPING = "STOPPING"
    FAULT = "FAULT"


class RolloutStates:
    IDLE = "IDLE"
    CONNECTING = "CONNECTING"
    FETCHING = "FETCHING"
    EXECUTING = "EXECUTING"
    STOPPING = "STOPPING"
    FAULT = "FAULT"


class Commands:
    CONNECT_SYSTEM = "ConnectSystem"
    DISCONNECT_SYSTEM = "DisconnectSystem"
    EXECUTE_POLICY = "ExecutePolicy"
    STOP_POLICY = "StopPolicy"
    GO_HOME = "GoHome"


_ALLOWED_COMMANDS = {
    SystemStates.IDLE: [Commands.CONNECT_SYSTEM],
    SystemStates.STARTING: [],
    SystemStates.READY: [
        Commands.DISCONNECT_SYSTEM,
        Commands.EXECUTE_POLICY,
        Commands.GO_HOME,
    ],
    SystemStates.EXECUTING: [
        Commands.STOP_POLICY,
        Commands.GO_HOME,
        Commands.DISCONNECT_SYSTEM,
    ],
    SystemStates.HOMING: [],
    SystemStates.STOPPING: [],
    SystemStates.FAULT: [Commands.DISCONNECT_SYSTEM],
}


def allowed_commands_for(state: str) -> list[str]:
    return list(_ALLOWED_COMMANDS.get(state, []))


def is_command_allowed(state: str, command: str) -> bool:
    return command in _ALLOWED_COMMANDS.get(state, [])
