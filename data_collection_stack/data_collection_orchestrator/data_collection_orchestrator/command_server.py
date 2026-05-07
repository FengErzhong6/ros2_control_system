from __future__ import annotations

from rclpy.action import ActionServer, CancelResponse, GoalResponse

from data_collection_interfaces.action import (
    GoHome,
    ShutdownSystem,
    StartSession,
    StartSystem,
    StartTeleoperation,
    StopSession,
    StopTeleoperation,
)
from .state_machine import Commands


class CommandServer:
    def __init__(self, node, supervisor) -> None:
        self._node = node
        self._supervisor = supervisor
        self._servers = [
            ActionServer(
                node,
                StartSystem,
                "start_system",
                execute_callback=self._wrap_execute(
                    Commands.START_SYSTEM, self._supervisor.execute_start_system
                ),
                goal_callback=self._make_goal_callback(Commands.START_SYSTEM),
                cancel_callback=self._cancel_callback,
            ),
            ActionServer(
                node,
                ShutdownSystem,
                "shutdown_system",
                execute_callback=self._wrap_execute(
                    Commands.SHUTDOWN_SYSTEM, self._supervisor.execute_shutdown_system
                ),
                goal_callback=self._make_goal_callback(Commands.SHUTDOWN_SYSTEM),
                cancel_callback=self._cancel_callback,
            ),
            ActionServer(
                node,
                StartTeleoperation,
                "start_teleoperation",
                execute_callback=self._wrap_execute(
                    Commands.START_TELEOPERATION,
                    self._supervisor.execute_start_teleoperation,
                ),
                goal_callback=self._make_goal_callback(Commands.START_TELEOPERATION),
                cancel_callback=self._cancel_callback,
            ),
            ActionServer(
                node,
                StopTeleoperation,
                "stop_teleoperation",
                execute_callback=self._wrap_execute(
                    Commands.STOP_TELEOPERATION,
                    self._supervisor.execute_stop_teleoperation,
                ),
                goal_callback=self._make_goal_callback(Commands.STOP_TELEOPERATION),
                cancel_callback=self._cancel_callback,
            ),
            ActionServer(
                node,
                StartSession,
                "start_session",
                execute_callback=self._wrap_execute(
                    Commands.START_SESSION, self._supervisor.execute_start_session
                ),
                goal_callback=self._make_goal_callback(Commands.START_SESSION),
                cancel_callback=self._cancel_callback,
            ),
            ActionServer(
                node,
                StopSession,
                "stop_session",
                execute_callback=self._wrap_execute(
                    Commands.STOP_SESSION, self._supervisor.execute_stop_session
                ),
                goal_callback=self._make_goal_callback(Commands.STOP_SESSION),
                cancel_callback=self._cancel_callback,
            ),
            ActionServer(
                node,
                GoHome,
                "go_home",
                execute_callback=self._wrap_execute(
                    Commands.GO_HOME, self._supervisor.execute_go_home
                ),
                goal_callback=self._make_goal_callback(Commands.GO_HOME),
                cancel_callback=self._cancel_callback,
            ),
        ]

    def describe(self) -> str:
        return "Action servers registered for start, shutdown, teleop, session, and home commands."

    def _make_goal_callback(self, command: str):
        def _goal_callback(goal_request):
            del goal_request
            accepted, reason = self._supervisor.try_begin_command(command)
            if not accepted:
                self._node.get_logger().warn(reason)
                return GoalResponse.REJECT
            self._node.get_logger().info(f"{command} accepted.")
            return GoalResponse.ACCEPT

        return _goal_callback

    def _wrap_execute(self, command: str, execute_callback):
        def _execute(goal_handle):
            detail = self._describe_goal(goal_handle.request)
            if detail:
                self._node.get_logger().info(f"{command} started. {detail}")
            else:
                self._node.get_logger().info(f"{command} started.")
            try:
                result = execute_callback(goal_handle)
            except Exception as exc:
                self._supervisor.finish_command(
                    command,
                    success=False,
                    detail=f"raised {exc!r}",
                )
                raise

            success = bool(getattr(result, "success", False))
            message = str(getattr(result, "message", "")).strip()
            self._supervisor.finish_command(
                command,
                success=success,
                detail=message,
            )
            return result

        return _execute

    @staticmethod
    def _describe_goal(goal_request) -> str:
        fields = []
        for name in (
            "recipe_id",
            "operator_id",
            "site_name",
            "session_name",
            "session_root",
            "session_tag",
            "reason",
        ):
            value = getattr(goal_request, name, None)
            if value not in {None, ""}:
                fields.append(f"{name}={value}")
        if hasattr(goal_request, "arm_after_home"):
            fields.append(f"arm_after_home={bool(goal_request.arm_after_home)}")
        if hasattr(goal_request, "force"):
            fields.append(f"force={bool(goal_request.force)}")
        return " ".join(fields)

    @staticmethod
    def _cancel_callback(goal_handle):
        del goal_handle
        return CancelResponse.REJECT
