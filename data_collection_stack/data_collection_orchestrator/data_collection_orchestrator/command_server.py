from __future__ import annotations

from rclpy.action import ActionServer, CancelResponse, GoalResponse

from data_collection_interfaces.action import (
    GoHome,
    ShutdownSystem,
    StartSession,
    StartSystem,
    StopSession,
)


class CommandServer:
    def __init__(self, node, supervisor) -> None:
        self._supervisor = supervisor
        self._servers = [
            ActionServer(
                node,
                StartSystem,
                "start_system",
                execute_callback=self._supervisor.execute_start_system,
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
            ),
            ActionServer(
                node,
                ShutdownSystem,
                "shutdown_system",
                execute_callback=self._supervisor.execute_shutdown_system,
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
            ),
            ActionServer(
                node,
                StartSession,
                "start_session",
                execute_callback=self._supervisor.execute_start_session,
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
            ),
            ActionServer(
                node,
                StopSession,
                "stop_session",
                execute_callback=self._supervisor.execute_stop_session,
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
            ),
            ActionServer(
                node,
                GoHome,
                "go_home",
                execute_callback=self._supervisor.execute_go_home,
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
            ),
        ]

    def describe(self) -> str:
        return "Action servers registered for start, shutdown, session, and home commands."

    @staticmethod
    def _goal_callback(goal_request):
        del goal_request
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(goal_handle):
        del goal_handle
        return CancelResponse.REJECT
