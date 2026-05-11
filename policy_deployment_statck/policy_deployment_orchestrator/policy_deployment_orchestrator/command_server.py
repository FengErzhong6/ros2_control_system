from __future__ import annotations

from rclpy.action import ActionServer, CancelResponse, GoalResponse

from policy_deployment_interfaces.action import (
    ConnectSystem,
    DisconnectSystem,
    ExecutePolicy,
    GoHome,
    StopPolicy,
)

from .state_machine import Commands


class CommandServer:
    def __init__(self, node, supervisor) -> None:
        self._node = node
        self._supervisor = supervisor
        self._servers = [
            ActionServer(
                node,
                ConnectSystem,
                "connect_system",
                execute_callback=self._wrap_execute(
                    Commands.CONNECT_SYSTEM,
                    self._supervisor.execute_connect_system,
                ),
                goal_callback=self._make_goal_callback(Commands.CONNECT_SYSTEM),
                cancel_callback=self._cancel_callback,
            ),
            ActionServer(
                node,
                DisconnectSystem,
                "disconnect_system",
                execute_callback=self._wrap_execute(
                    Commands.DISCONNECT_SYSTEM,
                    self._supervisor.execute_disconnect_system,
                ),
                goal_callback=self._make_goal_callback(Commands.DISCONNECT_SYSTEM),
                cancel_callback=self._cancel_callback,
            ),
            ActionServer(
                node,
                ExecutePolicy,
                "execute_policy",
                execute_callback=self._wrap_execute(
                    Commands.EXECUTE_POLICY,
                    self._supervisor.execute_policy,
                ),
                goal_callback=self._make_goal_callback(Commands.EXECUTE_POLICY),
                cancel_callback=self._cancel_callback,
            ),
            ActionServer(
                node,
                StopPolicy,
                "stop_policy",
                execute_callback=self._wrap_execute(
                    Commands.STOP_POLICY,
                    self._supervisor.execute_stop_policy,
                ),
                goal_callback=self._make_goal_callback(Commands.STOP_POLICY),
                cancel_callback=self._cancel_callback,
            ),
            ActionServer(
                node,
                GoHome,
                "go_home",
                execute_callback=self._wrap_execute(
                    Commands.GO_HOME,
                    self._supervisor.execute_go_home,
                ),
                goal_callback=self._make_goal_callback(Commands.GO_HOME),
                cancel_callback=self._cancel_callback,
            ),
        ]

    def describe(self) -> str:
        return "Action servers registered for connect, disconnect, policy execution, stop, and go_home."

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
                self._supervisor.finish_command(command, success=False, detail=f"raised {exc!r}")
                raise

            success = bool(getattr(result, "success", False))
            message = str(getattr(result, "message", "")).strip()
            self._supervisor.finish_command(command, success=success, detail=message)
            return result

        return _execute

    @staticmethod
    def _describe_goal(goal_request) -> str:
        fields = []
        for name in (
            "recipe_id",
            "policy_profile_id",
            "site_name",
            "operator_id",
            "prompt",
            "reason",
        ):
            value = getattr(goal_request, name, None)
            if value not in {None, ""}:
                fields.append(f"{name}={value}")
        if hasattr(goal_request, "force"):
            fields.append(f"force={bool(goal_request.force)}")
        if hasattr(goal_request, "arm_after_home"):
            fields.append(f"arm_after_home={bool(goal_request.arm_after_home)}")
        if hasattr(goal_request, "max_steps"):
            fields.append(f"max_steps={int(goal_request.max_steps)}")
        if hasattr(goal_request, "open_loop_horizon"):
            fields.append(f"open_loop_horizon={int(goal_request.open_loop_horizon)}")
        return " ".join(fields)

    @staticmethod
    def _cancel_callback(goal_handle):
        del goal_handle
        return CancelResponse.REJECT
