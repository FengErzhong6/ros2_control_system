#!/usr/bin/env python3

from __future__ import annotations

from ament_index_python.packages import get_package_prefix
import os
from pathlib import Path
import rclpy
from rclpy.node import Node
import signal
import subprocess
import sys
import time


def _move_group_executable() -> str:
    prefix = Path(get_package_prefix("moveit_ros_move_group"))
    executable = prefix / "lib" / "moveit_ros_move_group" / "move_group"
    if not executable.is_file():
        raise FileNotFoundError(f"move_group executable not found: {executable}")
    return str(executable)


def _node_fqn(namespace: str, name: str) -> str:
    ns = (namespace or "").strip()
    if not ns or ns == "/":
        return f"/{name}"
    return f"{ns.rstrip('/')}/{name}"


def _wait_for_graph_cleanup(timeout_sec: float = 6.0, poll_sec: float = 0.2) -> None:
    prefixes = (
        "/move_group",
        "/move_group/moveit",
        "/move_group_private_",
        "/moveit_",
        "/moveit_simple_controller_manager",
        "/planning_scene_interface_",
        "/transform_listener_impl_",
    )

    rclpy.init(args=None)
    node = Node("move_group_wrapper_waiter")
    try:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        stable = 0
        while time.monotonic() < deadline and rclpy.ok():
            discovered = [
                _node_fqn(namespace, name)
                for name, namespace in node.get_node_names_and_namespaces()
            ]
            remaining = [
                name
                for name in discovered
                if any(name == prefix or name.startswith(prefix) for prefix in prefixes)
            ]
            if not remaining:
                stable += 1
                if stable >= 3:
                    return
            else:
                stable = 0
            time.sleep(max(0.05, poll_sec))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _forwarded_child_signal(signum: signal.Signals) -> signal.Signals:
    if signum == signal.SIGINT:
        return signal.SIGINT
    return signal.SIGTERM


def main() -> int:
    child: subprocess.Popen | None = None
    forwarded_signal = signal.SIGTERM
    shutdown_requested = False
    child_sigterm_deadline: float | None = None
    child_sigterm_sent = False

    def _forward_shutdown(signum, _frame):
        nonlocal forwarded_signal, shutdown_requested, child_sigterm_deadline, child_sigterm_sent
        forwarded_signal = signal.Signals(signum)
        shutdown_requested = True
        if forwarded_signal == signal.SIGINT and child_sigterm_deadline is None:
            child_sigterm_deadline = time.monotonic() + 4.0
        if child is None or child.poll() is not None:
            return
        try:
            os.killpg(child.pid, _forwarded_child_signal(forwarded_signal))
        except ProcessLookupError:
            return

    signal.signal(signal.SIGINT, _forward_shutdown)
    signal.signal(signal.SIGTERM, _forward_shutdown)

    cmd = [_move_group_executable(), *sys.argv[1:]]
    child = subprocess.Popen(cmd, start_new_session=True)

    try:
        while child.poll() is None:
            if (
                shutdown_requested
                and not child_sigterm_sent
                and child_sigterm_deadline is not None
                and time.monotonic() >= child_sigterm_deadline
            ):
                try:
                    os.killpg(child.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                else:
                    child_sigterm_sent = True
            time.sleep(0.05)
    except KeyboardInterrupt:
        _forward_shutdown(signal.SIGINT, None)
    finally:
        if child is not None and child.poll() is None:
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and child.poll() is None:
                time.sleep(0.05)
            if child.poll() is None:
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                child.wait(timeout=2.0)

    if child is None:
        return 1

    if shutdown_requested:
        if child.returncode not in (0, None):
            print(
                "[move_group_wrapper] move_group exited with non-zero status during coordinated "
                f"shutdown ({child.returncode}); treating as clean shutdown.",
                file=sys.stderr,
                flush=True,
            )
        _wait_for_graph_cleanup()
        return 0

    if child.returncode is None:
        return 128 + int(forwarded_signal)
    if child.returncode < 0:
        return 128 + abs(child.returncode)
    return child.returncode


if __name__ == "__main__":
    sys.exit(main())
