#!/usr/bin/env python3

from __future__ import annotations

import time

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


class MarvinMotionServer(Node):
    def __init__(self) -> None:
        super().__init__(
            "marvin_motion_server",
            automatically_declare_parameters_from_overrides=True,
        )
        self._callback_group = ReentrantCallbackGroup()

        self._backend = str(self._get_parameter_value("backend", "legacy")).strip().lower()
        self._go_home_service_name = str(
            self._get_parameter_value("go_home_service_name", "/marvin_motion/go_home")
        )
        self._legacy_go_home_service = str(
            self._get_parameter_value(
                "legacy_go_home_service", "/tracker_teleop_controller/go_home"
            )
        )
        self._allow_legacy_go_home_fallback = bool(
            self._get_parameter_value("allow_legacy_go_home_fallback", False)
        )

        self._legacy_go_home_client = self.create_client(
            Trigger,
            self._legacy_go_home_service,
            callback_group=self._callback_group,
        )
        self._go_home_server = self.create_service(
            Trigger,
            self._go_home_service_name,
            self._handle_go_home,
            callback_group=self._callback_group,
        )

        home_left = self._safe_double_array("named_poses.home.left")
        home_right = self._safe_double_array("named_poses.home.right")
        self._scene_object_count = self._count_scene_objects()
        self._moveit_runtime_available, self._moveit_missing_packages = self._detect_moveit_runtime()
        self.get_logger().info(
            "Motion layer ready. "
            f"backend={self._backend} "
            f"service={self._go_home_service_name} "
            f"legacy_fallback={'ON' if self._allow_legacy_go_home_fallback else 'OFF'} "
            f"home_left={len(home_left)} home_right={len(home_right)} "
            f"scene_objects={self._scene_object_count}"
        )
        if self._backend == "moveit":
            if self._moveit_runtime_available:
                self.get_logger().info("MoveIt runtime packages detected.")
            else:
                self.get_logger().error(
                    "MoveIt backend requested but required packages are missing: "
                    f"{', '.join(self._moveit_missing_packages)}"
                )

    def _safe_double_array(self, name: str) -> list[float]:
        if not self.has_parameter(name):
            return []
        value = self.get_parameter(name).value
        if not isinstance(value, (list, tuple)):
            return []
        return [float(item) for item in value]

    def _get_parameter_value(self, name: str, default_value):
        if self.has_parameter(name):
            return self.get_parameter(name).value
        return self.declare_parameter(name, default_value).value

    def _count_scene_objects(self) -> int:
        prefixes = self.list_parameters(["scene"], depth=3).names
        object_names = set()
        for name in prefixes:
            parts = name.split(".")
            if len(parts) >= 3 and parts[0] == "scene":
                object_names.add(parts[1])
        return len(object_names)

    def _detect_moveit_runtime(self) -> tuple[bool, list[str]]:
        required_packages = [
            "moveit_ros_move_group",
            "moveit_ros_planning_interface",
            "moveit_configs_utils",
        ]
        missing: list[str] = []
        for package_name in required_packages:
            try:
                get_package_share_directory(package_name)
            except PackageNotFoundError:
                missing.append(package_name)
        return len(missing) == 0, missing

    def _handle_go_home(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        if self._backend == "moveit":
            response.success = False
            if self._moveit_runtime_available:
                response.message = (
                    "MoveIt backend is configured but go_home planning is not implemented yet."
                )
            else:
                response.message = (
                    "MoveIt backend requested but runtime packages are missing: "
                    f"{', '.join(self._moveit_missing_packages)}"
                )
            self.get_logger().error(response.message)
            return response

        if self._backend != "legacy":
            response.success = False
            response.message = f"Unsupported motion backend: {self._backend}"
            self.get_logger().error(response.message)
            return response

        if not self._allow_legacy_go_home_fallback:
            response.success = False
            response.message = (
                "Legacy go_home fallback is disabled. "
                "Select the MoveIt backend only after the planner is implemented."
            )
            self.get_logger().error(response.message)
            return response

        if not self._legacy_go_home_client.wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = (
                f"Legacy go_home service unavailable: {self._legacy_go_home_service}"
            )
            self.get_logger().error(response.message)
            return response

        self.get_logger().warn(
            "Using legacy go_home fallback via "
            f"{self._legacy_go_home_service}. This path is transitional and not MoveIt-backed."
        )
        future = self._legacy_go_home_client.call_async(Trigger.Request())
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if future.done():
                break
            time.sleep(0.01)

        if not future.done():
            response.success = False
            response.message = "Legacy go_home fallback timed out."
            self.get_logger().error(response.message)
            return response

        if future.exception() is not None:
            response.success = False
            response.message = f"Legacy go_home fallback failed: {future.exception()}"
            self.get_logger().error(response.message)
            return response

        legacy_response = future.result()
        response.success = bool(legacy_response.success)
        response.message = str(legacy_response.message)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MarvinMotionServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
