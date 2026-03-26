#!/usr/bin/env python3

import sys
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node

from manus_system.srv import CancelGloveCalibration
from manus_system.srv import FinishGloveCalibration
from manus_system.srv import ListGloves
from manus_system.srv import RunGloveCalibrationStep
from manus_system.srv import StartGloveCalibration


class ManusCalibrationWorkflowNode(Node):
    def __init__(self) -> None:
        super().__init__("manus_calibration_workflow_node")

        self.calibration_node_name = self.declare_parameter(
            "calibration_node_name", "/manus_calibration_node"
        ).value
        self.target_sides = self._normalize_sides(
            list(self.declare_parameter("target_sides", ["left", "right"]).value)
        )
        self.wait_for_services_timeout_sec = float(
            self.declare_parameter("wait_for_services_timeout_sec", 15.0).value
        )
        self.wait_for_glove_timeout_sec = float(
            self.declare_parameter("wait_for_glove_timeout_sec", 60.0).value
        )
        self.poll_period_sec = float(
            self.declare_parameter("poll_period_sec", 1.0).value
        )
        self.step_prepare_delay_sec = float(
            self.declare_parameter("step_prepare_delay_sec", 5.0).value
        )
        self.save_to_file = bool(
            self.declare_parameter("save_to_file", True).value
        )
        self.left_output_file = self.declare_parameter(
            "left_output_file", "left_glove.mcal"
        ).value
        self.right_output_file = self.declare_parameter(
            "right_output_file", "right_glove.mcal"
        ).value

        self.list_gloves_client = self.create_client(
            ListGloves, self._service_name("list_gloves")
        )
        self.start_calibration_client = self.create_client(
            StartGloveCalibration, self._service_name("start_calibration")
        )
        self.run_step_client = self.create_client(
            RunGloveCalibrationStep, self._service_name("run_calibration_step")
        )
        self.cancel_calibration_client = self.create_client(
            CancelGloveCalibration, self._service_name("cancel_calibration")
        )
        self.finish_calibration_client = self.create_client(
            FinishGloveCalibration, self._service_name("finish_calibration")
        )

    def run(self) -> bool:
        if not self.target_sides:
            self.get_logger().error("No valid target_sides configured.")
            return False

        if not self._wait_for_services():
            return False

        for side in self.target_sides:
            glove_id = self._wait_for_paired_glove(side)
            if glove_id is None:
                return False

            if not self._run_side_calibration(side, glove_id):
                return False

        self.get_logger().info("Calibration workflow finished successfully.")
        return True

    def _wait_for_services(self) -> bool:
        clients = [
            ("list_gloves", self.list_gloves_client),
            ("start_calibration", self.start_calibration_client),
            ("run_calibration_step", self.run_step_client),
            ("cancel_calibration", self.cancel_calibration_client),
            ("finish_calibration", self.finish_calibration_client),
        ]

        deadline = time.monotonic() + max(self.wait_for_services_timeout_sec, 0.0)
        for label, client in clients:
            while rclpy.ok():
                if client.wait_for_service(timeout_sec=self.poll_period_sec):
                    break
                if time.monotonic() > deadline:
                    self.get_logger().error(
                        f"Timed out waiting for service {label}."
                    )
                    return False
                self.get_logger().info(f"Waiting for service {label} ...")
        return True

    def _wait_for_paired_glove(self, side: str) -> Optional[int]:
        deadline = time.monotonic() + max(self.wait_for_glove_timeout_sec, 0.0)
        while rclpy.ok():
            response = self._call(self.list_gloves_client, ListGloves.Request())
            if response is not None and response.success:
                for glove_id, glove_side, paired_state in zip(
                    response.glove_ids,
                    response.sides,
                    response.paired_states,
                ):
                    if glove_side == side and paired_state == "paired":
                        self.get_logger().info(
                            f"Detected paired {side} glove with glove_id={glove_id}."
                        )
                        return glove_id

            if time.monotonic() > deadline:
                self.get_logger().error(
                    f"Timed out waiting for a paired {side} glove."
                )
                return None

            self.get_logger().info(
                f"Waiting for paired {side} glove and MANUS landscape ..."
            )
            time.sleep(max(self.poll_period_sec, 0.1))

        return None

    def _run_side_calibration(self, side: str, expected_glove_id: int) -> bool:
        self.get_logger().info(f"Starting calibration workflow for {side} glove.")

        start_request = StartGloveCalibration.Request()
        start_request.glove_id = 0
        start_request.side = side
        start_response = self._call(self.start_calibration_client, start_request)
        if start_response is None or not start_response.success:
            message = (
                "no response"
                if start_response is None
                else start_response.message
            )
            self.get_logger().error(
                f"Failed to start {side} glove calibration: {message}"
            )
            return False

        glove_id = int(start_response.resolved_glove_id)
        if glove_id != expected_glove_id:
            self.get_logger().warn(
                f"{side} glove_id changed from {expected_glove_id} to {glove_id} before calibration started."
            )

        for step_index in range(int(start_response.step_count)):
            title = self._safe_index(start_response.step_titles, step_index, "")
            description = self._safe_index(
                start_response.step_descriptions, step_index, ""
            )
            duration = self._safe_index(start_response.step_durations, step_index, 0.0)

            if title or description:
                self.get_logger().info(
                    f"[{side}] Step {step_index + 1}/{start_response.step_count}: {title} | {description} | hint={duration:.2f}s"
                )
            else:
                self.get_logger().info(
                    f"[{side}] Step {step_index + 1}/{start_response.step_count}: hint={duration:.2f}s"
                )

            if self.step_prepare_delay_sec > 0.0:
                self.get_logger().info(
                    f"[{side}] Starting step {step_index + 1} in {self.step_prepare_delay_sec:.1f}s ..."
                )
                time.sleep(self.step_prepare_delay_sec)

            step_request = RunGloveCalibrationStep.Request()
            step_request.glove_id = glove_id
            step_request.step_index = step_index
            step_response = self._call(self.run_step_client, step_request)
            if step_response is None or not step_response.success:
                message = (
                    "no response"
                    if step_response is None
                    else step_response.message
                )
                self.get_logger().error(
                    f"[{side}] Calibration step {step_index + 1} failed: {message}"
                )
                self._cancel_calibration(glove_id)
                return False

        finish_request = FinishGloveCalibration.Request()
        finish_request.glove_id = glove_id
        finish_request.save_to_file = self.save_to_file
        finish_request.file_name = self._output_file_for_side(side)
        finish_response = self._call(self.finish_calibration_client, finish_request)
        if finish_response is None or not finish_response.success:
            message = (
                "no response"
                if finish_response is None
                else finish_response.message
            )
            self.get_logger().error(
                f"Failed to finish {side} glove calibration: {message}"
            )
            return False

        saved_path = finish_response.saved_path.strip()
        if saved_path:
            self.get_logger().info(
                f"Finished {side} glove calibration and saved {saved_path}."
            )
        else:
            self.get_logger().info(f"Finished {side} glove calibration.")

        return True

    def _cancel_calibration(self, glove_id: int) -> None:
        cancel_request = CancelGloveCalibration.Request()
        cancel_request.glove_id = glove_id
        cancel_response = self._call(self.cancel_calibration_client, cancel_request)
        if cancel_response is None:
            self.get_logger().warn(
                f"Cancel calibration for glove_id={glove_id} returned no response."
            )
            return
        if not cancel_response.success:
            self.get_logger().warn(
                f"Cancel calibration for glove_id={glove_id} failed: {cancel_response.message}"
            )

    def _service_name(self, suffix: str) -> str:
        node_name = self.calibration_node_name.rstrip("/")
        return f"{node_name}/{suffix}"

    def _normalize_sides(self, sides: List[str]) -> List[str]:
        normalized = []
        for side in sides:
            side_value = str(side).strip().lower()
            if side_value not in {"left", "right"}:
                self.get_logger().warn(f"Ignoring unsupported side '{side}'.")
                continue
            if side_value in normalized:
                continue
            normalized.append(side_value)
        return normalized

    def _output_file_for_side(self, side: str) -> str:
        if side == "left":
            return self.left_output_file
        if side == "right":
            return self.right_output_file
        return ""

    def _call(self, client, request, timeout_sec: float = 120.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            self.get_logger().error(
                f"Timed out waiting for service {client.srv_name}."
            )
            return None
        if future.exception() is not None:
            self.get_logger().error(
                f"Service {client.srv_name} failed: {future.exception()}"
            )
            return None
        return future.result()

    @staticmethod
    def _safe_index(values, index: int, default):
        if index < len(values):
            return values[index]
        return default


def main() -> int:
    rclpy.init()
    node = ManusCalibrationWorkflowNode()
    try:
        return 0 if node.run() else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
