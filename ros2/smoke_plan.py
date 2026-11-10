"""Verify parent-task completion and preflight rejection for ordered tool calls."""

import os
import time
from unittest.mock import patch

import rclpy

from bridge_node import MuJoCoBridge
from control_platform import AgentControlPlatform
from mujoco_demo.contracts import PickPlaceTask
from mujoco_demo.deepseek_planner import DeepSeekPlanner
from mujoco_demo.task_plan import TaskPlan


def wait_for_task(platform: AgentControlPlatform, bridge: MuJoCoBridge, request_id: str) -> dict:
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        rclpy.spin_once(bridge, timeout_sec=0.01)
        record = platform.task(request_id)
        if record["status"] in {"succeeded", "failed", "rejected", "cancelled"}:
            return record
    raise TimeoutError(f"task {request_id} did not finish")


def main() -> None:
    rclpy.init()
    bridge = MuJoCoBridge(multi_scene=True)
    platform = AgentControlPlatform()
    platform.start()
    try:
        deadline = time.monotonic() + 5
        while (not platform.connected()) and time.monotonic() < deadline:
            rclpy.spin_once(bridge, timeout_sec=0.05)
        assert platform.connected(), "platform did not receive the bridge scene"

        invalid = TaskPlan(steps=(
            PickPlaceTask(object_name="red_cube", target_name="blue_zone"),
            PickPlaceTask(object_name="green_cube", target_name="blue_zone"),
        ))
        valid = TaskPlan(steps=(
            PickPlaceTask(object_name="green_cube", target_name="red_zone"),
            PickPlaceTask(object_name="red_cube", target_name="green_zone"),
        ))
        with patch.dict(os.environ, {"DEEPSEEK_API_KEY": "test-key"}), patch.object(
            DeepSeekPlanner, "parse_plan",
            side_effect=lambda instruction, _scene: invalid if instruction == "invalid" else valid,
        ):
            rejected = wait_for_task(platform, bridge, platform.submit("invalid")["request_id"])
            assert rejected["status"] == "rejected", rejected
            assert bridge.scene_revision == 0, "invalid plan moved the robot"

            completed = wait_for_task(platform, bridge, platform.submit("valid")["request_id"])
            assert completed["status"] == "succeeded", completed
            assert completed["total_steps"] == 2
            assert len(completed["result"]["steps"]) == 2
            assert bridge.scene_revision == 2
        print("ROS 2 compound task orchestration OK")
    finally:
        platform.stop()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
