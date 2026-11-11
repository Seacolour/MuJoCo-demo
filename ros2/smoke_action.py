"""Exercise the custom Action across platform and persistent MuJoCo runtime."""

import os
import threading
import time
from unittest.mock import patch

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from mujoco_demo_interfaces.action import PickPlace

from control_platform import AgentControlPlatform
from bridge_node import MuJoCoBridge
from mujoco_demo.platform.deepseek_planner import DeepSeekPlanner
from mujoco_demo.shared.contracts import PickPlaceTask
from mujoco_demo.shared.task_plan import TaskPlan

from smoke_domain import isolate_ros_domain


def wait_for_task(platform: AgentControlPlatform, request_id: str) -> dict:
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        record = platform.task(request_id)
        if record["status"] in {"succeeded", "failed", "rejected", "cancelled"}:
            return record
        time.sleep(0.02)
    raise TimeoutError(f"task {request_id} did not finish")


def main() -> None:
    isolate_ros_domain()
    rclpy.init()
    bridge = MuJoCoBridge(multi_scene=True, enable_action=True)
    bridge_executor = MultiThreadedExecutor(num_threads=4)
    bridge_executor.add_node(bridge)
    bridge_thread = threading.Thread(target=bridge_executor.spin, daemon=True)
    bridge_thread.start()
    platform = AgentControlPlatform(use_action=True)
    platform.start()
    try:
        deadline = time.monotonic() + 5
        while not platform.connected() and time.monotonic() < deadline:
            time.sleep(0.05)
        assert platform.connected(), "Action server or scene was not discovered"

        valid = TaskPlan(steps=(
            PickPlaceTask(object_name="green_cube", target_name="red_zone"),
            PickPlaceTask(object_name="red_cube", target_name="green_zone"),
        ))
        swap = TaskPlan(steps=(
            PickPlaceTask(object_name="red_cube", target_name="red_zone"),
            PickPlaceTask(object_name="green_cube", target_name="green_zone"),
        ))
        invalid = TaskPlan(steps=(
            PickPlaceTask(object_name="red_cube", target_name="blue_zone"),
            PickPlaceTask(object_name="green_cube", target_name="blue_zone"),
        ))
        to_blue = TaskPlan(steps=(
            PickPlaceTask(object_name="red_cube", target_name="blue_zone"),
        ))
        with patch.dict(os.environ, {"DEEPSEEK_API_KEY": "test-key"}), patch.object(
            DeepSeekPlanner, "parse_plan",
            side_effect=lambda instruction, _scene: {
                "invalid": invalid, "valid": valid, "swap": swap,
                "already": swap, "cancel": to_blue,
            }[instruction],
        ):
            rejected = wait_for_task(platform, platform.submit("invalid")["request_id"])
            assert rejected["status"] == "rejected", rejected
            assert bridge.scene_revision == 0
            assert not bridge._seen_ids, "invalid plan sent an Action goal"

            valid_id = platform.submit("valid")["request_id"]
            completed = wait_for_task(platform, valid_id)
            assert completed["status"] == "succeeded", completed
            assert len(completed["result"]["steps"]) == 2
            assert any(event["phase"] == "approach" for event in completed["events"])
            assert [event["step_index"] for event in completed["events"]
                    if event["phase"] == "return_home"] == [2]
            assert bridge.scene_revision == 2

            exchanged = wait_for_task(platform, platform.submit("swap")["request_id"])
            assert exchanged["status"] == "succeeded", exchanged
            assert len(exchanged["result"]["steps"]) == 3
            assert [event["step_index"] for event in exchanged["events"]
                    if event["phase"] == "return_home"] == [3]
            assert bridge.scene_revision == 5
            scene = bridge.simulation.catalog.snapshot(bridge.simulation.data)
            assert scene["zones"]["red_zone"]["occupied_by"] == "red_cube"
            assert scene["zones"]["green_zone"]["occupied_by"] == "green_cube"

            goals_before = len(bridge._seen_ids)
            already = wait_for_task(platform, platform.submit("already")["request_id"])
            assert already["status"] == "succeeded", already
            assert already["total_steps"] == 0
            assert already["result"]["already_satisfied"] is True
            assert bridge.scene_revision == 5
            assert len(bridge._seen_ids) == goals_before, "zero-step task sent an Action goal"

            grasped = threading.Event()
            resume_motion = threading.Event()
            original_run = bridge.simulation.run

            def pause_after_grasp(task: PickPlaceTask, *, return_home: bool = True):
                report_phase = bridge.simulation.on_phase

                def report_and_pause(phase) -> None:
                    report_phase(phase)
                    if phase.name == "close_gripper":
                        grasped.set()
                        if not resume_motion.wait(5):
                            raise TimeoutError("test did not release the grasp checkpoint")

                bridge.simulation.on_phase = report_and_pause
                return original_run(task, return_home=return_home)

            with patch.object(bridge.simulation, "run", side_effect=pause_after_grasp):
                cancel_id = platform.submit("cancel")["request_id"]
                assert grasped.wait(5), "robot did not grasp before cancellation"
                platform.cancel(cancel_id)
                deadline = time.monotonic() + 5
                while (not bridge._active_cancel.is_set()) and time.monotonic() < deadline:
                    time.sleep(0.01)
                assert bridge._active_cancel.is_set(), "Action cancellation did not reach the robot"
                resume_motion.set()
                cancelled = wait_for_task(platform, cancel_id)
                assert cancelled["status"] == "cancelled", cancelled
                assert bridge.scene_revision == 5

            retried = wait_for_task(platform, platform.submit("cancel")["request_id"])
            assert retried["status"] == "succeeded", retried
            assert bridge.scene_revision == 6

            direct_node = Node("mujoco_action_contract_smoke")
            direct_client = ActionClient(direct_node, PickPlace, "/mujoco_demo/pick_place")
            try:
                assert direct_client.wait_for_server(timeout_sec=5)
                for request_id, revision in ((f"{valid_id}:step:1", 6), ("stale-revision", 0)):
                    goal = PickPlace.Goal()
                    goal.request_id = request_id
                    goal.object_name = "red_cube"
                    goal.target_name = "blue_zone"
                    goal.expected_scene_revision = revision
                    response = direct_client.send_goal_async(goal)
                    rclpy.spin_until_future_complete(direct_node, response, timeout_sec=5)
                    assert response.done() and not response.result().accepted
                invalid = PickPlace.Goal()
                invalid.request_id = "invalid-object"
                invalid.object_name = "missing_cube"
                invalid.target_name = "blue_zone"
                invalid.expected_scene_revision = 6
                accepted = direct_client.send_goal_async(invalid)
                rclpy.spin_until_future_complete(direct_node, accepted, timeout_sec=5)
                assert accepted.done() and accepted.result().accepted
                result_future = accepted.result().get_result_async()
                rclpy.spin_until_future_complete(direct_node, result_future, timeout_sec=5)
                assert result_future.done()
                assert result_future.result().status == GoalStatus.STATUS_ABORTED
                assert result_future.result().result.error_code == "INVALID_GOAL"
                assert bridge.scene_revision == 6
            finally:
                direct_client.destroy()
                direct_node.destroy_node()
        print("ROS 2 Action feedback, result, cancel, dedupe and stale-scene checks OK")
    finally:
        platform.stop()
        bridge_executor.shutdown()
        bridge_thread.join(timeout=5)
        if bridge._action_server is not None:
            bridge._action_server.destroy()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
