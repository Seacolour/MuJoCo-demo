"""Verify that a generated PickPlace Action works across Windows processes."""

from pathlib import Path
import subprocess
import sys

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node

from mujoco_demo_interfaces.action import PickPlace
from mujoco_demo.shared.ros_contracts import ACTION_NAME

from smoke_domain import isolate_ros_domain


def main() -> None:
    isolate_ros_domain()
    bridge = subprocess.Popen(
        [sys.executable, "bridge_node.py", "--multi", "--action"],
        cwd=Path(__file__).resolve().parent,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        creationflags=subprocess.CREATE_NO_WINDOW,
    )
    rclpy.init()
    node = Node("mujoco_action_process_smoke")
    client = ActionClient(node, PickPlace, ACTION_NAME)
    try:
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError("bridge Action server did not start in its own process")
        goal = PickPlace.Goal()
        goal.request_id = "process-smoke-red-blue"
        goal.object_name = "red_cube"
        goal.target_name = "blue_zone"
        goal.expected_scene_revision = 0
        accepted = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, accepted, timeout_sec=10)
        assert accepted.done() and accepted.result().accepted
        result_future = accepted.result().get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=20)
        assert result_future.done()
        outcome = result_future.result()
        assert outcome.status == GoalStatus.STATUS_SUCCEEDED, outcome.status
        assert outcome.result.success
        assert outcome.result.scene_revision == 1
        print("ROS 2 PickPlace Action across processes OK")
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()
        bridge.terminate()
        try:
            bridge.wait(timeout=5)
        except subprocess.TimeoutExpired:
            bridge.kill()
            bridge.wait(timeout=5)
        if bridge.stdout is not None:
            bridge.stdout.close()


if __name__ == "__main__":
    main()
