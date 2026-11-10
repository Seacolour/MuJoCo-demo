"""Verify that two ROS 2 task commands share one simulated robot in order."""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from bridge_node import MuJoCoBridge
from mujoco_demo.contracts import PickPlaceTask
from mujoco_demo.ros_contracts import COMMAND_TOPIC, EVENT_TOPIC, TaskCommand, TaskEvent


def main() -> None:
    rclpy.init()
    bridge = MuJoCoBridge()
    client = Node("mujoco_queue_smoke_client")
    events: list[TaskEvent] = []
    client.create_subscription(
        String, EVENT_TOPIC, lambda message: events.append(TaskEvent.model_validate_json(message.data)), 10
    )
    publisher = client.create_publisher(String, COMMAND_TOPIC, 10)
    task = PickPlaceTask(object_name="red_cube", target_name="target_zone")
    try:
        deadline = time.monotonic() + 15
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(client, timeout_sec=0.1)
        assert publisher.get_subscription_count() > 0, "bridge command subscription was not discovered"
        for request_id in ("queue-smoke-1", "queue-smoke-2"):
            publisher.publish(String(data=TaskCommand(request_id=request_id, task=task).model_dump_json()))
        while time.monotonic() < deadline:
            rclpy.spin_once(bridge, timeout_sec=0.01)
            rclpy.spin_once(client, timeout_sec=0.01)
            if len([event for event in events if event.status == "completed"]) == 2:
                break
        statuses = [(event.request_id, event.status) for event in events]
        assert ("queue-smoke-2", "queued") in statuses, statuses
        assert ("queue-smoke-1", "completed") in statuses, statuses
        assert ("queue-smoke-2", "completed") in statuses, statuses
        assert statuses.index(("queue-smoke-1", "completed")) < statuses.index(
            ("queue-smoke-2", "completed")
        ), statuses
        print("ROS 2 sequential task queue OK")
    finally:
        client.destroy_node()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
