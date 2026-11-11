"""Interpret a user instruction, send the task over ROS 2, and read feedback."""

import argparse
import json
import time
from uuid import uuid4

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from mujoco_demo.platform.deepseek_planner import DeepSeekPlanner
from mujoco_demo.shared.contracts import PickPlaceTask
from mujoco_demo.shared.ros_contracts import (
    CANCEL_TOPIC, COMMAND_TOPIC, EVENT_TOPIC, SCENE_TOPIC,
    TaskCancel, TaskCommand, TaskEvent
)


class AgentTaskClient(Node):
    def __init__(self, request_id: str) -> None:
        super().__init__("mujoco_agent_task_client")
        self.request_id = request_id
        self.final_event: TaskEvent | None = None
        self.scene_snapshot: dict[str, object] | None = None
        self.publisher = self.create_publisher(String, COMMAND_TOPIC, 10)
        self.cancel_publisher = self.create_publisher(String, CANCEL_TOPIC, 10)
        self.create_subscription(String, EVENT_TOPIC, self._on_event, 10)
        self.create_subscription(
            String, SCENE_TOPIC, self._on_scene,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE),
        )

    def _on_scene(self, message: String) -> None:
        self.scene_snapshot = json.loads(message.data)

    def wait_scene(self, timeout: float = 3.0) -> dict[str, object] | None:
        deadline = time.monotonic() + timeout
        while self.scene_snapshot is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.scene_snapshot

    def _on_event(self, message: String) -> None:
        event = TaskEvent.model_validate_json(message.data)
        if event.request_id != self.request_id:
            return
        print(f"{event.status}: {event.phase or event.message or ''}", flush=True)
        if event.status in ("completed", "failed", "cancelled", "rejected"):
            self.final_event = event

    def run_task(self, task: PickPlaceTask, timeout: float, cancel_after: float | None) -> TaskEvent:
        deadline = time.monotonic() + timeout
        while self.publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.publisher.get_subscription_count() == 0:
            raise TimeoutError("MuJoCo ROS 2 bridge is not running")
        command = TaskCommand(request_id=self.request_id, task=task)
        self.publisher.publish(String(data=command.model_dump_json()))
        started_at = time.monotonic()
        cancel_sent = False
        while self.final_event is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (cancel_after is not None and not cancel_sent
                    and time.monotonic() - started_at >= cancel_after):
                cancel = TaskCancel(request_id=self.request_id)
                self.cancel_publisher.publish(String(data=cancel.model_dump_json()))
                cancel_sent = True
        if self.final_event is None:
            raise TimeoutError("MuJoCo ROS 2 bridge did not return a result")
        return self.final_event


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("instruction", nargs="?", default="把红色方块放到绿色目标区")
    parser.add_argument("--offline", action="store_true", help="exercise ROS 2 without a model API call")
    parser.add_argument("--object-name", default="red_cube", help="object for --offline mode")
    parser.add_argument("--target-name", default="target_zone", help="zone for --offline mode")
    parser.add_argument("--timeout", type=float, default=45)
    parser.add_argument("--cancel-after", type=float, help="request cancellation after N seconds")
    args = parser.parse_args()

    rclpy.init()
    client = AgentTaskClient(str(uuid4()))
    try:
        task = (
            PickPlaceTask(object_name=args.object_name, target_name=args.target_name)
            if args.offline
            else DeepSeekPlanner().parse(args.instruction, client.wait_scene())
        )
        print(f"task: {task.model_dump_json()}", flush=True)
        event = client.run_task(task, args.timeout, args.cancel_after)
        if event.result is not None:
            print(f"cube_final_m: {event.result['cube_final_m']}")
        raise SystemExit(0 if event.status == "completed" else 1)
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
