"""ROS 2 Topic transport for one robot skill call at a time."""

import json
from typing import Callable

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from mujoco_demo.shared.contracts import PickPlaceTask
from mujoco_demo.shared.ros_contracts import (
    CANCEL_TOPIC, COMMAND_TOPIC, EVENT_TOPIC, SCENE_TOPIC,
    TaskCancel, TaskCommand, TaskEvent,
)


class RosSkillClient(Node):
    def __init__(
        self,
        on_event: Callable[[TaskEvent], None],
        on_scene: Callable[[dict[str, object]], None],
    ) -> None:
        super().__init__("mujoco_agent_control_platform")
        self._on_event_callback = on_event
        self._on_scene_callback = on_scene
        self._command_publisher = self.create_publisher(String, COMMAND_TOPIC, 10)
        self._cancel_publisher = self.create_publisher(String, CANCEL_TOPIC, 10)
        self.create_subscription(String, EVENT_TOPIC, self._on_event, 10)
        self.create_subscription(
            String, SCENE_TOPIC, self._on_scene,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE),
        )

    def has_bridge(self) -> bool:
        return self._command_publisher.get_subscription_count() > 0

    def send_task(self, request_id: str, task: PickPlaceTask) -> None:
        command = TaskCommand(request_id=request_id, task=task)
        self._command_publisher.publish(String(data=command.model_dump_json()))

    def cancel_task(self, request_id: str) -> None:
        cancel = TaskCancel(request_id=request_id)
        self._cancel_publisher.publish(String(data=cancel.model_dump_json()))

    def _on_event(self, message: String) -> None:
        self._on_event_callback(TaskEvent.model_validate_json(message.data))

    def _on_scene(self, message: String) -> None:
        self._on_scene_callback(json.loads(message.data))
