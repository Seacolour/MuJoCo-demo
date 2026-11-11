"""ROS 2 transport for one robot skill call at a time."""

import json
import threading
from typing import Callable

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from mujoco_demo.shared.contracts import PickPlaceTask
from mujoco_demo.shared.ros_contracts import (
    ACTION_NAME, CANCEL_TOPIC, COMMAND_TOPIC, EVENT_TOPIC, SCENE_TOPIC,
    TaskCancel, TaskCommand, TaskEvent,
)


class RosSkillClient(Node):
    def __init__(
        self,
        on_event: Callable[[TaskEvent], None],
        on_scene: Callable[[dict[str, object]], None],
        *,
        use_action: bool = False,
    ) -> None:
        super().__init__("mujoco_agent_control_platform")
        self._on_event_callback = on_event
        self._on_scene_callback = on_scene
        self._use_action = use_action
        self._lock = threading.Lock()
        self._goal_handles: dict[str, object] = {}
        self._cancel_pending: set[str] = set()
        self._action_client = None
        self._command_publisher = None
        self._cancel_publisher = None
        if use_action:
            from mujoco_demo_interfaces.action import PickPlace

            self._action_type = PickPlace
            self._action_client = ActionClient(self, PickPlace, ACTION_NAME)
        else:
            self._command_publisher = self.create_publisher(String, COMMAND_TOPIC, 10)
            self._cancel_publisher = self.create_publisher(String, CANCEL_TOPIC, 10)
            self.create_subscription(String, EVENT_TOPIC, self._on_event, 10)
        self.create_subscription(
            String, SCENE_TOPIC, self._on_scene,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE),
        )

    def has_bridge(self) -> bool:
        if self._use_action:
            return self._action_client.server_is_ready()
        return self._command_publisher.get_subscription_count() > 0

    def send_task(self, request_id: str, task: PickPlaceTask,
                  scene_revision: int = 0, *, continue_sequence: bool = False) -> None:
        if not self._use_action:
            command = TaskCommand(
                request_id=request_id, task=task, continue_sequence=continue_sequence,
            )
            self._command_publisher.publish(String(data=command.model_dump_json()))
            return
        goal = self._action_type.Goal()
        goal.request_id = request_id
        goal.object_name = task.object_name
        goal.target_name = task.target_name
        goal.expected_scene_revision = scene_revision
        goal.continue_sequence = continue_sequence
        future = self._action_client.send_goal_async(
            goal, feedback_callback=lambda feedback: self._on_action_feedback(request_id, feedback)
        )
        future.add_done_callback(lambda done: self._on_goal_response(request_id, done))

    def cancel_task(self, request_id: str) -> None:
        if not self._use_action:
            cancel = TaskCancel(request_id=request_id)
            self._cancel_publisher.publish(String(data=cancel.model_dump_json()))
            return
        with self._lock:
            handle = self._goal_handles.get(request_id)
            if handle is None:
                self._cancel_pending.add(request_id)
        if handle is not None:
            handle.cancel_goal_async()

    def _on_goal_response(self, request_id: str, future) -> None:
        try:
            handle = future.result()
            if not handle.accepted:
                with self._lock:
                    self._cancel_pending.discard(request_id)
                self._on_event_callback(TaskEvent(
                    request_id=request_id, status="rejected",
                    message="Action Goal 被拒绝：机器人忙碌、场景版本过期或请求 ID 重复",
                ))
                return
            with self._lock:
                self._goal_handles[request_id] = handle
                cancel_pending = request_id in self._cancel_pending
                self._cancel_pending.discard(request_id)
            self._on_event_callback(TaskEvent(request_id=request_id, status="accepted"))
            if cancel_pending:
                handle.cancel_goal_async()
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda done: self._on_action_result(request_id, done)
            )
        except Exception as exc:
            self._on_event_callback(TaskEvent(request_id=request_id, status="failed", message=str(exc)))

    def _on_action_feedback(self, request_id: str, feedback) -> None:
        self._on_event_callback(TaskEvent(
            request_id=request_id, status="phase", phase=feedback.feedback.phase,
        ))

    def _on_action_result(self, request_id: str, future) -> None:
        with self._lock:
            self._goal_handles.pop(request_id, None)
        try:
            outcome = future.result()
            result = outcome.result
            if outcome.status == GoalStatus.STATUS_SUCCEEDED and result.success:
                status = "completed"
            elif outcome.status == GoalStatus.STATUS_CANCELED:
                status = "cancelled"
            else:
                status = "failed"
            self._on_event_callback(TaskEvent(
                request_id=request_id, status=status, message=result.message or None,
                result={
                    "success": result.success,
                    "error_code": result.error_code,
                    "cube_final_m": list(result.object_position_m),
                    "scene_revision": result.scene_revision,
                },
            ))
        except Exception as exc:
            self._on_event_callback(TaskEvent(request_id=request_id, status="failed", message=str(exc)))

    def _on_event(self, message: String) -> None:
        self._on_event_callback(TaskEvent.model_validate_json(message.data))

    def _on_scene(self, message: String) -> None:
        self._on_scene_callback(json.loads(message.data))
