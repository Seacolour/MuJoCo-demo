"""Validated payloads for the first ROS 2 topic bridge."""

from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field

from mujoco_demo.shared.contracts import PickPlaceTask


COMMAND_TOPIC = "/mujoco_demo/task_command"
ACTION_NAME = "/mujoco_demo/pick_place"
EVENT_TOPIC = "/mujoco_demo/task_event"
CANCEL_TOPIC = "/mujoco_demo/task_cancel"
JOINT_STATE_TOPIC = "/mujoco_demo/joint_states"
SCENE_TOPIC = "/mujoco_demo/scene"


class TaskCommand(BaseModel):
    model_config = ConfigDict(extra="forbid")

    request_id: str = Field(min_length=1)
    task: PickPlaceTask
    continue_sequence: bool = False


class TaskCancel(BaseModel):
    model_config = ConfigDict(extra="forbid")

    request_id: str = Field(min_length=1)


class TaskEvent(BaseModel):
    model_config = ConfigDict(extra="forbid")

    request_id: str
    status: Literal[
        "accepted", "queued", "phase", "cancel_requested", "cancelled",
        "completed", "failed", "rejected"
    ]
    phase: str | None = None
    message: str | None = None
    result: dict[str, Any] | None = None
