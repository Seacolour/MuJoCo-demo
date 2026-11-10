"""A compound command must preserve every move and validate the whole plan."""

from io import BytesIO
import json

import pytest

from mujoco_demo.contracts import PickPlaceTask
from mujoco_demo.deepseek_planner import DeepSeekPlanner
from mujoco_demo.task_plan import TaskPlan, validate_plan


SCENE = {
    "objects": {"red_cube": {"label": "红色方块"}, "green_cube": {"label": "绿色方块"}},
    "zones": {
        "red_zone": {"occupied_by": None},
        "green_zone": {"occupied_by": None},
        "blue_zone": {"occupied_by": None},
    },
}


def model_response(payload: dict) -> BytesIO:
    body = {"choices": [{"message": {"content": json.dumps(payload)}}]}
    return BytesIO(json.dumps(body).encode("utf-8"))


def test_two_requested_moves_remain_two_tool_calls(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        "mujoco_demo.deepseek_planner.request.urlopen",
        lambda *_args, **_kwargs: model_response({"steps": [
            {"object_name": "green_cube", "target_name": "red_zone"},
            {"object_name": "red_cube", "target_name": "green_zone"},
        ]}),
    )
    plan = DeepSeekPlanner(api_key="test-key").parse_plan(
        "先把绿色方块放到红色区域，然后把红色方块放到绿色区域", SCENE
    )
    assert len(plan.steps) == 2
    validate_plan(SCENE, plan)


def test_compound_instruction_rejects_a_truncated_one_step_plan(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        "mujoco_demo.deepseek_planner.request.urlopen",
        lambda *_args, **_kwargs: model_response({"steps": [
            {"object_name": "green_cube", "target_name": "red_zone"},
        ]}),
    )
    with pytest.raises(ValueError, match="未完整解析"):
        DeepSeekPlanner(api_key="test-key").parse_plan(
            "先把绿色方块放到红色区域，然后把红色方块放到绿色区域", SCENE
        )


def test_plan_rejects_a_later_occupied_target_before_any_motion() -> None:
    plan = TaskPlan(steps=(
        PickPlaceTask(object_name="red_cube", target_name="blue_zone"),
        PickPlaceTask(object_name="green_cube", target_name="blue_zone"),
    ))
    with pytest.raises(ValueError, match="第 2 步.*已被 红色方块 占用"):
        validate_plan(SCENE, plan)
