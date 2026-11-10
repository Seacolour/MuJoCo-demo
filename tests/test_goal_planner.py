"""Goal planning must use the free zone when two placed objects trade destinations."""

import pytest

from mujoco_demo.platform.goal_planner import expand_goal_plan
from mujoco_demo.shared.contracts import PickPlaceTask
from mujoco_demo.shared.task_plan import TaskPlan, validate_plan


def swapped_scene(*, blue_occupied: str | None = None) -> dict:
    return {
        "objects": {"red_cube": {"label": "红色方块"}, "green_cube": {"label": "绿色方块"}},
        "zones": {
            "red_zone": {"occupied_by": "green_cube"},
            "green_zone": {"occupied_by": "red_cube"},
            "blue_zone": {"occupied_by": blue_occupied},
        },
    }


def test_swap_uses_blue_zone_as_a_temporary_buffer() -> None:
    goals = TaskPlan(steps=(
        PickPlaceTask(object_name="red_cube", target_name="red_zone"),
        PickPlaceTask(object_name="green_cube", target_name="green_zone"),
    ))
    result = expand_goal_plan(swapped_scene(), goals)
    assert [(step.object_name, step.target_name) for step in result.steps] == [
        ("red_cube", "blue_zone"),
        ("green_cube", "green_zone"),
        ("red_cube", "red_zone"),
    ]
    validate_plan(swapped_scene(), result)


def test_occupied_target_without_a_destination_for_its_occupant_is_rejected() -> None:
    request = TaskPlan(steps=(PickPlaceTask(object_name="red_cube", target_name="red_zone"),))
    with pytest.raises(ValueError, match="无法在四步内完成目标"):
        expand_goal_plan(swapped_scene(), request)


def test_two_objects_cannot_finish_in_the_same_zone() -> None:
    request = TaskPlan(steps=(
        PickPlaceTask(object_name="red_cube", target_name="blue_zone"),
        PickPlaceTask(object_name="green_cube", target_name="blue_zone"),
    ))
    with pytest.raises(ValueError, match="不能同时占用"):
        expand_goal_plan(swapped_scene(), request)


def test_already_satisfied_goals_need_no_robot_moves() -> None:
    scene = swapped_scene()
    scene["zones"]["red_zone"]["occupied_by"] = "red_cube"
    scene["zones"]["green_zone"]["occupied_by"] = "green_cube"
    request = TaskPlan(steps=(
        PickPlaceTask(object_name="red_cube", target_name="red_zone"),
        PickPlaceTask(object_name="green_cube", target_name="green_zone"),
    ))
    assert expand_goal_plan(scene, request) is None
