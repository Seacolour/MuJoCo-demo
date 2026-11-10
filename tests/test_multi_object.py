"""Multi-object scene acceptance checks against actual MuJoCo physics."""

import pytest

from mujoco_demo.contracts import PickPlaceTask
from mujoco_demo.pick_place import MULTI_SCENE_PATH, PickPlaceSimulation


@pytest.mark.parametrize("object_name", ["red_cube", "green_cube"])
@pytest.mark.parametrize("target_name", ["red_zone", "green_zone", "blue_zone"])
def test_each_cube_can_reach_each_zone(object_name: str, target_name: str) -> None:
    simulation = PickPlaceSimulation(scene_path=MULTI_SCENE_PATH)
    result = simulation.run(PickPlaceTask(object_name=object_name, target_name=target_name))

    assert result.success, result.failure_reason
    assert result.grasp_verified and result.placed_verified
    assert result.max_other_object_displacement_m < 0.015
    assert simulation.catalog.zone_occupant(
        simulation.data, simulation.catalog.zones[target_name]
    ) == object_name


def test_two_tasks_share_scene_state_and_occupied_zone_is_rejected() -> None:
    simulation = PickPlaceSimulation(scene_path=MULTI_SCENE_PATH)

    first = simulation.run(PickPlaceTask(object_name="red_cube", target_name="blue_zone"))
    second = simulation.run(PickPlaceTask(object_name="green_cube", target_name="red_zone"))

    assert first.success and second.success
    assert simulation.catalog.zone_occupant(
        simulation.data, simulation.catalog.zones["blue_zone"]
    ) == "red_cube"
    assert simulation.catalog.zone_occupant(
        simulation.data, simulation.catalog.zones["red_zone"]
    ) == "green_cube"
    with pytest.raises(ValueError, match="target occupied by red_cube"):
        simulation.run(PickPlaceTask(object_name="green_cube", target_name="blue_zone"))


def test_unknown_object_is_rejected_before_motion() -> None:
    simulation = PickPlaceSimulation(scene_path=MULTI_SCENE_PATH)
    start_time = simulation.data.time
    with pytest.raises(ValueError, match="unknown object"):
        simulation.run(PickPlaceTask(object_name="blue_cube", target_name="red_zone"))
    assert simulation.data.time == start_time
