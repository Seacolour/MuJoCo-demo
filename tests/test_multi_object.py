"""Multi-object scene acceptance checks against actual MuJoCo physics."""

from unittest.mock import patch

import numpy as np
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


def test_repeated_cancellation_restores_a_retryable_scene() -> None:
    simulation = PickPlaceSimulation(scene_path=MULTI_SCENE_PATH)
    assert simulation.run(PickPlaceTask(object_name="red_cube", target_name="blue_zone")).success
    green_task = PickPlaceTask(object_name="green_cube", target_name="red_zone")
    qpos_before = simulation.data.qpos.copy()
    qvel_before = simulation.data.qvel.copy()
    ctrl_before = simulation.data.ctrl.copy()
    time_before = simulation.data.time

    for phase_name in ("settled", "close_gripper"):
        cancel_requested = False

        def on_phase(phase) -> None:
            nonlocal cancel_requested
            if phase.name == phase_name:
                cancel_requested = True

        simulation.on_phase = on_phase
        simulation.should_cancel = lambda: cancel_requested
        result = simulation.run(green_task)
        assert result.cancelled and not result.success
        np.testing.assert_allclose(simulation.data.qpos, qpos_before)
        np.testing.assert_allclose(simulation.data.qvel, qvel_before)
        np.testing.assert_allclose(simulation.data.ctrl, ctrl_before)
        assert simulation.data.time == time_before

    simulation.on_phase = None
    simulation.should_cancel = None
    retry = simulation.run(green_task)
    assert retry.success, retry.failure_reason


def test_failure_after_grasp_restores_scene_for_the_next_task() -> None:
    simulation = PickPlaceSimulation(scene_path=MULTI_SCENE_PATH)
    task = PickPlaceTask(object_name="green_cube", target_name="red_zone")
    qpos_before = simulation.data.qpos.copy()
    after_grasp = False
    original_move = simulation.move_tcp

    def on_phase(phase) -> None:
        nonlocal after_grasp
        if phase.name == "close_gripper":
            after_grasp = True

    def fail_lift(*args, **kwargs) -> float:
        if after_grasp:
            raise RuntimeError("forced IK failure")
        return original_move(*args, **kwargs)

    simulation.on_phase = on_phase
    with patch.object(simulation, "move_tcp", side_effect=fail_lift):
        result = simulation.run(task)
    assert not result.success
    assert result.failure_reason == "forced IK failure"
    np.testing.assert_allclose(simulation.data.qpos, qpos_before)

    simulation.on_phase = None
    retry = simulation.run(task)
    assert retry.success, retry.failure_reason
