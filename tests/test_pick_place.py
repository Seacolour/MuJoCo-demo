"""End-to-end physics checks for the xArm7 tabletop task."""

import mujoco
import pytest

from mujoco_demo.contracts import PickPlaceTask
from mujoco_demo.pick_place import PickPlaceSimulation


TASK = PickPlaceTask(object_name="red_cube", target_name="target_zone")


def test_cube_is_grasped_and_released_in_target() -> None:
    result = PickPlaceSimulation().run(TASK)

    assert result.success, result.failure_reason
    assert result.grasp_verified and result.placed_verified
    assert next(phase for phase in result.phases if phase.name == "lift").cube_position_m[2] > 0.4
    assert next(phase for phase in result.phases if phase.name == "lift").pad_contacts >= 2
    assert next(phase for phase in result.phases if phase.name == "release").pad_contacts == 0


def test_open_gripper_cannot_claim_a_successful_grasp() -> None:
    result = PickPlaceSimulation(close_command=0).run(TASK)

    assert not result.success
    assert not result.grasp_verified
    assert result.failure_reason == "cube did not rise with the gripper"


def test_cancelled_task_stops_before_claiming_success() -> None:
    checks = 0

    def should_cancel() -> bool:
        nonlocal checks
        checks += 1
        return checks >= 600

    result = PickPlaceSimulation(should_cancel=should_cancel).run(TASK)
    assert result.cancelled
    assert not result.success
    assert result.failure_reason == "task cancelled"


@pytest.mark.parametrize("x,y", [(0.43, 0.0), (0.47, 0.0), (0.45, 0.01), (0.45, -0.01)])
def test_small_object_position_changes_are_observed(x: float, y: float) -> None:
    simulation = PickPlaceSimulation()
    cube_qpos = int(simulation.model.jnt_qposadr[simulation.model.joint("red_cube_free").id])
    simulation.data.qpos[cube_qpos : cube_qpos + 2] = [x, y]
    mujoco.mj_forward(simulation.model, simulation.data)

    result = simulation.run(TASK)
    assert result.success, result.failure_reason
