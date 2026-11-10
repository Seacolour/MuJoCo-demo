"""Deterministic, physics-verified pick-and-place baseline for the xArm7."""

from dataclasses import dataclass, field
from pathlib import Path
import time
from typing import Callable, Protocol

import mujoco
import numpy as np

from mujoco_demo.contracts import PickPlaceTask
from mujoco_demo.kinematics import ARM_JOINT_NAMES, site_quaternion, solve_tcp_pose


SCENE_PATH = Path(__file__).resolve().parents[2] / "model" / "ufactory_xarm7" / "pick_place_scene.xml"
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 255.0


class Viewer(Protocol):
    def sync(self) -> None: ...

    def is_running(self) -> bool: ...


class TaskCancelled(RuntimeError):
    """The scheduler requested that this simulated task stop."""


@dataclass(frozen=True)
class PhaseReport:
    name: str
    tcp_error_m: float
    tcp_position_m: tuple[float, float, float]
    cube_position_m: tuple[float, float, float]
    joint_positions_rad: tuple[float, ...]
    pad_contacts: int


@dataclass(frozen=True)
class PickPlaceResult:
    success: bool
    cancelled: bool
    grasp_verified: bool
    placed_verified: bool
    cube_start_m: tuple[float, float, float]
    cube_final_m: tuple[float, float, float]
    target_m: tuple[float, float, float]
    phases: tuple[PhaseReport, ...] = field(default_factory=tuple)
    failure_reason: str | None = None


class PickPlaceSimulation:
    def __init__(
        self,
        *,
        viewer: Viewer | None = None,
        close_command: float = GRIPPER_CLOSED,
        on_phase: Callable[[PhaseReport], None] | None = None,
        should_cancel: Callable[[], bool] | None = None,
    ) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        self.viewer = viewer
        self.close_command = close_command
        self.on_phase = on_phase
        self.should_cancel = should_cancel
        self.joint_ids = np.array([self.model.joint(name).id for name in ARM_JOINT_NAMES])
        self.qpos_ids = self.model.jnt_qposadr[self.joint_ids]
        self.tcp_id = self.model.site("link_tcp").id
        self.gripper_id = self.model.actuator("gripper").id
        self.orientation = site_quaternion(self.data, self.tcp_id)

    def step_for(self, seconds: float) -> None:
        steps = round(seconds / self.model.opt.timestep)
        for step in range(steps):
            self._check_cancel()
            if self.viewer is not None and not self.viewer.is_running():
                raise RuntimeError("viewer closed before task completed")
            mujoco.mj_step(self.model, self.data)
            if self.viewer is not None and step % 5 == 0:
                self.viewer.sync()
                time.sleep(5 * self.model.opt.timestep)

    def move_tcp(self, position: np.ndarray, *, seconds: float = 1.5) -> float:
        self._check_cancel()
        solution = solve_tcp_pose(self.model, self.data, position, self.orientation)
        if solution.position_error > 0.01 or solution.orientation_error > 0.1:
            solution = solve_tcp_pose(
                self.model,
                self.data,
                position,
                self.orientation,
                initial_angles=self.model.key_qpos[0, :7],
            )
        if solution.position_error > 0.01 or solution.orientation_error > 0.1:
            raise RuntimeError(
                f"IK failed: position error={solution.position_error:.3f} m, "
                f"orientation error={solution.orientation_error:.3f} rad"
            )
        start = self.data.qpos[self.qpos_ids].copy()
        steps = max(1, round(seconds / self.model.opt.timestep))
        for step in range(steps):
            self._check_cancel()
            progress = (step + 1) / steps
            blend = progress * progress * (3 - 2 * progress)
            self.data.ctrl[:7] = start + blend * (solution.joint_positions - start)
            mujoco.mj_step(self.model, self.data)
            if self.viewer is not None and step % 5 == 0:
                self.viewer.sync()
                time.sleep(5 * self.model.opt.timestep)
        self.step_for(0.35)
        return float(np.linalg.norm(self.data.site_xpos[self.tcp_id] - position))

    def _check_cancel(self) -> None:
        if self.should_cancel is not None and self.should_cancel():
            self.data.ctrl[:7] = self.data.qpos[self.qpos_ids]
            raise TaskCancelled("task cancelled")

    def pad_contacts(self, cube_geom_id: int) -> int:
        pad_names = {
            "left_finger_pad_1", "left_finger_pad_2", "right_finger_pad_1", "right_finger_pad_2"
        }
        contacts: set[str] = set()
        for contact in self.data.contact[: self.data.ncon]:
            if cube_geom_id not in (contact.geom1, contact.geom2):
                continue
            other = contact.geom2 if contact.geom1 == cube_geom_id else contact.geom1
            name = self.model.geom(other).name
            if name in pad_names:
                contacts.add(name)
        return len(contacts)

    def run(self, task: PickPlaceTask) -> PickPlaceResult:
        cube = self.data.body(task.object_name)
        target = self.data.site(task.target_name)
        cube_geom_id = self.model.geom("red_cube_geom").id
        start = cube.xpos.copy()
        target_position = target.xpos.copy()
        phases: list[PhaseReport] = []
        grasp_verified = False
        placed_verified = False
        cancelled = False
        failure_reason: str | None = None

        def record(name: str, tcp_error: float = 0.0) -> None:
            report = PhaseReport(
                name=name,
                tcp_error_m=tcp_error,
                tcp_position_m=tuple(float(x) for x in self.data.site_xpos[self.tcp_id]),
                cube_position_m=tuple(float(x) for x in cube.xpos),
                joint_positions_rad=tuple(float(x) for x in self.data.qpos[self.qpos_ids]),
                pad_contacts=self.pad_contacts(cube_geom_id),
            )
            phases.append(report)
            if self.on_phase is not None:
                self.on_phase(report)

        try:
            self.data.ctrl[self.gripper_id] = GRIPPER_OPEN
            self.step_for(0.5)
            record("settled")

            pick_xy = cube.xpos[:2].copy()
            place_xy = target_position[:2].copy()
            grasp_z = float(cube.xpos[2] - 0.038)
            above_z = 0.45

            for name, point, seconds in (
                ("approach", np.array([*pick_xy, above_z]), 1.5),
                ("lower", np.array([*pick_xy, grasp_z]), 1.2),
            ):
                error = self.move_tcp(point, seconds=seconds)
                record(name, error)
                if error > 0.025:
                    raise RuntimeError(f"{name}: TCP missed target by {error:.3f} m")

            self.data.ctrl[self.gripper_id] = self.close_command
            self.step_for(1.0)
            record("close_gripper")

            error = self.move_tcp(np.array([*pick_xy, above_z]))
            record("lift", error)
            grasp_verified = bool(cube.xpos[2] > start[2] + 0.07)
            if not grasp_verified:
                raise RuntimeError("cube did not rise with the gripper")

            error = self.move_tcp(np.array([*place_xy, above_z]), seconds=2.0)
            record("transfer", error)
            if error > 0.025:
                raise RuntimeError(f"transfer: TCP missed target by {error:.3f} m")

            error = self.move_tcp(np.array([*place_xy, grasp_z]), seconds=1.2)
            record("lower_to_target", error)
            self.data.ctrl[self.gripper_id] = GRIPPER_OPEN
            self.step_for(0.8)
            record("release")

            error = self.move_tcp(np.array([*place_xy, above_z]))
            self.step_for(0.5)
            record("retreat", error)
            horizontal_error = float(np.linalg.norm(cube.xpos[:2] - place_xy))
            placed_verified = bool(horizontal_error < 0.055 and 0.325 < cube.xpos[2] < 0.36)
            if not placed_verified:
                raise RuntimeError(
                    f"cube missed target: horizontal error={horizontal_error:.3f} m, "
                    f"height={cube.xpos[2]:.3f} m"
                )
        except TaskCancelled as exc:
            cancelled = True
            failure_reason = str(exc)
        except RuntimeError as exc:
            failure_reason = str(exc)

        return PickPlaceResult(
            success=grasp_verified and placed_verified,
            cancelled=cancelled,
            grasp_verified=grasp_verified,
            placed_verified=placed_verified,
            cube_start_m=tuple(float(x) for x in start),
            cube_final_m=tuple(float(x) for x in cube.xpos),
            target_m=tuple(float(x) for x in target_position),
            phases=tuple(phases),
            failure_reason=failure_reason,
        )
