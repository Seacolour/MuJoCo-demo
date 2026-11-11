"""Deterministic, physics-verified pick-and-place baseline for the xArm7."""

from dataclasses import dataclass, field
from pathlib import Path
import time
from typing import Callable, Protocol

import mujoco
import numpy as np

from mujoco_demo.shared.contracts import PickPlaceTask
from mujoco_demo.robot_runtime.kinematics import ARM_JOINT_NAMES, site_quaternion, solve_tcp_pose
from mujoco_demo.robot_runtime.scene_catalog import SceneCatalog


SCENE_PATH = Path(__file__).resolve().parents[3] / "model" / "ufactory_xarm7" / "pick_place_scene.xml"
MULTI_SCENE_PATH = SCENE_PATH.with_name("multi_object_scene.xml")
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 255.0


class Viewer(Protocol):
    def sync(self) -> None: ...

    def is_running(self) -> bool: ...


class TaskCancelled(RuntimeError):
    """The scheduler requested that this simulated task stop."""


@dataclass(frozen=True)
class HomePose:
    """Robot joint pose and open gripper from the scene's startup keyframe."""

    joint_positions_rad: tuple[float, ...]
    tcp_position_m: tuple[float, float, float]
    gripper_command: float


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
    max_other_object_displacement_m: float = 0.0
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
        scene_path: Path = SCENE_PATH,
    ) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(scene_path))
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
        self.home_pose = HomePose(
            joint_positions_rad=tuple(float(x) for x in self.model.key_qpos[0, self.qpos_ids]),
            tcp_position_m=tuple(float(x) for x in self.data.site_xpos[self.tcp_id]),
            gripper_command=float(self.data.ctrl[self.gripper_id]),
        )
        self.orientation = site_quaternion(self.data, self.tcp_id)
        self.catalog = SceneCatalog(self.model)
        pad_z = np.mean([
            self.data.geom(name).xpos[2]
            for name in ("left_finger_pad_1", "right_finger_pad_1")
        ])
        self.pad_z_offset = float(pad_z - self.data.site_xpos[self.tcp_id][2])

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
                initial_angles=np.asarray(self.home_pose.joint_positions_rad),
            )
        if solution.position_error > 0.01 or solution.orientation_error > 0.1:
            raise RuntimeError(
                f"IK failed: position error={solution.position_error:.3f} m, "
                f"orientation error={solution.orientation_error:.3f} rad"
            )
        self.move_joints(solution.joint_positions, seconds=seconds)
        self.step_for(0.35)
        return float(np.linalg.norm(self.data.site_xpos[self.tcp_id] - position))

    def move_joints(self, target: np.ndarray, *, seconds: float) -> None:
        start = self.data.qpos[self.qpos_ids].copy()
        steps = max(1, round(seconds / self.model.opt.timestep))
        for step in range(steps):
            self._check_cancel()
            if self.viewer is not None and not self.viewer.is_running():
                raise RuntimeError("viewer closed before task completed")
            progress = (step + 1) / steps
            blend = progress * progress * (3 - 2 * progress)
            self.data.ctrl[:7] = start + blend * (target - start)
            mujoco.mj_step(self.model, self.data)
            if self.viewer is not None and step % 5 == 0:
                self.viewer.sync()
                time.sleep(5 * self.model.opt.timestep)

    def move_home(self) -> float:
        """Return to the scene's startup joint pose after the cube is released."""
        self.data.ctrl[self.gripper_id] = self.home_pose.gripper_command
        target = np.asarray(self.home_pose.joint_positions_rad)
        self.move_joints(target, seconds=2.4)
        self.step_for(0.35)
        joint_error = float(np.max(np.abs(self.data.qpos[self.qpos_ids] - target)))
        if joint_error > 0.05:
            raise RuntimeError(f"home pose missed by {joint_error:.3f} rad")
        return joint_error

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

    def run(self, task: PickPlaceTask, *, return_home: bool = True) -> PickPlaceResult:
        object_info, zone_info = self.catalog.validate(self.data, task)
        # A simulated task is committed only after physical verification. Rewind
        # its partial motion on cancel/failure so later tasks start from a stable state.
        state_spec = mujoco.mjtState.mjSTATE_INTEGRATION
        initial_state = np.empty(mujoco.mj_stateSize(self.model, state_spec))
        mujoco.mj_getState(self.model, self.data, initial_state, state_spec)
        cube = self.data.body(task.object_name)
        target = self.data.site(task.target_name)
        cube_geom_id = object_info.geom_id
        start = cube.xpos.copy()
        target_position = target.xpos.copy()
        other_starts = {
            name: self.data.xpos[info.body_id].copy()
            for name, info in self.catalog.objects.items()
            if name != task.object_name
        }
        phases: list[PhaseReport] = []
        grasp_verified = False
        placed_verified = False
        cancelled = False
        failure_reason: str | None = None
        max_other_displacement = 0.0
        completed = False

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
            grasp_z = float(cube.xpos[2] - self.pad_z_offset)
            place_z = float(
                target_position[2] + object_info.half_size_m[2] - self.pad_z_offset
            )
            above_z = max(float(cube.xpos[2]), float(target_position[2])) + 0.12

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

            error = self.move_tcp(np.array([*place_xy, place_z]), seconds=1.2)
            record("lower_to_target", error)
            self.data.ctrl[self.gripper_id] = GRIPPER_OPEN
            self.step_for(0.8)
            record("release")
            if self.pad_contacts(cube_geom_id) > 0:
                raise RuntimeError("cube still contacts the gripper after release")

            error = self.move_tcp(np.array([*place_xy, above_z]))
            self.step_for(0.5)
            record("retreat", error)
            if return_home:
                self.move_home()
                record("return_home")
            horizontal_error = np.abs(cube.xpos[:2] - place_xy)
            allowed_error = zone_info.half_size_m[:2] - object_info.half_size_m[:2] + 0.01
            expected_height = target_position[2] + object_info.half_size_m[2]
            placed_verified = bool(
                np.all(horizontal_error <= allowed_error)
                and abs(cube.xpos[2] - expected_height) < 0.03
            )
            if not placed_verified:
                raise RuntimeError(
                    f"cube missed target: horizontal error={horizontal_error.tolist()} m, "
                    f"height={cube.xpos[2]:.3f} m"
                )
            max_other_displacement = max(
                (
                    float(np.linalg.norm(self.data.xpos[self.catalog.objects[name].body_id] - position))
                    for name, position in other_starts.items()
                ),
                default=0.0,
            )
            if max_other_displacement > 0.015:
                placed_verified = False
                raise RuntimeError(
                    f"another cube moved by {max_other_displacement:.3f} m"
                )
            completed = True
        except TaskCancelled as exc:
            cancelled = True
            failure_reason = str(exc)
        except RuntimeError as exc:
            failure_reason = str(exc)
        finally:
            if not completed:
                mujoco.mj_setState(self.model, self.data, initial_state, state_spec)
                mujoco.mj_forward(self.model, self.data)
                if self.viewer is not None and self.viewer.is_running():
                    self.viewer.sync()

        return PickPlaceResult(
            success=completed,
            cancelled=cancelled,
            grasp_verified=grasp_verified,
            placed_verified=placed_verified,
            cube_start_m=tuple(float(x) for x in start),
            cube_final_m=tuple(float(x) for x in cube.xpos),
            target_m=tuple(float(x) for x in target_position),
            max_other_object_displacement_m=max_other_displacement,
            phases=tuple(phases),
            failure_reason=failure_reason,
        )
