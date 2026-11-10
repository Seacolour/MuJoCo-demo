"""Small damped-least-squares IK solver for the simulated xArm7 end effector."""

from dataclasses import dataclass

import mujoco
import numpy as np


ARM_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 8))
TCP_SITE_NAME = "link_tcp"


@dataclass(frozen=True)
class IkResult:
    joint_positions: np.ndarray
    position_error: float
    orientation_error: float
    iterations: int


def site_quaternion(data: mujoco.MjData, site_id: int) -> np.ndarray:
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, data.site_xmat[site_id])
    return quat


def solve_tcp_pose(
    model: mujoco.MjModel,
    state: mujoco.MjData,
    target_position: np.ndarray,
    target_quaternion: np.ndarray,
    *,
    initial_angles: np.ndarray | None = None,
    max_iterations: int = 250,
) -> IkResult:
    """Solve one pose from the current robot state without changing that state."""
    site_id = model.site(TCP_SITE_NAME).id
    joint_ids = np.array([model.joint(name).id for name in ARM_JOINT_NAMES])
    qpos_ids = model.jnt_qposadr[joint_ids]
    dof_ids = model.jnt_dofadr[joint_ids]
    limits = model.jnt_range[joint_ids]

    probe = mujoco.MjData(model)
    probe.qpos[:] = state.qpos
    if initial_angles is not None:
        probe.qpos[qpos_ids] = initial_angles

    position_error = float("inf")
    orientation_error = float("inf")
    for iteration in range(1, max_iterations + 1):
        mujoco.mj_forward(model, probe)
        delta_position = target_position - probe.site_xpos[site_id]
        delta_orientation = np.empty(3)
        mujoco.mju_subQuat(
            delta_orientation, target_quaternion, site_quaternion(probe, site_id)
        )
        position_error = float(np.linalg.norm(delta_position))
        orientation_error = float(np.linalg.norm(delta_orientation))
        if position_error < 0.003 and orientation_error < 0.03:
            break

        jac_position = np.zeros((3, model.nv))
        jac_rotation = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, probe, jac_position, jac_rotation, site_id)
        jacobian = np.vstack((jac_position[:, dof_ids], jac_rotation[:, dof_ids]))
        error = np.concatenate((delta_position, delta_orientation))
        damping = 0.06
        step = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + damping**2 * np.eye(6), error
        )
        probe.qpos[qpos_ids] = np.clip(
            probe.qpos[qpos_ids] + np.clip(step, -0.12, 0.12), limits[:, 0], limits[:, 1]
        )

    return IkResult(probe.qpos[qpos_ids].copy(), position_error, orientation_error, iteration)
