"""Compatibility import; new code should use mujoco_demo.shared.task_plan."""

from mujoco_demo.shared.task_plan import TaskPlan, validate_plan

__all__ = ["TaskPlan", "validate_plan"]
