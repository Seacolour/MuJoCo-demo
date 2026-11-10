"""Compatibility import; new code should use mujoco_demo.platform.deepseek_planner."""

from urllib import request

from mujoco_demo.platform.deepseek_planner import DeepSeekPlanner

__all__ = ["DeepSeekPlanner", "request"]
