"""Compatibility import; new code should use mujoco_demo.robot_runtime.scene_catalog."""

from mujoco_demo.robot_runtime.scene_catalog import (
    OBJECT_LABELS, ZONE_LABELS, ObjectInfo, SceneCatalog, ZoneInfo,
)

__all__ = ["OBJECT_LABELS", "ZONE_LABELS", "ObjectInfo", "SceneCatalog", "ZoneInfo"]
