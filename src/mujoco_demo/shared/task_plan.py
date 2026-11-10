"""Ordered robot tool calls and deterministic scene checks."""

from pydantic import BaseModel, ConfigDict, Field

from mujoco_demo.shared.contracts import PickPlaceTask


class TaskPlan(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    steps: tuple[PickPlaceTask, ...] = Field(min_length=1, max_length=4)


def validate_plan(scene: dict[str, object], plan: TaskPlan) -> None:
    """Reject impossible goal occupancy before moving the first object."""
    objects = scene["objects"]
    zones = scene["zones"]
    occupancy = {name: value.get("occupied_by") for name, value in zones.items()}

    for index, step in enumerate(plan.steps, start=1):
        if step.object_name not in objects:
            raise ValueError(f"第 {index} 步的方块不存在：{step.object_name}")
        if step.target_name not in zones:
            raise ValueError(f"第 {index} 步的目标区域不存在：{step.target_name}")
        occupant = occupancy[step.target_name]
        if occupant == step.object_name:
            raise ValueError(f"第 {index} 步的方块已在目标区域")
        if occupant is not None:
            label = objects.get(occupant, {}).get("label", occupant)
            raise ValueError(f"第 {index} 步的目标区域已被 {label} 占用")
        for zone_name, current_occupant in occupancy.items():
            if current_occupant == step.object_name:
                occupancy[zone_name] = None
        occupancy[step.target_name] = step.object_name
