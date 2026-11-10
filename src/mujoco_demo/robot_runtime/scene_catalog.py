"""Named objects and zones exposed to the task planner and verifier."""

from dataclasses import dataclass

import mujoco
import numpy as np

from mujoco_demo.shared.contracts import PickPlaceTask


OBJECT_LABELS = {"red_cube": "红色方块", "green_cube": "绿色方块"}
ZONE_LABELS = {
    "target_zone": "绿色目标区域",
    "red_zone": "红色区域",
    "green_zone": "绿色区域",
    "blue_zone": "蓝色区域",
}


@dataclass(frozen=True)
class ObjectInfo:
    name: str
    label: str
    body_id: int
    geom_id: int
    half_size_m: np.ndarray


@dataclass(frozen=True)
class ZoneInfo:
    name: str
    label: str
    site_id: int
    half_size_m: np.ndarray


class SceneCatalog:
    def __init__(self, model: mujoco.MjModel) -> None:
        self.objects = {
            name: ObjectInfo(
                name=name,
                label=label,
                body_id=model.body(name).id,
                geom_id=model.geom(f"{name}_geom").id,
                half_size_m=model.geom(f"{name}_geom").size.copy(),
            )
            for name, label in OBJECT_LABELS.items()
            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) >= 0
        }
        self.zones = {
            name: ZoneInfo(
                name=name,
                label=label,
                site_id=model.site(name).id,
                half_size_m=model.site(name).size.copy(),
            )
            for name, label in ZONE_LABELS.items()
            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name) >= 0
        }

    def zone_occupant(self, data: mujoco.MjData, zone: ZoneInfo) -> str | None:
        center = data.site_xpos[zone.site_id]
        for obj in self.objects.values():
            position = data.xpos[obj.body_id]
            if (np.all(np.abs(position[:2] - center[:2]) <= zone.half_size_m[:2])
                    and abs(position[2] - (center[2] + obj.half_size_m[2])) < 0.04):
                return obj.name
        return None

    def validate(self, data: mujoco.MjData, task: PickPlaceTask) -> tuple[ObjectInfo, ZoneInfo]:
        if task.object_name not in self.objects:
            raise ValueError(f"unknown object: {task.object_name}")
        if task.target_name not in self.zones:
            raise ValueError(f"unknown target zone: {task.target_name}")
        zone = self.zones[task.target_name]
        occupant = self.zone_occupant(data, zone)
        if occupant is not None:
            raise ValueError(
                "object already in target" if occupant == task.object_name
                else f"target occupied by {occupant}"
            )
        return self.objects[task.object_name], zone

    def snapshot(self, data: mujoco.MjData) -> dict[str, object]:
        return {
            "objects": {
                obj.name: {"label": obj.label, "position_m": data.xpos[obj.body_id].tolist()}
                for obj in self.objects.values()
            },
            "zones": {
                zone.name: {
                    "label": zone.label,
                    "position_m": data.site_xpos[zone.site_id].tolist(),
                    "occupied_by": self.zone_occupant(data, zone),
                }
                for zone in self.zones.values()
            },
        }
