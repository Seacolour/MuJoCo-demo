"""Expand requested placements into executable moves using free zones as buffers."""

from collections import deque

from mujoco_demo.shared.contracts import PickPlaceTask
from mujoco_demo.shared.task_plan import TaskPlan, validate_plan


def expand_goal_plan(scene: dict[str, object], requested: TaskPlan) -> TaskPlan:
    """Keep feasible ordered steps; repair blocked distinct-object goals within four moves."""
    objects = scene["objects"]
    zones = scene["zones"]
    for index, step in enumerate(requested.steps, start=1):
        if step.object_name not in objects:
            raise ValueError(f"第 {index} 步的方块不存在：{step.object_name}")
        if step.target_name not in zones:
            raise ValueError(f"第 {index} 步的目标区域不存在：{step.target_name}")

    # Repeated moves of one object express an ordered route, not final placements.
    requested_objects = [step.object_name for step in requested.steps]
    if len(set(requested_objects)) != len(requested_objects):
        validate_plan(scene, requested)
        return requested

    # An already executable sequence retains the order chosen by the user/model.
    try:
        validate_plan(scene, requested)
        return requested
    except ValueError:
        pass

    target_zones = [step.target_name for step in requested.steps]
    if len(set(target_zones)) != len(target_zones):
        raise ValueError("多个方块不能同时占用同一个目标区域")

    object_names = tuple(objects)
    zone_names = tuple(zones)
    current_zone: dict[str, str | None] = {name: None for name in object_names}
    for zone_name, zone in zones.items():
        occupant = zone.get("occupied_by")
        if occupant is None:
            continue
        if occupant not in current_zone or current_zone[occupant] is not None:
            raise ValueError("当前场景的区域占用信息不一致")
        current_zone[occupant] = zone_name

    initial = tuple(current_zone[name] for name in object_names)
    goals = {step.object_name: step.target_name for step in requested.steps}
    final = tuple(goals.get(name, current_zone[name]) for name in object_names)
    if initial == final:
        raise ValueError("方块已处于请求的目标区域")

    ordered_objects = tuple(dict.fromkeys(requested_objects + list(object_names)))
    ordered_zones = tuple(dict.fromkeys(target_zones + list(zone_names)))
    queue = deque([(initial, ())])
    visited = {initial}
    while queue:
        state, moves = queue.popleft()
        if state == final:
            plan = TaskPlan(steps=moves)
            validate_plan(scene, plan)
            return plan
        if len(moves) == 4:
            continue
        occupied = {zone for zone in state if zone is not None}
        for object_name in ordered_objects:
            index = object_names.index(object_name)
            for zone_name in ordered_zones:
                if zone_name in occupied or state[index] == zone_name:
                    continue
                next_state = list(state)
                next_state[index] = zone_name
                candidate = tuple(next_state)
                if candidate in visited:
                    continue
                visited.add(candidate)
                move = PickPlaceTask(object_name=object_name, target_name=zone_name)
                queue.append((candidate, (*moves, move)))

    raise ValueError("无法在四步内完成目标；请检查目标区占用或提供空闲区域")
