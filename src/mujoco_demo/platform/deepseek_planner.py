"""Use DeepSeek only for high-level, validated task interpretation."""

import json
import os
from pathlib import Path
import re
from urllib import error, request

from dotenv import dotenv_values
from pydantic import ValidationError

from mujoco_demo.shared.contracts import PickPlaceTask
from mujoco_demo.shared.task_plan import TaskPlan


DEEPSEEK_URL = "https://api.deepseek.com/chat/completions"
PROJECT_ROOT = Path(__file__).resolve().parents[3]


class DeepSeekPlanner:
    def __init__(self, *, api_key: str | None = None, model: str | None = None) -> None:
        local_settings = dotenv_values(PROJECT_ROOT / ".env")
        self.api_key = api_key or os.getenv("DEEPSEEK_API_KEY") or local_settings.get("DEEPSEEK_API_KEY")
        self.model = model or os.getenv("DEEPSEEK_MODEL") or local_settings.get("DEEPSEEK_MODEL") or "deepseek-flash"
        if not self.api_key:
            raise RuntimeError("Set DEEPSEEK_API_KEY in the process environment before running the agent")

    def parse(self, instruction: str, scene: dict[str, object] | None = None) -> PickPlaceTask:
        if scene is None:
            scene = {
                "objects": {"red_cube": {"label": "红色方块"}},
                "zones": {"target_zone": {"label": "绿色目标区域", "occupied_by": None}},
            }
        objects = scene["objects"]
        zones = scene["zones"]
        system_prompt = (
            "你是机械臂任务解析器。只返回 JSON 对象，不输出解释或代码。"
            "根据当前场景选择一个物体和一个未占用的目标区域。"
            "只允许输出形如 "
            '{"object_name":"red_cube","target_name":"blue_zone"} 的 JSON。'
            '若指令不明确、物体或区域不存在，返回 {"error":"unsupported task"}。'
            '若目标区域的 occupied_by 非空，返回 {"error":"target zone is occupied"}。'
            "不得输出坐标、关节角或控制量。"
            f"当前场景 JSON：{json.dumps({'objects': objects, 'zones': zones}, ensure_ascii=False)}"
        )
        parsed = self._complete_json(instruction, system_prompt, max_tokens=320)
        task = PickPlaceTask.model_validate(parsed)
        if task.object_name not in objects or task.target_name not in zones:
            raise ValueError("DeepSeek selected an object or target outside the current scene")
        if zones[task.target_name].get("occupied_by") is not None:
            raise ValueError("目标区域已被占用")
        return task

    def parse_plan(self, instruction: str, scene: dict[str, object]) -> TaskPlan:
        system_prompt = (
            "你是机械臂任务规划器。只返回 JSON 对象，不输出解释或代码。"
            "识别用户要求的每个方块及其目标区域，按原文顺序输出；不得遗漏方块。"
            "若两个方块当前占据彼此的目标区域，仍输出两个目标，平台会用空闲区域安排临时搬运。"
            "若用户要求交换两个方块的位置，根据当前场景中的占用关系确定双方目标。"
            "不要因为目标当前被另一个待移动方块占用而拒绝，也不要自行生成临时搬运。"
            "最多四个要求。合法格式示例："
            '{"steps":[{"object_name":"green_cube","target_name":"red_zone"},'
            '{"object_name":"red_cube","target_name":"green_zone"}]}。'
            '如果无法确定所有步骤，返回 {"error":"unsupported_task"}。'
            "只选择场景中提供的物体和区域 ID；不要生成坐标、关节角或控制代码。"
            f"当前场景 JSON：{json.dumps(scene, ensure_ascii=False)}"
        )
        parsed = self._complete_json(instruction, system_prompt, max_tokens=1024)
        try:
            plan = TaskPlan.model_validate(parsed)
        except ValidationError as exc:
            raise ValueError("任务规划格式不符合要求，最多支持四步") from exc
        explicit_moves = len(re.findall(r"(?:把|将)(?:红色|绿色)", instruction))
        mentioned_objects = {
            name for name, info in scene["objects"].items()
            if isinstance(info.get("label"), str) and info["label"] in instruction
        }
        planned_objects = {step.object_name for step in plan.steps}
        requires_multiple = bool(re.search(r"然后|接着|随后|再把|再将|并且", instruction))
        incomplete_sequence = (requires_multiple or explicit_moves > 1) and len(plan.steps) < 2
        missing_objects = len(mentioned_objects) > 1 and not mentioned_objects <= planned_objects
        if incomplete_sequence or missing_objects:
            raise ValueError("多步骤指令未完整解析，未执行机械臂动作")
        for step in plan.steps:
            if step.object_name not in scene["objects"] or step.target_name not in scene["zones"]:
                raise ValueError("DeepSeek selected an object or target outside the current scene")
        return plan

    def _complete_json(self, instruction: str, system_prompt: str, *, max_tokens: int) -> dict:
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": instruction},
            ],
            "response_format": {"type": "json_object"},
            "max_tokens": max_tokens,
            "stream": False,
        }
        parsed = None
        finish_reason = None
        for attempt in range(2):
            payload["max_tokens"] = max_tokens * (attempt + 1)
            http_request = request.Request(
                DEEPSEEK_URL,
                data=json.dumps(payload).encode("utf-8"),
                headers={
                    "Authorization": f"Bearer {self.api_key}",
                    "Content-Type": "application/json",
                },
                method="POST",
            )
            try:
                with request.urlopen(http_request, timeout=30) as response:
                    body = json.load(response)
            except error.HTTPError as exc:
                raise RuntimeError(f"DeepSeek request failed with HTTP {exc.code}") from exc
            choice = body["choices"][0]
            finish_reason = choice.get("finish_reason")
            content = choice["message"]["content"]
            if not content or not content.strip():
                continue
            try:
                parsed = json.loads(content)
                break
            except json.JSONDecodeError:
                continue
        if parsed is None:
            detail = "（输出达到长度限制）" if finish_reason == "length" else ""
            raise ValueError(f"DeepSeek 未返回有效的任务 JSON{detail}，请重试")
        if "error" in parsed:
            reason = str(parsed["error"])
            messages = {
                "unsupported task": "当前场景无法执行这条指令",
                "unsupported_task": "当前场景无法执行这条指令",
                "target zone is occupied": "目标区域已被占用",
                "target_occupied": "目标区域已被占用",
            }
            raise ValueError(messages.get(reason, "当前场景无法执行这条指令"))
        return parsed
