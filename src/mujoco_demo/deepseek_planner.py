"""Use DeepSeek only for high-level, validated task interpretation."""

import json
import os
from pathlib import Path
from urllib import error, request

from dotenv import dotenv_values

from mujoco_demo.contracts import PickPlaceTask


DEEPSEEK_URL = "https://api.deepseek.com/chat/completions"
PROJECT_ROOT = Path(__file__).resolve().parents[2]


class DeepSeekPlanner:
    def __init__(self, *, api_key: str | None = None, model: str | None = None) -> None:
        local_settings = dotenv_values(PROJECT_ROOT / ".env")
        self.api_key = api_key or os.getenv("DEEPSEEK_API_KEY") or local_settings.get("DEEPSEEK_API_KEY")
        self.model = model or os.getenv("DEEPSEEK_MODEL") or local_settings.get("DEEPSEEK_MODEL") or "deepseek-flash"
        if not self.api_key:
            raise RuntimeError("Set DEEPSEEK_API_KEY in the process environment before running the agent")

    def parse(self, instruction: str) -> PickPlaceTask:
        system_prompt = (
            "你是机械臂任务解析器。只返回 JSON 对象，不输出解释或代码。"
            "场景中只有 red_cube（红色方块）和 target_zone（绿色目标区域）。"
            "如果指令明确要求将红色方块放到绿色目标区域，返回 "
            '{"object_name":"red_cube","target_name":"target_zone"}。'
            '若指令不明确或请求其他动作，返回 {"error":"unsupported task"}。'
            "不得输出坐标、关节角或控制量。"
        )
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": instruction},
            ],
            "response_format": {"type": "json_object"},
            "max_tokens": 160,
            "stream": False,
        }
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
        content = body["choices"][0]["message"]["content"]
        if not content or not content.strip():
            raise ValueError("DeepSeek returned an empty task description")
        parsed = json.loads(content)
        if "error" in parsed:
            raise ValueError(str(parsed["error"]))
        task = PickPlaceTask.model_validate(parsed)
        if task.object_name != "red_cube" or task.target_name != "target_zone":
            raise ValueError("DeepSeek selected an object or target outside the current scene")
        return task
