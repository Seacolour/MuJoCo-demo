"""The model response must stay inside the simulated scene's task vocabulary."""

from io import BytesIO
import json

import pytest

from mujoco_demo.deepseek_planner import DeepSeekPlanner


def fake_response(task: dict[str, str]) -> BytesIO:
    body = {"choices": [{"message": {"content": json.dumps(task)}}]}
    return BytesIO(json.dumps(body).encode("utf-8"))


def test_valid_deepseek_task_is_accepted(monkeypatch: pytest.MonkeyPatch) -> None:
    def respond(http_request: object, timeout: int) -> BytesIO:
        assert timeout == 30
        assert json.loads(http_request.data)["response_format"] == {"type": "json_object"}
        return fake_response({"object_name": "red_cube", "target_name": "target_zone"})

    monkeypatch.setattr("mujoco_demo.deepseek_planner.request.urlopen", respond)
    task = DeepSeekPlanner(api_key="test-key").parse("把红色方块放进目标区")
    assert task.object_name == "red_cube"
    assert task.target_name == "target_zone"


def test_model_cannot_select_an_unknown_target(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        "mujoco_demo.deepseek_planner.request.urlopen",
        lambda *_args, **_kwargs: fake_response(
            {"object_name": "red_cube", "target_name": "robot_base"}
        ),
    )
    with pytest.raises(ValueError, match="outside the current scene"):
        DeepSeekPlanner(api_key="test-key").parse("把方块放到机器人底座")
