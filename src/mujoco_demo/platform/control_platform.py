"""Local HTTP control platform for the persistent MuJoCo ROS 2 bridge."""

from contextlib import asynccontextmanager
import json
from pathlib import Path
from queue import Empty, Queue
import threading
import time
from uuid import uuid4

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, StreamingResponse
from pydantic import BaseModel, Field
import rclpy
from rclpy.executors import SingleThreadedExecutor
import uvicorn

from mujoco_demo.platform.deepseek_planner import DeepSeekPlanner
from mujoco_demo.platform.ros_skill_client import RosSkillClient
from mujoco_demo.shared.task_plan import TaskPlan, validate_plan
from mujoco_demo.shared.ros_contracts import TaskEvent


WEB_PAGE = Path(__file__).resolve().parents[3] / "web" / "control.html"
FINAL_STATES = {"succeeded", "failed", "cancelled", "rejected"}


class InstructionRequest(BaseModel):
    instruction: str = Field(min_length=2, max_length=500)


class AgentControlPlatform:
    def __init__(self) -> None:
        self._ros = RosSkillClient(self._on_event, self._on_scene)
        self._lock = threading.RLock()
        self._tasks: dict[str, dict[str, object]] = {}
        self._order: list[str] = []
        self._scene: dict[str, object] | None = None
        self._listeners: set[Queue[str]] = set()
        self._pending: Queue[str | None] = Queue()
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._ros)
        self._child_to_parent: dict[str, tuple[str, int]] = {}
        self._active_parent_id: str | None = None
        self._active_step_id: str | None = None
        self._step_done = threading.Event()
        self._step_result: TaskEvent | None = None
        self._stop = threading.Event()
        self._ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self._planner_thread = threading.Thread(target=self._plan_tasks, daemon=True)

    def start(self) -> None:
        self._ros_thread.start()
        self._planner_thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._pending.put(None)
        self._planner_thread.join(timeout=5)
        self._ros_thread.join(timeout=5)
        self._executor.shutdown()
        self._ros.destroy_node()

    def _spin_ros(self) -> None:
        while not self._stop.is_set() and rclpy.ok():
            self._executor.spin_once(timeout_sec=0.1)

    def _emit(self, payload: dict[str, object]) -> None:
        message = json.dumps(payload, ensure_ascii=False)
        with self._lock:
            listeners = tuple(self._listeners)
        for listener in listeners:
            listener.put(message)

    def _task_snapshot(self, request_id: str) -> dict[str, object]:
        with self._lock:
            task = self._tasks[request_id]
            return {**task, "events": list(task["events"])}

    def _set_task(self, request_id: str, **changes: object) -> None:
        with self._lock:
            self._tasks[request_id].update(changes)
            self._tasks[request_id]["updated_at"] = time.time()
        self._emit({"kind": "task", "task": self._task_snapshot(request_id)})

    def submit(self, instruction: str) -> dict[str, object]:
        request_id = str(uuid4())
        now = time.time()
        with self._lock:
            self._tasks[request_id] = {
                "request_id": request_id,
                "instruction": instruction,
                "status": "received",
                "phase": None,
                "parsed_task": None,
                "plan": None,
                "current_step": 0,
                "total_steps": 0,
                "result": None,
                "error": None,
                "events": [],
                "created_at": now,
                "updated_at": now,
            }
            self._order.append(request_id)
        self._pending.put(request_id)
        self._emit({"kind": "task", "task": self._task_snapshot(request_id)})
        return self._task_snapshot(request_id)

    def cancel(self, request_id: str) -> dict[str, object]:
        with self._lock:
            if request_id not in self._tasks:
                raise KeyError(request_id)
            status = self._tasks[request_id]["status"]
            if status in FINAL_STATES:
                return self._task_snapshot(request_id)
            active_step_id = self._active_step_id if self._active_parent_id == request_id else None
            next_status = "cancel_requested" if active_step_id else "cancelled"
        self._set_task(request_id, status=next_status)
        if active_step_id is not None:
            self._ros.cancel_task(active_step_id)
        return self._task_snapshot(request_id)

    def _plan_tasks(self) -> None:
        while not self._stop.is_set():
            request_id = self._pending.get()
            if request_id is None:
                break
            with self._lock:
                if self._tasks[request_id]["status"] == "cancelled":
                    continue
            self._set_task(request_id, status="parsing")
            try:
                deadline = time.monotonic() + 10
                while self.scene() is None and time.monotonic() < deadline and not self._stop.is_set():
                    time.sleep(0.1)
                scene = self.scene()
                if scene is None:
                    raise RuntimeError("MuJoCo scene is unavailable; start the multi-scene bridge")
                instruction = str(self._task_snapshot(request_id)["instruction"])
                plan = DeepSeekPlanner().parse_plan(instruction, scene)
                validate_plan(scene, plan)
                with self._lock:
                    if self._tasks[request_id]["status"] == "cancelled":
                        continue
                if not self._ros.has_bridge():
                    raise RuntimeError("ROS 2 task bridge is unavailable")
                self._set_task(
                    request_id,
                    status="queued",
                    plan=[step.model_dump() for step in plan.steps],
                    total_steps=len(plan.steps),
                )
            except Exception as exc:
                if self._task_snapshot(request_id)["status"] != "cancelled":
                    self._set_task(request_id, status="rejected", error=str(exc))
                continue
            self._execute_plan(request_id, plan)

    def _execute_plan(self, request_id: str, plan: TaskPlan) -> None:
        step_results: list[dict[str, object]] = []
        for index, step in enumerate(plan.steps, start=1):
            if self._task_snapshot(request_id)["status"] in {"cancelled", "cancel_requested"}:
                self._set_task(request_id, status="cancelled")
                return
            scene = self.scene()
            if scene is None:
                self._set_task(request_id, status="failed", error="仿真场景已断开")
                return
            try:
                validate_plan(scene, TaskPlan(steps=plan.steps[index - 1:]))
            except ValueError as exc:
                self._set_task(
                    request_id, status="rejected" if index == 1 else "failed", error=str(exc)
                )
                return
            if not self._ros.has_bridge():
                self._set_task(request_id, status="failed", error="ROS 2 执行桥接已断开")
                return

            child_id = f"{request_id}:step:{index}"
            with self._lock:
                self._child_to_parent[child_id] = (request_id, index)
                self._active_parent_id = request_id
                self._active_step_id = child_id
                self._step_result = None
                self._step_done.clear()
            self._set_task(
                request_id, status="executing", current_step=index,
                parsed_task=step.model_dump(), phase=None,
            )
            self._ros.send_task(child_id, step)
            if not self._step_done.wait(timeout=90):
                self._ros.cancel_task(child_id)
                with self._lock:
                    self._active_parent_id = None
                    self._active_step_id = None
                    self._child_to_parent.pop(child_id, None)
                self._set_task(request_id, status="failed", error=f"第 {index} 步执行超时")
                return
            with self._lock:
                outcome = self._step_result
                self._active_parent_id = None
                self._active_step_id = None
                self._child_to_parent.pop(child_id, None)
            if outcome is None or outcome.status != "completed":
                status = "cancelled" if outcome is not None and outcome.status == "cancelled" else "failed"
                message = outcome.message if outcome is not None else "执行结果缺失"
                self._set_task(request_id, status=status, error=f"第 {index} 步：{message}")
                return
            step_results.append({
                "index": index, "tool": "pick_place", "task": step.model_dump(),
                "result": outcome.result,
            })
            self._set_task(request_id, result={"steps": list(step_results)})
            if index < len(plan.steps):
                previous_revision = int(scene.get("revision", 0))
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    current = self.scene()
                    if current is not None and int(current.get("revision", 0)) > previous_revision:
                        break
                    time.sleep(0.05)
                else:
                    self._set_task(request_id, status="failed", error="执行后场景状态未更新")
                    return
        self._set_task(request_id, status="succeeded", phase=None)

    def _on_event(self, event: TaskEvent) -> None:
        with self._lock:
            mapping = self._child_to_parent.get(event.request_id)
            if mapping is None:
                return
            request_id, step_index = mapping
            task = self._tasks[request_id]
            task["events"].append({
                "status": event.status,
                "phase": event.phase,
                "message": event.message,
                "step_index": step_index,
                "time": time.time(),
            })
            if event.request_id == self._active_step_id and event.status in {
                "completed", "failed", "rejected", "cancelled"
            }:
                self._step_result = event
                self._step_done.set()
        if event.status == "phase":
            if self._task_snapshot(request_id)["status"] != "cancel_requested":
                self._set_task(request_id, phase=event.phase)
        elif event.status == "accepted":
            self._set_task(request_id, status="executing")
        elif event.status == "queued":
            self._set_task(request_id, status="queued")
        else:
            self._emit({"kind": "task", "task": self._task_snapshot(request_id)})

    def _on_scene(self, scene: dict[str, object]) -> None:
        with self._lock:
            self._scene = scene
        self._emit({"kind": "scene", "scene": scene})

    def scene(self) -> dict[str, object] | None:
        with self._lock:
            return self._scene

    def tasks(self) -> list[dict[str, object]]:
        with self._lock:
            ids = list(reversed(self._order))
        return [self._task_snapshot(request_id) for request_id in ids]

    def task(self, request_id: str) -> dict[str, object]:
        return self._task_snapshot(request_id)

    def connected(self) -> bool:
        return self._ros.has_bridge() and self.scene() is not None

    def subscribe(self) -> Queue[str]:
        listener: Queue[str] = Queue()
        with self._lock:
            self._listeners.add(listener)
        listener.put(json.dumps({
            "kind": "snapshot", "tasks": self.tasks(), "scene": self.scene(),
            "connected": self.connected(),
        }, ensure_ascii=False))
        return listener

    def unsubscribe(self, listener: Queue[str]) -> None:
        with self._lock:
            self._listeners.discard(listener)


@asynccontextmanager
async def lifespan(app: FastAPI):
    rclpy.init()
    platform = AgentControlPlatform()
    app.state.platform = platform
    platform.start()
    try:
        yield
    finally:
        platform.stop()
        rclpy.shutdown()


app = FastAPI(title="MuJoCo Agent Control Platform", lifespan=lifespan)


def get_platform(request: Request) -> AgentControlPlatform:
    return request.app.state.platform


@app.get("/")
def index() -> FileResponse:
    return FileResponse(WEB_PAGE)


@app.get("/api/health")
def health(request: Request) -> dict[str, object]:
    platform = get_platform(request)
    return {"bridge_connected": platform.connected(), "scene_revision": (
        platform.scene() or {}
    ).get("revision")}


@app.get("/api/scene")
def scene(request: Request) -> dict[str, object]:
    return get_platform(request).scene() or {"status": "waiting_for_bridge"}


@app.get("/api/tasks")
def tasks(request: Request) -> list[dict[str, object]]:
    return get_platform(request).tasks()


@app.get("/api/tasks/{request_id}")
def task(request_id: str, request: Request) -> dict[str, object]:
    try:
        return get_platform(request).task(request_id)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="task not found") from exc


@app.post("/api/tasks", status_code=202)
def submit(body: InstructionRequest, request: Request) -> dict[str, object]:
    return get_platform(request).submit(body.instruction)


@app.post("/api/tasks/{request_id}/cancel")
def cancel(request_id: str, request: Request) -> dict[str, object]:
    try:
        return get_platform(request).cancel(request_id)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="task not found") from exc


@app.get("/api/events")
def events(request: Request) -> StreamingResponse:
    platform = get_platform(request)

    def stream():
        listener = platform.subscribe()
        try:
            while True:
                try:
                    payload = listener.get(timeout=15)
                    yield f"data: {payload}\n\n"
                except Empty:
                    yield ": keepalive\n\n"
        finally:
            platform.unsubscribe(listener)

    return StreamingResponse(stream(), media_type="text/event-stream")


def main() -> None:
    uvicorn.run(app, host="127.0.0.1", port=8765)


if __name__ == "__main__":
    main()
