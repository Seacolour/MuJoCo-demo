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
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import uvicorn

from mujoco_demo.deepseek_planner import DeepSeekPlanner
from mujoco_demo.ros_contracts import (
    CANCEL_TOPIC, COMMAND_TOPIC, EVENT_TOPIC, SCENE_TOPIC,
    TaskCancel, TaskCommand, TaskEvent,
)


WEB_PAGE = Path(__file__).resolve().parents[1] / "web" / "control.html"
FINAL_STATES = {"succeeded", "failed", "cancelled", "rejected"}


class InstructionRequest(BaseModel):
    instruction: str = Field(min_length=2, max_length=500)


class AgentControlPlatform(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_agent_control_platform")
        self.command_publisher = self.create_publisher(String, COMMAND_TOPIC, 10)
        self.cancel_publisher = self.create_publisher(String, CANCEL_TOPIC, 10)
        self.create_subscription(String, EVENT_TOPIC, self._on_event, 10)
        self.create_subscription(
            String, SCENE_TOPIC, self._on_scene,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE),
        )
        self._lock = threading.RLock()
        self._tasks: dict[str, dict[str, object]] = {}
        self._order: list[str] = []
        self._scene: dict[str, object] | None = None
        self._listeners: set[Queue[str]] = set()
        self._pending: Queue[str | None] = Queue()
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
        self.destroy_node()

    def _spin_ros(self) -> None:
        while not self._stop.is_set() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

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
            if status in {"received", "parsing"}:
                next_status = "cancelled"
            else:
                next_status = "cancel_requested"
        self._set_task(request_id, status=next_status)
        if next_status == "cancel_requested":
            self.cancel_publisher.publish(
                String(data=TaskCancel(request_id=request_id).model_dump_json())
            )
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
                task = DeepSeekPlanner().parse(instruction, scene)
                with self._lock:
                    if self._tasks[request_id]["status"] == "cancelled":
                        continue
                if self.command_publisher.get_subscription_count() == 0:
                    raise RuntimeError("ROS 2 task bridge is unavailable")
                self._set_task(request_id, status="queued", parsed_task=task.model_dump())
                self.command_publisher.publish(
                    String(data=TaskCommand(request_id=request_id, task=task).model_dump_json())
                )
            except Exception as exc:
                self._set_task(request_id, status="rejected", error=str(exc))

    def _on_event(self, message: String) -> None:
        event = TaskEvent.model_validate_json(message.data)
        with self._lock:
            if event.request_id not in self._tasks:
                return
            task = self._tasks[event.request_id]
            task["events"].append({
                "status": event.status,
                "phase": event.phase,
                "message": event.message,
                "time": time.time(),
            })
        status = {
            "accepted": "executing", "queued": "queued", "phase": "executing",
            "completed": "succeeded", "failed": "failed", "rejected": "rejected",
            "cancel_requested": "cancel_requested", "cancelled": "cancelled",
        }[event.status]
        self._set_task(
            event.request_id,
            status=status,
            phase=event.phase if event.phase else self._task_snapshot(event.request_id)["phase"],
            result=event.result if event.result is not None else self._task_snapshot(event.request_id)["result"],
            error=event.message if event.message else self._task_snapshot(event.request_id)["error"],
        )

    def _on_scene(self, message: String) -> None:
        scene = json.loads(message.data)
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
        return self.command_publisher.get_subscription_count() > 0 and self.scene() is not None

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


if __name__ == "__main__":
    uvicorn.run(app, host="127.0.0.1", port=8765)
