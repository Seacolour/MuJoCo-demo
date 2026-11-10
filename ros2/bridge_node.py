"""Expose the MuJoCo pick-and-place executor through ROS 2 topics."""

from dataclasses import asdict
import argparse
from collections import deque
import json
import threading
import time

import mujoco.viewer
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from mujoco_demo.pick_place import MULTI_SCENE_PATH, PhaseReport, PickPlaceSimulation
from mujoco_demo.ros_contracts import (
    CANCEL_TOPIC,
    COMMAND_TOPIC,
    EVENT_TOPIC,
    JOINT_STATE_TOPIC,
    SCENE_TOPIC,
    TaskCancel,
    TaskCommand,
    TaskEvent,
)


class MuJoCoBridge(Node):
    def __init__(self, *, show_viewer: bool = False, multi_scene: bool = False) -> None:
        super().__init__("mujoco_pick_place_bridge")
        self.show_viewer = show_viewer
        self.simulation = PickPlaceSimulation(scene_path=MULTI_SCENE_PATH) if multi_scene else None
        self.scene_revision = 0
        self._events = self.create_publisher(String, EVENT_TOPIC, 10)
        self._joint_states = self.create_publisher(JointState, JOINT_STATE_TOPIC, 10)
        self._scene = self.create_publisher(
            String,
            SCENE_TOPIC,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE),
        )
        self.create_subscription(String, COMMAND_TOPIC, self._on_command, 10)
        self.create_subscription(String, CANCEL_TOPIC, self._on_cancel, 10)
        self._lock = threading.Lock()
        self._active_id: str | None = None
        self._active_cancel: threading.Event | None = None
        self._queue: deque[TaskCommand] = deque()
        self._seen_ids: set[str] = set()
        self._worker: threading.Thread | None = None
        self.completed_at: float | None = None
        self._publish_scene()

    def _publish_scene(self) -> None:
        if self.simulation is None:
            return
        snapshot = {
            "revision": self.scene_revision,
            **self.simulation.catalog.snapshot(self.simulation.data),
        }
        self._scene.publish(String(data=json.dumps(snapshot, ensure_ascii=False)))

    def _publish(self, event: TaskEvent) -> None:
        self._events.publish(String(data=event.model_dump_json()))
        self.get_logger().info(f"{event.request_id}: {event.status} {event.phase or event.message or ''}")

    def _on_command(self, message: String) -> None:
        try:
            command = TaskCommand.model_validate_json(message.data)
        except ValueError as exc:
            self._publish(TaskEvent(request_id="unknown", status="rejected", message=str(exc)))
            return
        with self._lock:
            if command.request_id in self._seen_ids:
                rejection = "duplicate request_id"
                queued = False
            elif self._active_id is not None:
                rejection = "robot queue is full" if len(self._queue) >= 8 else None
                queued = rejection is None
                if queued:
                    self._seen_ids.add(command.request_id)
                    self._queue.append(command)
            else:
                rejection = None
                queued = False
                self._active_id = command.request_id
                self._active_cancel = threading.Event()
                self._seen_ids.add(command.request_id)
        if rejection:
            self._publish(TaskEvent(request_id=command.request_id, status="rejected", message=rejection))
            return
        self._publish(TaskEvent(request_id=command.request_id, status="queued" if queued else "accepted"))
        if not queued:
            self._start_worker(command)

    def _start_worker(self, command: TaskCommand) -> None:
        cancel = self._active_cancel
        assert cancel is not None
        self._worker = threading.Thread(target=self._execute, args=(command, cancel), daemon=True)
        self._worker.start()

    def _on_cancel(self, message: String) -> None:
        try:
            cancel = TaskCancel.model_validate_json(message.data)
        except ValueError:
            return
        with self._lock:
            if cancel.request_id == self._active_id and self._active_cancel is not None:
                self._active_cancel.set()
                status = "cancel_requested"
            else:
                queued = next((item for item in self._queue if item.request_id == cancel.request_id), None)
                if queued is not None:
                    self._queue.remove(queued)
                    status = "cancelled"
                else:
                    status = "rejected"
        self._publish(TaskEvent(request_id=cancel.request_id, status=status))

    def _execute(self, command: TaskCommand, cancel: threading.Event) -> None:
        def report_phase(phase: PhaseReport) -> None:
            self._publish(TaskEvent(request_id=command.request_id, status="phase", phase=phase.name))
            state = JointState()
            state.header.stamp = self.get_clock().now().to_msg()
            state.name = [f"joint{i}" for i in range(1, 8)]
            state.position = list(phase.joint_positions_rad)
            self._joint_states.publish(state)

        try:
            simulation = self.simulation or PickPlaceSimulation()
            simulation.on_phase = report_phase
            simulation.should_cancel = cancel.is_set

            def run_and_publish() -> None:
                start_time = simulation.data.time
                result = simulation.run(command.task)
                if self.simulation is not None and simulation.data.time > start_time:
                    self.scene_revision += 1
                    self._publish_scene()
                self._publish(
                    TaskEvent(
                        request_id=command.request_id,
                        status="cancelled" if result.cancelled else "completed" if result.success else "failed",
                        message=result.failure_reason,
                        result=asdict(result),
                    )
                )

            if self.simulation is not None:
                run_and_publish()
            elif self.show_viewer:
                with mujoco.viewer.launch_passive(simulation.model, simulation.data) as viewer:
                    simulation.viewer = viewer
                    viewer.cam.lookat[:] = [0.35, 0, 0.45]
                    viewer.cam.distance = 1.2
                    viewer.cam.azimuth = 145
                    viewer.cam.elevation = -25
                    run_and_publish()
                    print("Task finished. Close the MuJoCo viewer to stop the bridge.", flush=True)
                    while viewer.is_running():
                        viewer.sync()
                        time.sleep(1 / 60)
            else:
                run_and_publish()
        except ValueError as exc:
            self._publish(TaskEvent(request_id=command.request_id, status="rejected", message=str(exc)))
        except Exception as exc:
            self._publish(TaskEvent(request_id=command.request_id, status="failed", message=str(exc)))
        finally:
            with self._lock:
                self.completed_at = time.monotonic()
                next_command = self._queue.popleft() if self._queue else None
                self._active_id = next_command.request_id if next_command else None
                self._active_cancel = threading.Event() if next_command else None
            if next_command is not None:
                self._publish(TaskEvent(request_id=next_command.request_id, status="accepted"))
                self._start_worker(next_command)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--once", action="store_true", help="exit after one completed task")
    parser.add_argument("--viewer", action="store_true", help="show one task in the MuJoCo viewer")
    parser.add_argument("--multi", action="store_true", help="keep a two-cube, three-zone scene across tasks")
    args = parser.parse_args()
    rclpy.init()
    node = MuJoCoBridge(show_viewer=args.viewer, multi_scene=args.multi)
    try:
        if args.multi and args.viewer:
            assert node.simulation is not None
            with mujoco.viewer.launch_passive(node.simulation.model, node.simulation.data) as viewer:
                node.simulation.viewer = viewer
                viewer.cam.lookat[:] = [0.4, 0, 0.45]
                viewer.cam.distance = 1.35
                viewer.cam.azimuth = 145
                viewer.cam.elevation = -25
                viewer.sync()
                while rclpy.ok() and viewer.is_running():
                    rclpy.spin_once(node, timeout_sec=0.05)
                    if node._active_id is None:
                        viewer.sync()
                    if (args.once and node.completed_at is not None and node._active_id is None
                            and time.monotonic() - node.completed_at > 1):
                        break
        else:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                if ((args.once or args.viewer) and node.completed_at is not None and node._active_id is None
                        and time.monotonic() - node.completed_at > 1):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if node._active_cancel is not None:
            node._active_cancel.set()
        if node._worker is not None:
            node._worker.join(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
