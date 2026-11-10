# Windows ROS 2 Jazzy environment

This separate Pixi workspace contains ROS 2 Jazzy `ros-base`, `xacro`, and an editable installation of the root MuJoCo demo package.

From the project root:

```powershell
pixi run --manifest-path ros2/pixi.toml ros2 pkg list
pixi run --manifest-path ros2/pixi.toml smoke
pixi run --manifest-path ros2/pixi.toml smoke-queue
pixi run --manifest-path ros2/pixi.toml python ros2/bridge_node.py
pixi run --manifest-path ros2/pixi.toml python ros2/agent_client.py --offline
```

The xArm7 task bridge publishes JSON task events on `/mujoco_demo/task_event`, accepts JSON task commands on `/mujoco_demo/task_command`, cancellation requests on `/mujoco_demo/task_cancel`, and publishes `sensor_msgs/JointState` on `/mujoco_demo/joint_states`. It accepts one active task and queues up to eight more. This is a Topic-based prototype; it does not yet expose a ROS 2 Action interface.

Add `--viewer` to the `bridge_node.py` command to watch one ROS-triggered task in real time. The viewer stays open after the result and the bridge exits when the viewer is closed. Without `--viewer`, the bridge runs headless and can process a queue of tasks.

For the persistent two-cube, three-zone scene, use `bridge_node.py --multi --viewer`. Start `control_platform.py` in a second terminal, then open `http://127.0.0.1:8765` in a browser. The control platform parses natural-language instructions with DeepSeek, exposes task and scene APIs, and streams task events to the browser. Task history is currently in memory, while the bridge owns the one-robot execution queue.

`pixi run --manifest-path ros2/pixi.toml smoke-multi` validates two queued commands against the same simulation state.

Use `pixi run --manifest-path ros2/pixi.toml python ros2/agent_client.py "把红色方块放到绿色目标区"` instead of `--offline` to parse the instruction with DeepSeek. The agent reads `DEEPSEEK_API_KEY` from the process environment or the project root `.env` file.
