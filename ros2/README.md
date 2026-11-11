# Windows ROS 2 Jazzy environment

This separate Pixi workspace contains ROS 2 Jazzy `ros-base`, `xacro`, interface generators, `colcon`, and an editable installation of the root MuJoCo demo package.

The application code now lives in `mujoco_demo.platform`, `mujoco_demo.robot_runtime`, and `mujoco_demo.shared`. The scripts in this directory remain compatibility entry points. Both runtime modules currently use this same Pixi environment.

From the project root:

```powershell
pixi run --manifest-path ros2/pixi.toml ros2 pkg list
pixi run --manifest-path ros2/pixi.toml smoke
pixi run --manifest-path ros2/pixi.toml smoke-queue
pixi run --manifest-path ros2/pixi.toml python ros2/bridge_node.py
pixi run --manifest-path ros2/pixi.toml python ros2/agent_client.py --offline
```

The primary demo now uses `mujoco_demo_interfaces/action/PickPlace`. From the project root, build the interface with `pixi run --manifest-path ros2/pixi.toml build-interfaces` (Visual Studio C++ Build Tools are required on Windows); rebuild after editing the `.action` file. Run `bridge-action` and `platform-action` in separate terminals. The generated interface is loaded through `run_with_interfaces.cmd`; plain Python commands do not source it.

The older Topic path remains available for comparison: the bridge accepts JSON on `/mujoco_demo/task_command` and `/mujoco_demo/task_cancel`, publishes events on `/mujoco_demo/task_event`, and queues up to eight requests. The Action server accepts one Goal at a time and rejects stale scene revisions and duplicate request IDs. Both modes publish scene snapshots and `sensor_msgs/JointState`.

Add `--viewer` to the `bridge_node.py` command to watch one ROS-triggered task in real time. The viewer stays open after the result and the bridge exits when the viewer is closed. Without `--viewer`, the bridge runs headless and can process a queue of tasks.

For the persistent two-cube, three-zone scene, use `bridge_node.py --multi --viewer`. Start `control_platform.py` in a second terminal, then open `http://127.0.0.1:8765` in a browser. The control platform parses natural-language instructions with DeepSeek, exposes task and scene APIs, and streams task events to the browser. Task history is currently in memory, while the bridge owns the one-robot execution queue.

`pixi run --manifest-path ros2/pixi.toml smoke-multi` validates two queued commands against the same simulation state.

`pixi run --manifest-path ros2/pixi.toml smoke-plan` verifies the Topic fallback: preflight rejection, a valid two-step parent task, a three-step swap through the empty blue zone, and zero-step success when the final goals already hold. The platform performs natural-language decomposition in either transport mode.

`pixi run --manifest-path ros2/pixi.toml smoke-action` exercises the platform and Action runtime with feedback, results, cancellation, duplicate IDs, and stale scene revisions. `smoke-action-process` verifies Action transport between separate Windows processes.

The MuJoCo runtime restores the state from the start of a cancelled or failed step before returning its result. This simulation-only rollback keeps later commands executable; a physical robot would require a separate recovery procedure.

Use `pixi run --manifest-path ros2/pixi.toml python ros2/agent_client.py "把红色方块放到绿色目标区"` instead of `--offline` to parse the instruction with DeepSeek. The agent reads `DEEPSEEK_API_KEY` from the process environment or the project root `.env` file.
