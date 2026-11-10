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

Use `pixi run --manifest-path ros2/pixi.toml python ros2/agent_client.py "把红色方块放到绿色目标区"` instead of `--offline` to parse the instruction with DeepSeek. The agent reads `DEEPSEEK_API_KEY` from the process environment or the project root `.env` file.
