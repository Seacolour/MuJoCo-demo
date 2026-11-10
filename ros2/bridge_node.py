"""Compatibility entry point for the MuJoCo robot runtime."""

from mujoco_demo.robot_runtime.bridge_node import MuJoCoBridge, main

__all__ = ["MuJoCoBridge", "main"]


if __name__ == "__main__":
    main()
