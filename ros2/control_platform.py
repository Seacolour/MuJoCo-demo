"""Compatibility entry point for the Agent platform."""

from mujoco_demo.platform.control_platform import AgentControlPlatform, app, main

__all__ = ["AgentControlPlatform", "app", "main"]


if __name__ == "__main__":
    main()
