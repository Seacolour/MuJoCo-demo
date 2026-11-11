"""Keep smoke checks away from a user's running ROS 2 demo."""

import os
import secrets


def isolate_ros_domain() -> None:
    domain = os.getenv("MUJOCO_DEMO_SMOKE_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = domain if domain is not None else str(100 + secrets.randbelow(100))
