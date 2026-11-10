"""Check that native Windows ROS 2 nodes can exchange a message."""

import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


def main() -> None:
    rclpy.init()
    publisher_node = Node("mujoco_demo_smoke_publisher")
    subscriber_node = Node("mujoco_demo_smoke_subscriber")
    received: list[str] = []
    subscriber_node.create_subscription(
        String, "/mujoco_demo/smoke", lambda message: received.append(message.data), 10
    )
    publisher = publisher_node.create_publisher(String, "/mujoco_demo/smoke", 10)
    executor = SingleThreadedExecutor()
    executor.add_node(publisher_node)
    executor.add_node(subscriber_node)

    try:
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and not received:
            publisher.publish(String(data="ros2-ok"))
            executor.spin_once(timeout_sec=0.1)
        if received != ["ros2-ok"]:
            raise RuntimeError(f"ROS 2 topic loopback failed: {received!r}")
        print("ROS 2 topic loopback OK")
    finally:
        executor.shutdown()
        subscriber_node.destroy_node()
        publisher_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
