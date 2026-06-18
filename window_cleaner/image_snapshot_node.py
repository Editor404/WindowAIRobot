from __future__ import annotations

from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageSnapshotNode(Node):
    def __init__(self) -> None:
        super().__init__("image_snapshot")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("output", "/tmp/window_cleaner_snapshot.jpg")
        self.declare_parameter("timeout_sec", 10.0)

        self.bridge = CvBridge()
        self.output = Path(str(self.get_parameter("output").value)).expanduser()
        self.done = False
        topic = str(self.get_parameter("image_topic").value)
        self.subscription = self.create_subscription(Image, topic, self.on_image, 10)
        self.deadline = self.get_clock().now().nanoseconds + int(
            float(self.get_parameter("timeout_sec").value) * 1_000_000_000
        )
        self.timer = self.create_timer(0.2, self.on_timer)
        self.get_logger().info(f"Waiting for one image on {topic}; output={self.output}")

    def on_image(self, msg: Image) -> None:
        if self.done:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.output.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(self.output), frame):
            raise RuntimeError(f"Failed to write image: {self.output}")
        self.done = True
        self.get_logger().info(f"Saved snapshot: {self.output}")

    def on_timer(self) -> None:
        if self.done:
            rclpy.shutdown()
            return
        if self.get_clock().now().nanoseconds > self.deadline:
            self.get_logger().error("Timed out waiting for image")
            rclpy.shutdown()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ImageSnapshotNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
