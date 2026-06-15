from __future__ import annotations

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image

from window_cleaner.detect_dirt import DirtSegmenter
from window_cleaner.image_input import resize_frame
from window_cleaner.paths import default_model_path


class DirtSegmentationNode(Node):
    def __init__(self) -> None:
        super().__init__("dirt_segmentation_node")

        self.declare_parameter("model_path", str(default_model_path()))
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("image_width", 0)
        self.declare_parameter("image_height", 0)
        self.declare_parameter("center_topic", "/dirt/center_pixel")
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("device", "cpu")

        model_path = self.get_parameter("model_path").value
        image_topic = self.get_parameter("image_topic").value
        center_topic = self.get_parameter("center_topic").value
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        device = self.get_parameter("device").value or None

        self.bridge = CvBridge()
        self.segmenter = DirtSegmenter(
            model_path=model_path,
            confidence_threshold=confidence_threshold,
            device=device,
        )
        self.publisher = self.create_publisher(PointStamped, center_topic, 10)
        self.subscription = self.create_subscription(Image, image_topic, self.on_image, 10)

        self.get_logger().info(f"Listening: {image_topic}")
        self.get_logger().info(f"Input image size: {self.image_width}x{self.image_height} (set either to 0 to keep native size)")
        self.get_logger().info(f"Publishing dirt center pixels: {center_topic}")

    def on_image(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        frame = resize_frame(frame, self.image_width, self.image_height)
        detection = self.segmenter.detect_largest(frame)
        if detection is None:
            return

        center_msg = PointStamped()
        center_msg.header = msg.header
        center_msg.point.x = detection.center_px[0]
        center_msg.point.y = detection.center_px[1]
        center_msg.point.z = detection.confidence
        self.publisher.publish(center_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DirtSegmentationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
