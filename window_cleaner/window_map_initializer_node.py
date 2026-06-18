from __future__ import annotations

import math

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Pose2D, PoseArray
from rclpy.node import Node
from sensor_msgs.msg import Image

from window_cleaner.top_corner_detector import detect_top_corners, draw_top_corner_detection


class WindowMapInitializerNode(Node):
    def __init__(self) -> None:
        super().__init__("window_map_initializer_node")

        self.declare_parameter("image_topic", "/robot_camera/image_raw")
        self.declare_parameter("map_topic", "/window/map")
        self.declare_parameter("top_corners_topic", "/window/top_corners_px")
        self.declare_parameter("debug_image_topic", "/window/top_corners_debug_image")
        self.declare_parameter("window_width_cm", 80.0)
        self.declare_parameter("window_height_cm", 160.0)
        self.declare_parameter("min_confidence", 0.05)
        self.declare_parameter("publish_once", True)

        self.bridge = CvBridge()
        self.initialized = False
        self.map_pub = self.create_publisher(Pose2D, str(self.get_parameter("map_topic").value), 10)
        self.corners_pub = self.create_publisher(PoseArray, str(self.get_parameter("top_corners_topic").value), 10)
        self.debug_pub = self.create_publisher(Image, str(self.get_parameter("debug_image_topic").value), 10)
        self.image_sub = self.create_subscription(
            Image, str(self.get_parameter("image_topic").value), self.on_image, 10
        )
        self.get_logger().info(
            "Window map initializer waiting for top two corners: "
            f"image={self.get_parameter('image_topic').value}, map={self.get_parameter('map_topic').value}"
        )

    def on_image(self, msg: Image) -> None:
        if self.initialized and bool(self.get_parameter("publish_once").value):
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        detection = detect_top_corners(frame)
        debug = draw_top_corner_detection(frame, detection)
        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)

        if detection is None:
            return
        if detection.confidence < float(self.get_parameter("min_confidence").value):
            return

        corners_msg = PoseArray()
        corners_msg.header = msg.header
        corners_msg.header.frame_id = msg.header.frame_id or "camera_image"
        for x_px, y_px in (detection.top_left, detection.top_right):
            pose = Pose()
            pose.position.x = float(x_px)
            pose.position.y = float(y_px)
            pose.orientation.w = detection.confidence
            corners_msg.poses.append(pose)
        self.corners_pub.publish(corners_msg)

        map_msg = Pose2D()
        map_msg.x = float(self.get_parameter("window_width_cm").value)
        map_msg.y = float(self.get_parameter("window_height_cm").value)
        map_msg.theta = detection.confidence
        self.map_pub.publish(map_msg)
        self.initialized = True

        pixel_width = math.hypot(
            detection.top_right[0] - detection.top_left[0],
            detection.top_right[1] - detection.top_left[1],
        )
        self.get_logger().info(
            "Initialized window map from top corners: "
            f"TL=({detection.top_left[0]:.1f},{detection.top_left[1]:.1f}), "
            f"TR=({detection.top_right[0]:.1f},{detection.top_right[1]:.1f}), "
            f"top_width_px={pixel_width:.1f}, "
            f"map=({map_msg.x:.1f}x{map_msg.y:.1f})cm, confidence={detection.confidence:.2f}"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WindowMapInitializerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
