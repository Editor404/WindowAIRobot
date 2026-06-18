from __future__ import annotations

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class RPiCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("rpi_camera_node")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("backend", "picamera2")  # picamera2 | opencv
        self.declare_parameter("opencv_device", 0)

        image_topic = str(self.get_parameter("image_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.backend = str(self.get_parameter("backend").value).lower()
        fps = float(self.get_parameter("fps").value)

        self.publisher_ = self.create_publisher(Image, image_topic, 10)
        self.bridge = CvBridge()
        self.picam2 = None
        self.cap = None

        if self.backend == "picamera2":
            try:
                self._start_picamera2()
            except Exception as exc:
                self.get_logger().error(
                    "Picamera2 초기화 실패. CSI 카메라라면 python3-picamera2 설치를 확인하세요: "
                    f"{exc}"
                )
                raise
        elif self.backend == "opencv":
            self._start_opencv()
        else:
            raise ValueError("backend must be 'picamera2' or 'opencv'")

        self.timer = self.create_timer(1.0 / max(fps, 1.0), self.timer_callback)
        self.get_logger().info(
            f"카메라 노드 활성화: backend={self.backend}, topic={image_topic}, "
            f"size={self.width}x{self.height}, fps={fps:.1f}"
        )

    def _start_picamera2(self) -> None:
        from picamera2 import Picamera2

        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"}
        )
        self.picam2.configure(config)
        self.picam2.start()

    def _start_opencv(self) -> None:
        device = int(self.get_parameter("opencv_device").value)
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if not self.cap.isOpened():
            raise RuntimeError(f"OpenCV camera device cannot be opened: /dev/video{device}")

    def timer_callback(self) -> None:
        frame = self._read_frame()
        if frame is None:
            self.get_logger().warning("프레임 읽기 실패")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher_.publish(msg)

    def _read_frame(self):
        if self.picam2 is not None:
            rgb_frame = self.picam2.capture_array()
            return cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)

        if self.cap is not None:
            ret, frame = self.cap.read()
            return frame if ret else None

        return None

    def destroy_node(self) -> bool:
        if self.picam2 is not None:
            self.picam2.stop()
            self.picam2.close()
        if self.cap is not None:
            self.cap.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RPiCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
