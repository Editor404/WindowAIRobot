from __future__ import annotations

import subprocess

import cv2
import numpy as np
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
        self.declare_parameter("backend", "auto")  # auto | rpicam | picamera2 | opencv
        self.declare_parameter("opencv_device", 0)
        self.declare_parameter("rpicam_executable", "rpicam-vid")

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
        self.rpicam_process = None
        self.rpicam_frame_bytes = self.width * self.height * 3 // 2

        self._start_camera_backend()

        self.timer = self.create_timer(1.0 / max(fps, 1.0), self.timer_callback)
        self.get_logger().info(
            f"카메라 노드 활성화: backend={self.backend}, topic={image_topic}, "
            f"size={self.width}x{self.height}, fps={fps:.1f}"
        )

    def _start_camera_backend(self) -> None:
        if self.backend == "auto":
            try:
                self._start_picamera2()
                self.backend = "picamera2"
                return
            except Exception as exc:
                self.get_logger().warning(f"Picamera2 사용 불가, rpicam-vid로 전환: {exc}")
            try:
                self._start_rpicam()
                self.backend = "rpicam"
                return
            except Exception as exc:
                self.get_logger().warning(f"rpicam-vid 사용 불가, OpenCV로 전환: {exc}")
            self._start_opencv()
            self.backend = "opencv"
            return

        if self.backend == "picamera2":
            self._start_picamera2()
            return
        if self.backend == "rpicam":
            self._start_rpicam()
            return
        if self.backend == "opencv":
            self._start_opencv()
            return
        raise ValueError("backend must be 'auto', 'rpicam', 'picamera2', or 'opencv'")

    def _start_picamera2(self) -> None:
        from picamera2 import Picamera2

        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"}
        )
        self.picam2.configure(config)
        self.picam2.start()

    def _start_rpicam(self) -> None:
        executable = str(self.get_parameter("rpicam_executable").value)
        fps = float(self.get_parameter("fps").value)
        command = [
            executable,
            "-n",
            "--codec",
            "yuv420",
            "--width",
            str(self.width),
            "--height",
            str(self.height),
            "--framerate",
            str(max(int(fps), 1)),
            "--timeout",
            "0",
            "-o",
            "-",
        ]
        self.rpicam_process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=self.rpicam_frame_bytes * 2,
        )
        if self.rpicam_process.stdout is None:
            raise RuntimeError("rpicam-vid stdout pipe was not created")

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

        if self.rpicam_process is not None and self.rpicam_process.stdout is not None:
            raw = self.rpicam_process.stdout.read(self.rpicam_frame_bytes)
            if len(raw) != self.rpicam_frame_bytes:
                return None
            yuv = np.frombuffer(raw, dtype=np.uint8).reshape((self.height * 3 // 2, self.width))
            return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)

        if self.cap is not None:
            ret, frame = self.cap.read()
            return frame if ret else None

        return None

    def destroy_node(self) -> bool:
        if self.picam2 is not None:
            self.picam2.stop()
            self.picam2.close()
        if self.rpicam_process is not None:
            self.rpicam_process.terminate()
            try:
                self.rpicam_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.rpicam_process.kill()
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
