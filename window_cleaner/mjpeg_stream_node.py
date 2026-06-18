from __future__ import annotations

import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class SharedJpegFrame:
    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.frame: bytes | None = None
        self.sequence = 0

    def update(self, frame: bytes) -> None:
        with self.condition:
            self.frame = frame
            self.sequence += 1
            self.condition.notify_all()

    def wait_next(self, previous_sequence: int, timeout: float = 2.0) -> tuple[int, bytes | None]:
        with self.condition:
            if self.sequence == previous_sequence:
                self.condition.wait(timeout=timeout)
            return self.sequence, self.frame


class MjpegRequestHandler(BaseHTTPRequestHandler):
    server_version = "WindowCleanerMJPEG/0.1"

    def do_GET(self) -> None:  # noqa: N802 - stdlib handler API
        if self.path in {"/", "/index.html"}:
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(
                b"<html><head><title>Window Cleaner Camera</title></head>"
                b"<body style='margin:0;background:#111;color:#eee;font-family:sans-serif'>"
                b"<h3 style='margin:8px'>/camera/image_raw</h3>"
                b"<img src='/stream.mjpg' style='max-width:100vw;max-height:90vh'/>"
                b"</body></html>"
            )
            return

        if self.path != "/stream.mjpg":
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        sequence = 0
        shared_frame: SharedJpegFrame = self.server.shared_frame  # type: ignore[attr-defined]
        while True:
            sequence, jpeg = shared_frame.wait_next(sequence)
            if jpeg is None:
                continue
            try:
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                break

    def log_message(self, format: str, *args) -> None:  # noqa: A002 - stdlib handler API
        return


class MjpegStreamNode(Node):
    def __init__(self) -> None:
        super().__init__("mjpeg_stream")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("jpeg_quality", 80)

        self.bridge = CvBridge()
        self.shared_frame = SharedJpegFrame()
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        image_topic = str(self.get_parameter("image_topic").value)
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)

        self.subscription = self.create_subscription(Image, image_topic, self.on_image, 10)
        self.server = ThreadingHTTPServer((host, port), MjpegRequestHandler)
        self.server.shared_frame = self.shared_frame  # type: ignore[attr-defined]
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        self.get_logger().info(f"MJPEG stream: http://{host}:{port}/  topic={image_topic}")

    def on_image(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if ok:
            self.shared_frame.update(encoded.tobytes())

    def destroy_node(self) -> bool:
        self.server.shutdown()
        self.server.server_close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MjpegStreamNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
