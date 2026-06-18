import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RPiCameraNode(Node):
    def __init__(self):
        super().__init__('rpi_camera_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # 중요: libcamera가 이미 하드웨어를 제어 중이므로, 
        # v4l2 장치를 통해 OpenCV가 영상을 읽게 함
        self.cap = cv2.VideoCapture(0)
        
        # 해상도 강제 지정
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info('카메라 노드 활성화: ROS 2 데이터 송출 시작')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # OpenCV 형식(BGR)을 ROS 2 이미지 메시지로 변환
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn("프레임 읽기 실패!")

def main(args=None):
    rclpy.init(args=args)
    node = RPiCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()
