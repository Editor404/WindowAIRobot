import argparse

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class VideoReceiver(Node):
    def __init__(self, topic_name: str, width: int, height: int):
        super().__init__('video_receiver')
        self.subscription = self.create_subscription(Image, topic_name, self.listener_callback, 10)
        self.bridge = CvBridge()
        self.topic_name = topic_name
        self.width = width
        self.height = height

        print('--- 영상 수신 노드가 시작되었습니다. ---')
        print(f'topic: {self.topic_name}')
        print(f'image size: {self.width}x{self.height}')
        print("종료하려면 영상 창에서 'q'를 누르세요.")

    def listener_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            cv_image = cv2.resize(cv_image, (self.width, self.height))

            cv2.imshow('Video Receiver', cv_image)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rclpy.shutdown()
        except Exception as e:
            print(f'에러 발생: {e}')


def parse_args():
    parser = argparse.ArgumentParser(description='Receive ROS2 image topic as video stream.')
    parser.add_argument('--topic', default='/camera/image_raw', help='ROS2 image topic name')
    parser.add_argument('--width', type=int, default=640, help='display image width')
    parser.add_argument('--height', type=int, default=480, help='display image height')
    return parser.parse_args()


def main(args=None):
    cli_args = parse_args()

    rclpy.init(args=args)
    node = VideoReceiver(
        topic_name=cli_args.topic,
        width=cli_args.width,
        height=cli_args.height,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
