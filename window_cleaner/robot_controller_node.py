from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from std_msgs.msg import Bool

from window_cleaner.arduino_imu import map_delta_to_robot


class RobotControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_controller_node")

        self.declare_parameter("target_topic", "/robot/target_pose")
        self.declare_parameter("motor_command_topic", "/arduino/motor_command")
        self.declare_parameter("position_tolerance_cm", 0.5)
        self.declare_parameter("imu_pose_topic", "/robot/imu_pose")
        self.declare_parameter("use_imu_heading", True)
        self.declare_parameter("adhesion_topic", "/adhesion/secure")
        self.declare_parameter("require_adhesion", True)

        target_topic = self.get_parameter("target_topic").value
        motor_command_topic = self.get_parameter("motor_command_topic").value
        imu_pose_topic = self.get_parameter("imu_pose_topic").value
        adhesion_topic = self.get_parameter("adhesion_topic").value

        self.current_pose = Pose2D()
        self.imu_yaw = 0.0
        self.has_imu_heading = False
        self.adhesion_secure = False
        self.motor_command_publisher = self.create_publisher(Pose2D, motor_command_topic, 10)
        self.subscription = self.create_subscription(Pose2D, target_topic, self.on_target, 10)
        self.imu_subscription = self.create_subscription(Pose2D, imu_pose_topic, self.on_imu_pose, 10)
        self.adhesion_subscription = self.create_subscription(Bool, adhesion_topic, self.on_adhesion, 10)

        self.get_logger().info(f"Listening for target coordinates: {target_topic}")
        self.get_logger().info(f"Publishing Arduino motor commands: {motor_command_topic}")
        self.get_logger().info(f"Using Arduino IMU heading from: {imu_pose_topic}")
        self.get_logger().info(f"Movement safety requires adhesion from: {adhesion_topic}")
        self.get_logger().info("Motor command contract: Pose2D.x=dx_cm, y=dy_cm, theta=clean_flag")

    def on_imu_pose(self, msg: Pose2D) -> None:
        self.imu_yaw = msg.theta
        self.has_imu_heading = True

    def on_adhesion(self, msg: Bool) -> None:
        self.adhesion_secure = msg.data

    def on_target(self, msg: Pose2D) -> None:
        if bool(self.get_parameter("require_adhesion").value) and not self.adhesion_secure:
            self.get_logger().error("Movement blocked: pressure sensor reports insufficient adhesion.")
            return

        map_dx = msg.x - self.current_pose.x
        map_dy = msg.y - self.current_pose.y
        tolerance = float(self.get_parameter("position_tolerance_cm").value)

        if abs(map_dx) <= tolerance and abs(map_dy) <= tolerance:
            self.get_logger().info("Target is already within tolerance.")
            return

        use_imu = bool(self.get_parameter("use_imu_heading").value) and self.has_imu_heading
        if use_imu:
            dx, dy = map_delta_to_robot(map_dx, map_dy, self.imu_yaw)
        else:
            dx, dy = map_dx, map_dy

        command = Pose2D()
        command.x = dx
        command.y = dy
        command.theta = 1.0  # Arduino side can treat theta > 0 as cleaning after movement.
        self.motor_command_publisher.publish(command)

        self.get_logger().info(
            f"Published motor command: robot_dx={dx:.2f} cm, robot_dy={dy:.2f} cm, "
            f"imu_heading={'used' if use_imu else 'not available'}, clean=1"
        )
        self.current_pose.x = msg.x
        self.current_pose.y = msg.y


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RobotControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
