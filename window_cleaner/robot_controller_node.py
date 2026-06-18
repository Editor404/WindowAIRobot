from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from std_msgs.msg import Bool

from window_cleaner.arduino_imu import map_delta_to_robot


class RobotControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_controller_node")

        self.declare_parameter("target_topic", "/robot/target_pose")
        self.declare_parameter("robot_pose_topic", "/robot_pose")
        self.declare_parameter("motor_command_topic", "/arduino/motor_command")
        self.declare_parameter("position_tolerance_cm", 0.5)
        self.declare_parameter("imu_pose_topic", "/robot/imu_pose")
        self.declare_parameter("use_imu_heading", True)
        self.declare_parameter("use_target_heading", False)
        self.declare_parameter("heading_tolerance_rad", 0.05)
        self.declare_parameter("adhesion_topic", "/adhesion/secure")
        self.declare_parameter("require_adhesion", True)

        target_topic = self.get_parameter("target_topic").value
        robot_pose_topic = self.get_parameter("robot_pose_topic").value
        motor_command_topic = self.get_parameter("motor_command_topic").value
        imu_pose_topic = self.get_parameter("imu_pose_topic").value
        adhesion_topic = self.get_parameter("adhesion_topic").value

        self.current_pose = Pose2D()
        self.imu_yaw = 0.0
        self.has_imu_heading = False
        self.adhesion_secure = False
        self.motor_command_publisher = self.create_publisher(Pose2D, motor_command_topic, 10)
        self.subscription = self.create_subscription(Pose2D, target_topic, self.on_target, 10)
        self.pose_subscription = self.create_subscription(Pose2D, robot_pose_topic, self.on_robot_pose, 10)
        self.imu_subscription = self.create_subscription(Pose2D, imu_pose_topic, self.on_imu_pose, 10)
        self.adhesion_subscription = self.create_subscription(Bool, adhesion_topic, self.on_adhesion, 10)

        self.get_logger().info(f"Listening for target coordinates: {target_topic}")
        self.get_logger().info(f"Listening for robot pose feedback: {robot_pose_topic}")
        self.get_logger().info(f"Publishing Arduino motor commands: {motor_command_topic}")
        self.get_logger().info(f"Using Arduino IMU heading from: {imu_pose_topic}")
        self.get_logger().info(f"Movement safety requires adhesion from: {adhesion_topic}")
        self.get_logger().info("Motor command contract: Pose2D.x=dx_cm, y=dy_cm, theta=clean_flag")

    def on_robot_pose(self, msg: Pose2D) -> None:
        self.current_pose.x = msg.x
        self.current_pose.y = msg.y
        self.current_pose.theta = msg.theta

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
        use_target_heading = bool(self.get_parameter("use_target_heading").value)
        heading_tolerance = float(self.get_parameter("heading_tolerance_rad").value)

        if use_target_heading:
            command_values = compute_wheel_navigation_command(
                map_dx=map_dx,
                map_dy=map_dy,
                current_heading=self.current_pose.theta,
                target_heading=msg.theta,
                position_tolerance_cm=tolerance,
                heading_tolerance_rad=heading_tolerance,
            )
            if command_values is None:
                self.get_logger().info("Target is already within tolerance.")
                return

            command = Pose2D()
            command.x, command.y, command.theta = command_values
            self.motor_command_publisher.publish(command)
            if abs(command.x) <= 1e-6 and abs(command.y) <= 1e-6:
                self.get_logger().info(f"Published wheel turn command: dtheta={command.theta:.2f} rad")
            else:
                self.get_logger().info(f"Published wheel forward command: distance={command.x:.2f} cm")
            return

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
        command.theta = 1.0
        self.motor_command_publisher.publish(command)

        self.get_logger().info(
            f"Published motor command: robot_dx={dx:.2f} cm, robot_dy={dy:.2f} cm, "
            f"imu_heading={'used' if use_imu else 'not available'}, clean=1"
        )



def heading_to_target(map_dx: float, map_dy: float) -> float:
    """Return heading where 0 rad is straight up along the window (+Y)."""
    return math.atan2(map_dx, map_dy)


def compute_wheel_navigation_command(
    map_dx: float,
    map_dy: float,
    current_heading: float,
    target_heading: float,
    position_tolerance_cm: float,
    heading_tolerance_rad: float,
) -> tuple[float, float, float] | None:
    """Convert a map target into turn-in-place or forward-only wheel commands.

    Pose2D command contract for the simulator/Arduino bridge:
    - x > 0, y = 0, theta = 0 means drive forward along the current heading.
    - x = 0, y = 0, theta != 0 means rotate in place left/right.

    This prevents side-slip commands: a target to the side is reached by first
    turning toward it, then driving forward.
    """
    distance = math.hypot(map_dx, map_dy)
    if distance <= position_tolerance_cm:
        heading_error = normalize_angle(target_heading - current_heading)
        if abs(heading_error) > heading_tolerance_rad:
            return 0.0, 0.0, heading_error
        return None

    desired_heading = heading_to_target(map_dx, map_dy)
    heading_error = normalize_angle(desired_heading - current_heading)
    if abs(heading_error) > heading_tolerance_rad:
        return 0.0, 0.0, heading_error
    return distance, 0.0, 0.0


def normalize_angle(angle_rad: float) -> float:
    import math

    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad

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
