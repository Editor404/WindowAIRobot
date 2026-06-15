from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Int32

from window_cleaner.arduino_imu import GyroYawEstimator, parse_sensor_line


class ArduinoSensorBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("arduino_sensor_bridge")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("frame_id", "camera_imu_link")
        self.declare_parameter("camera_map_x_cm", 15.0)
        self.declare_parameter("camera_map_y_cm", 30.0)
        self.declare_parameter("initial_yaw_deg", 0.0)
        self.declare_parameter("gyro_z_bias_dps", 0.0)

        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("pyserial is required: install python3-serial or pyserial") from exc

        port = str(self.get_parameter("port").value)
        baud = int(self.get_parameter("baud").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.camera_x_cm = float(self.get_parameter("camera_map_x_cm").value)
        self.camera_y_cm = float(self.get_parameter("camera_map_y_cm").value)
        self.yaw_estimator = GyroYawEstimator(
            initial_yaw_radians=math.radians(float(self.get_parameter("initial_yaw_deg").value)),
            z_bias_dps=float(self.get_parameter("gyro_z_bias_dps").value),
        )

        self.pressure_publisher = self.create_publisher(Int32, "/pressure/raw", 10)
        self.adhesion_publisher = self.create_publisher(Bool, "/adhesion/secure", 10)
        self.imu_publisher = self.create_publisher(Imu, "/gyro/data", 10)
        self.pose_publisher = self.create_publisher(Pose2D, "/robot/imu_pose", 10)

        try:
            self.serial = serial.Serial(port, baud, timeout=0)
        except serial.SerialException as exc:
            raise RuntimeError(f"Cannot open Arduino serial port {port}: {exc}") from exc

        self.receive_buffer = bytearray()
        self.timer = self.create_timer(0.005, self.read_serial)
        self.get_logger().info(
            f"Arduino IMU: {port} at {baud} baud; camera map position="
            f"({self.camera_x_cm:.1f}, {self.camera_y_cm:.1f}) cm"
        )

    def read_serial(self) -> None:
        waiting = self.serial.in_waiting
        if waiting <= 0:
            return
        self.receive_buffer.extend(self.serial.read(waiting))

        while b"\n" in self.receive_buffer:
            raw_line, _, remaining = self.receive_buffer.partition(b"\n")
            self.receive_buffer = bytearray(remaining)
            line = raw_line.decode("ascii", errors="replace").strip()
            try:
                sample = parse_sensor_line(line)
            except ValueError:
                self.get_logger().warning(f"Ignoring malformed Arduino line: {line}")
                continue
            if sample is None:
                continue

            pressure = Int32()
            pressure.data = sample.pressure_raw
            self.pressure_publisher.publish(pressure)
            if sample.adhesion_secure is not None:
                adhesion = Bool()
                adhesion.data = sample.adhesion_secure
                self.adhesion_publisher.publish(adhesion)
            if not sample.gyro_valid:
                continue

            yaw = self.yaw_estimator.update(sample.timestamp_ms, sample.gyro_z_dps)
            self.publish_imu(sample.gyro_x_dps, sample.gyro_y_dps, sample.gyro_z_dps, yaw)
            self.publish_camera_pose(yaw)

    def publish_imu(self, x_dps: float, y_dps: float, z_dps: float, yaw: float) -> None:
        message = Imu()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.orientation.z = math.sin(yaw / 2.0)
        message.orientation.w = math.cos(yaw / 2.0)
        message.orientation_covariance = [0.0] * 9
        message.orientation_covariance[8] = math.radians(5.0) ** 2
        message.angular_velocity.x = math.radians(x_dps)
        message.angular_velocity.y = math.radians(y_dps)
        message.angular_velocity.z = math.radians(z_dps)
        message.linear_acceleration_covariance[0] = -1.0
        self.imu_publisher.publish(message)

    def publish_camera_pose(self, yaw: float) -> None:
        pose = Pose2D()
        pose.x = self.camera_x_cm
        pose.y = self.camera_y_cm
        pose.theta = yaw
        self.pose_publisher.publish(pose)

    def destroy_node(self) -> bool:
        if hasattr(self, "serial") and self.serial.is_open:
            self.serial.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ArduinoSensorBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
