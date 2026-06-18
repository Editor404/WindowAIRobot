from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Int32

from window_cleaner.arduino_imu import GyroYawEstimator, parse_sensor_line
from window_cleaner.arduino_motion import MotionCalibration, pose2d_to_motion_command


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
        self.declare_parameter("motor_command_topic", "/arduino/motor_command")
        self.declare_parameter("enable_motor_serial_commands", True)
        self.declare_parameter("drive_cm_per_second", 5.0)
        self.declare_parameter("turn_rad_per_second", 0.5)
        self.declare_parameter("max_motion_seconds", 2.0)
        self.declare_parameter("min_motion_seconds", 0.1)
        self.declare_parameter("lateral_tolerance_cm", 0.5)

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
        self.stop_deadline_ns: int | None = None
        self.motor_serial_enabled = bool(self.get_parameter("enable_motor_serial_commands").value)

        try:
            self.serial = serial.Serial(port, baud, timeout=0)
        except serial.SerialException as exc:
            raise RuntimeError(f"Cannot open Arduino serial port {port}: {exc}") from exc

        self.receive_buffer = bytearray()
        self.timer = self.create_timer(0.005, self.read_serial)
        if self.motor_serial_enabled:
            self.motor_subscription = self.create_subscription(
                Pose2D,
                str(self.get_parameter("motor_command_topic").value),
                self.on_motor_command,
                10,
            )
        self.get_logger().info(
            f"Arduino IMU: {port} at {baud} baud; camera map position="
            f"({self.camera_x_cm:.1f}, {self.camera_y_cm:.1f}) cm"
        )
        if self.motor_serial_enabled:
            self.get_logger().info(
                "Forwarding /arduino/motor_command to Arduino serial; "
                "calibrate drive_cm_per_second and turn_rad_per_second before free driving."
            )

    def on_motor_command(self, msg: Pose2D) -> None:
        calibration = MotionCalibration(
            drive_cm_per_second=float(self.get_parameter("drive_cm_per_second").value),
            turn_rad_per_second=float(self.get_parameter("turn_rad_per_second").value),
            max_motion_seconds=float(self.get_parameter("max_motion_seconds").value),
            min_motion_seconds=float(self.get_parameter("min_motion_seconds").value),
            lateral_tolerance_cm=float(self.get_parameter("lateral_tolerance_cm").value),
        )
        command = pose2d_to_motion_command(msg.x, msg.y, msg.theta, calibration)
        self.write_arduino_line(command.serial_line)
        if command.duration_s > 0.0:
            self.stop_deadline_ns = self.get_clock().now().nanoseconds + int(command.duration_s * 1_000_000_000)
            self.get_logger().info(
                f"Arduino motion: {command.reason} -> {command.serial_line!r} for {command.duration_s:.2f}s "
                f"from Pose2D(x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f})"
            )
        else:
            self.stop_deadline_ns = None
            if command.reason == "lateral_command_rejected":
                self.get_logger().error(
                    f"Rejected lateral motor command for real tracked robot: y={msg.y:.2f} cm. "
                    "Run robot_controller_node with use_target_heading:=true."
                )
            else:
                self.get_logger().info("Arduino motion: stop")

    def write_arduino_line(self, line: str) -> None:
        self.serial.write((line.strip() + "\n").encode("ascii"))

    def read_serial(self) -> None:
        self.stop_expired_motion()
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

    def stop_expired_motion(self) -> None:
        if self.stop_deadline_ns is None:
            return
        if self.get_clock().now().nanoseconds < self.stop_deadline_ns:
            return
        self.write_arduino_line("x")
        self.stop_deadline_ns = None
        self.get_logger().info("Arduino motion: timed stop sent")

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
