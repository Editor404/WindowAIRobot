from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Pose2D, PoseArray
from rclpy.node import Node
from std_msgs.msg import Int32, String

from window_cleaner.window_cleaning_planner import (
    CameraProjectionConfig,
    DirtMemoryMap,
    LawnMowerScanPlanner,
    MapPoint,
    PixelDetection,
    clamp_to_window,
    distance_cm,
    project_pixel_detection_to_window,
)


class WindowCleaningPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("window_cleaning_planner_node")

        self.declare_parameter("window_width_cm", 80.0)
        self.declare_parameter("window_height_cm", 160.0)
        self.declare_parameter("robot_pose_topic", "/robot_pose")
        self.declare_parameter("detections_px_topic", "/dirt/detections_px")
        self.declare_parameter("target_topic", "/robot/target_pose")
        self.declare_parameter("map_topic", "/window/map")
        self.declare_parameter("require_map_initialization", False)
        self.declare_parameter("memory_count_topic", "/dirt/memory_count")
        self.declare_parameter("planner_state_topic", "/cleaning/state")
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("scan_margin_cm", 15.0)
        self.declare_parameter("scan_stripe_spacing_cm", 25.0)
        self.declare_parameter("scan_reached_tolerance_cm", 8.0)
        self.declare_parameter("clean_reached_radius_cm", 6.0)
        self.declare_parameter("merge_radius_cm", 5.0)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("cm_per_pixel_x", 0.05)
        self.declare_parameter("cm_per_pixel_y", 0.05)
        self.declare_parameter("camera_offset_x_cm", 0.0)
        self.declare_parameter("camera_offset_y_cm", 0.0)
        self.declare_parameter("home_x_cm", 15.0)
        self.declare_parameter("home_y_cm", 15.0)
        self.declare_parameter("home_yaw_rad", 0.0)
        self.declare_parameter("search_turn_step_rad", 0.35)
        self.declare_parameter("search_total_turn_rad", 6.283185307179586)

        self.window_width_cm = float(self.get_parameter("window_width_cm").value)
        self.window_height_cm = float(self.get_parameter("window_height_cm").value)
        self.map_initialized = not bool(self.get_parameter("require_map_initialization").value)
        self.robot_pose = MapPoint(0.0, 0.0)
        self.robot_yaw = 0.0
        self.home_pose = MapPoint(
            float(self.get_parameter("home_x_cm").value),
            float(self.get_parameter("home_y_cm").value),
        )
        self.home_yaw = float(self.get_parameter("home_yaw_rad").value)
        self.memory = DirtMemoryMap(merge_radius_cm=float(self.get_parameter("merge_radius_cm").value))
        self.scan = LawnMowerScanPlanner(
            window_width_cm=self.window_width_cm,
            window_height_cm=self.window_height_cm,
            margin_cm=float(self.get_parameter("scan_margin_cm").value),
            stripe_spacing_cm=float(self.get_parameter("scan_stripe_spacing_cm").value),
        )
        self.projection = CameraProjectionConfig(
            image_width_px=int(self.get_parameter("image_width").value),
            image_height_px=int(self.get_parameter("image_height").value),
            cm_per_pixel_x=float(self.get_parameter("cm_per_pixel_x").value),
            cm_per_pixel_y=float(self.get_parameter("cm_per_pixel_y").value),
            camera_offset_x_cm=float(self.get_parameter("camera_offset_x_cm").value),
            camera_offset_y_cm=float(self.get_parameter("camera_offset_y_cm").value),
        )
        self.mode = "scan" if self.map_initialized else "wait_map"
        self.search_accumulated_rad = 0.0

        self.target_pub = self.create_publisher(Pose2D, str(self.get_parameter("target_topic").value), 10)
        self.memory_count_pub = self.create_publisher(Int32, str(self.get_parameter("memory_count_topic").value), 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("planner_state_topic").value), 10)
        self.pose_sub = self.create_subscription(
            Pose2D, str(self.get_parameter("robot_pose_topic").value), self.on_robot_pose, 10
        )
        self.map_sub = self.create_subscription(
            Pose2D, str(self.get_parameter("map_topic").value), self.on_window_map, 10
        )
        self.detections_sub = self.create_subscription(
            PoseArray, str(self.get_parameter("detections_px_topic").value), self.on_detections, 10
        )
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate_hz, 0.1), self.on_timer)

        self.get_logger().info(
            "Window cleaning planner active: "
            f"mode={self.mode}, map={self.window_width_cm:.1f}x{self.window_height_cm:.1f}cm, "
            f"scan_waypoints={len(self.scan.waypoints)}"
        )

    def on_window_map(self, msg: Pose2D) -> None:
        if msg.x <= 0.0 or msg.y <= 0.0:
            self.get_logger().warning(f"Ignoring invalid window map: width={msg.x}, height={msg.y}")
            return
        self.window_width_cm = float(msg.x)
        self.window_height_cm = float(msg.y)
        self.scan = LawnMowerScanPlanner(
            window_width_cm=self.window_width_cm,
            window_height_cm=self.window_height_cm,
            margin_cm=float(self.get_parameter("scan_margin_cm").value),
            stripe_spacing_cm=float(self.get_parameter("scan_stripe_spacing_cm").value),
        )
        self.map_initialized = True
        if self.mode == "wait_map":
            self.mode = "scan"
        self.get_logger().info(
            f"Window map initialized from camera: {self.window_width_cm:.1f}x{self.window_height_cm:.1f}cm, "
            f"confidence={msg.theta:.2f}, scan_waypoints={len(self.scan.waypoints)}"
        )

    def on_robot_pose(self, msg: Pose2D) -> None:
        self.robot_pose = MapPoint(float(msg.x), float(msg.y))
        self.robot_yaw = float(msg.theta)
        cleaned = self.memory.mark_near_cleaned(
            self.robot_pose,
            radius_cm=float(self.get_parameter("clean_reached_radius_cm").value),
        )
        if cleaned:
            self.get_logger().info(f"Marked cleaned dirt points: {cleaned}")

    def on_detections(self, msg: PoseArray) -> None:
        added_before = len(self.memory.points)
        for pose in msg.poses:
            pixel = PixelDetection(x_px=pose.position.x, y_px=pose.position.y, area_px=pose.position.z)
            point = project_pixel_detection_to_window(pixel, self.robot_pose, self.projection)
            point = clamp_to_window(point, self.window_width_cm, self.window_height_cm)
            self.memory.add_observation(point)
        if len(self.memory.points) != added_before:
            if self.mode == "search_rotate":
                self.mode = "clean"
                self.get_logger().info("New dirt found during 360 search; switching back to cleaning mode.")
            self.get_logger().info(
                f"Dirt memory updated: total={len(self.memory.points)}, uncleaned={len(self.memory.uncleaned())}"
            )

    def on_timer(self) -> None:
        if not self.map_initialized:
            self._publish_status()
            return
        self.scan.advance_if_reached(
            self.robot_pose,
            tolerance_cm=float(self.get_parameter("scan_reached_tolerance_cm").value),
        )
        if self.mode == "scan" and self.scan.is_complete():
            self.mode = "clean"
            self.get_logger().info("Scan complete; switching to nearest-dirt cleaning mode.")

        if self.mode == "clean" and not self.memory.uncleaned():
            self.mode = "search_rotate"
            self.search_accumulated_rad = 0.0
            self.get_logger().info("No remembered dirt remains; starting 360-degree search.")

        target = self._target_for_current_mode()
        self._publish_status()
        if target is None:
            return
        self._publish_target(target)

    def _target_for_current_mode(self) -> tuple[MapPoint, float] | None:
        if self.mode == "wait_map":
            return None
        if self.mode == "scan":
            target = self.scan.current_target()
            if target is None:
                return None
            return target, heading_to_target(self.robot_pose, target)
        if self.mode == "clean":
            nearest = self.memory.nearest_uncleaned(self.robot_pose)
            if nearest is None:
                return None
            target = nearest.point()
            return target, heading_to_target(self.robot_pose, target)
        if self.mode == "search_rotate":
            step = float(self.get_parameter("search_turn_step_rad").value)
            total = float(self.get_parameter("search_total_turn_rad").value)
            if self.search_accumulated_rad >= total:
                self.mode = "return_home"
                self.get_logger().info("360-degree search complete; no dirt found, returning home.")
                return self.home_pose, self.home_yaw
            self.search_accumulated_rad += step
            return self.robot_pose, normalize_angle(self.robot_yaw + step)
        if self.mode == "return_home":
            if distance_cm(self.robot_pose, self.home_pose) <= float(self.get_parameter("scan_reached_tolerance_cm").value):
                return self.home_pose, self.home_yaw
            return self.home_pose, heading_to_target(self.robot_pose, self.home_pose)
        return None

    def _publish_target(self, target_with_heading: tuple[MapPoint, float]) -> None:
        target, yaw = target_with_heading
        msg = Pose2D()
        msg.x = target.x_cm
        msg.y = target.y_cm
        msg.theta = yaw
        self.target_pub.publish(msg)
        self.get_logger().info(
            f"Published target: mode={self.mode}, x={msg.x:.2f}, y={msg.y:.2f}, yaw={msg.theta:.2f}"
        )

    def _publish_status(self) -> None:
        count = Int32()
        count.data = len(self.memory.uncleaned())
        self.memory_count_pub.publish(count)

        state = String()
        state.data = (
            f"mode={self.mode}, map_initialized={self.map_initialized}, scan_complete={self.scan.is_complete()}, "
            f"search={self.search_accumulated_rad:.2f}rad, "
            f"memory_total={len(self.memory.points)}, uncleaned={len(self.memory.uncleaned())}"
        )
        self.state_pub.publish(state)



def heading_to_target(robot: MapPoint, target: MapPoint) -> float:
    """Return heading where 0 rad points up the window (+Y)."""
    return math.atan2(target.x_cm - robot.x_cm, target.y_cm - robot.y_cm)


def normalize_angle(angle_rad: float) -> float:
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WindowCleaningPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
