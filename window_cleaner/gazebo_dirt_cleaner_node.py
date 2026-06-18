from __future__ import annotations

import rclpy
from gazebo_msgs.srv import DeleteEntity, DeleteModel
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from std_msgs.msg import Int32, String

from window_cleaner.gazebo_dirt_cleaner import DEFAULT_SIM_DIRT_DOTS, SimDirtDot, dirt_under_robot_footprint


class GazeboDirtCleanerNode(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_dirt_cleaner_node")

        self.declare_parameter("robot_pose_topic", "/robot_pose")
        self.declare_parameter("remaining_count_topic", "/dirt/sim_remaining_count")
        self.declare_parameter("cleaned_topic", "/dirt/sim_cleaned")
        self.declare_parameter("clean_half_width_cm", 15.0)
        self.declare_parameter("clean_half_height_cm", 15.0)
        self.declare_parameter("delete_entity_service", "")
        self.declare_parameter("delete_model_service", "")
        self.declare_parameter("require_delete_service_ready", True)

        self.remaining: dict[str, SimDirtDot] = {dot.name: dot for dot in DEFAULT_SIM_DIRT_DOTS}
        self.delete_wait_logged = False
        self.delete_entity_clients = self._create_delete_entity_clients()
        self.delete_model_clients = self._create_delete_model_clients()
        self.remaining_pub = self.create_publisher(Int32, str(self.get_parameter("remaining_count_topic").value), 10)
        self.cleaned_pub = self.create_publisher(String, str(self.get_parameter("cleaned_topic").value), 10)
        self.pose_sub = self.create_subscription(
            Pose2D, str(self.get_parameter("robot_pose_topic").value), self.on_robot_pose, 10
        )
        self.timer = self.create_timer(1.0, self.publish_remaining_count)

        self.get_logger().info(
            f"Gazebo dirt cleaner active: dots={len(self.remaining)}, "
            f"footprint={float(self.get_parameter('clean_half_width_cm').value) * 2:.1f}x"
            f"{float(self.get_parameter('clean_half_height_cm').value) * 2:.1f}cm"
        )
        self.get_logger().info("Gazebo DeleteEntity candidates: " + ", ".join(self.delete_entity_clients.keys()))
        self.get_logger().info("Gazebo DeleteModel candidates: " + ", ".join(self.delete_model_clients.keys()))

    def _create_delete_entity_clients(self):
        configured = str(self.get_parameter("delete_entity_service").value).strip()
        service_names = [configured] if configured else ["/gazebo/delete_entity", "/delete_entity"]
        return {name: self.create_client(DeleteEntity, name) for name in service_names}

    def _create_delete_model_clients(self):
        configured = str(self.get_parameter("delete_model_service").value).strip()
        service_names = [configured] if configured else ["/gazebo/delete_model", "/delete_model"]
        return {name: self.create_client(DeleteModel, name) for name in service_names}

    def on_robot_pose(self, msg: Pose2D) -> None:
        if not self.remaining:
            self.publish_remaining_count()
            return
        if bool(self.get_parameter("require_delete_service_ready").value) and not self._delete_service_ready():
            if not self.delete_wait_logged:
                self.get_logger().info(
                    "Waiting for Gazebo delete service before cleaning visual dirt models."
                )
                self.delete_wait_logged = True
            self.publish_remaining_count()
            return

        cleaned = dirt_under_robot_footprint(
            robot_x_cm=float(msg.x),
            robot_y_cm=float(msg.y),
            dots=tuple(self.remaining.values()),
            half_width_cm=float(self.get_parameter("clean_half_width_cm").value),
            half_height_cm=float(self.get_parameter("clean_half_height_cm").value),
        )
        for dot in cleaned:
            self.remaining.pop(dot.name, None)
            self._delete_gazebo_model(dot.name)
            cleaned_msg = String()
            cleaned_msg.data = dot.name
            self.cleaned_pub.publish(cleaned_msg)
            self.get_logger().info(
                f"Cleaned simulated dirt: {dot.name} at ({dot.x_cm:.1f}, {dot.y_cm:.1f}) cm; "
                f"remaining={len(self.remaining)}"
            )
        if cleaned:
            self.publish_remaining_count()

    def _delete_service_ready(self) -> bool:
        return any(client.service_is_ready() for client in self.delete_entity_clients.values()) or any(
            client.service_is_ready() for client in self.delete_model_clients.values()
        )

    def _delete_gazebo_model(self, name: str) -> None:
        for service_name, client in self.delete_entity_clients.items():
            if client.service_is_ready():
                request = DeleteEntity.Request()
                request.name = name
                client.call_async(request)
                self.get_logger().info(f"Requested Gazebo DeleteEntity via {service_name}: {name}")
                return
        for service_name, client in self.delete_model_clients.items():
            if client.service_is_ready():
                request = DeleteModel.Request()
                request.model_name = name
                client.call_async(request)
                self.get_logger().info(f"Requested Gazebo DeleteModel via {service_name}: {name}")
                return
        self.get_logger().warning(
            f"Could not delete {name} yet: Gazebo delete service is not ready. It will remain hidden from cleaner state only."
        )

    def publish_remaining_count(self) -> None:
        msg = Int32()
        msg.data = len(self.remaining)
        self.remaining_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GazeboDirtCleanerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
