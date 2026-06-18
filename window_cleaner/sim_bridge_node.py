from __future__ import annotations

import math

import rclpy
from gazebo_msgs.msg import EntityState, ModelState
from gazebo_msgs.srv import SetEntityState, SetModelState
from geometry_msgs.msg import Pose2D, Quaternion
from rclpy.node import Node
from std_msgs.msg import Bool


class SimBridgeNode(Node):
    """Bridge Arduino-style commands to ROS pose topics and a Gazebo model.

    The real robot receives `/arduino/motor_command` from `robot_controller_node`.
    In Gazebo there is no Arduino, so this node consumes that command, updates a
    simple cm-based simulated pose, publishes the pose/adhesion topics expected by
    the rest of the stack, and optionally moves the spawned Gazebo entity.
    """

    def __init__(self) -> None:
        super().__init__("sim_bridge_node")

        self.declare_parameter("motor_command_topic", "/arduino/motor_command")
        self.declare_parameter("robot_pose_topic", "/robot_pose")
        self.declare_parameter("imu_pose_topic", "/robot/imu_pose")
        self.declare_parameter("adhesion_topic", "/adhesion/secure")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("max_step_cm", 1.0)
        self.declare_parameter("max_turn_rad", 0.08)
        self.declare_parameter("heading_tolerance_rad", 0.05)
        self.declare_parameter("window_width_cm", 80.0)
        self.declare_parameter("window_height_cm", 160.0)
        self.declare_parameter("robot_half_width_cm", 15.0)
        self.declare_parameter("robot_half_height_cm", 15.0)
        self.declare_parameter("initial_x_cm", 15.0)
        self.declare_parameter("initial_y_cm", 15.0)
        self.declare_parameter("initial_yaw_rad", 0.0)
        self.declare_parameter("camera_heading_offset_rad", 0.0)
        self.declare_parameter("adhesion_secure", True)
        self.declare_parameter("update_gazebo_entity", True)
        self.declare_parameter("gazebo_entity_name", "window_cleaner_robot")
        self.declare_parameter("set_entity_state_service", "")
        # Glass thickness is 2 cm, so the +X glass face is X=+1 cm.
        # STL visual underside is local z=-6.8886 cm, and the +X glass face is X=+1 cm.
        # Center X=1+6.8886=7.8886 cm keeps the visible robot bottom on the glass.
        self.declare_parameter("gazebo_wall_x_m", 0.078886)
        self.declare_parameter("pose_scale_m_per_cm", 0.01)

        self.pose = Pose2D()
        self.pose.x = float(self.get_parameter("initial_x_cm").value)
        self.pose.y = float(self.get_parameter("initial_y_cm").value)
        self.pose.theta = float(self.get_parameter("initial_yaw_rad").value)
        self.target_pose = Pose2D()
        self.target_pose.x = self.pose.x
        self.target_pose.y = self.pose.y
        self.target_pose.theta = self.pose.theta

        motor_command_topic = str(self.get_parameter("motor_command_topic").value)
        robot_pose_topic = str(self.get_parameter("robot_pose_topic").value)
        imu_pose_topic = str(self.get_parameter("imu_pose_topic").value)
        adhesion_topic = str(self.get_parameter("adhesion_topic").value)

        self.robot_pose_pub = self.create_publisher(Pose2D, robot_pose_topic, 10)
        self.imu_pose_pub = self.create_publisher(Pose2D, imu_pose_topic, 10)
        self.adhesion_pub = self.create_publisher(Bool, adhesion_topic, 10)
        self.command_sub = self.create_subscription(Pose2D, motor_command_topic, self.on_motor_command, 10)
        self.model_state_publishers = {
            "/gazebo/set_model_state": self.create_publisher(ModelState, "/gazebo/set_model_state", 10),
            "/set_model_state": self.create_publisher(ModelState, "/set_model_state", 10),
        }

        self.set_entity_state_clients = self._create_set_entity_state_clients()
        self.set_model_state_clients = self._create_set_model_state_clients()
        self.active_set_entity_state_client = None
        self.active_set_model_state_client = None
        self.pending_gazebo_request = None
        self.gazebo_warning_logged = False

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate_hz, 0.1), self.publish_state)

        self.get_logger().info(f"Sim bridge listening for motor commands: {motor_command_topic}")
        self.get_logger().info(f"Publishing simulated robot pose: {robot_pose_topic}, {imu_pose_topic}")
        self.get_logger().info(f"Publishing simulated adhesion: {adhesion_topic}")
        self.get_logger().info(
            f"Gazebo entity updates: {bool(self.get_parameter('update_gazebo_entity').value)} "
            f"entity={self.get_parameter('gazebo_entity_name').value}"
        )
        self.get_logger().info(
            "Gazebo SetEntityState candidates: "
            + ", ".join(self.set_entity_state_clients.keys())
        )
        self.get_logger().info(
            "Gazebo SetModelState candidates: "
            + ", ".join(self.set_model_state_clients.keys())
        )
        self.get_logger().info(
            "Gazebo ModelState publish topics: "
            + ", ".join(self.model_state_publishers.keys())
        )

    def _create_set_entity_state_clients(self):
        configured_service = str(self.get_parameter("set_entity_state_service").value).strip()
        service_names = [configured_service] if configured_service else ["/set_entity_state", "/gazebo/set_entity_state"]
        return {name: self.create_client(SetEntityState, name) for name in service_names}

    def _create_set_model_state_clients(self):
        # Gazebo Classic commonly exposes SetModelState instead of SetEntityState.
        return {name: self.create_client(SetModelState, name) for name in ["/gazebo/set_model_state", "/set_model_state"]}

    def _ready_set_entity_state_client(self):
        if self.active_set_entity_state_client is not None and self.active_set_entity_state_client.service_is_ready():
            return self.active_set_entity_state_client
        for service_name, client in self.set_entity_state_clients.items():
            if client.service_is_ready():
                if self.active_set_entity_state_client is not client:
                    self.get_logger().info(f"Using Gazebo SetEntityState service: {service_name}")
                self.active_set_entity_state_client = client
                return client
        return None

    def _ready_set_model_state_client(self):
        if self.active_set_model_state_client is not None and self.active_set_model_state_client.service_is_ready():
            return self.active_set_model_state_client
        for service_name, client in self.set_model_state_clients.items():
            if client.service_is_ready():
                if self.active_set_model_state_client is not client:
                    self.get_logger().info(f"Using Gazebo SetModelState service: {service_name}")
                self.active_set_model_state_client = client
                return client
        return None

    def on_motor_command(self, msg: Pose2D) -> None:
        if abs(msg.x) <= 1e-6 and abs(msg.y) <= 1e-6 and abs(msg.theta) > 1e-6:
            self.target_pose.x = self.pose.x
            self.target_pose.y = self.pose.y
            target_camera_heading = normalize_angle(self._camera_heading() + msg.theta)
            self.target_pose.theta = self._base_heading_from_camera_heading(target_camera_heading)
            self.get_logger().info(
                f"Queued simulated turn: dtheta={msg.theta:.2f} rad, target_camera_yaw={target_camera_heading:.2f}"
            )
            return

        yaw = self._camera_heading()
        forward_x = math.sin(yaw)
        forward_y = math.cos(yaw)
        left_x = math.cos(yaw)
        left_y = -math.sin(yaw)
        map_dx = msg.x * forward_x + msg.y * left_x
        map_dy = msg.x * forward_y + msg.y * left_y
        target_x, target_y = self._clamp_robot_center(self.pose.x + map_dx, self.pose.y + map_dy)
        self.target_pose.x = target_x
        self.target_pose.y = target_y
        target_camera_heading = heading_to_target(self.pose.x, self.pose.y, target_x, target_y)
        self.target_pose.theta = self._base_heading_from_camera_heading(target_camera_heading)
        self.get_logger().info(
            f"Queued simulated move: robot_dx={msg.x:.2f} cm, robot_dy={msg.y:.2f} cm, "
            f"target=({self.target_pose.x:.2f}, {self.target_pose.y:.2f}, yaw={self.target_pose.theta:.2f})"
        )

    def publish_state(self) -> None:
        self._step_toward_target()
        adhesion = Bool()
        adhesion.data = bool(self.get_parameter("adhesion_secure").value)
        self.adhesion_pub.publish(adhesion)
        public_pose = self._public_pose()
        self.robot_pose_pub.publish(public_pose)
        self.imu_pose_pub.publish(public_pose)
        self.update_gazebo_entity()

    def _step_toward_target(self) -> None:
        max_step = float(self.get_parameter("max_step_cm").value)
        next_x, next_y, next_theta = step_nonholonomic(
            current_x_cm=self.pose.x,
            current_y_cm=self.pose.y,
            current_theta_rad=self._camera_heading(),
            target_x_cm=self.target_pose.x,
            target_y_cm=self.target_pose.y,
            target_theta_rad=self._camera_heading_for_base(self.target_pose.theta),
            max_step_cm=max_step,
            max_turn_rad=float(self.get_parameter("max_turn_rad").value),
            heading_tolerance_rad=float(self.get_parameter("heading_tolerance_rad").value),
        )
        self.pose.x, self.pose.y = self._clamp_robot_center(next_x, next_y)
        self.pose.theta = self._base_heading_from_camera_heading(next_theta)

    def _camera_heading_offset(self) -> float:
        return float(self.get_parameter("camera_heading_offset_rad").value)

    def _camera_heading(self) -> float:
        return self._camera_heading_for_base(self.pose.theta)

    def _camera_heading_for_base(self, base_heading: float) -> float:
        return camera_heading_from_base_heading(base_heading, self._camera_heading_offset())

    def _base_heading_from_camera_heading(self, camera_heading: float) -> float:
        return base_heading_from_camera_heading(camera_heading, self._camera_heading_offset())

    def _public_pose(self) -> Pose2D:
        pose = Pose2D()
        pose.x = self.pose.x
        pose.y = self.pose.y
        pose.theta = self._camera_heading()
        return pose

    def _clamp_robot_center(self, x_cm: float, y_cm: float) -> tuple[float, float]:
        return clamp_robot_center(
            x_cm,
            y_cm,
            window_width_cm=float(self.get_parameter("window_width_cm").value),
            window_height_cm=float(self.get_parameter("window_height_cm").value),
            robot_half_width_cm=float(self.get_parameter("robot_half_width_cm").value),
            robot_half_height_cm=float(self.get_parameter("robot_half_height_cm").value),
        )

    def update_gazebo_entity(self) -> None:
        if not bool(self.get_parameter("update_gazebo_entity").value):
            return
        scale = float(self.get_parameter("pose_scale_m_per_cm").value)
        yaw = self.pose.theta
        entity_name = str(self.get_parameter("gazebo_entity_name").value)
        pose = self._gazebo_pose(yaw, scale)
        self._publish_model_state(entity_name, pose)

        # Also try services when available, but do not block topic-based movement on service completion.
        if self.pending_gazebo_request is not None and not self.pending_gazebo_request.done():
            return

        entity_client = self._ready_set_entity_state_client()
        if entity_client is not None:
            state = EntityState()
            state.name = entity_name
            state.reference_frame = "world"
            state.pose = pose
            request = SetEntityState.Request()
            request.state = state
            self.pending_gazebo_request = entity_client.call_async(request)
            return

        model_client = self._ready_set_model_state_client()
        if model_client is not None:
            state = ModelState()
            state.model_name = entity_name
            state.reference_frame = "world"
            state.pose = pose
            request = SetModelState.Request()
            request.model_state = state
            self.pending_gazebo_request = model_client.call_async(request)
            return

        if not self.gazebo_warning_logged:
            self.get_logger().warning(
                "No Gazebo state-setting service is ready yet; ROS pose will still publish. "
                "Checked entity: " + ", ".join(self.set_entity_state_clients.keys())
                + " | model: " + ", ".join(self.set_model_state_clients.keys())
            )
            self.gazebo_warning_logged = True

    def _publish_model_state(self, entity_name: str, pose) -> None:
        state = ModelState()
        state.model_name = entity_name
        state.reference_frame = "world"
        state.pose = pose
        for publisher in self.model_state_publishers.values():
            publisher.publish(state)

    def _gazebo_pose(self, yaw: float, scale: float):
        from geometry_msgs.msg import Pose

        pose = Pose()
        # Gazebo window coordinates: world Y is horizontal cm from the glass lower-left,
        # world Z is vertical cm from the glass lower-left, and world X is fixed so the
        # STL visual underside stays attached to the +X-side glass face, not the frame.
        pose.position.x = float(self.get_parameter("gazebo_wall_x_m").value)
        pose.position.y = self.pose.x * scale
        pose.position.z = self.pose.y * scale
        pose.orientation = self.wall_heading_to_quaternion(yaw)
        return pose

    @staticmethod
    def wall_heading_to_quaternion(theta: float) -> Quaternion:
        """Return an orientation for a robot attached to the +X side of the window.

        Heading 0 is the user's initial-heading convention: straight up along the
        window/world +Z axis. Positive heading turns toward window +X (world +Y).
        Local +X points along that heading inside the window plane, local +Y is
        the in-plane right axis, and local -Z is the underside normal toward the
        glass (world -X). This keeps the bottom face attached for every heading.
        """
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        matrix = (
            # Columns are local +X (heading), local +Y, local +Z in world axes.
            # local -Z must stay world -X so the underside remains on the glass.
            (0.0, 0.0, 1.0),
            (sin_t, -cos_t, 0.0),
            (cos_t, sin_t, 0.0),
        )
        return SimBridgeNode.rotation_matrix_to_quaternion(matrix)

    @staticmethod
    def rotation_matrix_to_quaternion(matrix: tuple[tuple[float, float, float], ...]) -> Quaternion:
        m00, m01, m02 = matrix[0]
        m10, m11, m12 = matrix[1]
        m20, m21, m22 = matrix[2]
        trace = m00 + m11 + m22
        q = Quaternion()
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            q.w = 0.25 * scale
            q.x = (m21 - m12) / scale
            q.y = (m02 - m20) / scale
            q.z = (m10 - m01) / scale
        elif m00 > m11 and m00 > m22:
            scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
            q.w = (m21 - m12) / scale
            q.x = 0.25 * scale
            q.y = (m01 + m10) / scale
            q.z = (m02 + m20) / scale
        elif m11 > m22:
            scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
            q.w = (m02 - m20) / scale
            q.x = (m01 + m10) / scale
            q.y = 0.25 * scale
            q.z = (m12 + m21) / scale
        else:
            scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
            q.w = (m10 - m01) / scale
            q.x = (m02 + m20) / scale
            q.y = (m12 + m21) / scale
            q.z = 0.25 * scale
        return q





def camera_heading_from_base_heading(base_heading: float, camera_heading_offset: float) -> float:
    return normalize_angle(base_heading + camera_heading_offset)


def base_heading_from_camera_heading(camera_heading: float, camera_heading_offset: float) -> float:
    return normalize_angle(camera_heading - camera_heading_offset)


def normalize_angle(angle_rad: float) -> float:
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad



def step_angle_toward(current_theta_rad: float, target_theta_rad: float, max_turn_rad: float) -> float:
    error = normalize_angle(target_theta_rad - current_theta_rad)
    if abs(error) <= max_turn_rad:
        return normalize_angle(target_theta_rad)
    return normalize_angle(current_theta_rad + math.copysign(max_turn_rad, error))


def heading_to_target(current_x_cm: float, current_y_cm: float, target_x_cm: float, target_y_cm: float) -> float:
    """Return user heading where 0 rad means window/world +Z (map +Y)."""
    dx = target_x_cm - current_x_cm
    dy = target_y_cm - current_y_cm
    return math.atan2(dx, dy)

def step_nonholonomic(
    current_x_cm: float,
    current_y_cm: float,
    current_theta_rad: float,
    target_x_cm: float,
    target_y_cm: float,
    target_theta_rad: float,
    max_step_cm: float,
    max_turn_rad: float,
    heading_tolerance_rad: float,
) -> tuple[float, float, float]:
    dx = target_x_cm - current_x_cm
    dy = target_y_cm - current_y_cm
    distance = math.hypot(dx, dy)
    if distance <= 1e-6:
        next_theta = step_angle_toward(current_theta_rad, target_theta_rad, max_turn_rad)
        return current_x_cm, current_y_cm, next_theta

    desired_heading = heading_to_target(current_x_cm, current_y_cm, target_x_cm, target_y_cm)
    heading_error = normalize_angle(desired_heading - current_theta_rad)
    if abs(heading_error) > heading_tolerance_rad:
        turn = min(abs(heading_error), max_turn_rad)
        next_theta = normalize_angle(current_theta_rad + math.copysign(turn, heading_error))
        return current_x_cm, current_y_cm, next_theta

    step = min(max_step_cm, distance)
    next_x = current_x_cm + math.sin(current_theta_rad) * step
    next_y = current_y_cm + math.cos(current_theta_rad) * step
    return next_x, next_y, normalize_angle(current_theta_rad)

def clamp_robot_center(
    x_cm: float,
    y_cm: float,
    window_width_cm: float,
    window_height_cm: float,
    robot_half_width_cm: float,
    robot_half_height_cm: float,
) -> tuple[float, float]:
    min_x = min(robot_half_width_cm, window_width_cm / 2.0)
    max_x = max(window_width_cm - robot_half_width_cm, min_x)
    min_y = min(robot_half_height_cm, window_height_cm / 2.0)
    max_y = max(window_height_cm - robot_half_height_cm, min_y)
    return min(max(x_cm, min_x), max_x), min(max(y_cm, min_y), max_y)

def step_toward(
    current_x_cm: float,
    current_y_cm: float,
    target_x_cm: float,
    target_y_cm: float,
    max_step_cm: float,
) -> tuple[float, float]:
    dx = target_x_cm - current_x_cm
    dy = target_y_cm - current_y_cm
    distance = math.hypot(dx, dy)
    if distance <= 0.0 or distance <= max_step_cm:
        return target_x_cm, target_y_cm
    scale = max_step_cm / distance
    return current_x_cm + dx * scale, current_y_cm + dy * scale

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
