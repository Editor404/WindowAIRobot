from __future__ import annotations

from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Pose, Pose2D, PoseArray
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from window_cleaner.calibration_io import load_window_calibration, save_window_calibration
from window_cleaner.camera_calibration import CameraCalibrationParameters, load_camera_calibration
from window_cleaner.coordinate import CameraCalibration, RobotPose, dirt_absolute_coordinate
from window_cleaner.detect_dirt import DirtDetection, DirtSegmenter, SimDirtDetector
from window_cleaner.image_input import resize_frame
from window_cleaner.paths import (
    default_camera_calibration_path,
    default_model_path,
    default_window_model_path,
)
from window_cleaner.window_frame_detector import detect_window_frame
from window_cleaner.window_geometry import WindowCalibration, build_window_calibration, is_inside_window, pixel_to_window
from window_cleaner.window_segmentation_detector import (
    WindowSegmentationDetector,
    WindowSegmentationDetectorConfig,
)


class DirtTargetNode(Node):
    def __init__(self) -> None:
        super().__init__("dirt_target_node")

        self.declare_parameter("model_path", str(default_model_path()))
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("image_width", 0)
        self.declare_parameter("image_height", 0)
        self.declare_parameter("robot_pose_topic", "/robot_pose")
        self.declare_parameter("target_topic", "/robot/target_pose")
        self.declare_parameter("publish_control_target", True)
        self.declare_parameter("window_position_normalized_topic", "/dirt/window_position_normalized")
        self.declare_parameter("window_position_cm_topic", "/dirt/window_position_cm")
        self.declare_parameter("detection_count_topic", "/dirt/detection_count")
        self.declare_parameter("detections_px_topic", "/dirt/detections_px")
        self.declare_parameter("debug_image_topic", "/dirt/debug_image")
        self.declare_parameter("window_calibration_path", "config/window_calibration.yaml")
        self.declare_parameter("use_window_homography", True)
        self.declare_parameter("auto_detect_window_corners", False)
        self.declare_parameter("window_corner_detector", "ai")
        self.declare_parameter("window_model_path", str(default_window_model_path()))
        self.declare_parameter("window_confidence_threshold", 0.5)
        self.declare_parameter("window_detector_fallback_to_rule_based", True)
        self.declare_parameter("auto_save_window_calibration", False)
        self.declare_parameter("window_width_cm", 0.0)
        self.declare_parameter("window_height_cm", 0.0)
        self.declare_parameter("undistort_image", True)
        self.declare_parameter("camera_calibration_path", str(default_camera_calibration_path()))
        self.declare_parameter("camera_calibration_width", 640)
        self.declare_parameter("camera_calibration_height", 480)
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("cm_per_pixel_x", 0.05)
        self.declare_parameter("cm_per_pixel_y", 0.05)
        self.declare_parameter("camera_offset_x_cm", 0.0)
        self.declare_parameter("camera_offset_y_cm", 0.0)
        self.declare_parameter("detector_mode", "yolo")
        self.declare_parameter("sim_min_area_px", 20)
        self.declare_parameter("sim_max_area_px", 8000)
        self.declare_parameter("sim_edge_margin_px", 8)

        self.current_pose = RobotPose(0.0, 0.0)
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        self.bridge = CvBridge()
        self.camera_calibration = self._load_camera_calibration()
        self.window_calibration = self._load_window_calibration()
        self.auto_detect_window_corners = bool(self.get_parameter("auto_detect_window_corners").value)
        self.window_auto_detection_attempted = False
        self.window_detector = self._create_window_detector()
        self.detector_mode = str(self.get_parameter("detector_mode").value).strip().lower()
        if self.detector_mode not in {"yolo", "sim", "yolo_with_sim_fallback"}:
            raise ValueError("detector_mode must be 'yolo', 'sim', or 'yolo_with_sim_fallback'")
        self.sim_detector = SimDirtDetector(
            min_area_px=int(self.get_parameter("sim_min_area_px").value),
            max_area_px=int(self.get_parameter("sim_max_area_px").value),
            edge_margin_px=int(self.get_parameter("sim_edge_margin_px").value),
        )
        self.frames_seen = 0
        self.segmenter = None
        if self.detector_mode != "sim":
            self.segmenter = DirtSegmenter(
                model_path=self.get_parameter("model_path").value,
                confidence_threshold=float(self.get_parameter("confidence_threshold").value),
                device=self.get_parameter("device").value or None,
            )

        image_topic = self.get_parameter("image_topic").value
        robot_pose_topic = self.get_parameter("robot_pose_topic").value
        target_topic = self.get_parameter("target_topic").value
        normalized_topic = self.get_parameter("window_position_normalized_topic").value
        cm_topic = self.get_parameter("window_position_cm_topic").value
        count_topic = self.get_parameter("detection_count_topic").value
        detections_px_topic = self.get_parameter("detections_px_topic").value
        debug_image_topic = self.get_parameter("debug_image_topic").value

        self.publish_control_target = bool(self.get_parameter("publish_control_target").value)
        self.target_publisher = self.create_publisher(Pose2D, target_topic, 10)
        self.normalized_publisher = self.create_publisher(PointStamped, normalized_topic, 10)
        self.cm_publisher = self.create_publisher(PointStamped, cm_topic, 10)
        self.detection_count_publisher = self.create_publisher(Int32, count_topic, 10)
        self.detections_px_publisher = self.create_publisher(PoseArray, detections_px_topic, 10)
        self.debug_image_publisher = self.create_publisher(Image, debug_image_topic, 10)
        self.pose_subscription = self.create_subscription(Pose2D, robot_pose_topic, self.on_robot_pose, 10)
        self.image_subscription = self.create_subscription(Image, image_topic, self.on_image, 10)

        self.get_logger().info(f"Listening: {image_topic}, {robot_pose_topic}")
        self.get_logger().info(f"Input image size: {self.image_width}x{self.image_height} (set either to 0 to keep native size)")
        self.get_logger().info(f"Dirt detector mode: {self.detector_mode}")
        self.get_logger().info(
            f"Publishing legacy robot target coordinates: {target_topic} "
            f"enabled={self.publish_control_target}"
        )
        self.get_logger().info(f"Publishing normalized window coordinates: {normalized_topic}")
        self.get_logger().info(f"Publishing metric window coordinates: {cm_topic}")
        self.get_logger().info(f"Publishing all dirt detection count: {count_topic}")
        self.get_logger().info(f"Publishing all dirt detection pixels: {detections_px_topic}")
        self.get_logger().info(f"Publishing dirt debug image: {debug_image_topic}")

    def _load_camera_calibration(self) -> CameraCalibrationParameters | None:
        if not bool(self.get_parameter("undistort_image").value):
            self.get_logger().info("Camera undistortion disabled.")
            return None

        calibration_path = Path(str(self.get_parameter("camera_calibration_path").value))
        try:
            calibration = load_camera_calibration(
                calibration_path,
                calibration_width=int(self.get_parameter("camera_calibration_width").value),
                calibration_height=int(self.get_parameter("camera_calibration_height").value),
            )
        except Exception as exc:
            self.get_logger().warning(
                f"Could not load camera calibration from {calibration_path}: {exc}. "
                "Continuing without lens distortion correction."
            )
            return None

        self.get_logger().info(f"Loaded camera calibration from {calibration_path}")
        return calibration

    def _create_window_detector(self) -> WindowSegmentationDetector | None:
        if not self.auto_detect_window_corners:
            return None
        detector_name = str(self.get_parameter("window_corner_detector").value).strip().lower()
        if detector_name == "rule_based":
            self.get_logger().info("Window corner detector: rule_based")
            return None
        if detector_name != "ai":
            raise ValueError("window_corner_detector must be 'ai' or 'rule_based'")

        model_path = Path(str(self.get_parameter("window_model_path").value))
        try:
            detector = WindowSegmentationDetector(
                model_path=model_path,
                config=WindowSegmentationDetectorConfig(
                    confidence_threshold=float(self.get_parameter("window_confidence_threshold").value)
                ),
                device=self.get_parameter("device").value or None,
            )
        except Exception as exc:
            if not bool(self.get_parameter("window_detector_fallback_to_rule_based").value):
                raise
            self.get_logger().warning(
                f"Could not initialize AI window detector from {model_path}: {exc}. "
                "Falling back to rule-based corner detection."
            )
            return None

        self.get_logger().info(f"Window corner detector: AI segmentation ({model_path})")
        return detector

    def _load_window_calibration(self) -> WindowCalibration | None:
        if not bool(self.get_parameter("use_window_homography").value):
            self.get_logger().info("Window homography disabled; using legacy cm_per_pixel target conversion only.")
            return None

        calibration_path = Path(str(self.get_parameter("window_calibration_path").value))
        try:
            calibration = load_window_calibration(calibration_path)
        except Exception as exc:
            self.get_logger().warning(
                f"Could not load window calibration from {calibration_path}: {exc}. "
                "Falling back to legacy cm_per_pixel target conversion."
            )
            return None

        self.get_logger().info(
            f"Loaded window calibration from {calibration_path}; "
            f"localization_mode={calibration.mode}, unit={calibration.unit}"
        )
        return calibration

    def on_robot_pose(self, msg: Pose2D) -> None:
        self.current_pose = RobotPose(x_cm=msg.x, y_cm=msg.y)

    def on_image(self, msg: Image) -> None:
        self.frames_seen += 1
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        frame = resize_frame(frame, self.image_width, self.image_height)
        if self.camera_calibration is not None:
            frame = self.camera_calibration.undistort(frame)
        self._maybe_auto_detect_window_calibration(frame)
        detections = self._detect_dirt(frame)
        self._publish_detection_debug(msg, frame, detections)
        if not detections:
            if self.detector_mode == "sim" and self.frames_seen % 30 == 0:
                self.get_logger().warning(
                    "No simulated dirt detected yet: "
                    f"frames={self.frames_seen}, "
                    f"bg_gray={self.sim_detector.last_background_gray:.1f}, "
                    f"raw_components={self.sim_detector.last_raw_component_count}, "
                    f"accepted={self.sim_detector.last_detection_count}"
                )
            return

        detection = self._select_control_target(detections)
        if self.frames_seen % 15 == 0:
            self.get_logger().info(
                f"Detected dirt spots: count={len(detections)}, "
                f"target_pixel=({detection.center_px[0]:.1f}, {detection.center_px[1]:.1f})"
            )

        if self.window_calibration is not None:
            self._publish_window_position(msg, detection.center_px, detection.confidence)

        if self.publish_control_target and (self.window_calibration is None or self.window_calibration.mode == "metric"):
            self._publish_legacy_robot_target(frame, detection.center_px, detection.confidence)

    def _detect_dirt(self, frame) -> list[DirtDetection]:
        if self.detector_mode == "sim":
            return self.sim_detector.detect(frame)

        assert self.segmenter is not None
        detections = self.segmenter.detect(frame)
        if detections or self.detector_mode == "yolo":
            return detections

        return self.sim_detector.detect(frame)

    def _select_control_target(self, detections: list[DirtDetection]) -> DirtDetection:
        # Detection lists are sorted largest-first by each detector. Keep that
        # behavior for control so the robot goes to the most reliable visible
        # spot, while debug topics still expose every visible spot.
        return detections[0]

    def _publish_detection_debug(self, image_msg: Image, frame, detections: list[DirtDetection]) -> None:
        count_msg = Int32()
        count_msg.data = len(detections)
        self.detection_count_publisher.publish(count_msg)

        detections_msg = PoseArray()
        detections_msg.header = image_msg.header
        detections_msg.header.frame_id = image_msg.header.frame_id or "camera_image"
        for detection in detections:
            pose = Pose()
            pose.position.x = float(detection.center_px[0])
            pose.position.y = float(detection.center_px[1])
            pose.position.z = float(detection.area_px)
            pose.orientation.w = float(detection.confidence)
            detections_msg.poses.append(pose)
        self.detections_px_publisher.publish(detections_msg)
        debug_frame = frame.copy()
        for index, detection in enumerate(detections, start=1):
            center = (int(round(detection.center_px[0])), int(round(detection.center_px[1])))
            cv2.circle(debug_frame, center, 12, (0, 255, 255), 2)
            cv2.putText(
                debug_frame,
                str(index),
                (center[0] + 8, center[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )
        debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
        debug_msg.header = image_msg.header
        self.debug_image_publisher.publish(debug_msg)

    def _configured_window_size(self) -> tuple[float | None, float | None]:
        width_param = float(self.get_parameter("window_width_cm").value)
        height_param = float(self.get_parameter("window_height_cm").value)
        if width_param > 0.0 and height_param > 0.0:
            return width_param, height_param
        if self.window_calibration is not None:
            return self.window_calibration.width_cm, self.window_calibration.height_cm
        return None, None

    def _maybe_auto_detect_window_calibration(self, frame) -> None:
        if not self.auto_detect_window_corners or self.window_auto_detection_attempted:
            return
        self.window_auto_detection_attempted = True

        detection = self.window_detector.detect(frame) if self.window_detector is not None else detect_window_frame(frame)
        detector_used = "ai" if self.window_detector is not None else "rule_based"
        if (
            detection is None
            and self.window_detector is not None
            and bool(self.get_parameter("window_detector_fallback_to_rule_based").value)
        ):
            detection = detect_window_frame(frame)
            detector_used = "rule_based_fallback"
        if detection is None:
            self.get_logger().warning(
                "Auto window corner detection failed; using configured calibration or legacy conversion."
            )
            return

        width_cm, height_cm = self._configured_window_size()
        self.window_calibration = build_window_calibration(
            detection.corners,
            width_cm=width_cm,
            height_cm=height_cm,
        )
        self.get_logger().info(
            f"Auto-detected window corners with {detector_used}; "
            f"score={detection.score:.3f}, area_px={detection.area_px:.1f}, "
            f"localization_mode={self.window_calibration.mode}, unit={self.window_calibration.unit}"
        )

        if bool(self.get_parameter("auto_save_window_calibration").value):
            calibration_path = Path(str(self.get_parameter("window_calibration_path").value))
            try:
                save_window_calibration(calibration_path, self.window_calibration)
                self.get_logger().info(f"Saved auto-detected window calibration to {calibration_path}")
            except Exception as exc:
                self.get_logger().warning(f"Could not save auto-detected window calibration: {exc}")

    def _publish_window_position(
        self,
        image_msg: Image,
        center_px: tuple[float, float],
        confidence: float,
    ) -> None:
        assert self.window_calibration is not None
        window_point = pixel_to_window(center_px, self.window_calibration.homography)
        if not is_inside_window(window_point, self.window_calibration):
            self.get_logger().warning(
                f"Ignoring out-of-window dirt point: "
                f"pixel=({center_px[0]:.1f}, {center_px[1]:.1f}), "
                f"window=({window_point[0]:.3f}, {window_point[1]:.3f}) {self.window_calibration.unit}"
            )
            return

        point_msg = PointStamped()
        point_msg.header = image_msg.header
        point_msg.header.frame_id = "window_frame"
        point_msg.point.x = window_point[0]
        point_msg.point.y = window_point[1]
        point_msg.point.z = confidence

        if self.window_calibration.mode == "metric":
            self.cm_publisher.publish(point_msg)
        else:
            self.normalized_publisher.publish(point_msg)

        self.get_logger().info(
            f"Dirt window position: "
            f"pixel=({center_px[0]:.1f}, {center_px[1]:.1f}), "
            f"conf={confidence:.2f}, "
            f"position=({window_point[0]:.3f}, {window_point[1]:.3f}) {self.window_calibration.unit}, "
            f"mode={self.window_calibration.mode}"
        )

    def _publish_legacy_robot_target(
        self,
        frame,
        center_px: tuple[float, float],
        confidence: float,
    ) -> None:
        height, width = frame.shape[:2]
        calibration = CameraCalibration(
            image_width_px=width,
            image_height_px=height,
            cm_per_pixel_x=float(self.get_parameter("cm_per_pixel_x").value),
            cm_per_pixel_y=float(self.get_parameter("cm_per_pixel_y").value),
            camera_offset_x_cm=float(self.get_parameter("camera_offset_x_cm").value),
            camera_offset_y_cm=float(self.get_parameter("camera_offset_y_cm").value),
        )
        target = dirt_absolute_coordinate(
            robot_pose=self.current_pose,
            center_px=center_px,
            calibration=calibration,
        )

        target_msg = Pose2D()
        target_msg.x = target.x_cm
        target_msg.y = target.y_cm
        target_msg.theta = 0.0
        self.target_publisher.publish(target_msg)

        self.get_logger().info(
            "Dirt target: "
            f"pixel=({center_px[0]:.1f}, {center_px[1]:.1f}), "
            f"conf={confidence:.2f}, "
            f"target=({target.x_cm:.2f}, {target.y_cm:.2f}) cm"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DirtTargetNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
