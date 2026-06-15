from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="window_cleaner",
                executable="dirt_target_node",
                name="dirt_target_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": "/camera/image_raw",
                        "image_width": 0,
                        "image_height": 0,
                        "robot_pose_topic": "/robot_pose",
                        "target_topic": "/robot/target_pose",
                        "window_position_normalized_topic": "/dirt/window_position_normalized",
                        "window_position_cm_topic": "/dirt/window_position_cm",
                        "window_calibration_path": "config/window_calibration.yaml",
                        "use_window_homography": True,
                        "auto_detect_window_corners": True,
                        "window_corner_detector": "ai",
                        "window_confidence_threshold": 0.5,
                        "window_detector_fallback_to_rule_based": True,
                        "auto_save_window_calibration": False,
                        "window_width_cm": 0.0,
                        "window_height_cm": 0.0,
                        "undistort_image": True,
                        "camera_calibration_width": 640,
                        "camera_calibration_height": 480,
                        "confidence_threshold": 0.25,
                        "device": "cpu",
                        "cm_per_pixel_x": 0.05,
                        "cm_per_pixel_y": 0.05,
                        "camera_offset_x_cm": 0.0,
                        "camera_offset_y_cm": 0.0,
                    }
                ],
            ),
            Node(
                package="window_cleaner",
                executable="robot_controller_node",
                name="robot_controller_node",
                output="screen",
                parameters=[
                    {
                        "target_topic": "/robot/target_pose",
                        "motor_command_topic": "/arduino/motor_command",
                        "imu_pose_topic": "/robot/imu_pose",
                        "use_imu_heading": True,
                        "adhesion_topic": "/adhesion/secure",
                        "require_adhesion": True,
                        "position_tolerance_cm": 0.5,
                    }
                ],
            ),
        ]
    )
