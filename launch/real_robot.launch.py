from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("gyro_z_bias_dps", default_value="-1.35"),
            DeclareLaunchArgument("drive_cm_per_second", default_value="5.0"),
            DeclareLaunchArgument("turn_rad_per_second", default_value="0.5"),
            DeclareLaunchArgument("max_motion_seconds", default_value="1.0"),
            Node(
                package="window_cleaner",
                executable="rpi_camera_node",
                name="rpi_camera_node",
                output="screen",
                parameters=[
                    {
                        "backend": "auto",
                        "image_topic": "/camera/image_raw",
                        "width": 640,
                        "height": 480,
                        "fps": 30.0,
                    }
                ],
            ),
            Node(
                package="window_cleaner",
                executable="arduino_sensor_bridge",
                name="arduino_sensor_bridge",
                output="screen",
                parameters=[
                    {
                        "port": LaunchConfiguration("port"),
                        "baud": 115200,
                        "camera_map_x_cm": 15.0,
                        "camera_map_y_cm": 30.0,
                        "gyro_z_bias_dps": LaunchConfiguration("gyro_z_bias_dps"),
                        "enable_motor_serial_commands": True,
                        "drive_cm_per_second": LaunchConfiguration("drive_cm_per_second"),
                        "turn_rad_per_second": LaunchConfiguration("turn_rad_per_second"),
                        "max_motion_seconds": LaunchConfiguration("max_motion_seconds"),
                        "min_motion_seconds": 0.1,
                        "lateral_tolerance_cm": 0.5,
                    }
                ],
            ),
            Node(
                package="window_cleaner",
                executable="dirt_target_node",
                name="dirt_target_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": "/camera/image_raw",
                        "robot_pose_topic": "/robot/imu_pose",
                        "target_topic": "/robot/target_pose",
                        "use_window_homography": True,
                        "auto_detect_window_corners": True,
                        "window_corner_detector": "ai",
                        "window_detector_fallback_to_rule_based": True,
                        "undistort_image": True,
                        "device": "cpu",
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
                        "robot_pose_topic": "/robot/imu_pose",
                        "motor_command_topic": "/arduino/motor_command",
                        "imu_pose_topic": "/robot/imu_pose",
                        "use_imu_heading": True,
                        "use_target_heading": True,
                        "adhesion_topic": "/adhesion/secure",
                        "require_adhesion": True,
                        "position_tolerance_cm": 0.5,
                        "heading_tolerance_rad": 0.05,
                    }
                ],
            ),
        ]
    )
