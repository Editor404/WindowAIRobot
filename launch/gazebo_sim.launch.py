import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    package_share = Path(get_package_share_directory("window_cleaner"))
    gazebo_log_path = Path("/tmp") / f"gazebo-{os.environ.get('USER', 'user')}"
    gazebo_log_path.mkdir(parents=True, exist_ok=True)
    world = PathJoinSubstitution([
        FindPackageShare("window_cleaner"),
        "worlds",
        "window_cleaner_demo.world",
    ])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_share, "launch", "gazebo.launch.py"])
        ),
        launch_arguments={"world": world}.items(),
    )

    return LaunchDescription(
        [
            # Helps Gazebo resolve package://window_cleaner/meshes/jagosipda.stl.
            SetEnvironmentVariable("GAZEBO_MODEL_PATH", str(package_share.parent)),
            # Bind Gazebo transport to localhost, but do not override ROS_LOCALHOST_ONLY.
            # ROS 2 graph discovery must use the same ROS_LOCALHOST_ONLY value in the
            # launch terminal and any inspection/rqt terminal.
            SetEnvironmentVariable("GAZEBO_IP", "127.0.0.1"),
            SetEnvironmentVariable("GAZEBO_LOG_PATH", str(gazebo_log_path)),
            gazebo,
            Node(
                package="window_cleaner",
                executable="sim_bridge_node",
                name="sim_bridge_node",
                output="screen",
                parameters=[
                    {
                        "max_step_cm": 1.0,
                        "max_turn_rad": 0.08,
                        "heading_tolerance_rad": 0.05,
                        "publish_rate_hz": 10.0,
                        "window_width_cm": 80.0,
                        "window_height_cm": 160.0,
                        "robot_half_width_cm": 15.0,
                        "robot_half_height_cm": 15.0,
                        "initial_x_cm": 15.0,
                        "initial_y_cm": 15.0,
                        "initial_yaw_rad": 1.570796,
                        "camera_heading_offset_rad": -1.570796,
                    }
                ],
            ),

            Node(
                package="window_cleaner",
                executable="window_map_initializer_node",
                name="window_map_initializer_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": "/robot_camera/image_raw",
                        "map_topic": "/window/map",
                        "top_corners_topic": "/window/top_corners_px",
                        "debug_image_topic": "/window/top_corners_debug_image",
                        "window_width_cm": 80.0,
                        "window_height_cm": 160.0,
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
                        "image_topic": "/robot_camera/image_raw",
                        "robot_pose_topic": "/robot_pose",
                        "target_topic": "/dirt/immediate_target",
                        "publish_control_target": False,
                        "use_window_homography": False,
                        "auto_detect_window_corners": False,
                        "undistort_image": False,
                        "device": "cpu",
                        "detector_mode": "sim",
                        "sim_min_area_px": 8,
                        "sim_max_area_px": 8000,
                        "window_width_cm": 80.0,
                        "window_height_cm": 160.0,
                        "cm_per_pixel_x": 0.05,
                        "cm_per_pixel_y": 0.05,
                    }
                ],
            ),

            Node(
                package="window_cleaner",
                executable="window_cleaning_planner_node",
                name="window_cleaning_planner_node",
                output="screen",
                parameters=[
                    {
                        "window_width_cm": 80.0,
                        "window_height_cm": 160.0,
                        "robot_pose_topic": "/robot_pose",
                        "detections_px_topic": "/dirt/detections_px",
                        "target_topic": "/robot/target_pose",
                        "map_topic": "/window/map",
                        "require_map_initialization": False,
                        "image_width": 640,
                        "image_height": 480,
                        "cm_per_pixel_x": 0.05,
                        "cm_per_pixel_y": 0.05,
                        "camera_offset_x_cm": 0.0,
                        "camera_offset_y_cm": 15.0,
                        "home_x_cm": 15.0,
                        "home_y_cm": 15.0,
                        "home_yaw_rad": 1.570796,
                        "search_turn_step_rad": 0.35,
                    }
                ],
            ),

            Node(
                package="window_cleaner",
                executable="gazebo_dirt_cleaner_node",
                name="gazebo_dirt_cleaner_node",
                output="screen",
                parameters=[
                    {
                        "robot_pose_topic": "/robot_pose",
                        "clean_half_width_cm": 15.0,
                        "clean_half_height_cm": 15.0,
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
                        "require_adhesion": True,
                        "use_imu_heading": True,
                        "use_target_heading": True,
                        "heading_tolerance_rad": 0.05,
                    }
                ],
            ),
        ]
    )
