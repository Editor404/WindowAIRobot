from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("gyro_z_bias_dps", default_value="0.0"),
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
                    }
                ],
            ),
        ]
    )
