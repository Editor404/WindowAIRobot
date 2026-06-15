from glob import glob

from setuptools import setup

package_name = "window_cleaner"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/models", ["seg_best.pt", "best.pt"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", ["config/window_calibration.yaml", "calib_parameters.npz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="capstone",
    maintainer_email="student@example.com",
    description="YOLOv8 dirt segmentation and coordinate conversion nodes.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "dirt_segmentation_node = window_cleaner.dirt_segmentation_node:main",
            "dirt_target_node = window_cleaner.dirt_target_node:main",
            "robot_controller_node = window_cleaner.robot_controller_node:main",
            "arduino_sensor_bridge = window_cleaner.arduino_sensor_bridge_node:main",
            "calibrate_window = window_cleaner.window_frame_detector:main",
        ],
    },
)
