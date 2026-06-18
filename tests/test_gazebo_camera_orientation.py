from pathlib import Path
import xml.etree.ElementTree as ET


def test_robot_camera_faces_robot_local_top_axis():
    world = ET.parse(Path("worlds/window_cleaner_demo.world"))
    sensor = world.find(".//model[@name='window_cleaner_robot']/link/sensor[@name='robot_camera']")
    assert sensor is not None

    pose = sensor.findtext("pose")
    assert pose is not None
    x, y, z, roll, pitch, yaw = [float(value) for value in pose.split()]

    assert (x, y, z) == (0.0, 0.15, 0.061114)
    assert roll == 0.0
    assert pitch == 0.0983
    assert round(yaw, 6) == round(1.570796, 6)
