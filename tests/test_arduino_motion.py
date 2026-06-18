import pytest

from window_cleaner.arduino_motion import MotionCalibration, pose2d_to_motion_command


def test_forward_command_duration_is_distance_over_speed():
    command = pose2d_to_motion_command(10.0, 0.0, 0.0, MotionCalibration(drive_cm_per_second=5.0))

    assert command.serial_line == "w"
    assert command.duration_s == pytest.approx(2.0)
    assert command.reason == "drive"


def test_backward_command():
    command = pose2d_to_motion_command(-3.0, 0.0, 0.0, MotionCalibration(drive_cm_per_second=10.0))

    assert command.serial_line == "s"
    assert command.reason == "drive"


def test_positive_turn_maps_to_right_command():
    command = pose2d_to_motion_command(0.0, 0.0, 0.25, MotionCalibration(turn_rad_per_second=0.5))

    assert command.serial_line == "d"
    assert command.duration_s == pytest.approx(0.5)
    assert command.reason == "turn"


def test_lateral_command_is_rejected_for_real_robot():
    command = pose2d_to_motion_command(0.0, 2.0, 0.0)

    assert command.serial_line == "x"
    assert command.duration_s == 0.0
    assert command.reason == "lateral_command_rejected"
