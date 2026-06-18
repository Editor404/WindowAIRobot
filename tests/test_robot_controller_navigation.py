import math

from window_cleaner.robot_controller_node import compute_wheel_navigation_command


def test_side_target_turns_before_forward_motion():
    assert compute_wheel_navigation_command(
        map_dx=10.0,
        map_dy=0.0,
        current_heading=0.0,
        target_heading=0.0,
        position_tolerance_cm=0.5,
        heading_tolerance_rad=0.05,
    ) == (0.0, 0.0, math.pi / 2.0)


def test_aligned_target_moves_forward_only():
    assert compute_wheel_navigation_command(
        map_dx=0.0,
        map_dy=10.0,
        current_heading=0.0,
        target_heading=0.0,
        position_tolerance_cm=0.5,
        heading_tolerance_rad=0.05,
    ) == (10.0, 0.0, 0.0)


def test_at_position_turns_to_requested_final_heading():
    assert compute_wheel_navigation_command(
        map_dx=0.1,
        map_dy=0.1,
        current_heading=0.0,
        target_heading=0.5,
        position_tolerance_cm=0.5,
        heading_tolerance_rad=0.05,
    ) == (0.0, 0.0, 0.5)


def test_at_pose_returns_no_command():
    assert compute_wheel_navigation_command(
        map_dx=0.1,
        map_dy=0.1,
        current_heading=0.0,
        target_heading=0.01,
        position_tolerance_cm=0.5,
        heading_tolerance_rad=0.05,
    ) is None
