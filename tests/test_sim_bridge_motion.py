from window_cleaner.sim_bridge_node import step_toward


def test_step_toward_limits_motion_to_max_step():
    x, y = step_toward(0.0, 0.0, 3.0, 4.0, max_step_cm=1.0)

    assert round(x, 6) == 0.6
    assert round(y, 6) == 0.8


def test_step_toward_snaps_when_target_is_within_step():
    assert step_toward(0.0, 0.0, 0.3, 0.4, max_step_cm=1.0) == (0.3, 0.4)

from window_cleaner.sim_bridge_node import clamp_robot_center


def test_clamp_robot_center_keeps_30cm_robot_inside_80x160_window():
    assert clamp_robot_center(
        -20.0,
        999.0,
        window_width_cm=80.0,
        window_height_cm=160.0,
        robot_half_width_cm=15.0,
        robot_half_height_cm=15.0,
    ) == (15.0, 145.0)

    assert clamp_robot_center(
        70.0,
        5.0,
        window_width_cm=80.0,
        window_height_cm=160.0,
        robot_half_width_cm=15.0,
        robot_half_height_cm=15.0,
    ) == (65.0, 15.0)

from window_cleaner.sim_bridge_node import normalize_angle, step_nonholonomic


def test_nonholonomic_step_turns_before_moving_to_side_target():
    x, y, theta = step_nonholonomic(
        current_x_cm=0.0,
        current_y_cm=0.0,
        current_theta_rad=0.0,
        target_x_cm=10.0,
        target_y_cm=0.0,
        target_theta_rad=0.0,
        max_step_cm=1.0,
        max_turn_rad=0.2,
        heading_tolerance_rad=0.05,
    )

    assert (x, y) == (0.0, 0.0)
    assert round(theta, 6) == 0.2


def test_nonholonomic_step_moves_world_z_when_heading_zero():
    x, y, theta = step_nonholonomic(
        current_x_cm=0.0,
        current_y_cm=0.0,
        current_theta_rad=0.0,
        target_x_cm=0.0,
        target_y_cm=10.0,
        target_theta_rad=0.0,
        max_step_cm=1.0,
        max_turn_rad=0.2,
        heading_tolerance_rad=0.05,
    )

    assert round(x, 6) == 0.0
    assert round(y, 6) == 1.0
    assert round(theta, 6) == 0.0


def test_normalize_angle_wraps_to_pi_range():
    assert round(normalize_angle(3.5), 6) == round(3.5 - 2 * 3.141592653589793, 6)


def test_nonholonomic_step_rotates_in_place_when_position_reached():
    x, y, theta = step_nonholonomic(
        current_x_cm=10.0,
        current_y_cm=20.0,
        current_theta_rad=0.0,
        target_x_cm=10.0,
        target_y_cm=20.0,
        target_theta_rad=1.0,
        max_step_cm=1.0,
        max_turn_rad=0.2,
        heading_tolerance_rad=0.05,
    )

    assert (x, y) == (10.0, 20.0)
    assert round(theta, 6) == 0.2


def _quaternion_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def _column(matrix, index):
    return tuple(row[index] for row in matrix)


def test_wall_heading_quaternion_keeps_underside_on_glass_at_initial_heading():
    from window_cleaner.sim_bridge_node import SimBridgeNode

    matrix = _quaternion_to_matrix(SimBridgeNode.wall_heading_to_quaternion(0.0))

    assert tuple(round(value, 6) for value in _column(matrix, 0)) == (0.0, 0.0, 1.0)
    assert tuple(round(value, 6) for value in _column(matrix, 2)) == (1.0, 0.0, 0.0)


def test_wall_heading_quaternion_turns_inside_window_plane_without_detaching():
    from math import pi

    from window_cleaner.sim_bridge_node import SimBridgeNode

    matrix = _quaternion_to_matrix(SimBridgeNode.wall_heading_to_quaternion(pi / 2.0))

    assert tuple(round(value, 6) for value in _column(matrix, 0)) == (0.0, 1.0, 0.0)
    assert tuple(round(value, 6) for value in _column(matrix, 2)) == (1.0, 0.0, 0.0)



def test_camera_heading_offset_converts_restored_startup_yaw_to_window_up():
    from window_cleaner.sim_bridge_node import (
        base_heading_from_camera_heading,
        camera_heading_from_base_heading,
    )

    base_yaw = 1.570796
    offset = -1.570796

    camera_yaw = camera_heading_from_base_heading(base_yaw, offset)
    assert round(camera_yaw, 6) == 0.0
    assert round(base_heading_from_camera_heading(camera_yaw, offset), 6) == round(base_yaw, 6)
