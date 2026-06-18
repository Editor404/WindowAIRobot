import math

import pytest

from window_cleaner.arduino_imu import GyroYawEstimator, map_delta_to_robot, parse_sensor_line


def test_parse_arduino_sensor_line():
    sample = parse_sensor_line("SENSOR,1500,512,1,1.25,-2.5,90.0")

    assert sample is not None
    assert sample.timestamp_ms == 1500
    assert sample.pressure_raw == 512
    assert sample.gyro_valid
    assert sample.gyro_z_dps == 90.0




def test_parse_sensor_line_with_adhesion_without_blower_control():
    sample = parse_sensor_line("SENSOR,1500,512,1,1.25,-2.5,90.0,1")

    assert sample is not None
    assert sample.blower_pwm is None
    assert sample.adhesion_secure


def test_parse_sensor_line_with_adhesion_control_state():
    sample = parse_sensor_line("SENSOR,1500,512,1,1.25,-2.5,90.0,210,1")

    assert sample is not None
    assert sample.blower_pwm == 210
    assert sample.adhesion_secure


def test_ignore_non_sensor_serial_log():
    assert parse_sensor_line("FORWARD") is None


def test_integrate_gyro_z_into_yaw():
    estimator = GyroYawEstimator()
    estimator.update(1000, 90.0)

    yaw = estimator.update(2000, 90.0)

    assert yaw == pytest.approx(math.pi / 2.0)


def test_apply_gyro_bias():
    estimator = GyroYawEstimator(z_bias_dps=2.0)
    estimator.update(0, 2.0)

    assert estimator.update(500, 2.0) == pytest.approx(0.0)


def test_transform_map_motion_using_robot_yaw():
    forward_cm, lateral_cm = map_delta_to_robot(0.0, 10.0, 0.0)

    assert forward_cm == pytest.approx(10.0)
    assert lateral_cm == pytest.approx(0.0, abs=1e-9)


def test_transform_side_motion_after_positive_quarter_turn():
    forward_cm, lateral_cm = map_delta_to_robot(10.0, 0.0, math.pi / 2.0)

    assert forward_cm == pytest.approx(10.0)
    assert lateral_cm == pytest.approx(0.0, abs=1e-9)
