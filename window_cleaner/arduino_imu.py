from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ArduinoSensorSample:
    timestamp_ms: int
    pressure_raw: int
    gyro_valid: bool
    gyro_x_dps: float
    gyro_y_dps: float
    gyro_z_dps: float
    blower_pwm: int | None = None
    adhesion_secure: bool | None = None


def parse_sensor_line(line: str) -> ArduinoSensorSample | None:
    if not line.startswith("SENSOR,"):
        return None

    fields = line.strip().split(",")
    if len(fields) not in (7, 8, 9):
        raise ValueError(f"Expected 7, 8, or 9 sensor fields, got {len(fields)}")

    blower_pwm = None
    adhesion_secure = None
    if len(fields) == 8:
        adhesion_secure = bool(int(fields[7]))
    elif len(fields) == 9:
        # Backward-compatible parser for older blower-control sketches.
        blower_pwm = int(fields[7])
        adhesion_secure = bool(int(fields[8]))

    return ArduinoSensorSample(
        timestamp_ms=int(fields[1]),
        pressure_raw=int(fields[2]),
        gyro_valid=bool(int(fields[3])),
        gyro_x_dps=float(fields[4]),
        gyro_y_dps=float(fields[5]),
        gyro_z_dps=float(fields[6]),
        blower_pwm=blower_pwm,
        adhesion_secure=adhesion_secure,
    )


def normalize_angle(angle_radians: float) -> float:
    return math.atan2(math.sin(angle_radians), math.cos(angle_radians))


def map_delta_to_robot(
    delta_x_cm: float,
    delta_y_cm: float,
    robot_yaw_radians: float,
) -> tuple[float, float]:
    # User/map heading convention: yaw=0 means robot forward is map +Y
    # (Gazebo/world +Z), and positive yaw turns toward map +X (world +Y).
    sine = math.sin(robot_yaw_radians)
    cosine = math.cos(robot_yaw_radians)
    return (
        sine * delta_x_cm + cosine * delta_y_cm,
        cosine * delta_x_cm - sine * delta_y_cm,
    )


class GyroYawEstimator:
    def __init__(self, initial_yaw_radians: float = 0.0, z_bias_dps: float = 0.0) -> None:
        self.yaw_radians = normalize_angle(initial_yaw_radians)
        self.z_bias_dps = float(z_bias_dps)
        self._previous_timestamp_ms: int | None = None

    def update(self, timestamp_ms: int, gyro_z_dps: float) -> float:
        if self._previous_timestamp_ms is None:
            self._previous_timestamp_ms = int(timestamp_ms)
            return self.yaw_radians

        elapsed_ms = (int(timestamp_ms) - self._previous_timestamp_ms) & 0xFFFFFFFF
        self._previous_timestamp_ms = int(timestamp_ms)
        if elapsed_ms > 1000:
            return self.yaw_radians

        yaw_rate_radians = math.radians(float(gyro_z_dps) - self.z_bias_dps)
        self.yaw_radians = normalize_angle(self.yaw_radians + yaw_rate_radians * elapsed_ms / 1000.0)
        return self.yaw_radians
