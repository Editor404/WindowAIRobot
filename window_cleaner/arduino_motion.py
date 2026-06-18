from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MotionCommand:
    serial_line: str
    duration_s: float
    reason: str


@dataclass(frozen=True)
class MotionCalibration:
    drive_cm_per_second: float = 5.0
    turn_rad_per_second: float = 0.5
    max_motion_seconds: float = 2.0
    min_motion_seconds: float = 0.1
    lateral_tolerance_cm: float = 0.5
    command_deadband: float = 1e-6


def pose2d_to_motion_command(
    x_cm: float,
    y_cm: float,
    theta_rad: float,
    calibration: MotionCalibration | None = None,
) -> MotionCommand:
    """Translate a ROS Pose2D motor command into one Arduino serial command.

    Real tracked hardware cannot execute side-slip commands. For real robot use,
    `robot_controller_node` should run with `use_target_heading:=true`, which emits
    either turn-in-place commands (`theta != 0`) or forward/backward distance
    commands (`x != 0, y ~= 0`). This helper refuses lateral-only commands by
    returning the Arduino ROS-bridge stop command `CMD,S`.
    """

    calibration = calibration or MotionCalibration()
    if abs(y_cm) > calibration.lateral_tolerance_cm:
        return MotionCommand("CMD,S", 0.0, "lateral_command_rejected")

    if abs(theta_rad) > calibration.command_deadband and abs(x_cm) <= calibration.command_deadband:
        command = "CMD,R" if theta_rad > 0.0 else "CMD,L"
        duration = _bounded_duration(
            abs(theta_rad) / max(calibration.turn_rad_per_second, calibration.command_deadband),
            calibration,
        )
        return MotionCommand(command, duration, "turn")

    if abs(x_cm) > calibration.command_deadband:
        command = "CMD,F" if x_cm > 0.0 else "CMD,B"
        duration = _bounded_duration(
            abs(x_cm) / max(calibration.drive_cm_per_second, calibration.command_deadband),
            calibration,
        )
        return MotionCommand(command, duration, "drive")

    return MotionCommand("CMD,S", 0.0, "stop")


def _bounded_duration(duration_s: float, calibration: MotionCalibration) -> float:
    return min(
        max(float(duration_s), float(calibration.min_motion_seconds)),
        float(calibration.max_motion_seconds),
    )
