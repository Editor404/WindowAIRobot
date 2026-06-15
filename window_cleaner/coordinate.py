from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RobotPose:
    x_cm: float
    y_cm: float


@dataclass(frozen=True)
class CameraCalibration:
    image_width_px: int
    image_height_px: int
    cm_per_pixel_x: float
    cm_per_pixel_y: float
    camera_offset_x_cm: float = 0.0
    camera_offset_y_cm: float = 0.0


@dataclass(frozen=True)
class TargetCoordinate:
    x_cm: float
    y_cm: float


def pixel_to_robot_relative_cm(
    center_px: tuple[float, float],
    calibration: CameraCalibration,
) -> tuple[float, float]:
    image_center_x = calibration.image_width_px / 2
    image_center_y = calibration.image_height_px / 2

    pixel_dx = center_px[0] - image_center_x
    pixel_dy = image_center_y - center_px[1]

    return (
        pixel_dx * calibration.cm_per_pixel_x,
        pixel_dy * calibration.cm_per_pixel_y,
    )


def dirt_absolute_coordinate(
    robot_pose: RobotPose,
    center_px: tuple[float, float],
    calibration: CameraCalibration,
) -> TargetCoordinate:
    relative_x_cm, relative_y_cm = pixel_to_robot_relative_cm(center_px, calibration)

    return TargetCoordinate(
        x_cm=robot_pose.x_cm + calibration.camera_offset_x_cm + relative_x_cm,
        y_cm=robot_pose.y_cm + calibration.camera_offset_y_cm + relative_y_cm,
    )
