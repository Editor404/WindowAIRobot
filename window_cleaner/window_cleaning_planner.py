from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class MapPoint:
    x_cm: float
    y_cm: float


@dataclass
class DirtMemoryPoint:
    x_cm: float
    y_cm: float
    observations: int = 1
    cleaned: bool = False

    def point(self) -> MapPoint:
        return MapPoint(self.x_cm, self.y_cm)


@dataclass(frozen=True)
class PixelDetection:
    x_px: float
    y_px: float
    area_px: float = 0.0


@dataclass(frozen=True)
class CameraProjectionConfig:
    image_width_px: int = 640
    image_height_px: int = 480
    cm_per_pixel_x: float = 0.05
    cm_per_pixel_y: float = 0.05
    camera_offset_x_cm: float = 0.0
    camera_offset_y_cm: float = 0.0


class DirtMemoryMap:
    def __init__(self, merge_radius_cm: float = 5.0) -> None:
        self.merge_radius_cm = float(merge_radius_cm)
        self.points: list[DirtMemoryPoint] = []

    def add_observation(self, point: MapPoint) -> DirtMemoryPoint:
        nearest = self._nearest(point, include_cleaned=False)
        if nearest is not None and distance_cm(nearest.point(), point) <= self.merge_radius_cm:
            total = nearest.observations + 1
            nearest.x_cm = (nearest.x_cm * nearest.observations + point.x_cm) / total
            nearest.y_cm = (nearest.y_cm * nearest.observations + point.y_cm) / total
            nearest.observations = total
            return nearest

        memory = DirtMemoryPoint(point.x_cm, point.y_cm)
        self.points.append(memory)
        return memory

    def uncleaned(self) -> list[DirtMemoryPoint]:
        return [point for point in self.points if not point.cleaned]

    def mark_near_cleaned(self, robot: MapPoint, radius_cm: float) -> int:
        cleaned = 0
        for point in self.points:
            if not point.cleaned and distance_cm(point.point(), robot) <= radius_cm:
                point.cleaned = True
                cleaned += 1
        return cleaned

    def nearest_uncleaned(self, robot: MapPoint) -> DirtMemoryPoint | None:
        return self._nearest(robot, include_cleaned=False)

    def _nearest(self, point: MapPoint, include_cleaned: bool) -> DirtMemoryPoint | None:
        candidates = self.points if include_cleaned else self.uncleaned()
        if not candidates:
            return None
        return min(candidates, key=lambda candidate: distance_cm(candidate.point(), point))


class LawnMowerScanPlanner:
    def __init__(
        self,
        window_width_cm: float,
        window_height_cm: float,
        margin_cm: float = 15.0,
        stripe_spacing_cm: float = 25.0,
    ) -> None:
        self.window_width_cm = float(window_width_cm)
        self.window_height_cm = float(window_height_cm)
        self.margin_cm = float(margin_cm)
        self.stripe_spacing_cm = float(stripe_spacing_cm)
        self._waypoints = self._build_waypoints()
        self._index = 0

    @property
    def waypoints(self) -> list[MapPoint]:
        return list(self._waypoints)

    def is_complete(self) -> bool:
        return self._index >= len(self._waypoints)

    def current_target(self) -> MapPoint | None:
        if self.is_complete():
            return None
        return self._waypoints[self._index]

    def advance_if_reached(self, robot: MapPoint, tolerance_cm: float) -> bool:
        target = self.current_target()
        if target is None:
            return False
        if distance_cm(robot, target) <= tolerance_cm:
            self._index += 1
            return True
        return False

    def _build_waypoints(self) -> list[MapPoint]:
        min_x = min(self.margin_cm, self.window_width_cm / 2.0)
        max_x = max(self.window_width_cm - self.margin_cm, min_x)
        min_y = min(self.margin_cm, self.window_height_cm / 2.0)
        max_y = max(self.window_height_cm - self.margin_cm, min_y)

        xs: list[float] = []
        x = min_x
        while x < max_x:
            xs.append(x)
            x += max(self.stripe_spacing_cm, 1.0)
        if not xs or abs(xs[-1] - max_x) > 1e-6:
            xs.append(max_x)

        waypoints: list[MapPoint] = []
        for index, x_value in enumerate(xs):
            if index % 2 == 0:
                waypoints.append(MapPoint(x_value, min_y))
                waypoints.append(MapPoint(x_value, max_y))
            else:
                waypoints.append(MapPoint(x_value, max_y))
                waypoints.append(MapPoint(x_value, min_y))
        return waypoints


def project_pixel_detection_to_window(
    detection: PixelDetection,
    robot_pose: MapPoint,
    calibration: CameraProjectionConfig,
) -> MapPoint:
    image_center_x = calibration.image_width_px / 2.0
    image_center_y = calibration.image_height_px / 2.0
    dx_cm = (detection.x_px - image_center_x) * calibration.cm_per_pixel_x
    dy_cm = (image_center_y - detection.y_px) * calibration.cm_per_pixel_y
    return MapPoint(
        x_cm=robot_pose.x_cm + calibration.camera_offset_x_cm + dx_cm,
        y_cm=robot_pose.y_cm + calibration.camera_offset_y_cm + dy_cm,
    )


def clamp_to_window(point: MapPoint, width_cm: float, height_cm: float) -> MapPoint:
    return MapPoint(
        x_cm=min(max(point.x_cm, 0.0), float(width_cm)),
        y_cm=min(max(point.y_cm, 0.0), float(height_cm)),
    )


def distance_cm(a: MapPoint, b: MapPoint) -> float:
    return math.hypot(a.x_cm - b.x_cm, a.y_cm - b.y_cm)
