from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import cv2
import numpy as np

CornerName = Literal["top_left", "top_right", "bottom_right", "bottom_left"]
LocalizationMode = Literal["normalized", "metric"]

CORNER_ORDER: tuple[CornerName, ...] = ("top_left", "top_right", "bottom_right", "bottom_left")


@dataclass(frozen=True)
class WindowCornersPx:
    top_left: tuple[float, float]
    top_right: tuple[float, float]
    bottom_right: tuple[float, float]
    bottom_left: tuple[float, float]

    def ordered(self) -> list[tuple[float, float]]:
        return [self.top_left, self.top_right, self.bottom_right, self.bottom_left]


@dataclass(frozen=True)
class WindowCalibration:
    image_corners_px: WindowCornersPx
    homography: np.ndarray
    mode: LocalizationMode
    width_cm: float | None = None
    height_cm: float | None = None

    @property
    def unit(self) -> str:
        return "cm" if self.mode == "metric" else "normalized"


def resolve_mode(width_cm: float | None, height_cm: float | None) -> LocalizationMode:
    return "metric" if width_cm is not None and height_cm is not None else "normalized"


def window_destination_points(
    width_cm: float | None = None,
    height_cm: float | None = None,
) -> np.ndarray:
    """Return homography destination points in TL, TR, BR, BL order.

    If metric dimensions are missing, coordinates are normalized to [0, 1].
    The window frame convention is bottom-left origin, +x right, +y up.
    """
    if width_cm is not None and height_cm is not None:
        width = float(width_cm)
        height = float(height_cm)
    else:
        width = 1.0
        height = 1.0

    return np.array(
        [
            [0.0, height],
            [width, height],
            [width, 0.0],
            [0.0, 0.0],
        ],
        dtype=np.float32,
    )


def compute_homography(
    image_corners_px: WindowCornersPx | list[tuple[float, float]],
    width_cm: float | None = None,
    height_cm: float | None = None,
) -> np.ndarray:
    if isinstance(image_corners_px, WindowCornersPx):
        ordered_corners = image_corners_px.ordered()
    else:
        ordered_corners = image_corners_px

    if len(ordered_corners) != 4:
        raise ValueError("Exactly four image corners are required: top_left, top_right, bottom_right, bottom_left")

    src = np.array(ordered_corners, dtype=np.float32)
    dst = window_destination_points(width_cm=width_cm, height_cm=height_cm)
    return cv2.getPerspectiveTransform(src, dst)


def pixel_to_window(
    pixel: tuple[float, float],
    homography: np.ndarray,
) -> tuple[float, float]:
    src = np.array([[[float(pixel[0]), float(pixel[1])]]], dtype=np.float32)
    dst = cv2.perspectiveTransform(src, homography)
    return float(dst[0, 0, 0]), float(dst[0, 0, 1])


def is_inside_window(
    point: tuple[float, float],
    calibration: WindowCalibration,
    tolerance: float = 1e-6,
) -> bool:
    max_x = calibration.width_cm if calibration.mode == "metric" and calibration.width_cm is not None else 1.0
    max_y = calibration.height_cm if calibration.mode == "metric" and calibration.height_cm is not None else 1.0
    x, y = point
    return -tolerance <= x <= max_x + tolerance and -tolerance <= y <= max_y + tolerance


def build_window_calibration(
    image_corners_px: WindowCornersPx,
    width_cm: float | None = None,
    height_cm: float | None = None,
) -> WindowCalibration:
    mode = resolve_mode(width_cm, height_cm)
    homography = compute_homography(image_corners_px, width_cm=width_cm, height_cm=height_cm)
    return WindowCalibration(
        image_corners_px=image_corners_px,
        homography=homography,
        mode=mode,
        width_cm=width_cm,
        height_cm=height_cm,
    )
