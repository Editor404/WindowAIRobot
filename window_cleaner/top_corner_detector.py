from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class TopCornerDetection:
    top_left: tuple[float, float]
    top_right: tuple[float, float]
    confidence: float
    mask_area_px: int


def detect_top_corners(frame: np.ndarray) -> TopCornerDetection | None:
    if frame is None or frame.size == 0:
        raise ValueError("frame must be a non-empty image")

    mask = _glass_like_mask(frame)
    component = _largest_component(mask)
    if component is None:
        return None

    contour = _largest_contour(component)
    if contour is None:
        return None

    hull = cv2.convexHull(contour).reshape(-1, 2).astype(np.float32)
    if len(hull) < 4:
        return None

    top_left, top_right = _top_edge_points(hull)
    if top_right[0] - top_left[0] < frame.shape[1] * 0.15:
        return None

    area = int(component.sum() // 255)
    confidence = min(1.0, max(0.0, area / float(frame.shape[0] * frame.shape[1]) * 4.0))
    return TopCornerDetection(
        top_left=(float(top_left[0]), float(top_left[1])),
        top_right=(float(top_right[0]), float(top_right[1])),
        confidence=confidence,
        mask_area_px=area,
    )


def draw_top_corner_detection(frame: np.ndarray, detection: TopCornerDetection | None) -> np.ndarray:
    output = frame.copy()
    if detection is None:
        return output
    left = tuple(int(round(v)) for v in detection.top_left)
    right = tuple(int(round(v)) for v in detection.top_right)
    cv2.circle(output, left, 6, (0, 0, 255), -1)
    cv2.circle(output, right, 6, (0, 0, 255), -1)
    cv2.line(output, left, right, (0, 255, 255), 2)
    cv2.putText(output, "TL", (left[0] + 8, left[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.putText(output, "TR", (right[0] + 8, right[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    return output


def _glass_like_mask(frame: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]

    # Gazebo glass is a large low/medium-saturation blue-gray surface. This
    # excludes the black frame/background and most dark robot body pixels.
    mask = (gray > 55) & (value > 65) & (saturation < 120)
    binary = mask.astype(np.uint8) * 255
    kernel = np.ones((5, 5), dtype=np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
    return binary


def _largest_component(mask: np.ndarray) -> np.ndarray | None:
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count <= 1:
        return None
    areas = stats[1:, cv2.CC_STAT_AREA]
    label = int(np.argmax(areas)) + 1
    if int(stats[label, cv2.CC_STAT_AREA]) < mask.shape[0] * mask.shape[1] * 0.03:
        return None
    return ((labels == label).astype(np.uint8)) * 255


def _largest_contour(mask: np.ndarray) -> np.ndarray | None:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    return max(contours, key=cv2.contourArea)


def _top_edge_points(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    min_y = float(np.min(points[:, 1]))
    max_y = float(np.max(points[:, 1]))
    top_band = max(8.0, (max_y - min_y) * 0.12)
    candidates = points[points[:, 1] <= min_y + top_band]
    if len(candidates) < 2:
        by_y = points[np.argsort(points[:, 1])[: min(6, len(points))]]
        candidates = by_y
    left = candidates[int(np.argmin(candidates[:, 0]))]
    right = candidates[int(np.argmax(candidates[:, 0]))]
    return left, right
