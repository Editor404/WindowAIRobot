from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from window_cleaner.calibration_io import save_window_calibration
from window_cleaner.window_geometry import WindowCornersPx, build_window_calibration


@dataclass(frozen=True)
class WindowFrameDetection:
    corners: WindowCornersPx
    area_px: float
    score: float


@dataclass(frozen=True)
class WindowFrameDetectorConfig:
    min_area_ratio: float = 0.05
    canny_low: int | None = None
    canny_high: int | None = None
    blur_kernel_size: int = 5
    close_kernel_size: int = 5


def order_corners_tl_tr_br_bl(points: np.ndarray) -> WindowCornersPx:
    """Order four image points as top_left, top_right, bottom_right, bottom_left."""
    pts = np.asarray(points, dtype=np.float32).reshape(4, 2)
    sums = pts.sum(axis=1)
    diffs = pts[:, 0] - pts[:, 1]

    top_left = pts[int(np.argmin(sums))]
    bottom_right = pts[int(np.argmax(sums))]
    top_right = pts[int(np.argmax(diffs))]
    bottom_left = pts[int(np.argmin(diffs))]

    ordered = np.array([top_left, top_right, bottom_right, bottom_left], dtype=np.float32)
    if len({tuple(p) for p in ordered.tolist()}) != 4:
        # Fallback for near-degenerate cases: sort by y, then x.
        by_y = pts[np.argsort(pts[:, 1])]
        top = by_y[:2][np.argsort(by_y[:2, 0])]
        bottom = by_y[2:][np.argsort(by_y[2:, 0])]
        ordered = np.array([top[0], top[1], bottom[1], bottom[0]], dtype=np.float32)

    return WindowCornersPx(
        top_left=(float(ordered[0, 0]), float(ordered[0, 1])),
        top_right=(float(ordered[1, 0]), float(ordered[1, 1])),
        bottom_right=(float(ordered[2, 0]), float(ordered[2, 1])),
        bottom_left=(float(ordered[3, 0]), float(ordered[3, 1])),
    )


def _auto_canny_thresholds(gray: np.ndarray) -> tuple[int, int]:
    median = float(np.median(gray))
    low = int(max(0, (1.0 - 0.33) * median))
    high = int(min(255, (1.0 + 0.33) * median))
    if low == high:
        low, high = 50, 150
    return low, high


def _preprocess_edges(frame: np.ndarray, config: WindowFrameDetectorConfig) -> np.ndarray:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame.copy()
    kernel_size = max(3, int(config.blur_kernel_size) | 1)
    blurred = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)
    low, high = _auto_canny_thresholds(blurred)
    if config.canny_low is not None:
        low = int(config.canny_low)
    if config.canny_high is not None:
        high = int(config.canny_high)
    edges = cv2.Canny(blurred, low, high)
    close_size = max(1, int(config.close_kernel_size))
    if close_size > 1:
        kernel = np.ones((close_size, close_size), dtype=np.uint8)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    return edges


def _candidate_quadrilaterals(contour: np.ndarray) -> list[np.ndarray]:
    perimeter = cv2.arcLength(contour, True)
    candidates: list[np.ndarray] = []
    for epsilon_ratio in (0.01, 0.015, 0.02, 0.03, 0.04, 0.06, 0.08):
        approx = cv2.approxPolyDP(contour, epsilon_ratio * perimeter, True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            candidates.append(approx.reshape(4, 2).astype(np.float32))
    return candidates


def detect_window_frame(
    frame: np.ndarray,
    config: WindowFrameDetectorConfig | None = None,
) -> WindowFrameDetection | None:
    """Detect the most likely rectangular window frame in an image.

    The detector is intentionally calibration-oriented: it picks the largest
    convex quadrilateral found by edge/contour analysis, then returns its
    corners in the project's required TL, TR, BR, BL order.
    """
    if frame is None or frame.size == 0:
        raise ValueError("frame must be a non-empty image")

    config = config or WindowFrameDetectorConfig()
    height, width = frame.shape[:2]
    image_area = float(width * height)
    min_area = image_area * float(config.min_area_ratio)
    edges = _preprocess_edges(frame, config)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best: WindowFrameDetection | None = None
    for contour in contours:
        contour_area = abs(float(cv2.contourArea(contour)))
        if contour_area < min_area:
            continue
        for quad in _candidate_quadrilaterals(contour):
            area = abs(float(cv2.contourArea(quad)))
            if area < min_area:
                continue
            ordered = order_corners_tl_tr_br_bl(quad)
            rect = cv2.minAreaRect(quad)
            box_width, box_height = rect[1]
            if box_width <= 1.0 or box_height <= 1.0:
                continue
            rectangularity = min(1.0, area / max(contour_area, 1.0))
            coverage = min(1.0, area / image_area)
            score = 0.7 * coverage + 0.3 * rectangularity
            detection = WindowFrameDetection(corners=ordered, area_px=area, score=score)
            if best is None or detection.score > best.score:
                best = detection

    return best


def detect_window_corners(frame: np.ndarray) -> WindowCornersPx | None:
    detection = detect_window_frame(frame)
    return None if detection is None else detection.corners


def draw_window_detection(frame: np.ndarray, detection: WindowFrameDetection) -> np.ndarray:
    output = frame.copy()
    points = np.array(detection.corners.ordered(), dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(output, [points], isClosed=True, color=(0, 255, 0), thickness=2)
    for label, point in zip(("TL", "TR", "BR", "BL"), detection.corners.ordered()):
        x, y = int(point[0]), int(point[1])
        cv2.circle(output, (x, y), 5, (0, 0, 255), -1)
        cv2.putText(output, label, (x + 6, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    return output


def _read_frame(image_path: str | None, camera_index: int) -> np.ndarray:
    if image_path:
        frame = cv2.imread(str(Path(image_path)))
        if frame is None:
            raise FileNotFoundError(f"Could not read image: {image_path}")
        return frame

    capture = cv2.VideoCapture(camera_index)
    try:
        ok, frame = capture.read()
    finally:
        capture.release()
    if not ok:
        raise RuntimeError(f"Could not read frame from camera index {camera_index}")
    return frame


def _optional_dimension(value: float) -> float | None:
    return float(value) if value > 0 else None


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Detect window frame corners and write homography calibration YAML.")
    parser.add_argument("--image", help="Input image path. If omitted, one frame is read from --camera.")
    parser.add_argument("--camera", type=int, default=0, help="Camera index for live capture.")
    parser.add_argument("--output", default="config/window_calibration.yaml", help="Calibration YAML output path.")
    parser.add_argument("--window-width-cm", type=float, default=0.0, help="Set >0 with height for metric cm mode.")
    parser.add_argument("--window-height-cm", type=float, default=0.0, help="Set >0 with width for metric cm mode.")
    parser.add_argument("--annotated-output", help="Optional image path for visualizing detected corners.")
    parsed = parser.parse_args(args)

    frame = _read_frame(parsed.image, parsed.camera)
    detection = detect_window_frame(frame)
    if detection is None:
        raise RuntimeError("Could not detect a rectangular window frame in the input image")

    calibration = build_window_calibration(
        detection.corners,
        width_cm=_optional_dimension(parsed.window_width_cm),
        height_cm=_optional_dimension(parsed.window_height_cm),
    )
    save_window_calibration(parsed.output, calibration)

    if parsed.annotated_output:
        annotated = draw_window_detection(frame, detection)
        if not cv2.imwrite(str(Path(parsed.annotated_output)), annotated):
            raise RuntimeError(f"Could not write annotated output: {parsed.annotated_output}")

    print(f"Saved calibration: {parsed.output}")
    print(f"corners={detection.corners}")
    print(f"score={detection.score:.3f}, area_px={detection.area_px:.1f}, mode={calibration.mode}")


if __name__ == "__main__":
    main()
