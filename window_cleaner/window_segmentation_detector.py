from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from window_cleaner.window_frame_detector import WindowFrameDetection, order_corners_tl_tr_br_bl


@dataclass(frozen=True)
class WindowSegmentationDetectorConfig:
    confidence_threshold: float = 0.5
    frame_class_id: int = 0
    glass_class_id: int = 1
    boundary_kernel_size: int = 7
    min_area_ratio: float = 0.05


def _combined_class_masks(
    masks: np.ndarray,
    classes: np.ndarray,
    image_shape: tuple[int, int],
    frame_class_id: int,
    glass_class_id: int,
) -> tuple[np.ndarray, np.ndarray]:
    height, width = image_shape
    frame_mask = np.zeros((height, width), dtype=np.uint8)
    glass_mask = np.zeros((height, width), dtype=np.uint8)

    for mask, class_id in zip(masks, classes):
        resized = cv2.resize(mask.astype(np.float32), (width, height), interpolation=cv2.INTER_LINEAR)
        binary = np.where(resized > 0.5, 255, 0).astype(np.uint8)
        if int(class_id) == frame_class_id:
            frame_mask = cv2.bitwise_or(frame_mask, binary)
        elif int(class_id) == glass_class_id:
            glass_mask = cv2.bitwise_or(glass_mask, binary)

    return frame_mask, glass_mask


def _quadrilateral_from_contour(contour: np.ndarray) -> np.ndarray | None:
    perimeter = cv2.arcLength(contour, True)
    for epsilon_ratio in (0.02, 0.03, 0.04, 0.06, 0.08, 0.1):
        approx = cv2.approxPolyDP(contour, epsilon_ratio * perimeter, True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            return approx.reshape(4, 2).astype(np.float32)

    rect = cv2.minAreaRect(contour)
    if rect[1][0] <= 1.0 or rect[1][1] <= 1.0:
        return None
    return cv2.boxPoints(rect).astype(np.float32)


def detect_window_from_class_masks(
    frame_mask: np.ndarray,
    glass_mask: np.ndarray,
    boundary_kernel_size: int = 7,
    min_area_ratio: float = 0.05,
) -> WindowFrameDetection | None:
    if frame_mask.shape != glass_mask.shape:
        raise ValueError("frame and glass masks must have the same shape")
    if frame_mask.ndim != 2:
        raise ValueError("frame and glass masks must be single-channel images")

    height, width = glass_mask.shape
    image_area = float(width * height)
    min_area = image_area * min_area_ratio
    kernel_size = max(1, int(boundary_kernel_size))
    kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)

    # CornerDetection.py used the contact band between dilated glass and frame.
    boundary_band = cv2.bitwise_and(cv2.dilate(glass_mask, kernel), frame_mask)
    candidates = [boundary_band, glass_mask]

    best: WindowFrameDetection | None = None
    for candidate_mask in candidates:
        contours, _ = cv2.findContours(candidate_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            contour_area = abs(float(cv2.contourArea(contour)))
            if contour_area < min_area:
                continue
            quad = _quadrilateral_from_contour(contour)
            if quad is None:
                continue
            area = abs(float(cv2.contourArea(quad)))
            if area < min_area:
                continue
            coverage = min(1.0, area / image_area)
            fit = min(1.0, contour_area / max(area, 1.0))
            detection = WindowFrameDetection(
                corners=order_corners_tl_tr_br_bl(quad),
                area_px=area,
                score=0.7 * coverage + 0.3 * fit,
            )
            if best is None or detection.score > best.score:
                best = detection
        if best is not None:
            break

    return best


class WindowSegmentationDetector:
    def __init__(
        self,
        model_path: str | Path,
        config: WindowSegmentationDetectorConfig | None = None,
        device: str | None = None,
    ) -> None:
        try:
            from ultralytics import YOLO
        except Exception as exc:
            raise RuntimeError("Could not import ultralytics") from exc

        self.model_path = Path(model_path)
        if not self.model_path.exists():
            raise FileNotFoundError(f"Window segmentation model not found: {self.model_path}")
        self.config = config or WindowSegmentationDetectorConfig()
        self.device = device
        self.model = YOLO(str(self.model_path))

    def detect(self, frame: np.ndarray) -> WindowFrameDetection | None:
        results = self.model.predict(
            source=frame,
            conf=self.config.confidence_threshold,
            device=self.device,
            verbose=False,
        )
        if not results:
            return None
        result = results[0]
        if result.masks is None or result.boxes is None:
            return None

        masks = result.masks.data.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy().astype(int)
        frame_mask, glass_mask = _combined_class_masks(
            masks=masks,
            classes=classes,
            image_shape=frame.shape[:2],
            frame_class_id=self.config.frame_class_id,
            glass_class_id=self.config.glass_class_id,
        )
        return detect_window_from_class_masks(
            frame_mask=frame_mask,
            glass_mask=glass_mask,
            boundary_kernel_size=self.config.boundary_kernel_size,
            min_area_ratio=self.config.min_area_ratio,
        )
