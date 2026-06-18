from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class DirtDetection:
    class_id: int
    confidence: float
    center_px: tuple[float, float]
    area_px: int
    mask: np.ndarray


class SimDirtDetector:
    """Simple detector for Gazebo demo dirt dots.

    The simulated white dirt marks are rendered as low-saturation blobs on the
    glass and may appear gray depending on Gazebo lighting. This detector is not
    meant to replace the trained model on hardware; it exists so the full ROS
    control pipeline can be exercised in Gazebo even when YOLO does not recognize
    synthetic dots.
    """

    def __init__(
        self,
        min_area_px: int = 8,
        max_area_px: int = 8000,
        edge_margin_px: int = 8,
    ) -> None:
        self.min_area_px = int(min_area_px)
        self.max_area_px = int(max_area_px)
        self.edge_margin_px = int(edge_margin_px)
        self.last_background_gray = 0.0
        self.last_raw_component_count = 0
        self.last_detection_count = 0

    def detect(self, image: np.ndarray) -> list[DirtDetection]:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)

        saturation = hsv[:, :, 1]
        valid_background = (gray_blur > 25) & (gray_blur < 235)
        background_pixels = gray_blur[valid_background]
        background_gray = float(np.median(background_pixels)) if background_pixels.size else float(np.median(gray_blur))
        self.last_background_gray = background_gray

        # In Gazebo the nominally white dirt disks often appear as darker gray
        # disks through the blue glass. Detect both dark contrast disks and truly
        # bright low-saturation disks, while avoiding the black outside world.
        dark_dirt = (gray_blur > 25) & (gray_blur < background_gray - 14.0)
        bright_dirt = (gray_blur > background_gray + 30.0) & (saturation < 120)
        mask = dark_dirt | bright_dirt

        # Exclude image borders where the black outside-world and robot body can
        # create large false positives.
        margin = self.edge_margin_px
        if margin > 0:
            mask[:margin, :] = False
            mask[-margin:, :] = False
            mask[:, :margin] = False
            mask[:, -margin:] = False

        binary = mask.astype(np.uint8) * 255
        kernel = np.ones((3, 3), dtype=np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        count, labels, stats, centroids = cv2.connectedComponentsWithStats(binary, connectivity=8)
        self.last_raw_component_count = max(0, count - 1)
        detections: list[DirtDetection] = []
        image_area = image.shape[0] * image.shape[1]
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.min_area_px or area > min(self.max_area_px, image_area // 20):
                continue
            w = int(stats[label, cv2.CC_STAT_WIDTH])
            h = int(stats[label, cv2.CC_STAT_HEIGHT])
            if w <= 0 or h <= 0:
                continue
            aspect = max(w / h, h / w)
            if aspect > 2.8:
                continue
            extent = area / float(w * h)
            if extent < 0.25:
                continue
            component_mask = (labels == label).astype(np.uint8)
            cx, cy = centroids[label]
            detections.append(
                DirtDetection(
                    class_id=0,
                    confidence=0.5,
                    center_px=(float(cx), float(cy)),
                    area_px=area,
                    mask=component_mask,
                )
            )

        self.last_detection_count = len(detections)
        # Prefer larger/nearer spots first.
        return sorted(detections, key=lambda item: item.area_px, reverse=True)

    def detect_largest(self, image: np.ndarray) -> DirtDetection | None:
        detections = self.detect(image)
        return detections[0] if detections else None


class DirtSegmenter:
    def __init__(
        self,
        model_path: str | Path,
        confidence_threshold: float = 0.25,
        device: str | None = None,
    ) -> None:
        os.environ.setdefault("YOLO_CONFIG_DIR", "/tmp/capstone-ultralytics")
        os.environ.setdefault("MPLCONFIGDIR", "/tmp/capstone-matplotlib")

        try:
            from ultralytics import YOLO
        except Exception as exc:
            raise RuntimeError(
                "Could not import ultralytics. Install or repair dependencies with: "
                "python3 -m pip install -r requirements.txt"
            ) from exc

        self.model_path = Path(model_path)
        if not self.model_path.exists():
            raise FileNotFoundError(f"Model file not found: {self.model_path}")

        self.confidence_threshold = confidence_threshold
        self.device = device
        self.model = YOLO(str(self.model_path))

    def detect(self, image: np.ndarray) -> list[DirtDetection]:
        results = self.model.predict(
            source=image,
            conf=self.confidence_threshold,
            device=self.device,
            verbose=False,
        )
        if not results:
            return []

        result = results[0]
        if result.masks is None or result.boxes is None:
            return []

        masks = result.masks.data.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy().astype(int)
        confidences = result.boxes.conf.cpu().numpy()

        detections: list[DirtDetection] = []
        for mask, class_id, confidence in zip(masks, classes, confidences):
            binary_mask = mask > 0.5
            area_px = int(binary_mask.sum())
            if area_px == 0:
                continue

            ys, xs = np.where(binary_mask)
            center_px = (float(xs.mean()), float(ys.mean()))
            detections.append(
                DirtDetection(
                    class_id=int(class_id),
                    confidence=float(confidence),
                    center_px=center_px,
                    area_px=area_px,
                    mask=binary_mask.astype(np.uint8),
                )
            )

        return sorted(detections, key=lambda item: item.area_px, reverse=True)

    def detect_largest(self, image: np.ndarray) -> DirtDetection | None:
        detections = self.detect(image)
        return detections[0] if detections else None
