from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class DirtDetection:
    class_id: int
    confidence: float
    center_px: tuple[float, float]
    area_px: int
    mask: np.ndarray


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
