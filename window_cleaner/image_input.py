from __future__ import annotations

import cv2
import numpy as np


def resize_frame(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """Resize an OpenCV frame when both width and height are positive.

    Use width=0 or height=0 to keep the camera topic's native image size.
    """
    if width <= 0 or height <= 0:
        return frame
    return cv2.resize(frame, (int(width), int(height)))
