import cv2
import numpy as np

from window_cleaner.top_corner_detector import detect_top_corners


def test_detect_top_corners_on_large_glass_polygon():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    polygon = np.array([[90, 80], [550, 95], [590, 470], [45, 470]], dtype=np.int32)
    cv2.fillPoly(frame, [polygon], (150, 135, 120))

    detection = detect_top_corners(frame)

    assert detection is not None
    assert abs(detection.top_left[0] - 90) <= 20
    assert abs(detection.top_right[0] - 550) <= 20
    assert detection.confidence > 0.05
