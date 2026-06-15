import cv2
import numpy as np

from window_cleaner.window_frame_detector import detect_window_frame, order_corners_tl_tr_br_bl
from window_cleaner.window_geometry import compute_homography, pixel_to_window


def test_order_corners_tl_tr_br_bl():
    points = np.array([[530, 400], [100, 100], [80, 420], [500, 80]], dtype=np.float32)
    corners = order_corners_tl_tr_br_bl(points)
    assert corners.top_left == (100.0, 100.0)
    assert corners.top_right == (500.0, 80.0)
    assert corners.bottom_right == (530.0, 400.0)
    assert corners.bottom_left == (80.0, 420.0)


def test_detect_window_frame_on_synthetic_quadrilateral():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    polygon = np.array([[100, 90], [520, 105], [500, 390], [120, 410]], dtype=np.int32)
    cv2.fillPoly(frame, [polygon], (245, 245, 245))
    cv2.polylines(frame, [polygon], True, (255, 255, 255), 4)

    detection = detect_window_frame(frame)

    assert detection is not None
    corners = detection.corners
    expected = {
        "top_left": (100, 90),
        "top_right": (520, 105),
        "bottom_right": (500, 390),
        "bottom_left": (120, 410),
    }
    actual = {
        "top_left": corners.top_left,
        "top_right": corners.top_right,
        "bottom_right": corners.bottom_right,
        "bottom_left": corners.bottom_left,
    }
    for name, point in actual.items():
        ex = expected[name]
        assert abs(point[0] - ex[0]) <= 8
        assert abs(point[1] - ex[1]) <= 8


def test_detected_corners_support_metric_homography():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    polygon = np.array([[100, 90], [520, 105], [500, 390], [120, 410]], dtype=np.int32)
    cv2.fillPoly(frame, [polygon], (255, 255, 255))

    detection = detect_window_frame(frame)
    assert detection is not None

    homography = compute_homography(detection.corners, width_cm=120.0, height_cm=80.0)
    x_cm, y_cm = pixel_to_window((310.0, 250.0), homography)

    assert 0.0 <= x_cm <= 120.0
    assert 0.0 <= y_cm <= 80.0
