import cv2
import numpy as np

from window_cleaner.detect_dirt import SimDirtDetector


def test_sim_dirt_detector_finds_dark_gazebo_like_dot_on_blue_glass():
    image = np.full((480, 640, 3), (150, 135, 120), dtype=np.uint8)
    cv2.circle(image, (320, 180), 14, (80, 80, 80), thickness=-1)

    detection = SimDirtDetector(min_area_px=8).detect_largest(image)

    assert detection is not None
    assert abs(detection.center_px[0] - 320) < 3
    assert abs(detection.center_px[1] - 180) < 3
    assert detection.area_px > 100


def test_sim_dirt_detector_ignores_uniform_glass_background():
    image = np.full((480, 640, 3), (150, 135, 120), dtype=np.uint8)

    assert SimDirtDetector(min_area_px=8).detect_largest(image) is None


def test_sim_dirt_detector_finds_multiple_visible_dots():
    image = np.full((480, 640, 3), (150, 135, 120), dtype=np.uint8)
    centers = [(120, 100), (320, 180), (500, 320)]
    for center in centers:
        cv2.circle(image, center, 12, (80, 80, 80), thickness=-1)

    detections = SimDirtDetector(min_area_px=8).detect(image)

    assert len(detections) == len(centers)
    detected_centers = sorted((round(d.center_px[0]), round(d.center_px[1])) for d in detections)
    assert detected_centers == sorted(centers)
