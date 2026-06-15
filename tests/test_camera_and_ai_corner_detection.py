import cv2
import numpy as np

from window_cleaner.camera_calibration import CameraCalibrationParameters, load_camera_calibration
from window_cleaner.window_segmentation_detector import detect_window_from_class_masks


def test_load_camera_calibration_and_scale_matrix():
    calibration = load_camera_calibration("calib_parameters.npz")
    scaled = calibration.camera_matrix_for_image(1280, 960)

    assert scaled[0, 0] == calibration.camera_matrix[0, 0] * 2
    assert scaled[1, 1] == calibration.camera_matrix[1, 1] * 2
    assert scaled[0, 2] == calibration.camera_matrix[0, 2] * 2
    assert scaled[1, 2] == calibration.camera_matrix[1, 2] * 2


def test_undistort_preserves_image_shape():
    calibration = CameraCalibrationParameters(
        camera_matrix=np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]]),
        distortion_coefficients=np.zeros((1, 5)),
    )
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    assert calibration.undistort(frame).shape == frame.shape


def test_detect_window_from_segmentation_masks():
    frame_mask = np.zeros((480, 640), dtype=np.uint8)
    glass_mask = np.zeros((480, 640), dtype=np.uint8)
    polygon = np.array([[110, 90], [530, 110], [505, 400], [125, 420]], dtype=np.int32)
    cv2.fillPoly(glass_mask, [polygon], 255)
    cv2.polylines(frame_mask, [polygon], True, 255, 12)

    detection = detect_window_from_class_masks(frame_mask, glass_mask)

    assert detection is not None
    expected = [(110, 90), (530, 110), (505, 400), (125, 420)]
    for actual, target in zip(detection.corners.ordered(), expected):
        assert abs(actual[0] - target[0]) <= 12
        assert abs(actual[1] - target[1]) <= 12
