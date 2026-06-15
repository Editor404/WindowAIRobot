import json

import cv2
import numpy as np

from window_cleaner.window_bim import build_rectangular_bim, estimate_window_pose, save_bim_result
from window_cleaner.window_geometry import WindowCornersPx


def test_build_rectangular_bim_from_map_bottom_left():
    bim = build_rectangular_bim(120.0, 80.0, map_bottom_left=(10.0, 20.0))

    assert (bim.bottom_left.x_cm, bim.bottom_left.y_cm) == (10.0, 20.0)
    assert (bim.top_left.x_cm, bim.top_left.y_cm) == (10.0, 100.0)
    assert (bim.top_right.x_cm, bim.top_right.y_cm) == (130.0, 100.0)
    assert (bim.bottom_right.x_cm, bim.bottom_right.y_cm) == (130.0, 20.0)


def test_estimate_window_pose_from_detected_rectangle():
    camera_matrix = np.array(
        [[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    object_points = np.array(
        [[0.0, 80.0, 0.0], [120.0, 80.0, 0.0], [120.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        dtype=np.float64,
    )
    expected_rotation = np.array([[0.08], [-0.12], [0.03]], dtype=np.float64)
    expected_translation = np.array([[-60.0], [-40.0], [350.0]], dtype=np.float64)
    projected, _ = cv2.projectPoints(
        object_points,
        expected_rotation,
        expected_translation,
        camera_matrix,
        np.zeros((1, 5)),
    )
    points = projected.reshape(-1, 2)
    corners = WindowCornersPx(
        top_left=tuple(points[0]),
        top_right=tuple(points[1]),
        bottom_right=tuple(points[2]),
        bottom_left=tuple(points[3]),
    )

    pose = estimate_window_pose(corners, 120.0, 80.0, camera_matrix)

    assert pose.reprojection_rmse_px < 1e-4
    assert np.allclose(pose.translation_vector_cm, expected_translation.reshape(3), atol=1e-3)


def test_save_bim_result_includes_top_corner_observations(tmp_path):
    bim = build_rectangular_bim(120.0, 80.0)
    camera_matrix = np.array(
        [[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    corners = WindowCornersPx(
        top_left=(220.0, 140.0),
        top_right=(420.0, 140.0),
        bottom_right=(420.0, 340.0),
        bottom_left=(220.0, 340.0),
    )
    pose = estimate_window_pose(corners, 120.0, 80.0, camera_matrix)
    output = tmp_path / "window_bim.json"

    save_bim_result(output, bim, pose, corners.top_left, corners.top_right)
    data = json.loads(output.read_text(encoding="utf-8"))

    assert data["bim_2d"]["bottom_left"] == {"x_cm": 0.0, "y_cm": 0.0}
    assert data["source_observations"]["map_origin_cm"] == [0.0, 0.0]
    assert data["source_observations"]["camera_map_position_cm"] == [15.0, 30.0]
    assert data["source_observations"]["detected_top_left_px"] == [220.0, 140.0]
