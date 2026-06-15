from __future__ import annotations

import argparse
from pathlib import Path

import cv2

from window_cleaner.camera_calibration import load_camera_calibration
from window_cleaner.paths import default_camera_calibration_path, default_window_model_path
from window_cleaner.window_frame_detector import draw_window_detection
from window_cleaner.window_segmentation_detector import (
    WindowSegmentationDetector,
    WindowSegmentationDetectorConfig,
)
from window_cleaner.window_bim import build_rectangular_bim, estimate_window_pose, save_bim_result


def main() -> None:
    parser = argparse.ArgumentParser(description="Detect window corners with best.pt.")
    parser.add_argument("--image", required=True)
    parser.add_argument("--model", default=str(default_window_model_path()))
    parser.add_argument("--camera-calibration", default=str(default_camera_calibration_path()))
    parser.add_argument("--calibration-width", type=int, default=640)
    parser.add_argument("--calibration-height", type=int, default=480)
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument("--output", default="window_corners_detected.jpg")
    parser.add_argument("--window-width-cm", type=float)
    parser.add_argument("--window-height-cm", type=float)
    parser.add_argument("--map-bottom-left-x-cm", type=float, default=0.0)
    parser.add_argument("--map-bottom-left-y-cm", type=float, default=0.0)
    parser.add_argument("--camera-map-x-cm", type=float, default=15.0)
    parser.add_argument("--camera-map-y-cm", type=float, default=30.0)
    parser.add_argument("--bim-output", default="window_bim.json")
    args = parser.parse_args()

    dimensions = (args.window_width_cm, args.window_height_cm)
    if (dimensions[0] is None) != (dimensions[1] is None):
        parser.error("--window-width-cm and --window-height-cm must be provided together")

    frame = cv2.imread(args.image)
    if frame is None:
        raise FileNotFoundError(f"Could not read image: {args.image}")

    camera_calibration = load_camera_calibration(
        args.camera_calibration,
        calibration_width=args.calibration_width,
        calibration_height=args.calibration_height,
    )
    undistorted = camera_calibration.undistort(frame)
    detector = WindowSegmentationDetector(
        args.model,
        WindowSegmentationDetectorConfig(confidence_threshold=args.confidence),
    )
    detection = detector.detect(undistorted)
    if detection is None:
        raise RuntimeError("Window/frame segmentation did not produce four corners")

    for label, point in zip(("TL", "TR", "BR", "BL"), detection.corners.ordered()):
        print(f"{label}: ({point[0]:.1f}, {point[1]:.1f})")

    if args.window_width_cm is not None and args.window_height_cm is not None:
        height, width = undistorted.shape[:2]
        camera_matrix = camera_calibration.camera_matrix_for_image(width, height)
        pose = estimate_window_pose(
            image_corners=detection.corners,
            width_cm=args.window_width_cm,
            height_cm=args.window_height_cm,
            camera_matrix=camera_matrix,
        )
        bim = build_rectangular_bim(
            width_cm=args.window_width_cm,
            height_cm=args.window_height_cm,
            map_bottom_left=(args.map_bottom_left_x_cm, args.map_bottom_left_y_cm),
        )
        save_bim_result(
            path=args.bim_output,
            bim=bim,
            pose=pose,
            detected_top_left_px=detection.corners.top_left,
            detected_top_right_px=detection.corners.top_right,
            camera_map_position_cm=(args.camera_map_x_cm, args.camera_map_y_cm),
        )
        print(
            "PnP: "
            f"t_cm=({pose.translation_vector_cm[0]:.2f}, "
            f"{pose.translation_vector_cm[1]:.2f}, "
            f"{pose.translation_vector_cm[2]:.2f}), "
            f"reprojection_rmse={pose.reprojection_rmse_px:.3f}px"
        )
        print(f"Saved 2D BIM: {args.bim_output}")

    output = draw_window_detection(undistorted, detection)
    if not cv2.imwrite(str(Path(args.output)), output):
        raise RuntimeError(f"Could not write output: {args.output}")
    print(f"Saved: {args.output}")


if __name__ == "__main__":
    main()
