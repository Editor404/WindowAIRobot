from __future__ import annotations

import argparse
from pathlib import Path

import cv2

from coordinate import CameraCalibration, dirt_absolute_coordinate
from detect_dirt import DirtSegmenter
from robot_controller import RobotController


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Detect dirt and move robot to the dirt coordinate.")
    parser.add_argument("--model", default="seg_best.pt", help="YOLOv8 segmentation model path.")
    parser.add_argument("--image", help="Input image path. If omitted, camera is used.")
    parser.add_argument("--camera", type=int, default=0, help="Camera index for live capture.")
    parser.add_argument("--conf", type=float, default=0.25, help="YOLO confidence threshold.")
    parser.add_argument("--cm-per-pixel-x", type=float, required=True)
    parser.add_argument("--cm-per-pixel-y", type=float, required=True)
    parser.add_argument("--camera-offset-x", type=float, default=0.0)
    parser.add_argument("--camera-offset-y", type=float, default=0.0)
    parser.add_argument("--device", help="Ultralytics device value, e.g. cpu, 0, cuda:0.")
    return parser.parse_args()


def read_frame(image_path: str | None, camera_index: int):
    if image_path:
        image = cv2.imread(str(Path(image_path)))
        if image is None:
            raise FileNotFoundError(f"Could not read image: {image_path}")
        return image

    capture = cv2.VideoCapture(camera_index)
    try:
        ok, frame = capture.read()
    finally:
        capture.release()

    if not ok:
        raise RuntimeError(f"Could not read frame from camera index {camera_index}")
    return frame


def main() -> None:
    args = parse_args()

    robot = RobotController()
    robot.home()

    frame = read_frame(args.image, args.camera)
    height, width = frame.shape[:2]

    calibration = CameraCalibration(
        image_width_px=width,
        image_height_px=height,
        cm_per_pixel_x=args.cm_per_pixel_x,
        cm_per_pixel_y=args.cm_per_pixel_y,
        camera_offset_x_cm=args.camera_offset_x,
        camera_offset_y_cm=args.camera_offset_y,
    )

    segmenter = DirtSegmenter(
        model_path=args.model,
        confidence_threshold=args.conf,
        device=args.device,
    )
    detection = segmenter.detect_largest(frame)
    if detection is None:
        print("No dirt detected.")
        return

    target = dirt_absolute_coordinate(
        robot_pose=robot.current_pose,
        center_px=detection.center_px,
        calibration=calibration,
    )

    print(
        "Dirt detected: "
        f"class={detection.class_id}, "
        f"conf={detection.confidence:.2f}, "
        f"center_px=({detection.center_px[0]:.1f}, {detection.center_px[1]:.1f}), "
        f"area_px={detection.area_px}"
    )
    print(f"Target coordinate: x={target.x_cm:.2f} cm, y={target.y_cm:.2f} cm")

    robot.move_to(target)
    robot.clean()


if __name__ == "__main__":
    main()
