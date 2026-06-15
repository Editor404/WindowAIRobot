from __future__ import annotations

import ast
from pathlib import Path

import numpy as np

from window_cleaner.window_geometry import CORNER_ORDER, WindowCalibration, WindowCornersPx, build_window_calibration

DEFAULT_CALIBRATION_PATH = Path("config/window_calibration.yaml")


def _format_optional_float(value: float | None) -> str:
    return "null" if value is None else repr(float(value))


def _format_pair(pair: tuple[float, float]) -> str:
    return f"[{float(pair[0])}, {float(pair[1])}]"


def save_window_calibration(path: str | Path, calibration: WindowCalibration) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    h = calibration.homography.tolist()
    lines = [
        "mode: auto",
        f"window_width_cm: {_format_optional_float(calibration.width_cm)}",
        f"window_height_cm: {_format_optional_float(calibration.height_cm)}",
        "image_corners_px:",
    ]
    for name, point in zip(CORNER_ORDER, calibration.image_corners_px.ordered()):
        lines.append(f"  {name}: {_format_pair(point)}")
    lines.append("homography:")
    for row in h:
        lines.append("  - [" + ", ".join(repr(float(value)) for value in row) + "]")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _parse_optional_float(value: str) -> float | None:
    value = value.strip()
    if value in {"", "null", "None", "~"}:
        return None
    return float(value)


def _parse_pair(value: str) -> tuple[float, float]:
    parsed = ast.literal_eval(value.strip())
    if not isinstance(parsed, (list, tuple)) or len(parsed) != 2:
        raise ValueError(f"Expected [x, y] pair, got: {value}")
    return float(parsed[0]), float(parsed[1])


def _parse_row(value: str) -> list[float]:
    parsed = ast.literal_eval(value.strip())
    if not isinstance(parsed, (list, tuple)) or len(parsed) != 3:
        raise ValueError(f"Expected homography row of 3 values, got: {value}")
    return [float(parsed[0]), float(parsed[1]), float(parsed[2])]


def load_window_calibration(path: str | Path = DEFAULT_CALIBRATION_PATH) -> WindowCalibration:
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Window calibration file not found: {path}")

    width_cm: float | None = None
    height_cm: float | None = None
    corners: dict[str, tuple[float, float]] = {}
    homography_rows: list[list[float]] = []
    section: str | None = None

    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        if line == "image_corners_px:":
            section = "corners"
            continue
        if line == "homography:":
            section = "homography"
            continue

        stripped = line.strip()
        if section == "corners" and ":" in stripped:
            name, value = stripped.split(":", 1)
            if name in CORNER_ORDER:
                corners[name] = _parse_pair(value)
            continue
        if section == "homography" and stripped.startswith("-"):
            homography_rows.append(_parse_row(stripped[1:].strip()))
            continue

        if ":" in stripped:
            key, value = stripped.split(":", 1)
            if key == "window_width_cm":
                width_cm = _parse_optional_float(value)
            elif key == "window_height_cm":
                height_cm = _parse_optional_float(value)

    missing = [name for name in CORNER_ORDER if name not in corners]
    if missing:
        raise ValueError(f"Calibration missing image corners: {', '.join(missing)}")

    image_corners = WindowCornersPx(
        top_left=corners["top_left"],
        top_right=corners["top_right"],
        bottom_right=corners["bottom_right"],
        bottom_left=corners["bottom_left"],
    )
    calibration = build_window_calibration(image_corners, width_cm=width_cm, height_cm=height_cm)

    if homography_rows:
        if len(homography_rows) != 3:
            raise ValueError("Homography must have exactly 3 rows")
        object.__setattr__(calibration, "homography", np.array(homography_rows, dtype=np.float32))

    return calibration
