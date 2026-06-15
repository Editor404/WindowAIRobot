from __future__ import annotations

from pathlib import Path


def default_model_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        installed_model = Path(get_package_share_directory("window_cleaner")) / "models" / "seg_best.pt"
        if installed_model.exists():
            return installed_model
    except Exception:
        pass

    return Path.cwd() / "seg_best.pt"


def default_window_model_path() -> Path:
    return _installed_or_local_path("models", "best.pt")


def default_camera_calibration_path() -> Path:
    return _installed_or_local_path("config", "calib_parameters.npz")


def _installed_or_local_path(share_directory: str, filename: str) -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        installed_path = Path(get_package_share_directory("window_cleaner")) / share_directory / filename
        if installed_path.exists():
            return installed_path
    except Exception:
        pass

    return Path.cwd() / filename
