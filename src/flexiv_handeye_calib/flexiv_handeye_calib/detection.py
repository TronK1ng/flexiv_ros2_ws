"""Shared detection result type and selection helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import numpy as np


@dataclass
class Detection:
    """One detected calibration target in the camera frame."""

    marker_id: int
    transform: np.ndarray  # T_camera_target, 4x4
    reprojection_error_px: float
    area_px: float
    corners: np.ndarray  # Nx2
    label: str = ""


def polygon_area(points: np.ndarray) -> float:
    x = np.asarray(points, dtype=float)[:, 0]
    y = np.asarray(points, dtype=float)[:, 1]
    return float(abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1))) * 0.5)


def convex_area(points: np.ndarray) -> float:
    import cv2

    hull = cv2.convexHull(np.asarray(points, dtype=np.float32).reshape(-1, 1, 2))
    return float(cv2.contourArea(hull))


def select_detection(
    detections: List[Detection],
    marker_id: Optional[int] = None,
    min_area_px: float = 0.0,
    max_reprojection_error_px: float = 5.0,
) -> Optional[Detection]:
    candidates = [
        d
        for d in detections
        if d.area_px >= min_area_px and d.reprojection_error_px <= max_reprojection_error_px
    ]
    if marker_id is not None and marker_id >= 0:
        candidates = [d for d in candidates if d.marker_id == marker_id]
    if not candidates:
        return None
    return max(candidates, key=lambda d: d.area_px)
