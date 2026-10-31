"""Chessboard (checkerboard) calibration target support.

The board frame is centred on the pattern, x along the corner columns, y along
the corner rows, z out of the board.  Using the centre as origin matters: a
plain square chessboard looks exactly the same after a 90 deg rotation about
its centre, so the detector may return any of four equivalent poses.  With the
origin at the centre those four candidates differ only by a rotation, which
lets :func:`align_symmetric_targets` pick the consistent branch later on.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .detection import Detection, convex_area

#: inner-corner counts to try when the board geometry is not configured; most
#: printed boards are in here.  (columns, rows)
COMMON_PATTERNS: Tuple[Tuple[int, int], ...] = (
    (5, 5), (6, 6), (7, 7), (8, 8), (9, 6), (6, 9), (11, 8), (8, 11),
    (7, 6), (6, 7), (8, 6), (6, 8), (9, 7), (7, 9), (4, 4), (5, 4), (4, 5),
    (10, 7), (12, 9), (9, 5), (5, 9), (6, 5), (5, 6),
)


def _object_points(cols: int, rows: int, square_size: float) -> np.ndarray:
    """Inner corners, centred on the pattern, in board coordinates."""
    xs = (np.arange(cols) - (cols - 1) / 2.0) * square_size
    ys = (np.arange(rows) - (rows - 1) / 2.0) * square_size
    grid = np.array([(x, y, 0.0) for y in ys for x in xs], dtype=np.float64)
    return grid


def symmetry_transforms(cols: int, rows: int) -> List[np.ndarray]:
    """Rotations about z that map the corner lattice onto itself.

    A square board is invariant under 90/180/270 deg; a rectangular one only
    under 180 deg.
    """
    candidates = [np.eye(4)]
    steps = (1, 2, 3) if cols == rows else (2,)
    for k in steps:
        angle = k * math.pi / 2.0
        transform = np.eye(4)
        transform[:3, :3] = np.array(
            [
                [math.cos(angle), -math.sin(angle), 0.0],
                [math.sin(angle), math.cos(angle), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        candidates.append(transform)
    return candidates


class ChessboardDetector:
    def __init__(
        self,
        cols: int = 8,
        rows: int = 11,
        square_size: float = 0.006,
        auto_pattern: bool = False,
    ):
        self.cols = int(cols)
        self.rows = int(rows)
        self.square_size = float(square_size)
        self.auto_pattern = bool(auto_pattern)
        self.last_pattern: Optional[Tuple[int, int]] = None
        self._object_cache = {}

    # ------------------------------------------------------------------ #
    def describe(self) -> str:
        return (
            f"chessboard {self.cols}x{self.rows} inner corners, "
            f"{self.square_size * 1000:.1f} mm squares"
        )

    def symmetry_candidates(self) -> List[np.ndarray]:
        return symmetry_transforms(self.cols, self.rows)

    def _object(self, cols, rows):
        key = (cols, rows)
        if key not in self._object_cache:
            self._object_cache[key] = _object_points(cols, rows, self.square_size)
        return self._object_cache[key]

    # ------------------------------------------------------------------ #
    def detect(self, image_bgr: np.ndarray, camera_matrix, dist_coeffs) -> List[Detection]:
        gray = image_bgr if image_bgr.ndim == 2 else cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        k = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
        if dist_coeffs is None or np.asarray(dist_coeffs).size == 0:
            d = np.zeros((5, 1))
        else:
            d = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1, 1)

        patterns = [(self.cols, self.rows)]
        if self.auto_pattern:
            for candidate in COMMON_PATTERNS:
                if candidate not in patterns:
                    patterns.append(candidate)

        best = None
        for cols, rows in patterns:
            found, corners = self._find(gray, cols, rows)
            if not found:
                continue
            corners = corners.reshape(-1, 2).astype(np.float64)
            solved = self._pose(corners, cols, rows, k, d)
            if solved is None:
                continue
            rvec, tvec, error = solved
            score = (cols * rows, -error)
            if best is None or score > best[0]:
                best = (score, cols, rows, corners, rvec, tvec, error)
            if not self.auto_pattern:
                break

        if best is None:
            return []

        _, cols, rows, corners, rvec, tvec, error = best
        self.last_pattern = (cols, rows)
        rotation, _ = cv2.Rodrigues(rvec)
        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = np.asarray(tvec).reshape(3)
        return [
            Detection(
                marker_id=0,
                transform=transform,
                reprojection_error_px=error,
                area_px=convex_area(corners),
                corners=corners,
                label=f"{cols}x{rows}",
            )
        ]

    # ------------------------------------------------------------------ #
    @staticmethod
    def _find(gray, cols, rows):
        try:
            found, corners = cv2.findChessboardCornersSB(
                gray,
                (cols, rows),
                flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY,
            )
            if found and corners is not None and len(corners) == cols * rows:
                return True, corners
        except cv2.error:
            pass

        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, (cols, rows), flags=flags)
        if not found or corners is None or len(corners) != cols * rows:
            return False, None
        corners = cv2.cornerSubPix(
            gray,
            corners.astype(np.float32),
            (5, 5),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.01),
        )
        return True, corners

    def _pose(self, image_points, cols, rows, k, d):
        object_points = self._object(cols, rows)
        ok, rvec, tvec = cv2.solvePnP(
            object_points, image_points, k, d, flags=cv2.SOLVEPNP_ITERATIVE
        )
        if not ok:
            return None
        error = self._error(object_points, rvec, tvec, image_points, k, d)
        refined_rvec = rvec.copy()
        refined_tvec = tvec.copy()
        try:
            cv2.solvePnPRefineLM(
                object_points, image_points, k, d, refined_rvec, refined_tvec
            )
            refined_error = self._error(
                object_points, refined_rvec, refined_tvec, image_points, k, d
            )
            if refined_error <= error:
                rvec, tvec, error = refined_rvec, refined_tvec, refined_error
        except cv2.error:
            pass
        if not np.isfinite(error) or error > 5.0:
            return None
        return rvec, tvec, error

    @staticmethod
    def _error(object_points, rvec, tvec, image_points, k, d) -> float:
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, k, d)
        return float(
            np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - image_points) ** 2, axis=1)))
        )


def _angle_of(rotation: np.ndarray) -> float:
    value = (np.trace(rotation) - 1.0) * 0.5
    return math.acos(max(-1.0, min(1.0, value)))
