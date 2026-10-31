"""AprilTag / ArUco detection and pose estimation.

Kept independent from ROS so it can be exercised with a rendered image in tests.

Marker frame: origin at the centre of the black square, x right, y "up" on the
printed tag, z along the outward normal (towards the camera).  That is the same
convention ``grasp_perception`` uses, and it is what ``solvePnP`` gives when the
object points are ordered top-left, top-right, bottom-right, bottom-left.
"""

from __future__ import annotations

from typing import List, Optional

import cv2
import numpy as np

from .detection import Detection, polygon_area, select_detection

FAMILIES = {
    "16h5": "DICT_APRILTAG_16H5",
    "25h9": "DICT_APRILTAG_25H9",
    "36h11": "DICT_APRILTAG_36H11",
    "4x4_50": "DICT_4X4_50",
    "4x4_100": "DICT_4X4_100",
    "5x5_50": "DICT_5X5_50",
    "5x5_100": "DICT_5X5_100",
    "6x6_250": "DICT_6X6_250",
    "7x7_1000": "DICT_7X7_1000",
}


def _object_points(size: float) -> np.ndarray:
    half = size / 2.0
    return np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )


class TagDetector:
    def __init__(self, family: str = "25h9", tag_size: float = 0.026):
        key = FAMILIES.get(family.lower())
        if key is None:
            raise ValueError(f"unknown tag family '{family}', known: {sorted(FAMILIES)}")
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, key))
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize = 5
        params.cornerRefinementMaxIterations = 50
        params.cornerRefinementMinAccuracy = 0.05
        # 25h9 modules are small; search a wider window than the default 23 px
        params.adaptiveThreshWinSizeMin = 5
        params.adaptiveThreshWinSizeMax = 35
        params.adaptiveThreshWinSizeStep = 4
        self._detector = cv2.aruco.ArucoDetector(dictionary, params)
        self.tag_size = float(tag_size)
        self.family = family.lower()
        self._object_points = _object_points(self.tag_size)

    def describe(self) -> str:
        return f"apriltag {self.family} {self.tag_size * 1000:.1f} mm"

    def detect(self, image_bgr: np.ndarray, camera_matrix: np.ndarray, dist_coeffs) -> List[Detection]:
        gray = image_bgr if image_bgr.ndim == 2 else cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)
        if ids is None or len(ids) == 0:
            return []

        k = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
        if dist_coeffs is None or np.asarray(dist_coeffs).size == 0:
            d = np.zeros((5, 1))
        else:
            d = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1, 1)
        results: List[Detection] = []
        for corner, marker_id in zip(corners, ids.flatten()):
            image_points = corner.reshape((4, 2)).astype(np.float64)
            solved = self._solve_pnp(image_points, k, d)
            if solved is None:
                continue
            rvec, tvec, error = solved
            rotation, _ = cv2.Rodrigues(rvec)
            transform = np.eye(4)
            transform[:3, :3] = rotation
            transform[:3, 3] = np.asarray(tvec).reshape(3)
            results.append(
                Detection(
                    marker_id=int(marker_id),
                    transform=transform,
                    reprojection_error_px=error,
                    area_px=float(polygon_area(image_points)),
                    corners=image_points,
                )
            )
        return results

    # ------------------------------------------------------------------ #
    def _reprojection_error(self, rvec, tvec, image_points, k, d) -> float:
        projected, _ = cv2.projectPoints(self._object_points, rvec, tvec, k, d)
        return float(
            np.sqrt(np.mean(np.sum((projected.reshape((4, 2)) - image_points) ** 2, axis=1)))
        )

    def _solve_pnp(self, image_points, k, d):
        """Homography-initialised solve + LM refinement.

        ``SOLVEPNP_IPPE_SQUARE`` is deliberately not used: on OpenCV 4.13 it
        returns the pose with the marker axes flipped for this object-point
        order (reprojection error ~ the marker size), which is worse than
        useless here.  The iterative solver handles a planar square well.
        """
        ok, rvec, tvec = cv2.solvePnP(
            self._object_points, image_points, k, d, flags=cv2.SOLVEPNP_ITERATIVE
        )
        if not ok:
            return None
        error = self._reprojection_error(rvec, tvec, image_points, k, d)
        refined_rvec = rvec.copy()
        refined_tvec = tvec.copy()
        try:
            cv2.solvePnPRefineLM(
                self._object_points, image_points, k, d, refined_rvec, refined_tvec
            )
            refined_error = self._reprojection_error(
                refined_rvec, refined_tvec, image_points, k, d
            )
            if refined_error <= error:
                rvec, tvec, error = refined_rvec, refined_tvec, refined_error
        except cv2.error:
            pass
        if not np.isfinite(error) or error > 5.0:
            return None
        return rvec, tvec, error


__all__ = ["Detection", "TagDetector", "select_detection", "FAMILIES"]
