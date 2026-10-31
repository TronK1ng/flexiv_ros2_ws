"""Detector tests that render a synthetic AprilTag at a known pose.

The tag image is warped with the homography induced by the ground truth pose, so
the recovered PnP pose can be compared against an exact reference.  This checks
the intrinsics handling, the corner order and - important - the definition of
``tag_size`` (side of the black square, border included).
"""

import cv2
import numpy as np
import pytest

from flexiv_handeye_calib import geometry as geom
from flexiv_handeye_calib.tag_detector import TagDetector, select_detection

# roughly a RealSense D415 colour stream at 1280x720, which is what the docs recommend
CAMERA_MATRIX = np.array([[920.0, 0.0, 640.0], [0.0, 920.0, 360.0], [0.0, 0.0, 1.0]])
IMAGE_SIZE = (1280, 720)
TAG_SIZE = 0.026
# fronto-parallel tag: tag frame is x-right, y-up, z-out-of-the-tag, camera is
# x-right, y-down, z-forward, hence the flip of y and z
R_FRONT = np.diag([1.0, -1.0, -1.0])


def render_tag(transform_cam_tag, tag_id=7, padding_ratio=0.35):
    """Draw one tag at the given T_camera_tag, return the BGR image."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25H9)
    marker_px = 240
    marker = cv2.aruco.generateImageMarker(dictionary, tag_id, marker_px)
    pad = int(marker_px * padding_ratio)
    canvas_size = marker_px + 2 * pad
    canvas = np.full((canvas_size, canvas_size), 255, dtype=np.uint8)
    canvas[pad:pad + marker_px, pad:pad + marker_px] = marker

    # physical half size of the white canvas around the black square
    half_outer = TAG_SIZE / 2.0 * canvas_size / marker_px
    object_corners = np.array(
        [
            [-half_outer, half_outer, 0.0],
            [half_outer, half_outer, 0.0],
            [half_outer, -half_outer, 0.0],
            [-half_outer, -half_outer, 0.0],
        ],
        dtype=np.float64,
    )
    rvec, _ = cv2.Rodrigues(transform_cam_tag[:3, :3])
    tvec = transform_cam_tag[:3, 3]
    projected, _ = cv2.projectPoints(object_corners, rvec, tvec, CAMERA_MATRIX, None)
    projected = projected.reshape(4, 2).astype(np.float32)

    width, height = IMAGE_SIZE
    image = np.full((height, width, 3), 255, dtype=np.uint8)
    source = np.array(
        [[0, 0], [canvas_size - 1, 0], [canvas_size - 1, canvas_size - 1], [0, canvas_size - 1]],
        dtype=np.float32,
    )
    homography = cv2.getPerspectiveTransform(source, projected)
    warped = cv2.warpPerspective(canvas, homography, (width, height), borderValue=255)
    image[:, :, 0] = warped
    image[:, :, 1] = warped
    image[:, :, 2] = warped
    return image


@pytest.mark.parametrize(
    "pose",
    [
        ([0.0, 0.0, 0.35], [0.0, 0.0, 0.0]),
        ([0.0, 0.0, 0.30], [0.0, 0.0, 0.4]),
        ([0.04, -0.03, 0.40], [0.6, -0.4, 0.2]),
        ([0.0, 0.0, 0.45], [-0.7, 0.5, 0.0]),
    ],
)
def test_pose_recovered_from_rendered_tag(pose):
    translation, rpy = pose
    truth = geom.make_transform(R_FRONT @ geom.euler_to_matrix(rpy), translation)
    image = render_tag(truth)
    detector = TagDetector("25h9", TAG_SIZE)
    detections = detector.detect(image, CAMERA_MATRIX, None)
    assert detections, "tag not detected in rendered image"
    best = max(detections, key=lambda d: d.area_px)
    assert best.marker_id == 7
    assert best.reprojection_error_px < 1.0

    delta = geom.invert(truth) @ best.transform
    translation_mm = float(np.linalg.norm(geom.translation_of(delta)) * 1000.0)
    rotation_deg = np.degrees(geom.rotation_angle(geom.rotation_of(delta)))
    # the synthetic rasterisation itself has a ~0.3 px corner bias, which is
    # ~2 mm of range at 0.35 m; a strongly foreshortened tag loses orientation
    # accuracy, exactly like a real one
    assert translation_mm < 5.0, f"{translation_mm:.2f} mm"
    assert rotation_deg < 2.0, f"{rotation_deg:.3f} deg"


def test_wrong_tag_size_shows_up_as_range_error():
    truth = geom.make_transform(R_FRONT, [0.0, 0.0, 0.35])
    image = render_tag(truth)
    detector = TagDetector("25h9", TAG_SIZE * 1.1)  # 10 % too large
    best = max(detector.detect(image, CAMERA_MATRIX, None), key=lambda d: d.area_px)
    recovered = geom.translation_of(best.transform)[2]
    assert abs(recovered - 0.35) > 0.02, "a 10 % size error should shift the range by ~1 cm"


def test_select_detection_filters():
    truth = geom.make_transform(R_FRONT, [0.0, 0.0, 0.35])
    image = render_tag(truth, tag_id=7)
    detector = TagDetector("25h9", TAG_SIZE)
    detections = detector.detect(image, CAMERA_MATRIX, None)
    assert select_detection(detections, marker_id=7) is not None
    assert select_detection(detections, marker_id=99) is None
    assert select_detection(detections, min_area_px=1e9) is None
    assert select_detection(detections, max_reprojection_error_px=0.0) is None


def test_range_estimate_is_method_independent():
    """IPPE and the iterative solver must agree, otherwise the PnP branch is broken."""
    truth = geom.make_transform(R_FRONT, [0.0, 0.0, 0.35])
    image = render_tag(truth)
    detector = TagDetector("25h9", TAG_SIZE)
    detection = detector.detect(image, CAMERA_MATRIX, None)[0]
    points = detection.corners
    ok, rvec, tvec = cv2.solvePnP(
        np.array([[-TAG_SIZE / 2, TAG_SIZE / 2, 0], [TAG_SIZE / 2, TAG_SIZE / 2, 0],
                  [TAG_SIZE / 2, -TAG_SIZE / 2, 0], [-TAG_SIZE / 2, -TAG_SIZE / 2, 0]]),
        points, CAMERA_MATRIX, None, flags=cv2.SOLVEPNP_ITERATIVE)
    assert ok
    assert abs(tvec[2][0] - detection.transform[2, 3]) < 0.01
