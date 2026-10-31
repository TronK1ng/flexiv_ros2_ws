"""Chessboard target tests: rendering, detection and symmetry handling."""

import math

import cv2
import numpy as np
import pytest

from flexiv_handeye_calib import geometry as geom
from flexiv_handeye_calib import solver
from flexiv_handeye_calib.chessboard import ChessboardDetector, symmetry_transforms
from flexiv_handeye_calib.detection import select_detection
from flexiv_handeye_calib.solver import HandEyeSample

CAMERA_MATRIX = np.array([[920.0, 0.0, 640.0], [0.0, 920.0, 360.0], [0.0, 0.0, 1.0]])
IMAGE_SIZE = (1280, 720)
R_FRONT = np.diag([1.0, -1.0, -1.0])


def render_board(transform_cam_board, cols=5, rows=5, square=0.012, px_per_m=2000):
    """Draw a chessboard with `cols` x `rows` inner corners at a known pose."""
    squares_x = cols + 1
    squares_y = rows + 1
    margin = 1  # one white square of quiet zone around the pattern
    tex_w = (squares_x + 2 * margin) * square
    tex_h = (squares_y + 2 * margin) * square
    width = int(round(tex_w * px_per_m))
    height = int(round(tex_h * px_per_m))
    side = int(round(square * px_per_m))

    texture = np.full((height, width), 255, dtype=np.uint8)
    for j in range(squares_y):
        for i in range(squares_x):
            if (i + j) % 2 == 0:
                y0 = (j + margin) * side
                x0 = (i + margin) * side
                texture[y0:y0 + side, x0:x0 + side] = 0

    half_x = (squares_x / 2.0 + margin) * square
    half_y = (squares_y / 2.0 + margin) * square
    # texture top-left is board (-x, +y): board y points "up"
    corners = np.array(
        [[-half_x, half_y, 0.0], [half_x, half_y, 0.0],
         [half_x, -half_y, 0.0], [-half_x, -half_y, 0.0]],
        dtype=np.float64,
    )
    rvec, _ = cv2.Rodrigues(transform_cam_board[:3, :3])
    projected, _ = cv2.projectPoints(
        corners, rvec, transform_cam_board[:3, 3], CAMERA_MATRIX, None
    )
    projected = projected.reshape(4, 2).astype(np.float32)

    source = np.array(
        [[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]],
        dtype=np.float32,
    )
    homography = cv2.getPerspectiveTransform(source, projected)
    warped = cv2.warpPerspective(
        texture, homography, IMAGE_SIZE, borderValue=255
    )
    return np.dstack([warped, warped, warped])


def _rotation_mod_90(rotation):
    angle = math.degrees(geom.rotation_angle(rotation)) % 90.0
    return min(angle, 90.0 - angle)


@pytest.mark.parametrize(
    "translation,rpy",
    [
        ([0.0, 0.0, 0.35], [0.0, 0.0, 0.0]),
        ([0.02, -0.01, 0.30], [0.0, 0.0, 0.5]),
        ([0.0, 0.0, 0.40], [0.5, -0.3, 0.0]),
        ([0.0, 0.0, 0.32], [-0.6, 0.4, 0.2]),
    ],
)
def test_board_pose_recovered(translation, rpy):
    truth = geom.make_transform(R_FRONT @ geom.euler_to_matrix(rpy), translation)
    image = render_board(truth)
    detector = ChessboardDetector(cols=5, rows=5, square_size=0.012, auto_pattern=False)
    detections = detector.detect(image, CAMERA_MATRIX, None)
    assert detections, "board not detected"
    best = detections[0]
    assert best.reprojection_error_px < 1.5

    # the frame origin is the pattern centre, so a symmetry flip does not move it
    translation_error = float(
        np.linalg.norm(geom.translation_of(best.transform) - translation) * 1000.0
    )
    assert translation_error < 3.0, f"{translation_error:.2f} mm"

    delta = geom.invert(truth) @ best.transform
    assert _rotation_mod_90(geom.rotation_of(delta)) < 1.0


def test_auto_pattern_finds_the_board():
    truth = geom.make_transform(R_FRONT, [0.0, 0.0, 0.35])
    image = render_board(truth, cols=6, rows=6, square=0.012)
    detector = ChessboardDetector(cols=4, rows=4, square_size=0.012, auto_pattern=True)
    detections = detector.detect(image, CAMERA_MATRIX, None)
    assert detections
    assert detector.last_pattern == (6, 6), detector.last_pattern


def test_square_size_error_shows_up_as_range_error():
    truth = geom.make_transform(R_FRONT, [0.0, 0.0, 0.35])
    image = render_board(truth)
    detector = ChessboardDetector(cols=5, rows=5, square_size=0.012 * 1.1, auto_pattern=False)
    best = detector.detect(image, CAMERA_MATRIX, None)[0]
    assert abs(geom.translation_of(best.transform)[2] - 0.35) > 0.02


# --------------------------------------------------------------------------- #
# symmetry
# --------------------------------------------------------------------------- #
def make_chessboard_dataset(rng, count=12, flip_fraction=0.0):
    X_true = geom.make_transform(geom.euler_to_matrix([0.0, -0.5, 1.5708]), [0.04, -0.02, 0.07])
    Y_true = geom.make_transform(geom.euler_to_matrix([math.pi, 0, 0.2]), [0.5, 0.02, 0.02])
    nominal = geom.make_transform(geom.euler_to_matrix([math.pi, 0, 0]), [0.5, 0.02, 0.4])
    symmetries = symmetry_transforms(5, 5)
    samples = []
    flips = []
    for index in range(count):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        delta = geom.exp_so3(axis * math.radians(rng.uniform(-30, 30)))
        lever = np.array([0.0, 0.0, 0.35])
        position = nominal[:3, 3] + nominal[:3, :3] @ (lever - delta @ lever)
        A = geom.make_transform(delta @ nominal[:3, :3], position)
        B = geom.invert(X_true) @ geom.invert(A) @ Y_true
        if rng.random() < flip_fraction:
            B = B @ symmetries[int(rng.integers(1, len(symmetries)))]
            flips.append(index)
        samples.append(HandEyeSample(base_flange=A, cam_target=B))
    return samples, X_true, flips


def test_symmetry_alignment_recovers_flipped_samples():
    rng = np.random.default_rng(31)
    samples, X_true, flips = make_chessboard_dataset(rng, count=14, flip_fraction=0.35)
    assert flips, "test data should contain flips"

    aligned, detected = solver.align_symmetric_targets(samples, symmetry_transforms(5, 5))
    assert set(detected) == set(flips), f"detected {detected}, expected {flips}"

    result = solver.solve(aligned, method="PARK")
    delta = geom.invert(result.X) @ X_true
    assert np.linalg.norm(geom.translation_of(delta)) * 1000.0 < 2.0
    assert math.degrees(geom.rotation_angle(geom.rotation_of(delta))) < 0.3


def test_alignment_is_a_noop_on_consistent_data():
    rng = np.random.default_rng(32)
    samples, _, _ = make_chessboard_dataset(rng, count=12, flip_fraction=0.0)
    aligned, flips = solver.align_symmetric_targets(samples, symmetry_transforms(5, 5))
    assert flips == []
    for before, after in zip(samples, aligned):
        assert np.allclose(before.cam_target, after.cam_target)


def test_selection_prefers_larger_board():
    truth = geom.make_transform(R_FRONT, [0.0, 0.0, 0.35])
    image = render_board(truth, cols=6, rows=6)
    detector = ChessboardDetector(cols=6, rows=6, square_size=0.012, auto_pattern=False)
    best = select_detection(detector.detect(image, CAMERA_MATRIX, None), 0, 100.0, 5.0)
    assert best is not None and best.area_px > 1000
