"""Numerical tests for the hand-eye solver.

The core check is a closed loop: build a synthetic data set from a known
T_flange_camera, feed the same samples the ROS node would produce, and require
the solver to recover the ground truth.
"""

import math

import numpy as np
import pytest

from flexiv_handeye_calib import geometry as geom
from flexiv_handeye_calib import solver
from flexiv_handeye_calib.solver import HandEyeSample


def _random_rotation(rng, max_angle_deg):
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    angle = math.radians(rng.uniform(-max_angle_deg, max_angle_deg))
    return geom.exp_so3(axis * angle)


def make_dataset(rng, count=15, pose_noise_mm=0.0, pose_noise_deg=0.0,
                 marker_noise_mm=0.0, marker_noise_deg=0.0):
    """Synthetic eye-in-hand observations around a nominal top grasp."""
    X_true = geom.make_transform(
        geom.euler_to_matrix([0.0, -0.6, 1.5708]),
        [0.045, -0.02, 0.075],
    )
    Y_true = geom.make_transform(
        geom.euler_to_matrix([math.pi, 0.0, 0.3]),
        [0.55, 0.05, 0.02],
    )

    nominal = geom.make_transform(geom.euler_to_matrix([math.pi, 0.0, 0.0]), [0.55, 0.05, 0.40])
    samples = []
    for index in range(count):
        # tilt the wrist around a point in front of the camera so the marker
        # stays visible, like the operator would do on the real robot
        delta = _random_rotation(rng, 30.0)
        lever = np.array([0.0, 0.0, 0.35])
        position = nominal[:3, 3] + nominal[:3, :3] @ (lever - delta @ lever)
        A = geom.make_transform(delta @ nominal[:3, :3], position)
        B = geom.invert(X_true) @ geom.invert(A) @ Y_true

        A = _perturb(A, rng, pose_noise_mm, pose_noise_deg)
        B = _perturb(B, rng, marker_noise_mm, marker_noise_deg)
        samples.append(HandEyeSample(base_flange=A, cam_target=B, meta={"index": index}))
    return samples, X_true, Y_true


def _perturb(transform, rng, noise_mm, noise_deg):
    if noise_mm == 0.0 and noise_deg == 0.0:
        return transform
    delta_r = geom.exp_so3(rng.normal(size=3) * math.radians(noise_deg))
    delta_t = rng.normal(size=3) * noise_mm / 1000.0
    noise = geom.make_transform(delta_r, delta_t)
    return transform @ noise


def transform_mismatch(estimate, truth):
    delta = geom.invert(truth) @ estimate
    return (
        float(np.linalg.norm(geom.translation_of(delta)) * 1000.0),
        math.degrees(geom.rotation_angle(geom.rotation_of(delta))),
    )


# --------------------------------------------------------------------------- #
# geometry
# --------------------------------------------------------------------------- #
def test_euler_roundtrip():
    rng = np.random.default_rng(3)
    for _ in range(50):
        rpy = rng.uniform(-3.0, 3.0, size=3)
        rotation = geom.euler_to_matrix(rpy)
        recovered = geom.matrix_to_euler(rotation)
        assert np.allclose(geom.euler_to_matrix(recovered), rotation, atol=1e-9)


def test_quaternion_roundtrip():
    rng = np.random.default_rng(4)
    for _ in range(50):
        rotation = geom.exp_so3(rng.normal(size=3))
        recovered = geom.quat_to_matrix(geom.matrix_to_quat(rotation))
        assert np.allclose(recovered, rotation, atol=1e-9)


def test_log_exp_roundtrip():
    rng = np.random.default_rng(5)
    for _ in range(50):
        omega = rng.normal(size=3) * 0.8
        assert np.allclose(geom.log_so3(geom.exp_so3(omega)), omega, atol=1e-9)


def test_near_pi_rotation():
    rotation = geom.exp_so3(np.array([math.pi - 1e-7, 0.0, 0.0]))
    assert np.allclose(geom.exp_so3(geom.log_so3(rotation)), rotation, atol=1e-6)


# --------------------------------------------------------------------------- #
# solver
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("method", ["TSAI", "PARK", "HORAUD", "ANDREFF", "DANIILIDIS", "PARK_LS"])
def test_recovers_ground_truth_without_noise(method):
    rng = np.random.default_rng(11)
    samples, X_true, _ = make_dataset(rng, count=12)
    result = solver.solve(samples, method=method, do_refine=True, reject_outliers=False)
    translation_mm, rotation_deg = transform_mismatch(result.X, X_true)
    assert translation_mm < 1e-4, f"{method}: {translation_mm} mm"
    assert rotation_deg < 1e-4, f"{method}: {rotation_deg} deg"


def test_recovers_ground_truth_with_noise():
    rng = np.random.default_rng(12)
    samples, X_true, Y_true = make_dataset(
        rng, count=20, pose_noise_mm=0.2, pose_noise_deg=0.02,
        marker_noise_mm=0.4, marker_noise_deg=0.1,
    )
    result = solver.solve(samples, method="PARK", do_refine=True, reject_outliers=False)
    translation_mm, rotation_deg = transform_mismatch(result.X, X_true)
    assert translation_mm < 3.0, f"translation error {translation_mm:.2f} mm"
    assert rotation_deg < 0.5, f"rotation error {rotation_deg:.3f} deg"
    assert result.metrics["translation_rms_mm"] < 1.0
    # the nuisance target pose should come out as well
    target_mm, target_deg = transform_mismatch(result.Y, Y_true)
    assert target_mm < 3.0 and target_deg < 0.5


def test_leave_one_out_is_sane():
    rng = np.random.default_rng(13)
    samples, _, _ = make_dataset(
        rng, count=16, pose_noise_mm=0.2, pose_noise_deg=0.02,
        marker_noise_mm=0.4, marker_noise_deg=0.1,
    )
    result = solver.solve(samples, method="PARK")
    loo = result.metrics["loo_translation_rms_mm"]
    assert math.isfinite(loo)
    assert loo < 3.0, f"leave-one-out rms {loo:.2f} mm"


def test_outlier_is_rejected():
    rng = np.random.default_rng(14)
    samples, X_true, _ = make_dataset(
        rng, count=14, pose_noise_mm=0.1, pose_noise_deg=0.01,
        marker_noise_mm=0.3, marker_noise_deg=0.05,
    )
    # corrupt one sample by 3 cm, as a mis-detected tag would
    samples[4].cam_target[:3, 3] += np.array([0.03, 0.0, 0.0])
    result = solver.solve(samples, method="PARK", reject_outliers=True)
    assert 4 in result.outliers
    translation_mm, _ = transform_mismatch(result.X, X_true)
    assert translation_mm < 2.0


def test_pure_translation_is_rejected():
    """Translating the arm alone cannot constrain the rotation: refuse to guess."""
    X_true = geom.make_transform(geom.euler_to_matrix([0.1, 0.2, 0.3]), [0.05, 0.0, 0.08])
    Y_true = geom.make_transform(np.eye(3), [0.5, 0.0, 0.1])
    samples = []
    for index in range(8):
        A = geom.make_transform(np.eye(3), [0.5 + 0.02 * index, 0.0, 0.4])
        B = geom.invert(X_true) @ geom.invert(A) @ Y_true
        samples.append(HandEyeSample(base_flange=A, cam_target=B))
    with pytest.raises(Exception):
        solver.solve(samples, method="PARK")


def test_low_rotation_diversity_is_flagged():
    rng = np.random.default_rng(16)
    X_true = geom.make_transform(geom.euler_to_matrix([0.0, -0.6, 1.5708]), [0.045, -0.02, 0.075])
    Y_true = geom.make_transform(geom.euler_to_matrix([math.pi, 0.0, 0.3]), [0.55, 0.05, 0.02])
    nominal = geom.make_transform(geom.euler_to_matrix([math.pi, 0.0, 0.0]), [0.55, 0.05, 0.40])
    samples = []
    for index in range(10):
        delta = _random_rotation(rng, 4.0)
        A = nominal @ geom.make_transform(delta, np.zeros(3))
        B = geom.invert(X_true) @ geom.invert(A) @ Y_true
        samples.append(HandEyeSample(base_flange=A, cam_target=B))
    result = solver.solve(samples, method="PARK", do_refine=False)
    assert "rotation diversity" in result.message
