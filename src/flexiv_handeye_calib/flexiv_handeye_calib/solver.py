"""Eye-in-hand calibration solver:  A_i · X · B_i = Y.

``A_i`` is ``T_base_flange`` (robot forward kinematics, from tf),
``B_i`` is ``T_camera_target`` (marker pose from PnP),
``X``  is the unknown ``T_flange_camera`` and ``Y`` the (nuisance)
``T_base_target``.

The module deliberately avoids scipy.  Every estimator is pure numpy, so the
numerics can be checked against synthetic ground truth in the unit tests.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence

import numpy as np

from . import geometry as geom

# OpenCV method constants by name; imported lazily so that the pure-python
# fallback still works on machines without cv2.
_CV_METHODS = ("TSAI", "PARK", "HORAUD", "ANDREFF", "DANIILIDIS")
_ALL_METHODS = _CV_METHODS + ("PARK_LS",)

# rotation residuals are converted to metres with this lever arm so that a
# 1 deg error weighs about as much as a 5 mm position error during refinement
_ROT_SCALE_M = 0.3


@dataclass
class HandEyeSample:
    """One paired observation."""

    base_flange: np.ndarray  # 4x4, T_base_flange
    cam_target: np.ndarray  # 4x4, T_camera_target
    meta: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "base_flange": np.asarray(self.base_flange).tolist(),
            "cam_target": np.asarray(self.cam_target).tolist(),
            "meta": self.meta,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "HandEyeSample":
        return cls(
            base_flange=np.asarray(data["base_flange"], dtype=float),
            cam_target=np.asarray(data["cam_target"], dtype=float),
            meta=data.get("meta", {}),
        )


@dataclass
class SolveResult:
    X: np.ndarray  # T_flange_camera
    Y: np.ndarray  # T_base_target
    method: str
    residuals_deg: np.ndarray
    residuals_mm: np.ndarray
    used: List[int]
    outliers: List[int]
    metrics: Dict[str, float]
    method_spread: Dict[str, Dict[str, float]] = field(default_factory=dict)
    ok: bool = True
    message: str = ""


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def transform(sample: HandEyeSample) -> np.ndarray:
    return np.asarray(sample.base_flange, dtype=float)


def _relative_motions(
    samples: Sequence[HandEyeSample], min_angle_deg: float = 3.0
) -> List[tuple]:
    """All pose pairs (A_rel, B_rel) with enough rotation to constrain X.

    A_rel = inv(A_j) A_i,  B_rel = B_j inv(B_i)   with  A_rel X = X B_rel.
    """
    pairs = []
    n = len(samples)
    for i in range(n):
        for j in range(i + 1, n):
            a_rel = geom.invert(transform(samples[j])) @ transform(samples[i])
            b_rel = samples[j].cam_target @ geom.invert(samples[i].cam_target)
            angle = math.degrees(geom.rotation_angle(geom.rotation_of(b_rel)))
            if angle >= min_angle_deg:
                pairs.append((a_rel, b_rel))
    return pairs


def _rotation_park(pairs: Iterable[tuple]) -> np.ndarray:
    """Closed-form rotation from  alpha = R_X beta  (Kabsch / Park-Martin)."""
    alpha = []
    beta = []
    for a_rel, b_rel in pairs:
        alpha.append(geom.log_so3(geom.rotation_of(a_rel)))
        beta.append(geom.log_so3(geom.rotation_of(b_rel)))
    if len(alpha) < 2:
        raise ValueError("not enough informative relative rotations")
    a_mat = np.column_stack(alpha)
    b_mat = np.column_stack(beta)
    cross = b_mat @ a_mat.T
    u, _, vt = np.linalg.svd(cross)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    return rotation


def _translation_ls(pairs: Iterable[tuple], rotation: np.ndarray) -> np.ndarray:
    """Solve (R_A - I) t_X = R_X t_B - t_A in least squares sense."""
    lhs = []
    rhs = []
    for a_rel, b_rel in pairs:
        r_a = geom.rotation_of(a_rel)
        lhs.append(r_a - np.eye(3))
        rhs.append(rotation @ geom.translation_of(b_rel) - geom.translation_of(a_rel))
    if not lhs:
        raise ValueError("no relative motions")
    solution, *_ = np.linalg.lstsq(np.vstack(lhs), np.concatenate(rhs), rcond=None)
    return solution


def _cv2_estimate(samples: Sequence[HandEyeSample], method: str):
    import cv2

    constants = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    r_gripper2base = np.array([geom.rotation_of(s.base_flange) for s in samples])
    t_gripper2base = np.array([geom.translation_of(s.base_flange) for s in samples])
    r_target2cam = np.array([geom.rotation_of(s.cam_target) for s in samples])
    t_target2cam = np.array([geom.translation_of(s.cam_target) for s in samples])
    rotation, translation = cv2.calibrateHandEye(
        r_gripper2base,
        t_gripper2base.reshape(-1, 3, 1),
        r_target2cam,
        t_target2cam.reshape(-1, 3, 1),
        method=constants[method],
    )
    return geom.make_transform(rotation, translation.reshape(3))


def _initial_estimate(samples: Sequence[HandEyeSample], method: str) -> np.ndarray:
    pairs = _relative_motions(samples)
    if method == "PARK_LS":
        rotation = _rotation_park(pairs)
    else:
        rotation = geom.rotation_of(_cv2_estimate(samples, method))
    translation = _translation_ls(pairs, rotation)
    return geom.make_transform(rotation, translation)


# --------------------------------------------------------------------------- #
# residual + non-linear refinement
# --------------------------------------------------------------------------- #
def _predicted(sample: HandEyeSample, X: np.ndarray) -> np.ndarray:
    return sample.base_flange @ X @ sample.cam_target


def _residual_vector(samples: Sequence[HandEyeSample], X: np.ndarray, Y: np.ndarray) -> np.ndarray:
    out = []
    for sample in samples:
        delta = geom.invert(_predicted(sample, X)) @ Y
        out.append(geom.log_so3(geom.rotation_of(delta)) * _ROT_SCALE_M)
        out.append(geom.translation_of(delta))
    return np.concatenate(out)


def _apply(p: np.ndarray, X: np.ndarray, Y: np.ndarray):
    x_new = geom.make_transform(geom.exp_so3(p[0:3]) @ X[:3, :3], X[:3, 3] + p[3:6])
    y_new = geom.make_transform(geom.exp_so3(p[6:9]) @ Y[:3, :3], Y[:3, 3] + p[9:12])
    return x_new, y_new


def refine(
    samples: Sequence[HandEyeSample],
    X0: np.ndarray,
    Y0: np.ndarray,
    iterations: int = 60,
    tolerance: float = 1e-12,
):
    """Levenberg-Marquardt on (X, Y) with a numerical Jacobian."""
    X = np.array(X0, dtype=float)
    Y = np.array(Y0, dtype=float)
    r = _residual_vector(samples, X, Y)
    cost = float(r @ r)
    lam = 1e-3
    for _ in range(iterations):
        jac = np.zeros((r.size, 12))
        for k in range(12):
            step = np.zeros(12)
            step[k] = 1e-6
            x_p, y_p = _apply(step, X, Y)
            jac[:, k] = (_residual_vector(samples, x_p, y_p) - r) / 1e-6
        normal = jac.T @ jac
        gradient = jac.T @ r
        improved = False
        for _ in range(8):
            try:
                delta = np.linalg.solve(normal + lam * np.diag(np.diag(normal) + 1e-12), -gradient)
            except np.linalg.LinAlgError:
                lam *= 10.0
                continue
            x_new, y_new = _apply(delta, X, Y)
            r_new = _residual_vector(samples, x_new, y_new)
            cost_new = float(r_new @ r_new)
            if cost_new < cost:
                X, Y, r = x_new, y_new, r_new
                improved = True
                lam = max(lam * 0.3, 1e-9)
                break
            lam *= 10.0
        if not improved:
            break
        if abs(cost - cost_new) < tolerance * max(cost, 1.0) and np.linalg.norm(delta) < 1e-10:
            cost = cost_new
            break
        cost = cost_new
    return X, Y


# --------------------------------------------------------------------------- #
# metrics
# --------------------------------------------------------------------------- #
def per_sample_errors(samples, X, Y):
    rotations = []
    translations = []
    for sample in samples:
        delta = geom.invert(_predicted(sample, X)) @ Y
        rotations.append(math.degrees(geom.rotation_angle(geom.rotation_of(delta))))
        translations.append(float(np.linalg.norm(geom.translation_of(delta)) * 1000.0))
    return np.asarray(rotations), np.asarray(translations)


def _rms(values) -> float:
    values = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(values**2))) if values.size else float("nan")


def rotation_diversity(samples) -> dict:
    angles = []
    for i in range(len(samples)):
        for j in range(i + 1, len(samples)):
            rel = geom.invert(transform(samples[j])) @ transform(samples[i])
            angles.append(math.degrees(geom.rotation_angle(geom.rotation_of(rel))))
    positions = np.array([geom.translation_of(transform(s)) for s in samples])
    spread = float(np.max(np.linalg.norm(positions - positions.mean(axis=0), axis=1)) * 1000.0)
    return {
        "max_relative_rotation_deg": float(max(angles)) if angles else 0.0,
        "mean_relative_rotation_deg": float(np.mean(angles)) if angles else 0.0,
        "max_translation_spread_mm": spread,
    }


def pair_deviation(sample_a, sample_b):
    """|angle(A_ij) - angle(B_ij)| in degrees, or None if the pair is uninformative.

    Frame independent and independent of X: for a rigid setup the two relative
    rotation angles must match.
    """
    a_rel = geom.invert(sample_b.base_flange) @ sample_a.base_flange
    b_rel = sample_b.cam_target @ geom.invert(sample_a.cam_target)
    angle_a = math.degrees(geom.rotation_angle(geom.rotation_of(a_rel)))
    angle_b = math.degrees(geom.rotation_angle(geom.rotation_of(b_rel)))
    if angle_a < 10.0:
        return None
    return abs(angle_a - angle_b)


def view_geometry(samples) -> dict:
    """Frame-independent checks on the motion set.

    * consistency: the rotation angle of ``A_ij = inv(A_j) A_i`` must equal the
      angle of ``B_ij = B_j inv(B_i)`` for a rigid setup.  Blurred frames, a
      target that moved, or a stale tf all break this, and it needs no X.
    * axis spread: if every relative rotation is about the same axis (pure roll
      about the optical axis), the translation along that axis is unobservable.
    """
    deviations = []
    axes = []
    for i in range(len(samples)):
        for j in range(i + 1, len(samples)):
            a_rel = geom.invert(samples[j].base_flange) @ samples[i].base_flange
            b_rel = samples[j].cam_target @ geom.invert(samples[i].cam_target)
            angle_a = math.degrees(geom.rotation_angle(geom.rotation_of(a_rel)))
            angle_b = math.degrees(geom.rotation_angle(geom.rotation_of(b_rel)))
            if angle_a < 10.0:
                continue
            deviations.append(abs(angle_a - angle_b))
            axis = geom.log_so3(geom.rotation_of(b_rel))
            norm = float(np.linalg.norm(axis))
            if norm > 1e-9:
                axes.append(axis / norm)

    spread = []
    for i in range(len(axes)):
        for j in range(i + 1, len(axes)):
            spread.append(math.degrees(math.acos(min(1.0, abs(float(axes[i] @ axes[j]))))))

    return {
        "rotation_consistency_deg": float(np.median(deviations)) if deviations else 0.0,
        "rotation_consistency_max_deg": float(np.max(deviations)) if deviations else 0.0,
        "rotation_axis_spread_deg": float(np.median(spread)) if spread else 0.0,
        "informative_pairs": float(len(deviations)),
    }


def _loo(samples, method: str, X_full: np.ndarray, Y_full: np.ndarray):
    """Leave-one-out: fit without sample i, predict sample i's target pose."""
    n = len(samples)
    if n < 6:
        return float("nan"), float("nan")
    errors_t = []
    errors_r = []
    for i in range(n):
        subset = [s for k, s in enumerate(samples) if k != i]
        try:
            X_i = _initial_estimate(subset, method)
            Y_i0 = geom.average_transform([_predicted(s, X_i) for s in subset])
            X_i, Y_i = refine(subset, X_i, Y_i0)
        except Exception:
            return float("nan"), float("nan")
        delta = geom.invert(_predicted(samples[i], X_i)) @ Y_full
        errors_t.append(float(np.linalg.norm(geom.translation_of(delta)) * 1000.0))
        errors_r.append(math.degrees(geom.rotation_angle(geom.rotation_of(delta))))
    return _rms(errors_t), _rms(errors_r)


# --------------------------------------------------------------------------- #
# main entry point
# --------------------------------------------------------------------------- #
def solve(
    samples: Sequence[HandEyeSample],
    method: str = "PARK",
    do_refine: bool = True,
    reject_outliers: bool = True,
    min_samples: int = 6,
    max_outlier_ratio: float = 0.25,
) -> SolveResult:
    samples = list(samples)
    if len(samples) < 3:
        raise ValueError(f"need at least 3 samples, got {len(samples)}")
    method = method.upper()
    if method not in _ALL_METHODS:
        raise ValueError(f"unknown method {method}, expected one of {_ALL_METHODS}")

    # 1. one initial estimate per available solver, keep the best by residual
    estimates = {}
    for name in _ALL_METHODS:
        try:
            estimates[name] = _initial_estimate(samples, name)
        except Exception:
            continue
    if not estimates:
        raise RuntimeError("all solvers failed on this data set")

    scored = []
    for name, X in estimates.items():
        Y = geom.average_transform([_predicted(s, X) for s in samples])
        r = _residual_vector(samples, X, Y)
        scored.append((float(r @ r), name, X, Y))
    scored.sort(key=lambda item: item[0])
    _, chosen, X, Y = scored[0]
    if method in estimates:
        chosen, X = method, estimates[method]
        Y = geom.average_transform([_predicted(s, X) for s in samples])

    if do_refine:
        X, Y = refine(samples, X, Y)

    # 2. robust outlier rejection, re-fit without the offending samples
    used = list(range(len(samples)))
    outliers: List[int] = []
    if reject_outliers and len(samples) >= 8:
        for _ in range(4):
            rot, tra = per_sample_errors([samples[i] for i in used], X, Y)
            median_t = float(np.median(tra))
            median_r = float(np.median(rot))
            limit_t = max(3.0, 3.0 * median_t)
            limit_r = max(1.0, 4.0 * median_r)
            bad = [i for i, (r_, t_) in enumerate(zip(rot, tra)) if t_ > limit_t or r_ > limit_r]
            if not bad:
                break
            if len(used) - len(bad) < max(min_samples, 6):
                break
            if len(bad) + len(outliers) > max_outlier_ratio * len(samples):
                break
            worst = max(bad, key=lambda i: tra[i] / limit_t + rot[i] / limit_r)
            outliers.append(used.pop(worst))
            subset = [samples[i] for i in used]
            try:
                X = _initial_estimate(subset, method if method in estimates else chosen)
                Y = geom.average_transform([_predicted(s, X) for s in subset])
                if do_refine:
                    X, Y = refine(subset, X, Y)
            except Exception:
                break

    subset = [samples[i] for i in used]
    x_rot, x_tra = per_sample_errors(subset, X, Y)
    loo_t, loo_r = _loo(subset, method if method in estimates else chosen, X, Y) if do_refine else (
        float("nan"),
        float("nan"),
    )

    diversity = rotation_diversity(subset)
    geometry = view_geometry(subset)
    metrics = {
        "samples": float(len(subset)),
        "outliers": float(len(outliers)),
        "rotation_rms_deg": _rms(x_rot),
        "rotation_max_deg": float(np.max(x_rot)) if x_rot.size else float("nan"),
        "translation_rms_mm": _rms(x_tra),
        "translation_max_mm": float(np.max(x_tra)) if x_tra.size else float("nan"),
        "loo_translation_rms_mm": loo_t,
        "loo_rotation_rms_deg": loo_r,
        "max_relative_rotation_deg": diversity["max_relative_rotation_deg"],
        "mean_relative_rotation_deg": diversity["mean_relative_rotation_deg"],
        "max_translation_spread_mm": diversity["max_translation_spread_mm"],
        "rotation_consistency_deg": geometry["rotation_consistency_deg"],
        "rotation_consistency_max_deg": geometry["rotation_consistency_max_deg"],
        "rotation_axis_spread_deg": geometry["rotation_axis_spread_deg"],
        "informative_pairs": geometry["informative_pairs"],
    }

    # 3. how far apart the independent estimators are - a data quality signal
    spread = {}
    for name, estimate in estimates.items():
        d_rot, d_tra = geom.transform_error(estimate, X)
        spread[name] = {"rotation_deg": d_rot, "translation_mm": d_tra}

    message = "ok"
    if metrics["rotation_consistency_deg"] > 1.0:
        message = (
            f"samples disagree by {metrics['rotation_consistency_deg']:.1f} deg "
            f"(max {metrics['rotation_consistency_max_deg']:.1f}): the arm was still "
            "moving when the frame was taken (blur), the target moved, or the tf is stale"
        )
    elif metrics["max_relative_rotation_deg"] < 25.0 or metrics["informative_pairs"] < 6:
        message = "low rotation diversity: rotate the wrist +/-30 deg about x and y"
    elif metrics["rotation_axis_spread_deg"] < 20.0:
        message = (
            "rotations are about a single axis "
            f"(axis spread {metrics['rotation_axis_spread_deg']:.1f} deg): "
            "tilt about two different axes"
        )
    elif metrics["translation_rms_mm"] > 5.0:
        message = "high residual: check tag size, intrinsics, time sync or loosen joints"

    return SolveResult(
        X=X,
        Y=Y,
        method=method if method in estimates else chosen,
        residuals_deg=x_rot,
        residuals_mm=x_tra,
        used=used,
        outliers=outliers,
        metrics=metrics,
        method_spread=spread,
        message=message,
    )


# --------------------------------------------------------------------------- #
# symmetric targets (plain chessboards)
# --------------------------------------------------------------------------- #
def _quick_fit(samples):
    X = _initial_estimate(samples, "PARK")
    if not np.all(np.isfinite(X)):
        raise ValueError("degenerate estimate")
    Y = geom.average_transform([_predicted(s, X) for s in samples])
    if not np.all(np.isfinite(Y)):
        raise ValueError("degenerate target pose")
    X, Y = refine(samples, X, Y)
    if not (np.all(np.isfinite(X)) and np.all(np.isfinite(Y))):
        raise ValueError("refinement diverged")
    return X, Y


def _best_symmetry_error(sample, X, Y, symmetries):
    best_choice, best_error = 0, float("inf")
    for index, symmetry in enumerate(symmetries):
        predicted = sample.base_flange @ X @ (sample.cam_target @ symmetry)
        delta = geom.invert(predicted) @ Y
        error = float(
            np.linalg.norm(geom.translation_of(delta))
            + _ROT_SCALE_M * geom.rotation_angle(geom.rotation_of(delta))
        )
        if error < best_error:
            best_choice, best_error = index, error
    return best_choice, best_error


def _assign_symmetries(samples, X, Y, symmetries):
    choices = []
    errors = []
    for sample in samples:
        choice, error = _best_symmetry_error(sample, X, Y, symmetries)
        choices.append(choice)
        errors.append(error)
    return choices, errors


def _with_symmetry(sample, symmetry):
    return HandEyeSample(
        base_flange=sample.base_flange,
        cam_target=sample.cam_target @ symmetry,
        meta=sample.meta,
    )


def align_symmetric_targets(
    samples,
    symmetries,
    iterations: int = 6,
    hypotheses: int = 24,
    seed: int = 0,
):
    """Pick one of the equivalent target orientations per sample.

    A plain square chessboard is invariant under 90 deg rotations about its
    centre, so the detector may return any of four poses that reproject
    identically.  Since the board is static, the correct branch is the one whose
    ``T_base_target`` agrees with the other samples.

    Flips can be a large fraction of the data set, which breaks a naive
    "fit once, then fix the outliers" loop.  A small RANSAC over triples finds a
    consistent branch first; the iterative pass then polishes it.

    Returns ``(adjusted_samples, flipped_indices)``; the input is untouched.
    """
    samples = list(samples)
    count = len(samples)
    branches = len(symmetries)
    if count < 4 or branches < 2:
        return samples, []

    choices = [0] * count

    # 1. RANSAC over sample triples: fix the first sample's branch (a global
    #    branch shift is absorbed by X, so it carries no information).
    if count >= 5:
        import itertools
        import random

        pool = list(itertools.combinations(range(count), 3))
        rng = random.Random(seed)
        if len(pool) > hypotheses:
            pool = rng.sample(pool, hypotheses)
        best_inliers = -1
        for triple in pool:
            for first in range(branches):
                for second in range(branches):
                    picks = (0, first, second)
                    subset = [
                        _with_symmetry(samples[index], symmetries[pick])
                        for index, pick in zip(triple, picks)
                    ]
                    try:
                        X = _initial_estimate(subset, "PARK")
                        if not np.all(np.isfinite(X)):
                            continue
                        Y = geom.average_transform([_predicted(item, X) for item in subset])
                        if not np.all(np.isfinite(Y)):
                            continue
                    except Exception:
                        continue
                    inliers = 0
                    for sample in samples:
                        _, error = _best_symmetry_error(sample, X, Y, symmetries)
                        if error < 0.03:
                            inliers += 1
                    if inliers > best_inliers:
                        best_inliers = inliers
                        choices, _ = _assign_symmetries(samples, X, Y, symmetries)

    # 2. iterative refinement on the whole set
    for _ in range(iterations):
        adjusted = [
            _with_symmetry(sample, symmetries[choice])
            for sample, choice in zip(samples, choices)
        ]
        try:
            X, Y = _quick_fit(adjusted)
        except Exception:
            break
        updated, _ = _assign_symmetries(samples, X, Y, symmetries)
        if updated == choices:
            break
        choices = updated

    # 3. prefer the representative with the fewest flips: shifting every branch
    #    by the same amount only rotates X, which is an equivalent solution
    counts = [choices.count(branch) for branch in range(branches)]
    shift = int(np.argmax(counts))
    if shift:
        choices = [(choice - shift) % branches for choice in choices]

    adjusted = [
        _with_symmetry(sample, symmetries[choice])
        for sample, choice in zip(samples, choices)
    ]
    return adjusted, [index for index, choice in enumerate(choices) if choice]
