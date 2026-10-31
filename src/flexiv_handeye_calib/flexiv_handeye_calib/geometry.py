"""Small, dependency-free SE(3)/SO(3) helpers.

Everything is plain numpy so the calibration maths can be unit tested
without ROS, OpenCV or scipy.  Conventions follow tf2 / OpenCV:

* a transform is a 4x4 homogeneous matrix ``T_a_b`` mapping points from
  frame ``b`` into frame ``a``:  ``p_a = T_a_b @ p_b``
* euler angles are fixed-axis roll-pitch-yaw, ``R = Rz(yaw) Ry(pitch) Rx(roll)``
  which is exactly what ``tf2`` means by ``rpy``
* quaternions are ``(x, y, z, w)``
"""

from __future__ import annotations

import math

import numpy as np

_EPS = 1e-12


# --------------------------------------------------------------------------- #
# elementary rotations
# --------------------------------------------------------------------------- #
def rot_x(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# --------------------------------------------------------------------------- #
# euler <-> matrix
# --------------------------------------------------------------------------- #
def euler_to_matrix(rpy) -> np.ndarray:
    roll, pitch, yaw = (float(v) for v in rpy)
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def matrix_to_euler(rotation: np.ndarray):
    """Return fixed-axis (roll, pitch, yaw) in radians."""
    r = np.asarray(rotation, dtype=float)
    sy = math.hypot(r[0, 0], r[1, 0])
    if sy > 1e-9:
        roll = math.atan2(r[2, 1], r[2, 2])
        pitch = math.atan2(-r[2, 0], sy)
        yaw = math.atan2(r[1, 0], r[0, 0])
    else:  # gimbal lock
        roll = math.atan2(-r[1, 2], r[1, 1])
        pitch = math.atan2(-r[2, 0], sy)
        yaw = 0.0
    return np.array([roll, pitch, yaw])


# --------------------------------------------------------------------------- #
# quaternion <-> matrix
# --------------------------------------------------------------------------- #
def quat_to_matrix(quat) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < _EPS:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quat(rotation: np.ndarray):
    """Shepperd's method, numerically stable for all rotations."""
    r = np.asarray(rotation, dtype=float)
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    quat = np.array([x, y, z, w])
    if quat[3] < 0.0:
        quat = -quat
    return quat / np.linalg.norm(quat)


# --------------------------------------------------------------------------- #
# homogeneous transforms
# --------------------------------------------------------------------------- #
def make_transform(rotation: np.ndarray, translation) -> np.ndarray:
    t = np.asarray(translation, dtype=float).reshape(3)
    out = np.eye(4)
    out[:3, :3] = np.asarray(rotation, dtype=float)
    out[:3, 3] = t
    return out


def invert(transform: np.ndarray) -> np.ndarray:
    t = np.asarray(transform, dtype=float)
    r = t[:3, :3]
    out = np.eye(4)
    out[:3, :3] = r.T
    out[:3, 3] = -r.T @ t[:3, 3]
    return out


def rotation_of(transform: np.ndarray) -> np.ndarray:
    return np.asarray(transform, dtype=float)[:3, :3]


def translation_of(transform: np.ndarray) -> np.ndarray:
    return np.asarray(transform, dtype=float)[:3, 3]


def rotation_angle(rotation: np.ndarray) -> float:
    """Geodesic angle of a rotation matrix, in radians."""
    r = np.asarray(rotation, dtype=float)
    value = (np.trace(r) - 1.0) * 0.5
    return math.acos(max(-1.0, min(1.0, value)))


def log_so3(rotation: np.ndarray) -> np.ndarray:
    r = np.asarray(rotation, dtype=float)
    angle = rotation_angle(r)
    if angle < 1e-8:
        return 0.5 * np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    if angle > math.pi - 1e-5:
        # axis is the unit eigenvector of R with eigenvalue 1
        values, vectors = np.linalg.eig(r)
        axis = np.real(vectors[:, np.argmin(np.abs(values - 1.0))])
        axis = axis / np.linalg.norm(axis)
        if np.dot(axis, np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])) < 0:
            axis = -axis
        return axis * angle
    factor = angle / (2.0 * math.sin(angle))
    return factor * np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])


def exp_so3(omega) -> np.ndarray:
    w = np.asarray(omega, dtype=float).reshape(3)
    angle = float(np.linalg.norm(w))
    if angle < 1e-12:
        return np.eye(3)
    k = w / angle
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(angle) * kx + (1.0 - math.cos(angle)) * (kx @ kx)


def project_to_so3(matrix: np.ndarray) -> np.ndarray:
    """Closest proper rotation (Frobenius norm)."""
    u, _, vt = np.linalg.svd(np.asarray(matrix, dtype=float))
    r = u @ vt
    if np.linalg.det(r) < 0.0:
        u[:, -1] *= -1.0
        r = u @ vt
    return r


# --------------------------------------------------------------------------- #
# averaging
# --------------------------------------------------------------------------- #
def average_rotation(rotations) -> np.ndarray:
    """Markley's quaternion average."""
    acc = np.zeros((4, 4))
    for rotation in rotations:
        q = matrix_to_quat(rotation)
        acc += np.outer(q, q)
    values, vectors = np.linalg.eigh(acc)
    return quat_to_matrix(vectors[:, np.argmax(values)])


def average_transform(transforms) -> np.ndarray:
    transforms = list(transforms)
    rotation = average_rotation([rotation_of(t) for t in transforms])
    translation = np.mean([translation_of(t) for t in transforms], axis=0)
    return make_transform(rotation, translation)


def transform_error(measured: np.ndarray, reference: np.ndarray):
    """Return (rotation angle [deg], translation difference [mm])."""
    delta = invert(reference) @ measured
    return (
        math.degrees(rotation_angle(rotation_of(delta))),
        float(np.linalg.norm(translation_of(delta)) * 1000.0),
    )
