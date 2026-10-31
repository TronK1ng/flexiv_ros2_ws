#!/usr/bin/env python3
"""AprilTag 25h9 detection for the wrist-mounted RealSense D415.

The node keeps the newest colour frame and camera info, and answers the
/grasp_demo/detect service on demand: the task node calls it once the arm has
settled at the observe pose, which keeps detection away from motion blur.

Poses are returned in the configured base frame. The camera -> base transform is
looked up through TF, so the hand-eye calibration only has to be published as a
static transform (flange -> camera_link).

Deliberately no cv_bridge: this machine has a user-level OpenCV 4.13 (needed for
DICT_APRILTAG_25H9) next to the distro OpenCV 4.5.4 that cv_bridge links against,
and loading both in one process is a known source of crashes. Images are
converted with numpy instead.
"""

import math

import cv2
import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from flexiv_grasp_demo.msg import GraspTarget
from flexiv_grasp_demo.srv import Detect

FAMILIES = {
    "25h9": cv2.aruco.DICT_APRILTAG_25H9,
    "16h5": cv2.aruco.DICT_APRILTAG_16H5,
    "36h10": cv2.aruco.DICT_APRILTAG_36H10,
    "36h11": cv2.aruco.DICT_APRILTAG_36H11,
    "4x4_50": cv2.aruco.DICT_4X4_50,
    "4x4_100": cv2.aruco.DICT_4X4_100,
    "5x5_50": cv2.aruco.DICT_5X5_50,
    "5x5_100": cv2.aruco.DICT_5X5_100,
}


def quat_to_matrix(x, y, z, w):
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quat(matrix):
    """Rotation matrix -> (x, y, z, w), Shepperd's method."""
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return np.array([x / norm, y / norm, z / norm, w / norm])


def quat_rotate(q, v):
    """Rotate vector v by quaternion q (x, y, z, w)."""
    x, y, z, w = q
    u = np.array([x, y, z])
    return (
        v * (2.0 * w * w - 1.0)
        + 2.0 * np.dot(u, v) * u
        + 2.0 * w * np.cross(u, v)
    )


def quat_multiply(q1, q2):
    """Hamilton product, both (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]
    )


class GraspPerception(Node):
    def __init__(self):
        super().__init__("grasp_perception")

        family = self.declare_parameter("tag_family", "25h9").value
        if family not in FAMILIES:
            raise ValueError(f"unsupported tag_family '{family}', known: {sorted(FAMILIES)}")
        self.tag_size = float(self.declare_parameter("tag_size", 0.026).value)
        self.base_frame = self.declare_parameter("base_frame", "").value
        self.target_ids = list(self.declare_parameter("target_ids", [0]).value or [])
        # target_ids defaults to a single 0 which would filter everything out;
        # treat [0] as "no filter" unless the user really only wants id 0.
        if self.target_ids == [0]:
            self.target_ids = []
        self.max_age_s = float(self.declare_parameter("max_age_s", 0.2).value)
        self.min_area_px = float(self.declare_parameter("min_area_px", 150.0).value)
        self.max_reproj_error_px = float(
            self.declare_parameter("max_reproj_error_px", 3.0).value
        )
        # The target is a cube with a marker on every face, so an oblique view can
        # make a side face larger than the top one. Picking by area alone would then
        # return a side marker: its centre sits at half the cube height and its
        # normal is horizontal, which silently turns a top grasp into a side grasp.
        self.selection = str(self.declare_parameter("selection", "up_facing").value)
        self.min_normal_z = float(self.declare_parameter("min_normal_z", 0.7).value)
        self.publish_debug = bool(self.declare_parameter("publish_debug", True).value)
        self.tf_latest_fallback = bool(
            self.declare_parameter("tf_latest_fallback", True).value
        )
        self.tf_fallback_count = 0

        image_topic = self.declare_parameter(
            "image_topic", "/camera/camera/color/image_raw"
        ).value
        info_topic = self.declare_parameter(
            "camera_info_topic", "/camera/camera/color/camera_info"
        ).value

        if not self.base_frame:
            self.get_logger().warn("base_frame is empty: poses will be returned in the camera frame")

        self.dictionary = cv2.aruco.getPredefinedDictionary(FAMILIES[family])
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize = 5
        params.cornerRefinementMaxIterations = 50
        params.cornerRefinementMinAccuracy = 0.01
        params.adaptiveThreshWinSizeMin = 5
        params.adaptiveThreshWinSizeMax = 23
        params.adaptiveThreshWinSizeStep = 4
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, params)

        self.latest_image = None
        self.latest_info = None
        self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, info_topic, self._on_info, qos_profile_sensor_data
        )
        self.debug_pub = (
            self.create_publisher(Image, "/grasp_demo/debug_image", 1)
            if self.publish_debug
            else None
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_service(Detect, "/grasp_demo/detect", self._on_detect)
        self.get_logger().info(
            f"grasp_perception ready: family={family} size={self.tag_size * 1000:.0f} mm "
            f"base_frame='{self.base_frame or 'camera'}'"
        )

    # ---------------- subscriptions ----------------
    def _on_image(self, msg):
        self.latest_image = msg

    def _on_info(self, msg):
        self.latest_info = msg

    # ---------------- helpers ----------------
    @staticmethod
    def _image_to_bgr(msg):
        channels = 3 if msg.encoding in ("bgr8", "rgb8") else 1
        dtype = np.uint8
        row_bytes = msg.width * channels
        data = np.frombuffer(msg.data, dtype=dtype)
        if msg.step == row_bytes:
            image = data.reshape(msg.height, msg.width, channels)
        else:
            image = data.reshape(msg.height, msg.step)[:, :row_bytes].reshape(
                msg.height, msg.width, channels
            )
        if channels == 3 and msg.encoding == "rgb8":
            image = image[:, :, ::-1]
        return np.ascontiguousarray(image)

    def _lookup(self, camera_frame, stamp):
        """Camera -> base at the image stamp, falling back to the newest transform.

        Camera and joint-state timestamps never line up exactly, so a lookup at the
        image stamp fails now and then even with a perfectly static robot. Detection
        only runs while the arm is parked, so the newest transform is just as good.
        """
        if not self.base_frame or self.base_frame == camera_frame:
            return None
        try:
            return self.tf_buffer.lookup_transform(
                self.base_frame, camera_frame, Time.from_msg(stamp), Duration(seconds=0.2)
            )
        except Exception as exc:  # tf2 raises a family of exceptions
            if not self.tf_latest_fallback:
                self.get_logger().warn(f"TF {self.base_frame} <- {camera_frame} failed: {exc}")
                return None
            self.tf_fallback_count += 1
            if self.tf_fallback_count <= 5:
                self.get_logger().warn(
                    f"TF at image stamp failed ({exc}); using the latest transform instead "
                    "(logged 5 times at most)"
                )
        try:
            return self.tf_buffer.lookup_transform(
                self.base_frame, camera_frame, Time(), Duration(seconds=0.2)
            )
        except Exception as exc:
            self.get_logger().warn(f"latest TF {self.base_frame} <- {camera_frame} failed: {exc}")
            return None

    def _solve(self, corners, camera_matrix, dist_coeffs):
        half = self.tag_size / 2.0
        object_points = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float64,
        )
        image_points = corners.reshape((4, 2)).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            return None
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
        error = float(
            np.mean(np.linalg.norm(projected.reshape((4, 2)) - image_points, axis=1))
        )
        rotation, _ = cv2.Rodrigues(rvec)
        return rotation, tvec.reshape(3), error

    # ---------------- service ----------------
    def _on_detect(self, request, response):
        image_msg = self.latest_image
        info = self.latest_info
        if image_msg is None or info is None:
            response.image_stale = True
            response.message = "no camera frame received yet"
            return response

        age = (self.get_clock().now() - Time.from_msg(image_msg.header.stamp)).nanoseconds * 1e-9
        if age > self.max_age_s:
            response.image_stale = True
            response.message = f"newest frame is {age:.2f} s old"
            return response

        image = self._image_to_bgr(image_msg)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        corners, ids, _ = self.detector.detectMarkers(gray)

        wanted = None
        if request.object_id:
            try:
                wanted = int(request.object_id)
            except ValueError:
                response.message = f"object_id '{request.object_id}' is not a number"
                return response

        camera_matrix = np.array(info.k, dtype=np.float64).reshape(3, 3)
        dist_coeffs = np.array(info.d, dtype=np.float64)
        frame_id = image_msg.header.frame_id
        transform = self._lookup(frame_id, image_msg.header.stamp)
        if transform is None and self.base_frame:
            response.image_stale = False
            response.message = f"cannot transform {frame_id} to {self.base_frame}"
            return response

        response.targets = []
        if ids is None:
            response.message = "no marker detected"
            self._publish_debug(image, corners, ids)
            return response

        candidates = []
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            marker_id = int(marker_id)
            if wanted is not None and marker_id != wanted:
                continue
            if self.target_ids and marker_id not in self.target_ids:
                continue
            area = float(cv2.contourArea(marker_corners.reshape((-1, 1, 2))))
            if area < self.min_area_px:
                continue
            solved = self._solve(marker_corners, camera_matrix, dist_coeffs)
            if solved is None:
                continue
            rotation, translation, reproj = solved
            if reproj > self.max_reproj_error_px:
                continue

            quat_cam = matrix_to_quat(rotation)
            position = translation
            quat = quat_cam
            if transform is not None:
                tr = transform.transform.translation
                rq = transform.transform.rotation
                rotation_tf = quat_to_matrix(rq.x, rq.y, rq.z, rq.w)
                position = rotation_tf @ translation + np.array([tr.x, tr.y, tr.z])
                quat = quat_multiply(np.array([rq.x, rq.y, rq.z, rq.w]), quat_cam)

            target = GraspTarget()
            target.id = marker_id
            target.pose.header.stamp = image_msg.header.stamp
            target.pose.header.frame_id = self.base_frame or frame_id
            target.pose.pose.position.x = float(position[0])
            target.pose.pose.position.y = float(position[1])
            target.pose.pose.position.z = float(position[2])
            target.pose.pose.orientation.x = float(quat[0])
            target.pose.pose.orientation.y = float(quat[1])
            target.pose.pose.orientation.z = float(quat[2])
            target.pose.pose.orientation.w = float(quat[3])
            target.size.x = self.tag_size
            target.size.y = self.tag_size
            target.size.z = 0.0
            target.area = area
            target.reprojection_error = reproj
            # Normal of the marker plane in the base frame.
            normal_z = float(quat_rotate(quat, np.array([0.0, 0.0, 1.0]))[2])
            candidates.append((normal_z, area, target))

        if self.selection == "up_facing":
            up_facing = [c for c in candidates if c[0] >= self.min_normal_z]
            if not up_facing and candidates:
                response.message = (
                    f"{len(candidates)} marker(s) visible but none faces up "
                    f"(best normal_z={max(c[0] for c in candidates):.2f})"
                )
                self._publish_debug(image, corners, ids)
                return response
            up_facing.sort(key=lambda c: c[0], reverse=True)
            response.targets = [c[2] for c in up_facing]
        else:
            candidates.sort(key=lambda c: c[1], reverse=True)
            response.targets = [c[2] for c in candidates]
        response.image_stale = False
        best_normal = candidates and max(c[0] for c in candidates) or 0.0
        response.message = (
            f"{len(response.targets)} target(s), best normal_z={best_normal:.2f}"
        )
        self._publish_debug(image, corners, ids)
        return response

    def _publish_debug(self, image, corners, ids):
        if self.debug_pub is None:
            return
        if corners is not None and len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame or "camera"
        msg.height, msg.width = image.shape[:2]
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        msg.data = image.tobytes()
        self.debug_pub.publish(msg)


def main():
    rclpy.init()
    node = GraspPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
