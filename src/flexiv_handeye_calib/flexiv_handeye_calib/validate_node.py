"""Independent accuracy check for a published eye-in-hand calibration.

The validator does not know anything about the solver: it simply looks up
``base -> camera`` through tf (which includes whatever ``flange -> camera``
the calibrator published or the demo loaded from YAML), detects a marker that
stays put on the table, and reports how much the recovered marker pose moves
when the arm goes to fresh poses.

If the calibration is right, the marker does not move in the base frame.  The
spread of the estimates is therefore a direct, end-to-end accuracy figure
(calibration error + tag detection noise + robot repeatability).
"""

from __future__ import annotations

import math
import os

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from . import geometry as geom
from .detection import select_detection
from .detectors import make_detector


class HandeyeValidator(Node):
    def __init__(self):
        super().__init__("handeye_validator")
        self.base_frame = self.declare_parameter("base_frame", "").value
        self.robot_sn = self.declare_parameter("robot_sn", "").value
        self.camera_frame = self.declare_parameter("camera_frame", "camera_link").value
        self.image_topic = self.declare_parameter(
            "image_topic", "/camera/camera/color/image_raw"
        ).value
        self.camera_info_topic = self.declare_parameter(
            "camera_info_topic", "/camera/camera/color/camera_info"
        ).value
        self.target_type = self.declare_parameter("target_type", "chessboard").value
        self.tag_family = self.declare_parameter("tag_family", "25h9").value
        self.tag_size = float(self.declare_parameter("tag_size", 0.026).value)
        self.board_cols = int(self.declare_parameter("chessboard_cols", 8).value)
        self.board_rows = int(self.declare_parameter("chessboard_rows", 11).value)
        self.square_size = float(self.declare_parameter("square_size", 0.006).value)
        self.board_auto = bool(self.declare_parameter("chessboard_auto", False).value)
        self.tag_id = int(self.declare_parameter("tag_id", -1).value)
        self.min_area_px = float(self.declare_parameter("min_area_px", 150.0).value)
        self.max_reproj_error_px = float(
            self.declare_parameter("max_reprojection_error_px", 3.0).value
        )
        self.max_image_age_s = float(self.declare_parameter("max_image_age_s", 0.5).value)
        self.dedupe_translation = float(self.declare_parameter("dedupe_translation", 0.01).value)
        self.report_file = self.declare_parameter(
            "report_file",
            os.path.join(os.path.expanduser("~"), ".ros", "flexiv_handeye", "validation.txt"),
        ).value

        if not self.report_file:
            self.report_file = os.path.join(
                os.path.expanduser("~"), ".ros", "flexiv_handeye", "validation.txt"
            )
        self.report_file = os.path.abspath(os.path.expanduser(self.report_file))

        if not self.base_frame:
            self.base_frame = f"{self.robot_sn}_base_link" if self.robot_sn else "base_link"

        self.detector = make_detector(
            self.target_type,
            tag_family=self.tag_family,
            tag_size=self.tag_size,
            board_cols=self.board_cols,
            board_rows=self.board_rows,
            square_size=self.square_size,
            board_auto=self.board_auto,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._last_image = None
        self._last_info = None
        self.samples: list[np.ndarray] = []
        self.meta: list[dict] = []
        self._camera_poses: list[np.ndarray] = []

        self.create_subscription(
            Image, self.image_topic, self._on_image, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_info, qos_profile_sensor_data
        )
        self.create_service(Trigger, "~/record", self._srv_record)
        self.create_service(Trigger, "~/report", self._srv_report)
        self.create_service(Trigger, "~/reset", self._srv_reset)
        self.get_logger().info(
            f"validator ready: {self.base_frame} <- {self.camera_frame}, "
            f"target {self.detector.describe()}"
        )
        self._checked = False
        self.create_timer(1.0, self._check_once)

    def _check_once(self):
        """Warn early if the calibration transform is not published."""
        if self._checked:
            return
        try:
            self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame, Time()
            )
            self._checked = True
            self.get_logger().info("tf chain to the camera is complete")
        except Exception:
            self.get_logger().warn(
                f"no {self.base_frame} <- {self.camera_frame} yet: run the calibration "
                "(solve) and keep the calibrate node running, or start a "
                "static_transform_publisher with the saved xyz/rpy",
                throttle_duration_sec=5.0,
            )

    def _on_image(self, message: Image):
        self._last_image = message

    def _on_info(self, message: CameraInfo):
        self._last_info = message

    @staticmethod
    def _to_bgr(message: Image):
        channels = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(message.encoding)
        if channels is None:
            return None
        data = np.frombuffer(message.data, dtype=np.uint8)
        row_bytes = message.width * channels
        image = data.reshape(message.height, message.step)[:, :row_bytes].reshape(
            message.height, message.width, channels
        )
        if channels == 1:
            return np.ascontiguousarray(np.repeat(image, 3, axis=2))
        if message.encoding == "rgb8":
            image = image[:, :, ::-1]
        return np.ascontiguousarray(image)

    def _srv_record(self, request, response):
        image, info = self._last_image, self._last_info
        if image is None or info is None:
            response.success, response.message = False, "no camera frame yet"
            return response
        stamp = Time.from_msg(image.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if age > self.max_image_age_s:
            response.success, response.message = False, f"frame is {age:.2f} s old"
            return response
        try:
            bgr = self._to_bgr(image)
            k = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
            d = np.asarray(info.d, dtype=np.float64) if len(info.d) > 0 else None
            detection = select_detection(
                self.detector.detect(bgr, k, d),
                marker_id=self.tag_id,
                min_area_px=self.min_area_px,
                max_reprojection_error_px=self.max_reproj_error_px,
            )
            if detection is None:
                response.success, response.message = False, "no usable tag in view"
                return response
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, image.header.frame_id, stamp, Duration(seconds=0.2)
            )
            base_optical = self._to_matrix(transform)
        except Exception as exc:
            response.success, response.message = False, f"failed: {exc}"
            return response

        pose = base_optical @ detection.transform

        # de-duplicate on the ROBOT pose, not on the target: the target is
        # supposed to stay put, so near-identical target poses are expected and
        # are exactly what this validator measures
        camera_pose = None
        try:
            camera_pose = self._to_matrix(
                self.tf_buffer.lookup_transform(
                    self.base_frame, self.camera_frame, stamp, Duration(seconds=0.2)
                )
            )
        except Exception:
            camera_pose = None
        if camera_pose is not None:
            for existing in self._camera_poses:
                moved = np.linalg.norm(
                    geom.translation_of(camera_pose) - geom.translation_of(existing)
                )
                turned = math.degrees(geom.rotation_angle(
                    geom.rotation_of(geom.invert(existing) @ camera_pose)
                ))
                if moved < self.dedupe_translation and turned < 5.0:
                    response.success = False
                    response.message = (
                        "this is the same robot pose as a previous sample; "
                        "move the arm further"
                    )
                    return response
        if camera_pose is not None:
            self._camera_poses.append(camera_pose)
        self.samples.append(pose)
        self.meta.append(
            {
                "marker_id": detection.marker_id,
                "reproj_px": detection.reprojection_error_px,
                "area_px": detection.area_px,
            }
        )
        position = geom.translation_of(pose)
        response.success = True
        response.message = (
            f"recorded {len(self.samples)}: marker {detection.marker_id} at "
            f"[{position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}] m, "
            f"reproj {detection.reprojection_error_px:.2f} px"
        )
        self.get_logger().info(response.message)
        return response

    @staticmethod
    def _to_matrix(transform):
        return geom.make_transform(
            geom.quat_to_matrix(
                [
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                ]
            ),
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
        )

    def _srv_reset(self, request, response):
        self.samples, self.meta, self._camera_poses = [], [], []
        response.success, response.message = True, "validation samples cleared"
        return response

    def _srv_report(self, request, response):
        if len(self.samples) < 3:
            response.success = False
            response.message = f"need at least 3 samples, have {len(self.samples)}"
            return response

        # the board frame can come back rotated by 180 deg (rectangular
        # chessboard); normalise every sample to the first one before averaging
        symmetries = (
            self.detector.symmetry_candidates()
            if hasattr(self.detector, "symmetry_candidates")
            else [np.eye(4)]
        )
        reference = self.samples[0]
        normalised = [reference]
        for pose in self.samples[1:]:
            best, best_error = pose, float("inf")
            for symmetry in symmetries:
                candidate = pose @ symmetry
                error = geom.rotation_angle(
                    geom.rotation_of(geom.invert(reference) @ candidate)
                )
                if error < best_error:
                    best, best_error = candidate, error
            normalised.append(best)

        mean_pose = geom.average_transform(normalised)
        reference_position = geom.translation_of(mean_pose)
        translations = []
        rotations = []
        for pose in normalised:
            delta = geom.invert(mean_pose) @ pose
            translations.append(float(np.linalg.norm(geom.translation_of(delta)) * 1000.0))
            rotations.append(math.degrees(geom.rotation_angle(geom.rotation_of(delta))))
        translations = np.asarray(translations)
        rotations = np.asarray(rotations)

        positions = np.array([geom.translation_of(p) for p in self.samples])
        rms = float(np.sqrt(np.mean(translations**2)))
        max_error = float(np.max(translations))
        if rms < 2.0:
            grade = "GOOD"
        elif rms < 5.0:
            grade = "ACCEPTABLE"
        else:
            grade = "POOR"

        lines = [
            f"validation samples : {len(self.samples)}",
            f"fixed target (base): [{reference_position[0]:.4f}, {reference_position[1]:.4f}, {reference_position[2]:.4f}] m",
            f"repeatability      : {rms:.2f} mm rms, {max_error:.2f} mm max, "
            f"{float(np.sqrt(np.mean(rotations**2))):.3f} deg rms",
            f"per axis std       : x {positions[:, 0].std() * 1000:.2f} "
            f"y {positions[:, 1].std() * 1000:.2f} z {positions[:, 2].std() * 1000:.2f} mm",
            f"verdict            : {grade}",
        ]
        text = "\n".join(lines)
        try:
            os.makedirs(os.path.dirname(os.path.abspath(self.report_file)), exist_ok=True)
            with open(self.report_file, "w", encoding="utf-8") as handle:
                handle.write(text + "\n")
        except OSError as exc:
            self.get_logger().warn(f"cannot write {self.report_file}: {exc}")
        self.get_logger().info("\n" + text)
        response.success = True
        response.message = text
        return response


def main(args=None):
    rclpy.init(args=args)
    node = HandeyeValidator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
