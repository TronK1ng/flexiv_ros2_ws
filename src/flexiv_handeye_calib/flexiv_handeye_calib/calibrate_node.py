"""ROS 2 node for eye-in-hand calibration of a Flexiv arm and a wrist camera.

It does three things, each exposed as a ``std_srvs/Trigger`` service so that the
whole procedure can be scripted or driven from the shell:

* ``~/capture``  take one sample: detect the tag, read ``T_base_flange`` from tf,
  pair them and append to the sample file
* ``~/solve``    run AX = XB, print the quality report, write the YAML and
  publish ``parent_frame -> child_frame`` as a static transform
* ``~/save``     rewrite the YAML of the last solution
* ``~/reset``    drop all samples
* ``~/status``   human readable state

No cv_bridge: the image is decoded with numpy exactly like ``grasp_perception``
does, which keeps the pip OpenCV 4.13 out of the apt OpenCV used by cv_bridge.
"""

from __future__ import annotations

import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

from . import geometry as geom
from . import solver, storage
from .solver import HandEyeSample
from .detection import select_detection
from .detectors import make_detector


class HandeyeCalibrator(Node):
    def __init__(self):
        super().__init__("handeye_calibrator")

        self.robot_sn = self.declare_parameter("robot_sn", "").value
        self.base_frame = self.declare_parameter("base_frame", "").value
        self.flange_frame = self.declare_parameter("flange_frame", "").value
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
        self.tag_id = int(self.declare_parameter("tag_id", -1).value)
        self.board_cols = int(self.declare_parameter("chessboard_cols", 8).value)
        self.board_rows = int(self.declare_parameter("chessboard_rows", 11).value)
        self.square_size = float(self.declare_parameter("square_size", 0.006).value)
        self.board_auto = bool(self.declare_parameter("chessboard_auto", False).value)
        self.normalize_symmetric = bool(
            self.declare_parameter("normalize_symmetric_board", True).value
        )
        self.min_area_px = float(self.declare_parameter("min_area_px", 150.0).value)
        self.max_reproj_error_px = float(
            self.declare_parameter("max_reprojection_error_px", 3.0).value
        )
        default_dir = os.path.join(os.path.expanduser("~"), ".ros", "flexiv_handeye")
        self.samples_file = self.declare_parameter(
            "samples_file", os.path.join(default_dir, "samples.json")
        ).value
        self.output_yaml = self.declare_parameter(
            "output_yaml", os.path.join(default_dir, "handeye_calibration.yaml")
        ).value
        self.auto_capture = bool(self.declare_parameter("auto_capture", False).value)
        self.auto_min_translation = float(
            self.declare_parameter("auto_min_translation", 0.02).value
        )
        self.auto_min_rotation = math.radians(
            float(self.declare_parameter("auto_min_rotation_deg", 6.0).value)
        )
        self.auto_max_speed = float(self.declare_parameter("auto_max_speed", 0.05).value)
        self.auto_max_angular_speed = float(
            self.declare_parameter("auto_max_angular_speed_deg", 8.0).value
        )
        self.settle_time = float(self.declare_parameter("settle_time", 0.6).value)
        self._still_since = None
        self._velocity_pose = None
        self._velocity_time = None
        self.auto_cooldown_s = float(self.declare_parameter("auto_cooldown_s", 1.5).value)
        self.min_samples = int(self.declare_parameter("min_samples", 6).value)
        self.solve_method = self.declare_parameter("solve_method", "PARK").value
        self.reject_outliers = bool(self.declare_parameter("reject_outliers", True).value)
        self.publish_tf = bool(self.declare_parameter("publish_tf", True).value)
        self.dedupe_translation = float(
            self.declare_parameter("dedupe_translation", 0.005).value
        )
        self.dedupe_rotation = math.radians(
            float(self.declare_parameter("dedupe_rotation_deg", 2.0).value)
        )
        self.max_image_age_s = float(self.declare_parameter("max_image_age_s", 0.5).value)
        self.auto_save = bool(self.declare_parameter("auto_save", True).value)
        self.guided = bool(self.declare_parameter("guided", True).value)
        self.pose_tolerance_deg = float(
            self.declare_parameter("pose_tolerance_deg", 8.0).value
        )
        self.auto_solve_on_plan = bool(
            self.declare_parameter("auto_solve_on_plan", True).value
        )
        self.reject_inconsistent = bool(
            self.declare_parameter("reject_inconsistent", False).value
        )
        self.inconsistent_threshold = float(
            self.declare_parameter("inconsistent_threshold_deg", 1.2).value
        )

        if not self.samples_file:
            self.samples_file = os.path.join(default_dir, "samples.json")
        if not self.output_yaml:
            self.output_yaml = os.path.join(default_dir, "handeye_calibration.yaml")
        self.samples_file = os.path.abspath(os.path.expanduser(self.samples_file))
        self.output_yaml = os.path.abspath(os.path.expanduser(self.output_yaml))

        self._fill_frames()
        if not self.robot_sn and not self.flange_frame:
            self.get_logger().warn(
                "neither robot_sn nor flange_frame is set: tf lookups will fail"
            )

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
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.samples: list[HandEyeSample] = []
        self.last_result = None
        self._last_image = None
        self._last_info = None
        self._last_capture_time = None
        self._last_saved_pose = None
        self._last_saved_time = None
        self._plan_index = 0
        self._seed_pose = None

        self.create_subscription(
            Image, self.image_topic, self._on_image, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_info, qos_profile_sensor_data
        )
        self.create_service(Trigger, "~/capture", self._srv_capture)
        self.create_service(Trigger, "~/solve", self._srv_solve)
        self.create_service(Trigger, "~/save", self._srv_save)
        self.create_service(Trigger, "~/reset", self._srv_reset)
        self.create_service(Trigger, "~/status", self._srv_status)

        self._load_existing_samples()
        self._publish_stored_calibration()

        if self.auto_capture:
            self.create_timer(0.2, self._auto_capture_tick)
            self.get_logger().info(
                "auto_capture on: move the arm by hand/MoveIt, samples are taken when it settles"
            )
        if self.guided:
            self._announce_next()
        self.get_logger().info(
            "handeye calibrator ready\n"
            f"  base   : {self.base_frame}\n"
            f"  flange : {self.flange_frame}\n"
            f"  camera : {self.camera_frame}\n"
            f"  image  : {self.image_topic}\n"
            f"  target : {self.detector.describe()}\n"
            f"  samples: {len(self.samples)} from {self.samples_file}"
        )

    # ------------------------------------------------------------------ #
    # setup helpers
    # ------------------------------------------------------------------ #
    def _fill_frames(self):
        if not self.base_frame:
            self.base_frame = f"{self.robot_sn}_base_link" if self.robot_sn else "base_link"
        if not self.flange_frame:
            self.flange_frame = f"{self.robot_sn}_flange" if self.robot_sn else "flange"

    def _load_existing_samples(self):
        if not os.path.exists(self.samples_file):
            return
        try:
            samples, meta = storage.load_samples(self.samples_file)
            stored_sn = meta.get("robot_sn", "")
            if stored_sn and self.robot_sn and stored_sn != self.robot_sn:
                self.get_logger().warn(
                    f"{self.samples_file} belongs to {stored_sn}, not {self.robot_sn}: "
                    "ignoring the old samples"
                )
                return
            stored_type = meta.get("target_type", "")
            mismatch = stored_type and stored_type != self.target_type
            if stored_type == "chessboard" and self.target_type == "chessboard":
                mismatch = (
                    int(meta.get("chessboard_cols", 0)) != self.board_cols
                    or int(meta.get("chessboard_rows", 0)) != self.board_rows
                    or abs(float(meta.get("square_size", 0.0)) - self.square_size) > 1e-6
                )
            if mismatch:
                self.get_logger().warn(
                    f"{self.samples_file} was captured with different target settings "
                    f"({stored_type}, {meta.get('chessboard_cols')}x{meta.get('chessboard_rows')}, "
                    f"{meta.get('square_size')}): ignoring it. Call ~/reset to start clean."
                )
                return
            self.samples = list(samples)
        except Exception as exc:  # corrupt file should not block new work
            self.get_logger().warn(f"cannot read {self.samples_file}: {exc}")

    def _publish_stored_calibration(self):
        if not self.publish_tf or not os.path.exists(self.output_yaml):
            return
        try:
            import yaml

            with open(self.output_yaml, "r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle)
            entry = data["handeye_calibration"]
            xyz = entry["xyz"]
            quat = entry.get("quaternion") or list(
                geom.matrix_to_quat(geom.euler_to_matrix(entry["rpy"]))
            )
            self._send_tf(xyz, quat, str(entry["parent_frame"]), str(entry["child_frame"]))
            self.get_logger().info(
                f"republished calibration {entry['parent_frame']} -> {entry['child_frame']}"
            )
        except Exception as exc:
            self.get_logger().warn(f"cannot republish {self.output_yaml}: {exc}")

    def _send_tf(self, xyz, quat, parent, child):
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = parent
        message.child_frame_id = child
        message.transform.translation.x = float(xyz[0])
        message.transform.translation.y = float(xyz[1])
        message.transform.translation.z = float(xyz[2])
        message.transform.rotation.x = float(quat[0])
        message.transform.rotation.y = float(quat[1])
        message.transform.rotation.z = float(quat[2])
        message.transform.rotation.w = float(quat[3])
        self.static_broadcaster.sendTransform(message)

    # ------------------------------------------------------------------ #
    # subscriptions
    # ------------------------------------------------------------------ #
    def _on_image(self, message: Image):
        self._last_image = message

    def _on_info(self, message: CameraInfo):
        self._last_info = message

    @staticmethod
    def _to_bgr(message: Image) -> np.ndarray | None:
        channels = {"rgb8": 3, "bgr8": 3, "mono8": 1, "8UC3": 3, "8UC1": 1}.get(message.encoding)
        if channels is None:
            return None
        data = np.frombuffer(message.data, dtype=np.uint8)
        row_bytes = message.width * channels
        if message.step == row_bytes:
            image = data.reshape(message.height, message.width, channels)
        else:
            image = data.reshape(message.height, message.step)[:, :row_bytes].reshape(
                message.height, message.width, channels
            )
        if channels == 1:
            return np.ascontiguousarray(np.repeat(image, 3, axis=2))
        if message.encoding == "rgb8":
            image = image[:, :, ::-1]
        return np.ascontiguousarray(image)

    # ------------------------------------------------------------------ #
    # tf helpers
    # ------------------------------------------------------------------ #
    def _lookup(self, target: str, source: str, stamp=None):
        if target == source:
            return np.eye(4)
        try:
            if stamp is None:
                transform = self.tf_buffer.lookup_transform(
                    target, source, Time(), Duration(seconds=0.2)
                )
            else:
                transform = self.tf_buffer.lookup_transform(
                    target, source, stamp, Duration(seconds=0.2)
                )
        except Exception as exc:
            self.get_logger().warn(f"tf {target} <- {source} failed: {exc}", throttle_duration_sec=2.0)
            return None
        return self._transform_to_matrix(transform)

    @staticmethod
    def _transform_to_matrix(transform: TransformStamped) -> np.ndarray:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return geom.make_transform(
            geom.quat_to_matrix([rotation.x, rotation.y, rotation.z, rotation.w]),
            [translation.x, translation.y, translation.z],
        )

    # ------------------------------------------------------------------ #
    # capture
    # ------------------------------------------------------------------ #
    def _capture_once(self, quiet: bool = False):
        image = self._last_image
        info = self._last_info
        if image is None or info is None:
            return False, "no camera frame received yet"

        stamp = Time.from_msg(image.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if age > self.max_image_age_s:
            return False, f"newest frame is {age:.2f} s old"

        bgr = self._to_bgr(image)
        if bgr is None:
            return False, f"unsupported encoding {image.encoding}"
        if len(info.k) == 0 or info.k[0] == 0.0:
            return False, "camera info has no intrinsics yet"

        camera_matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(info.d, dtype=np.float64) if len(info.d) > 0 else None
        detections = self.detector.detect(bgr, camera_matrix, distortion)
        detection = select_detection(
            detections,
            marker_id=self.tag_id,
            min_area_px=self.min_area_px,
            max_reprojection_error_px=self.max_reproj_error_px,
        )
        if detection is None:
            if not quiet:
                self.get_logger().warn(
                    f"no usable tag in view ({len(detections)} detected, "
                    f"min_area={self.min_area_px:.0f}px, "
                    f"max_reproj={self.max_reproj_error_px:.1f}px)"
                )
            return False, "no usable tag in view"

        base_flange = self._lookup(self.base_frame, self.flange_frame, stamp)
        if base_flange is None:
            base_flange = self._lookup(self.base_frame, self.flange_frame)
            if base_flange is None:
                return False, f"tf {self.base_frame} <- {self.flange_frame} unavailable"
            self.get_logger().warn("using latest tf instead of image-stamp tf", throttle_duration_sec=2.0)

        camera_optical = self._lookup(self.camera_frame, image.header.frame_id, stamp)
        if camera_optical is None:
            return False, f"tf {self.camera_frame} <- {image.header.frame_id} unavailable"
        cam_target = camera_optical @ detection.transform

        T = cam_target
        if not np.all(np.isfinite(T)):
            return False, "non finite pose"

        sample = HandEyeSample(
            base_flange=base_flange,
            cam_target=cam_target,
            meta={
                "marker_id": detection.marker_id,
                "reprojection_error_px": round(detection.reprojection_error_px, 4),
                "area_px": round(detection.area_px, 1),
                "stamp": float(image.header.stamp.sec) + float(image.header.stamp.nanosec) * 1e-9,
            },
        )

        for existing in self.samples:
            d_rot, d_tra = geom.transform_error(sample.base_flange, existing.base_flange)
            if d_tra * 1e-3 < self.dedupe_translation and math.radians(d_rot) < self.dedupe_rotation:
                return False, "too close to an existing pose, move the arm further"

        self.samples.append(sample)
        storage.save_samples(
            self.samples_file,
            self.samples,
            meta={
                "robot_sn": self.robot_sn,
                "base_frame": self.base_frame,
                "flange_frame": self.flange_frame,
                "camera_frame": self.camera_frame,
                "target_type": self.target_type,
                "tag_family": self.tag_family,
                "tag_size": self.tag_size,
                "chessboard_cols": self.board_cols,
                "chessboard_rows": self.board_rows,
                "square_size": self.square_size,
            },
        )
        self._last_capture_time = self.get_clock().now()
        self._last_saved_pose = sample.base_flange.copy()
        self._last_saved_time = self.get_clock().now()

        self._after_capture(sample)
        if not quiet:
            self.get_logger().info(
                f"sample {len(self.samples)}: {self.detector.describe() or detection.label} "
                f"reproj={detection.reprojection_error_px:.2f}px "
                f"area={detection.area_px:.0f}px "
                f"range={np.linalg.norm(geom.translation_of(cam_target)):.3f} m"
            )
        return True, f"sample {len(self.samples)} captured"

    # ------------------------------------------------------------------ #
    # guided mode: tell the operator which pose to do next
    # ------------------------------------------------------------------ #
    _PLAN = (
        ("stay at the observation pose (seed sample)", (0.0, 0.0, 0.0)),
        ("roll +20 deg about the tool z axis", (0.0, 0.0, 20.0)),
        ("roll -20 deg about the tool z axis", (0.0, 0.0, -20.0)),
        ("roll +40 deg about the tool z axis", (0.0, 0.0, 40.0)),
        ("roll -40 deg about the tool z axis", (0.0, 0.0, -40.0)),
        ("tilt +15 deg about the tool x axis", (15.0, 0.0, 0.0)),
        ("tilt -15 deg about the tool x axis", (-15.0, 0.0, 0.0)),
        ("tilt +15 deg about the tool y axis", (0.0, 15.0, 0.0)),
        ("tilt -15 deg about the tool y axis", (0.0, -15.0, 0.0)),
        ("tilt +25 deg about the tool x axis", (25.0, 0.0, 0.0)),
        ("tilt -25 deg about the tool x axis", (-25.0, 0.0, 0.0)),
        ("tilt +25 deg about the tool y axis", (0.0, 25.0, 0.0)),
        ("tilt -25 deg about the tool y axis", (0.0, -25.0, 0.0)),
        ("tilt +18 deg about both x and y", (18.0, 18.0, 0.0)),
        ("tilt -18 deg about both x and y", (-18.0, -18.0, 0.0)),
        ("move ~3 cm closer to the board, slight tilt", (10.0, 0.0, 0.0)),
        ("move ~3 cm farther from the board, slight tilt", (-10.0, 0.0, 10.0)),
    )

    def _relative_rpy(self):
        if self._seed_pose is None or len(self.samples) == 0:
            return None
        current = self._lookup(self.base_frame, self.flange_frame)
        if current is None:
            return None
        delta = geom.invert(self._seed_pose) @ current
        return [math.degrees(v) for v in geom.matrix_to_euler(geom.rotation_of(delta))]

    def _announce_next(self):
        total = len(self._PLAN)
        if self._plan_index >= total:
            self.get_logger().info(
                f"all {total} suggested poses are covered - call ~/solve "
                "(or restart with auto_solve_on_plan:=true)"
            )
            return
        label, _ = self._PLAN[self._plan_index]
        rpy = self._relative_rpy()
        where = ""
        if rpy is not None:
            where = (f"  (current vs first sample: x {rpy[0]:+.0f}, "
                     f"y {rpy[1]:+.0f}, roll {rpy[2]:+.0f} deg)")
        self.get_logger().info(
            f">>> pose {self._plan_index + 1}/{total}: {label}{where} - "
            "move there, then hold still for 1 s"
        )

    def _after_capture(self, sample):
        """Quality gate + guidance after a frame has been stored."""
        if self._seed_pose is None:
            self._seed_pose = sample.base_flange.copy()

        deviations = []
        for other in self.samples[:-1]:
            value = solver.pair_deviation(sample, other)
            if value is not None:
                deviations.append(value)
        if len(deviations) >= 2:
            median = float(np.median(deviations))
            if median > self.inconsistent_threshold:
                self.get_logger().warn(
                    f"sample {len(self.samples)} disagrees with the earlier ones by "
                    f"{median:.1f} deg (should be < 0.3): the arm was probably still "
                    "moving, or the board shifted. Redo this pose."
                )
                if self.reject_inconsistent:
                    self.samples.pop()
                    storage.save_samples(self.samples_file, self.samples)
                    self.get_logger().warn("sample discarded (reject_inconsistent)")
                    return

        if self.guided:
            self._plan_index += 1
            if self.auto_solve_on_plan and self._plan_index >= len(self._PLAN):
                self.get_logger().info(
                    f"all {len(self._PLAN)} planned poses captured - solving automatically"
                )
                try:
                    self._solve_and_report()
                except Exception as exc:
                    self.get_logger().error(f"automatic solve failed: {exc}")
            else:
                self._announce_next()

    def _planned_rotation(self):
        if self._plan_index >= len(self._PLAN):
            return None
        _, (rx, ry, rz) = self._PLAN[self._plan_index]
        return (
            geom.rot_x(math.radians(rx))
            @ geom.rot_y(math.radians(ry))
            @ geom.rot_z(math.radians(rz))
        )

    def _near_planned_pose(self, current):
        if not self.guided or self._seed_pose is None:
            return True
        planned = self._planned_rotation()
        if planned is None:
            return True
        relative = geom.rotation_of(geom.invert(self._seed_pose) @ current)
        error = math.degrees(geom.rotation_angle(geom.invert(planned) @ relative))
        return error <= self.pose_tolerance_deg

    def _auto_capture_tick(self):
        if self._last_image is None or self._last_info is None:
            return
        now = self.get_clock().now()
        pose = self._lookup(self.base_frame, self.flange_frame)
        if pose is None:
            return
        if not self._near_planned_pose(pose):
            return

        # gate on BOTH linear and angular speed: a wrist roll has almost no
        # linear velocity, so a linear-only gate happily samples a spinning arm
        # and every frame comes out blurred
        previous = self._velocity_pose
        previous_time = self._velocity_time
        self._velocity_pose, self._velocity_time = pose, now
        if previous is not None and previous_time is not None:
            dt = (now - previous_time).nanoseconds / 1e9
            if dt > 1e-3:
                linear = np.linalg.norm(
                    geom.translation_of(pose) - geom.translation_of(previous)
                ) / dt
                angular = math.degrees(
                    geom.rotation_angle(geom.rotation_of(geom.invert(previous) @ pose))
                ) / dt
                if linear > self.auto_max_speed or angular > self.auto_max_angular_speed:
                    self._still_since = None
                    return

        if self._still_since is None:
            self._still_since = now
            return
        if (now - self._still_since).nanoseconds / 1e9 < self.settle_time:
            return
        if self._last_saved_time is not None:
            if (now - self._last_saved_time).nanoseconds / 1e9 < self.auto_cooldown_s:
                return
        if self._last_saved_pose is not None:
            d_rot, d_tra = geom.transform_error(pose, self._last_saved_pose)
            if (d_tra * 1e-3 < self.auto_min_translation
                    and math.radians(d_rot) < self.auto_min_rotation):
                return

        captured, message = self._capture_once(quiet=True)
        if captured:
            self.get_logger().info(message)
        elif "too close" not in message and "no usable tag" not in message:
            self.get_logger().debug(message)

    # ------------------------------------------------------------------ #
    # services
    # ------------------------------------------------------------------ #
    def _srv_capture(self, request, response):
        try:
            ok, message = self._capture_once(quiet=False)
        except Exception as exc:
            ok, message = False, f"capture failed: {exc}"
        response.success = ok
        response.message = f"{message} ({len(self.samples)} samples)"
        return response

    def _solve_and_report(self):
        if len(self.samples) < self.min_samples:
            return False, f"need at least {self.min_samples} samples, have {len(self.samples)}"
        samples = list(self.samples)
        flipped = []
        if self.normalize_symmetric and hasattr(self.detector, "symmetry_candidates"):
            try:
                samples, flipped = solver.align_symmetric_targets(
                    samples, self.detector.symmetry_candidates()
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"symmetry realignment skipped ({exc}); using the raw samples"
                )
                samples, flipped = list(self.samples), []
        result = solver.solve(
            samples,
            method=self.solve_method,
            reject_outliers=self.reject_outliers,
        )
        if flipped:
            self.get_logger().warn(
                f"board symmetry: flipped {len(flipped)} samples back ({flipped})"
            )
        calibration = storage.build_calibration(
            result, self.robot_sn, self.flange_frame, self.camera_frame
        )
        if self.auto_save:
            storage.write_calibration_yaml(self.output_yaml, calibration)
        if self.publish_tf:
            entry = calibration["handeye_calibration"]
            self._send_tf(entry["xyz"], entry["quaternion"],
                          entry["parent_frame"], entry["child_frame"])
        self.last_result = result
        report = storage.format_report(result, self.flange_frame, self.camera_frame)
        verdict = storage.verdict(result)
        if verdict.startswith("GOOD"):
            hint = ("keep this node running (it publishes the tf), then run "
                    "validate.launch.py to confirm on fresh poses")
        elif verdict.startswith("ACCEPTABLE"):
            hint = "usable, but confirm with the validator before using it for grasping"
        else:
            hint = "do NOT use this result: reset, fix what the warning says and recapture"
            if result.metrics.get("rotation_consistency_deg", 0.0) > 1.0:
                hint = ("do NOT use: the samples disagree with each other - recapture with "
                        "the arm fully stopped (the guided prompts wait for 1 s of stillness)")
        text = (f"{report}\nverdict: {verdict}\nnext: {hint}\n"
                f"written: {self.output_yaml if self.auto_save else '(not saved)'}"
                + (f"\nsymmetry: realigned {len(flipped)} samples ({flipped})" if flipped else ""))
        self.get_logger().info("\n" + text)
        return True, text

    def _srv_solve(self, request, response):
        try:
            ok, message = self._solve_and_report()
            response.success = ok
            response.message = message
        except Exception as exc:
            response.success = False
            response.message = f"solve failed: {exc}"
            self.get_logger().error(response.message)
        return response

    def _srv_save(self, request, response):
        if self.last_result is None:
            response.success = False
            response.message = "nothing to save, call ~/solve first"
            return response
        try:
            calibration = storage.build_calibration(
                self.last_result, self.robot_sn, self.flange_frame, self.camera_frame
            )
            storage.write_calibration_yaml(self.output_yaml, calibration)
            response.success = True
            response.message = f"written {self.output_yaml}"
        except Exception as exc:
            response.success = False
            response.message = f"save failed: {exc}"
        return response

    def _srv_reset(self, request, response):
        self.samples = []
        self.last_result = None
        self._seed_pose = None
        self._plan_index = 0
        storage.save_samples(self.samples_file, [], meta={"robot_sn": self.robot_sn})
        if self.guided:
            self._announce_next()
        response.success = True
        response.message = "samples cleared"
        return response

    def _srv_status(self, request, response):
        lines = [
            f"samples     : {len(self.samples)}",
            f"base/flange : {self.base_frame} / {self.flange_frame}",
            f"camera      : {self.camera_frame}",
            f"target      : {self.detector.describe()}",
            f"normalize   : {self.normalize_symmetric}",
            f"auto_capture: {self.auto_capture}",
            f"samples file: {self.samples_file}",
            f"output yaml : {self.output_yaml}",
        ]
        if self.last_result is not None:
            lines.append(storage.format_report(self.last_result))
            lines.append(f"verdict     : {storage.verdict(self.last_result)}")
        response.success = True
        response.message = "\n".join(lines)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = HandeyeCalibrator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
