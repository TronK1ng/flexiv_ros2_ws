"""Pre-flight check: frames, intrinsics, tf chain and tag visibility.

Run this before the first calibration session; it prints one line per check so
that a broken tf tree or a wrong tag size is found in ten seconds instead of
during the first sample.

    ros2 run flexiv_handeye_calib doctor --ros-args -p robot_sn:=Rizon4s-123456
"""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from . import geometry as geom
from .detection import select_detection
from .detectors import make_detector


class Doctor(Node):
    def __init__(self):
        super().__init__("handeye_doctor")
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
        self.board_cols = int(self.declare_parameter("chessboard_cols", 8).value)
        self.board_rows = int(self.declare_parameter("chessboard_rows", 11).value)
        self.square_size = float(self.declare_parameter("square_size", 0.006).value)
        self.board_auto = bool(self.declare_parameter("chessboard_auto", False).value)
        self.timeout_s = float(self.declare_parameter("timeout_s", 10.0).value)

        if not self.base_frame:
            self.base_frame = f"{self.robot_sn}_base_link" if self.robot_sn else "base_link"
        if not self.flange_frame:
            self.flange_frame = f"{self.robot_sn}_flange" if self.robot_sn else "flange"

        self._image = None
        self._info = None
        self._image_count = 0
        self._info_count = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # sensor-data QoS (best effort) so that both reliable and best-effort
        # camera drivers can be received
        self.create_subscription(
            Image, self.image_topic, self._on_image, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_info, qos_profile_sensor_data
        )

    def _on_image(self, message):
        self._image = message
        self._image_count += 1

    def _on_info(self, message):
        self._info = message
        self._info_count += 1

    def _wait(self):
        deadline = time.time() + self.timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._image is not None and self._info is not None:
                return True
        return False

    def run(self) -> int:
        checks = []

        def check(name, ok, detail=""):
            checks.append(ok)
            self.get_logger().info(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f" - {detail}" if detail else ""))

        received = self._wait()
        check("camera image", self._image is not None,
              f"{self.image_topic} - {self._image_count} msgs, "
              f"{self._endpoint_summary(self.image_topic)}")
        check("camera info", self._info is not None,
              f"{self.camera_info_topic} - {self._info_count} msgs, "
              f"{self._endpoint_summary(self.camera_info_topic)}")
        if not received:
            self.get_logger().info(
                "hint: is the camera driver running? try "
                f"'ros2 topic hz {self.image_topic}' and 'ros2 node list | grep camera'"
            )
            return 1
        self.get_logger().info(
            f"[INFO] image {self._image.width}x{self._image.height} "
            f"{self._image.encoding}, frame '{self._image.header.frame_id}'"
        )

        info = self._info
        k = (np.asarray(info.k, dtype=np.float64).reshape(3, 3)
             if len(info.k) > 0 else None)
        check("intrinsics", k is not None and k[0, 0] > 0 and k[1, 1] > 0,
              "" if k is None else f"fx={k[0, 0]:.1f} fy={k[1, 1]:.1f} cx={k[0, 2]:.1f} cy={k[1, 2]:.1f}")

        after_flange, error = self._lookup(self.base_frame, self.flange_frame)
        if after_flange is not None:
            position = geom.translation_of(after_flange)
            check("tf base <- flange", True,
                  f"{self.base_frame} <- {self.flange_frame} at "
                  f"[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}] m")
        else:
            check("tf base <- flange", False, error)

        if self.camera_frame == self._image.header.frame_id:
            eye, error = np.eye(4), ""
        else:
            eye, error = self._lookup(self.camera_frame, self._image.header.frame_id)
        check("tf camera <- optical", eye is not None,
              f"{self.camera_frame} <- {self._image.header.frame_id}" if eye is not None else error)

        age = (self.get_clock().now() - Time.from_msg(self._image.header.stamp)).nanoseconds / 1e9
        check("image age", 0.0 <= age < 1.0, f"{age:.3f} s")

        detector = make_detector(
            self.target_type,
            tag_family=self.tag_family,
            tag_size=self.tag_size,
            board_cols=self.board_cols,
            board_rows=self.board_rows,
            square_size=self.square_size,
            board_auto=self.board_auto,
        )
        detections = detector.detect(self._bgr(self._image), k, info.d)
        best = select_detection(detections, None, 0.0, 5.0)
        if best is None:
            check("target visible", False,
                  f"{len(detections)} detected, check lighting/focus/distance or the pattern")
        else:
            cam_tag = (eye @ best.transform) if eye is not None else best.transform
            distance = float(np.linalg.norm(geom.translation_of(cam_tag)))
            pixels = math.sqrt(best.area_px)
            pattern = getattr(detector, "last_pattern", None)
            square_px = self._square_pixels(best, pattern)
            detail = (
                f"{detector.describe()} {pixels:.0f}px wide, {distance:.3f} m away, "
                f"reproj={best.reprojection_error_px:.2f}px"
            )
            if pattern:
                detail += f", detected {pattern[0]}x{pattern[1]} corners"
            if square_px:
                detail += f", ~{square_px:.1f} px/square"
            check("target visible", True, detail)
            if square_px is None:
                check("target size sanity", pixels >= 20.0,
                      "a target below ~20 px gives poor pose accuracy")
            else:
                check("corner spacing", square_px >= 12.0,
                      f"{square_px:.1f} px per square"
                      + ("" if square_px >= 12.0 else
                         " - too far, below 12 px/square gives poor pose accuracy"))
            if after_flange is not None:
                # pose of the tag from the robot chain, useful to see if the
                # published calibration is roughly sane
                flange_camera, error = self._lookup(self.flange_frame, self.camera_frame, 1.0)
                if flange_camera is None:
                    # before the first solve there is no flange -> camera link yet;
                    # this is not a failure
                    self.get_logger().info(
                        f"[SKIP] tag pose in base - no {self.flange_frame} -> {self.camera_frame} "
                        "published yet (expected before the first solve)"
                    )
                else:
                    target = after_flange @ flange_camera @ cam_tag
                    position = geom.translation_of(target)
                    check("tag pose in base", True,
                          f"[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}] m")

        return 0 if all(checks) else 1

    @staticmethod
    def _square_pixels(detection, pattern):
        """Mean pixel distance between neighbouring corners (one square)."""
        if not pattern:
            return None
        cols, rows = pattern
        try:
            grid = detection.corners.reshape(rows, cols, 2)
            dx = np.linalg.norm(np.diff(grid, axis=1), axis=2)
            dy = np.linalg.norm(np.diff(grid, axis=0), axis=2)
            return float(np.mean(np.concatenate([dx.ravel(), dy.ravel()])))
        except Exception:
            return None

    def _endpoint_summary(self, topic):
        try:
            infos = self.get_publishers_info_by_topic(topic)
        except Exception as exc:  # graph introspection is best effort
            return f"publisher lookup failed: {exc}"
        if not infos:
            return "no publisher"
        reliability = infos[0].qos_profile.reliability
        kind = "RELIABLE" if int(reliability) == 1 else "BEST_EFFORT"
        return f"{len(infos)} publisher(s), {kind}"

    def _lookup(self, target, source, timeout=None):
        """Look a transform up, spinning until it arrives or the timeout expires."""
        deadline = time.time() + (self.timeout_s if timeout is None else timeout)
        error = "transform not available"
        while True:
            try:
                return self._to_matrix(
                    self.tf_buffer.lookup_transform(target, source, Time())
                ), ""
            except Exception as exc:
                error = str(exc)
            if time.time() >= deadline:
                return None, error
            rclpy.spin_once(self, timeout_sec=0.05)

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

    @staticmethod
    def _bgr(message: Image):
        channels = {"rgb8": 3, "bgr8": 3, "mono8": 1}[message.encoding]
        data = np.frombuffer(message.data, dtype=np.uint8)
        row = message.width * channels
        image = data.reshape(message.height, message.step)[:, :row].reshape(
            message.height, message.width, channels
        )
        if channels == 1:
            image = np.repeat(image, 3, axis=2)
        elif message.encoding == "rgb8":
            image = image[:, :, ::-1]
        return np.ascontiguousarray(image)


def main(args=None):
    rclpy.init(args=args)
    node = Doctor()
    code = 1
    try:
        code = node.run()
    except Exception as exc:
        node.get_logger().error(f"doctor failed: {exc}")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
