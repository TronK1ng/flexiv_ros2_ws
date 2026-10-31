"""Live readout of how far the flange has moved from a marked reference pose.

Handy while dragging the MoveIt interactive marker: it tells you the rotation
angle you actually achieved, instead of guessing from the RViz rings.

    ros2 run flexiv_handeye_calib pose_delta --ros-args -p robot_sn:=Rizon4s-064230

It prints one line whenever the pose changes by more than ~0.5 deg / 1 mm:

    d_rot  21.3 deg (axis 0.98 0.05 0.12)  d_trans  3.2 mm  |  total  21.3 deg

`~/mark` resets the reference to the current pose (call it at every new
calibration pose if you want to check each step in isolation).
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from . import geometry as geom


class PoseDelta(Node):
    def __init__(self):
        super().__init__("pose_delta")
        self.robot_sn = self.declare_parameter("robot_sn", "").value
        self.base_frame = self.declare_parameter("base_frame", "").value
        self.flange_frame = self.declare_parameter("flange_frame", "").value
        self.period = float(self.declare_parameter("period", 0.5).value)

        if not self.base_frame:
            self.base_frame = f"{self.robot_sn}_base_link" if self.robot_sn else "base_link"
        if not self.flange_frame:
            self.flange_frame = f"{self.robot_sn}_flange" if self.robot_sn else "flange"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_service(Trigger, "~/mark", self._mark)

        self.reference = None       # pose at the last ~/mark
        self.start = None           # very first pose seen
        self.last_printed = None
        self.create_timer(self.period, self._tick)
        self.get_logger().info(
            f"tracking {self.base_frame} <- {self.flange_frame}; "
            "call ~/mark to reset the reference"
        )

    # ------------------------------------------------------------------ #
    def _current(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.flange_frame, Time()
            )
        except Exception:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return geom.make_transform(
            geom.quat_to_matrix([rotation.x, rotation.y, rotation.z, rotation.w]),
            [translation.x, translation.y, translation.z],
        )

    def _tick(self):
        pose = self._current()
        if pose is None:
            return
        if self.start is None:
            self.start = pose
            self.reference = pose
            self.get_logger().info("reference pose captured")
            return
        if self.last_printed is not None:
            angle = math.degrees(geom.rotation_angle(geom.rotation_of(
                geom.invert(self.last_printed) @ pose)))
            distance = np.linalg.norm(
                geom.translation_of(pose) - geom.translation_of(self.last_printed)
            ) * 1000.0
            if angle < 0.5 and distance < 1.0:
                return

        delta = geom.invert(self.reference) @ pose
        angle = math.degrees(geom.rotation_angle(geom.rotation_of(delta)))
        axis = geom.log_so3(geom.rotation_of(delta))
        norm = float(np.linalg.norm(axis))
        axis_text = (
            f"{axis[0] / norm:+.2f} {axis[1] / norm:+.2f} {axis[2] / norm:+.2f}"
            if norm > 1e-9 else "0.00 0.00 0.00"
        )
        translation = float(np.linalg.norm(geom.translation_of(delta)) * 1000.0)
        total = math.degrees(geom.rotation_angle(
            geom.rotation_of(geom.invert(self.start) @ pose)))
        position = geom.translation_of(pose)
        self.get_logger().info(
            f"d_rot {angle:5.1f} deg (axis {axis_text})  d_trans {translation:5.1f} mm"
            f"  |  total {total:5.1f} deg  |  pos "
            f"[{position[0]:.3f} {position[1]:.3f} {position[2]:.3f}]"
        )
        self.last_printed = pose

    def _mark(self, request, response):
        pose = self._current()
        if pose is None:
            response.success = False
            response.message = f"tf {self.base_frame} <- {self.flange_frame} unavailable"
            return response
        self.reference = pose
        self.last_printed = None
        response.success = True
        response.message = "reference updated"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PoseDelta()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
