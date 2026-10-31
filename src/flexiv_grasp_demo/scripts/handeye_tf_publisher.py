#!/usr/bin/env python3
"""Publish the hand-eye calibration as a static TF.

Reads the YAML written by flexiv_handeye_calib:

    handeye_calibration:
      type: eye_in_hand
      robot_sn: Rizon4s-064230
      parent_frame: Rizon4s-064230_flange
      child_frame: camera_link
      xyz: [x, y, z]
      rpy: [r, p, y]            # rad, used only if "quaternion" is absent
      quaternion: [x, y, z, w]

The transform is published on /tf_static, so every other node (perception, RViz)
gets it without further wiring. `camera_link -> camera_color_optical_frame` is
published by the RealSense driver and is intentionally not duplicated here.
"""

import os

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


class HandeyeTfPublisher(Node):
    def __init__(self):
        super().__init__("handeye_tf_publisher")

        yaml_path = self.declare_parameter("yaml_path", "").value
        robot_sn = self.declare_parameter("robot_sn", "").value
        parent_override = self.declare_parameter("parent_frame", "").value
        child_override = self.declare_parameter("child_frame", "").value

        if not yaml_path or not os.path.isfile(yaml_path):
            raise FileNotFoundError(f"hand-eye calibration file not found: '{yaml_path}'")

        with open(yaml_path, "r", encoding="utf-8") as handle:
            document = yaml.safe_load(handle)
        if not isinstance(document, dict) or "handeye_calibration" not in document:
            raise ValueError(f"'{yaml_path}' has no 'handeye_calibration' section")
        calibration = document["handeye_calibration"]

        calib_type = calibration.get("type", "eye_in_hand")
        if calib_type != "eye_in_hand":
            self.get_logger().warn(
                f"calibration type is '{calib_type}', not 'eye_in_hand'; publishing it anyway"
            )

        calib_sn = calibration.get("robot_sn", "")
        if robot_sn and calib_sn and calib_sn != robot_sn:
            self.get_logger().error(
                f"calibration was made for '{calib_sn}' but the robot is '{robot_sn}'; "
                "the parent frame below will not match the TF tree"
            )

        parent = parent_override or calibration["parent_frame"]
        child = child_override or calibration["child_frame"]
        xyz = calibration["xyz"]
        quaternion = calibration.get("quaternion")

        if quaternion is None:
            roll, pitch, yaw = calibration["rpy"]
            from tf_transformations import quaternion_from_euler

            quaternion = quaternion_from_euler(roll, pitch, yaw)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = float(xyz[0])
        transform.transform.translation.y = float(xyz[1])
        transform.transform.translation.z = float(xyz[2])
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])

        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(transform)

        quality = calibration.get("quality", {})
        self.get_logger().info(
            f"published {parent} -> {child} "
            f"xyz=[{xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f}] m "
            f"from '{os.path.basename(yaml_path)}'"
        )
        if quality:
            self.get_logger().info(
                "calibration quality: "
                f"{quality.get('samples', '?')} samples, "
                f"{quality.get('translation_rms_mm', float('nan')):.2f} mm rms, "
                f"{quality.get('rotation_rms_deg', float('nan')):.2f} deg rms"
            )


def main():
    rclpy.init()
    node = HandeyeTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
