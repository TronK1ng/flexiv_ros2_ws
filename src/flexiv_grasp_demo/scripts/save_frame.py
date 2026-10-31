#!/usr/bin/env python3
"""Save one frame from an image topic to a PNG and exit.

Handy when a detection fails and the quickest way to know why is to look at the
frame. Works for the raw camera stream and for /grasp_demo/debug_image (which has
the detected markers drawn on it).

    ros2 run flexiv_grasp_demo save_frame.py --ros-args \
        -p topic:=/grasp_demo/debug_image -p output:=$PWD/.ros_home/debug.png
"""

import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class SaveFrame(Node):
    def __init__(self):
        super().__init__("save_frame")
        self.topic = self.declare_parameter("topic", "/grasp_demo/debug_image").value
        self.output = os.path.expanduser(
            self.declare_parameter("output", "frame.png").value
        )
        self.saved = False
        self.create_subscription(Image, self.topic, self._on_image, qos_profile_sensor_data)
        self.get_logger().info(f"waiting for one frame on {self.topic}")

    def _on_image(self, msg):
        if self.saved:
            return
        self.saved = True

        channels = 3 if msg.encoding in ("bgr8", "rgb8") else 1
        row = msg.width * channels
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.step == row:
            image = data.reshape(msg.height, msg.width, channels)
        else:
            image = data.reshape(msg.height, msg.step)[:, :row].reshape(
                msg.height, msg.width, channels
            )
        if channels == 3 and msg.encoding == "rgb8":
            image = image[:, :, ::-1]

        directory = os.path.dirname(self.output)
        if directory:
            os.makedirs(directory, exist_ok=True)
        ok = cv2.imwrite(self.output, image)
        self.get_logger().info(
            f"{'saved' if ok else 'FAILED to save'} {msg.width}x{msg.height} "
            f"{msg.encoding} -> {self.output}"
        )
        rclpy.shutdown()


def main():
    rclpy.init()
    node = SaveFrame()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
