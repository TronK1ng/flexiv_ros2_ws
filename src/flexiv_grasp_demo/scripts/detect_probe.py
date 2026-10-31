#!/usr/bin/env python3
"""Sample /grasp_demo/detect and report the spread of the detected pose.

Used for the M2 static check: put the tagged cube on the table, put the arm in
some pose, run this tool, then move the arm somewhere else and run it again.

Because the cube does not move, the *base frame* position reported by the
perception node must stay the same across arm poses. The spread is therefore a
direct measure of hand-eye + detection error, with no ruler required.

    ros2 run flexiv_grasp_demo detect_probe.py --ros-args -p samples:=5

Parameters:
  samples   number of service calls per run (default 5)
  interval  seconds between calls (default 0.3)
  object_id tag id filter; empty = largest visible tag
  service   service name (default /grasp_demo/detect)
"""

import statistics
import sys
import time

import rclpy
from rclpy.node import Node

from flexiv_grasp_demo.srv import Detect


class DetectProbe(Node):
    def __init__(self):
        super().__init__("detect_probe")
        self.samples = int(self.declare_parameter("samples", 5).value)
        self.interval = float(self.declare_parameter("interval", 0.3).value)
        self.object_id = self.declare_parameter("object_id", "").value
        self.retries = int(self.declare_parameter("retries", 3).value)
        service_name = self.declare_parameter("service", "/grasp_demo/detect").value

        self.client = self.create_client(Detect, service_name)
        if not self.client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"{service_name} is not available")

    def _report(self, text):
        """Print to the console and to the ROS log file, so a run can be reviewed later."""
        print(text)
        self.get_logger().info(text)

    def _call_once(self):
        request = Detect.Request()
        request.object_id = self.object_id
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result()

    def run(self):
        positions = []
        ids = []
        first_message = None

        for index in range(self.samples):
            # A single missed frame is normal (blur, TF skew, exposure); retry before
            # giving up so the stability measurement is not derailed by one drop-out.
            response = None
            attempts = 0
            for _ in range(max(1, self.retries)):
                attempts += 1
                response = self._call_once()
                if response is not None and not response.image_stale and response.targets:
                    break
                time.sleep(self.interval)

            if response is None:
                self._report(f"sample {index + 1}: no response from the service")
                return 1
            if response.image_stale:
                self._report(f"sample {index + 1}: image_stale=True ({response.message})")
                return 1
            if not response.targets:
                self._report(f"sample {index + 1}: no marker visible ({response.message})")
                return 1

            target = response.targets[0]
            position = target.pose.pose.position
            positions.append((position.x, position.y, position.z))
            ids.append(target.id)
            if first_message is None:
                first_message = (
                    f"id={target.id} frame={target.pose.header.frame_id} "
                    f"area={target.area:.0f}px reproj={target.reprojection_error:.2f}px"
                )
            retried = f" (after {attempts} attempts)" if attempts > 1 else ""
            self._report(
                f"sample {index + 1}: "
                f"[{position.x:+.4f}, {position.y:+.4f}, {position.z:+.4f}] m{retried}"
            )
            time.sleep(self.interval)

        if len(set(ids)) > 1:
            print("warning: different tag ids were detected between samples")

        means = [statistics.fmean(axis) for axis in zip(*positions)]
        deviations = []
        for axis_index, axis in enumerate(zip(*positions)):
            deviations.append(statistics.pstdev(axis) * 1000.0 if len(axis) > 1 else 0.0)

        self._report(f"RESULT {first_message}")
        self._report("RESULT mean: [{:+.4f}, {:+.4f}, {:+.4f}] m".format(*means))
        self._report(
            "RESULT std : [{:.2f}, {:.2f}, {:.2f}] mm  over {} samples".format(
                *deviations, len(positions)
            )
        )
        print(
            "\nrecord this mean, move the arm to a clearly different pose, run again.\n"
            "the means from different poses must agree to a few mm: the cube did not move."
        )
        return 0


def main():
    rclpy.init()
    try:
        probe = DetectProbe()
    except RuntimeError as error:
        print(error)
        rclpy.shutdown()
        return 1
    try:
        return probe.run()
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
