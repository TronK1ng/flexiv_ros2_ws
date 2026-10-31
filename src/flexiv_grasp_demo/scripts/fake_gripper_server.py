#!/usr/bin/env python3
"""Stand-in for flexiv_gripper_node when the stack runs with fake hardware.

The real gripper node needs an RDK connection, so `use_fake_hardware:=true` leaves
the /flexiv_gripper_node/{move,grasp} action servers missing and the demo cannot
exercise its gripper clients. This node fills that gap with the same interface and
a very small width simulation.

It is never launched alongside the real node: the demo launch file only starts it
when use_fake_hardware is true.

Parameters:
  open_width / object_width : m, fully open width and the width of the simulated
                              grasped object (0 -> grasp closes empty and fails)
  default_velocity          : m/s
  joint_name                : joint to publish, must match the URDF
                              ("<robot_sn>_finger_width_joint")
"""

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from flexiv_msgs.action import Grasp, Move


class FakeGripper(Node):
    def __init__(self):
        super().__init__("flexiv_gripper_node")
        self.open_width = float(self.declare_parameter("open_width", 0.09).value)
        self.object_width = float(self.declare_parameter("object_width", 0.02).value)
        self.velocity = float(self.declare_parameter("default_velocity", 0.1).value)
        self.joint_name = self.declare_parameter("joint_name", "finger_width_joint").value
        # The URDF drives the finger joints from finger_width_joint through a
        # mimic multiplier, so the published joint state has to be in that space.
        self.multiplier = float(
            self.declare_parameter("width_to_joint_multiplier", 9.404).value
        )
        self.offset = float(self.declare_parameter("width_to_joint_offset", -0.155).value)
        # Width published to the robot model. The URDF's finger collision meshes are
        # coarse and report self-collisions at some narrow openings, which would make
        # MoveIt reject perfectly fine poses. This node only stands in for the gripper,
        # so the model stays open while the reported width still simulates motion.
        self.state_width = float(
            self.declare_parameter("state_width", self.open_width).value
        )

        self.width = self.open_width
        self.stopped = False

        self.state_publisher = self.create_publisher(JointState, "~/gripper_joint_states", 10)
        self.create_timer(0.02, self._publish_state)

        self.move_server = ActionServer(
            self,
            Move,
            "~/move",
            self._execute_move,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
        )
        self.grasp_server = ActionServer(
            self,
            Grasp,
            "~/grasp",
            self._execute_grasp,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
        )
        self.stop_service = self.create_service(Trigger, "~/stop", self._on_stop)
        self.get_logger().info(
            f"fake gripper ready: open={self.open_width:.3f} m "
            f"object={self.object_width:.3f} m joint='{self.joint_name}'"
        )

    # ---------------- helpers ----------------
    def _on_goal(self, _goal_request):
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _on_stop(self, _request, response):
        self.stopped = True
        response.success = True
        response.message = "fake gripper stopped"
        self.get_logger().info("stop requested")
        return response

    def _publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.joint_name]
        msg.position = [self.multiplier * self.state_width + self.offset]
        msg.velocity = [0.0]
        msg.effort = [0.0]
        self.state_publisher.publish(msg)

    def _run_to(self, target, velocity, goal_handle, feedback_factory):
        self.stopped = False
        dt = 0.02
        velocity = max(velocity, 0.005)
        last_feedback = 0.0
        while abs(target - self.width) > 1e-4:
            if self.stopped or (goal_handle is not None and goal_handle.is_cancel_requested):
                return False
            step = target - self.width
            step = max(-velocity * dt, min(velocity * dt, step))
            self.width += step
            now = time.monotonic()
            if feedback_factory is not None and now - last_feedback > 0.05:
                goal_handle.publish_feedback(feedback_factory())
                last_feedback = now
            time.sleep(dt)
        self.width = target
        # The last in-loop feedback may be one step short of the target; publish the
        # final width so clients see the settled value.
        if feedback_factory is not None:
            goal_handle.publish_feedback(feedback_factory())
        return True

    # ---------------- actions ----------------
    def _execute_move(self, goal_handle):
        request = goal_handle.request
        target = max(0.0, min(request.width, self.open_width * 1.1))
        velocity = request.velocity if request.velocity > 0.0 else self.velocity
        self.get_logger().info(f"move: width {self.width:.3f} -> {target:.3f} m")

        reached = self._run_to(
            target,
            velocity,
            goal_handle,
            lambda: self._move_feedback(),
        )

        # Move.action only carries success/error in the result; width and force
        # are reported through feedback while the fingers move.
        result = Move.Result()
        result.success = reached
        result.error = "" if reached else "stopped before reaching the target"
        if reached:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _execute_grasp(self, goal_handle):
        request = goal_handle.request
        target = self.object_width if self.object_width > 1e-3 else 0.0
        self.get_logger().info(
            f"grasp: force {request.force:.1f} N, closing to {target:.3f} m"
        )

        reached = self._run_to(
            target,
            self.velocity,
            goal_handle,
            lambda: self._grasp_feedback(),
        )

        result = Grasp.Result()
        result.success = reached and self.object_width > 1e-3
        if result.success:
            result.error = ""
        elif not reached:
            result.error = "stopped before closing"
        else:
            result.error = "gripper closed on nothing"
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _move_feedback(self):
        feedback = Move.Feedback()
        feedback.current_width = self.width
        feedback.current_force = 0.0
        feedback.moving = True
        return feedback

    def _grasp_feedback(self):
        feedback = Grasp.Feedback()
        feedback.current_width = self.width
        # Report a force once the fingers are pressing the simulated object, so the task
        # node's force-based success check can be exercised offline.
        feedback.current_force = 10.0 if self.width <= self.object_width + 1e-3 else 0.0
        feedback.moving = True
        return feedback


def main():
    rclpy.init()
    node = FakeGripper()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
