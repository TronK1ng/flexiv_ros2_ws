"""Automatic pose sequencer for the eye-in-hand calibration.

The arm is moved through a fixed set of orientations using the same control path
the rest of the stack already uses - MoveIt IK plus the joint trajectory
controller - so the running ``ros2_control`` keeps ownership of the robot.  A
second RDK client would compete with it for motion commands, which is why this
node does not talk to the SDK directly.

Because the camera is not on the flange axis, tilting the wrist swings the board
across the image.  The node therefore measures, once, how a small flange
translation moves the board in the image, and uses that 2x2 Jacobian to
re-centre the board after every rotation.  Large rotations are split into two
halves so the board never leaves the frame between corrections.

    ros2 run flexiv_handeye_calib auto_move --ros-args -p robot_sn:=Rizon4s-064230
    ros2 service call /auto_move/run std_srvs/srv/Trigger

Nothing moves until ``~/run`` is called.  Keep your hand on the e-stop.
"""

from __future__ import annotations

import math
import time
from typing import Generator, Optional

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from . import geometry as geom
from .detection import select_detection
from .detectors import make_detector
from .image_utils import to_bgr

#: (label, (rx, ry, rz) in degrees about the tool axes, applied from the seed)
PLAN = (
    ("roll +20", (0.0, 0.0, 20.0)),
    ("roll -20", (0.0, 0.0, -20.0)),
    ("roll +40", (0.0, 0.0, 40.0)),
    ("roll -40", (0.0, 0.0, -40.0)),
    ("tilt x +12", (12.0, 0.0, 0.0)),
    ("tilt x -12", (-12.0, 0.0, 0.0)),
    ("tilt y +12", (0.0, 12.0, 0.0)),
    ("tilt y -12", (0.0, -12.0, 0.0)),
    ("tilt x +18", (18.0, 0.0, 0.0)),
    ("tilt x -18", (-18.0, 0.0, 0.0)),
    ("tilt y +18", (0.0, 18.0, 0.0)),
    ("tilt y -18", (0.0, -18.0, 0.0)),
    ("tilt +12/+12", (12.0, 12.0, 0.0)),
    ("tilt -12/-12", (-12.0, -12.0, 0.0)),
)


class AutoMove(Node):
    def __init__(self):
        super().__init__("auto_move")
        self.robot_sn = self.declare_parameter("robot_sn", "").value
        self.base_frame = self.declare_parameter("base_frame", "").value
        self.flange_frame = self.declare_parameter("flange_frame", "").value
        self.group = self.declare_parameter("planning_group", "").value
        self.ik_service = self.declare_parameter("ik_service", "/compute_ik").value
        self.controller = self.declare_parameter("controller", "rizon_arm_controller").value
        self.capture_service = self.declare_parameter(
            "capture_service", "/handeye_calibrator/capture"
        ).value
        self.solve_service = self.declare_parameter(
            "solve_service", "/handeye_calibrator/solve"
        ).value
        self.joint_names = list(self.declare_parameter("joint_names", [""]).value)
        self.move_duration = float(self.declare_parameter("move_duration", 3.0).value)
        self.settle = float(self.declare_parameter("settle", 1.0).value)
        self.max_translation = float(self.declare_parameter("max_translation", 0.15).value)
        self.probe_translation = float(
            self.declare_parameter("probe_translation", 0.02).value
        )
        self.correction_threshold_px = float(
            self.declare_parameter("correction_threshold_px", 30.0).value
        )
        self.max_corrections = int(self.declare_parameter("max_corrections", 3).value)
        self.target_type = self.declare_parameter("target_type", "chessboard").value
        self.board_cols = int(self.declare_parameter("chessboard_cols", 8).value)
        self.board_rows = int(self.declare_parameter("chessboard_rows", 11).value)
        self.square_size = float(self.declare_parameter("square_size", 0.006).value)
        self.tag_family = self.declare_parameter("tag_family", "25h9").value
        self.tag_size = float(self.declare_parameter("tag_size", 0.026).value)
        self.image_topic = self.declare_parameter(
            "image_topic", "/camera/camera/color/image_raw"
        ).value
        self.camera_info_topic = self.declare_parameter(
            "camera_info_topic", "/camera/camera/color/camera_info"
        ).value
        self.dry_run = bool(self.declare_parameter("dry_run", False).value)
        self.max_poses = int(self.declare_parameter("max_poses", 0).value)
        #: True  -> one pose per ~/run call (operator triggers every step)
        #: False -> the whole plan runs on a single ~/run
        self.step_mode = bool(self.declare_parameter("step_mode", True).value)

        if not self.base_frame:
            self.base_frame = f"{self.robot_sn}_base_link" if self.robot_sn else "base_link"
        if not self.flange_frame:
            self.flange_frame = f"{self.robot_sn}_flange" if self.robot_sn else "flange"
        if not self.group:
            self.group = f"{self.robot_sn}_arm" if self.robot_sn else "arm"
        if not self.joint_names or self.joint_names == [""]:
            self.joint_names = [f"{self.robot_sn}_joint{i}" for i in range(1, 8)]

        self.detector = make_detector(
            self.target_type,
            tag_family=self.tag_family,
            tag_size=self.tag_size,
            board_cols=self.board_cols,
            board_rows=self.board_rows,
            square_size=self.square_size,
            board_auto=False,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Image, self.image_topic, self._on_image, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_info, qos_profile_sensor_data
        )
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)

        self._ik_client = self.create_client(GetPositionIK, self.ik_service)
        self._traj_client = ActionClient(
            self, FollowJointTrajectory, f"/{self.controller}/follow_joint_trajectory"
        )
        self._capture_client = self.create_client(Trigger, self.capture_service)
        self._solve_client = self.create_client(Trigger, self.solve_service)
        self.create_service(Trigger, "~/run", self._srv_run)
        self.create_service(Trigger, "~/abort", self._srv_abort)
        self.create_service(Trigger, "~/status", self._srv_status)

        self._image = None
        self._info = None
        self._joints: dict[str, float] = {}
        self._gen: Optional[Generator] = None
        self._pending = None
        self._goal_handle = None
        self._pose: Optional[np.ndarray] = None
        self._seed: Optional[np.ndarray] = None
        self._resume = False
        self._pose_index = 0
        self._last_range = None
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            "auto_move ready (nothing moves until ~/run). "
            f"group={self.group} flange={self.flange_frame} controller={self.controller}"
        )

    # ------------------------------------------------------------------ #
    # subscriptions
    # ------------------------------------------------------------------ #
    def _on_image(self, message):
        self._image = message

    def _on_info(self, message):
        self._info = message

    def _on_joints(self, message):
        for name, position in zip(message.name, message.position):
            self._joints[name] = position

    # ------------------------------------------------------------------ #
    # services
    # ------------------------------------------------------------------ #
    def _preflight(self):
        if self._image is None or self._info is None:
            return False, f"no camera frame on {self.image_topic}"
        if self._lookup_pose() is None:
            return False, f"tf {self.base_frame} <- {self.flange_frame} unavailable"
        try:
            centre = self._measure()
        except RuntimeError as exc:
            return False, (
                f"the target must be fully visible at the start pose: {exc}. "
                "Move the arm so the whole board is in view (0.30-0.35 m), then run again"
            )
        distance = self._last_range
        suffix = f", {distance:.2f} m" if distance else ""
        return True, f"ready, target at pixel ({centre[0]:.0f}, {centre[1]:.0f}){suffix}"

    def _srv_status(self, request, response):
        ok, message = self._preflight()
        waiting = self._gen is not None and not self._resume
        response.success = ok
        response.message = (
            f"{message} | pose {self._pose_index}/{len(self._plan())} "
            f"| {'waiting for ~/run' if waiting else 'idle/running'} "
            f"| step_mode={self.step_mode}"
        )
        return response

    def _srv_run(self, request, response):
        if self._gen is not None:
            if self.step_mode:
                self._resume = True
                response.success = True
                response.message = (
                    f"continuing: pose {self._pose_index + 1}/{len(self._plan())}"
                )
            else:
                response.success = False
                response.message = "already running"
            return response
        ok, message = self._preflight()
        if not ok:
            response.success = False
            response.message = message
            self.get_logger().error(message)
            return response
        if self.dry_run:
            for index, (label, rot) in enumerate(self._plan(), start=1):
                self.get_logger().info(f"[dry run] {index:2d}/{len(PLAN)}: {label} {rot}")
            response.success = True
            response.message = f"dry run: {len(PLAN)} poses"
            return response
        self._pose_index = 0
        self._resume = True
        self._gen = self._sequence()
        self._pending = None
        response.success = True
        response.message = (
            "started (step_mode=true): this call runs the first pose, "
            "call ~/run again for each following pose"
            if self.step_mode
            else "started: the whole plan will run now"
        )
        return response

    def _plan(self):
        poses = PLAN if self.max_poses <= 0 else PLAN[: self.max_poses]
        return poses

    def _srv_abort(self, request, response):
        self._cancel_goal()
        self._gen = None
        self._pending = None
        self._resume = False
        response.success = True
        response.message = "aborted"
        return response

    def _cancel_goal(self):
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._goal_handle = None

    # ------------------------------------------------------------------ #
    # generator driver
    # ------------------------------------------------------------------ #
    def _tick(self):
        if self._gen is None:
            return
        try:
            if self._pending is not None:
                if not self._pending():
                    return
                self._pending = None
            self._pending = next(self._gen)
        except StopIteration:
            self.get_logger().info("sequence finished")
            self._gen = None
            self._pending = None
        except Exception as exc:
            self.get_logger().error(f"sequence aborted: {exc}")
            self._cancel_goal()
            self._gen = None
            self._pending = None

    # ------------------------------------------------------------------ #
    # small generators used by the sequence
    # ------------------------------------------------------------------ #
    def _wait_for_go(self):
        """Pause the sequence until the next ~/run (step mode)."""
        self._resume = False
        while not self._resume:
            yield lambda: self._resume

    def _sleep(self, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline:
            yield lambda: time.time() >= deadline

    def _wait_future(self, future, timeout, what):
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                raise RuntimeError(f"timeout waiting for {what}")
            yield lambda: future.done()
        return future.result()

    # ------------------------------------------------------------------ #
    def _lookup_pose(self):
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

    def _measure_or_wait(self, label):
        """Measure, or pause and wait for the operator to bring the board back."""
        while True:
            try:
                return self._measure()
            except RuntimeError as exc:
                text = str(exc)
                if "not visible" in text or "border" in text:
                    self.get_logger().warn(
                        f"{label}: {text}. Move the arm so the board is fully in "
                        "view, then call ~/run to continue"
                    )
                    yield from self._wait_for_go()
                    continue
                raise

    def _measure(self):
        """Board centre in pixels, or raise with a clear reason."""
        image, info = self._image, self._info
        if image is None or info is None:
            raise RuntimeError("no camera frame")
        age = (self.get_clock().now() - Time.from_msg(image.header.stamp)).nanoseconds / 1e9
        if age > 0.5:
            raise RuntimeError(f"camera frame is {age:.2f} s old")
        bgr = to_bgr(image)
        if bgr is None:
            raise RuntimeError(f"unsupported encoding {image.encoding}")
        k = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        detection = select_detection(
            self.detector.detect(bgr, k, np.asarray(info.d) if len(info.d) else None),
            None, 200.0, 3.0,
        )
        if detection is None:
            raise RuntimeError("target not visible")
        corners = detection.corners.reshape(-1, 2)
        width, height = image.width, image.height
        margin = 25.0
        if (corners[:, 0].min() < margin or corners[:, 0].max() > width - margin
                or corners[:, 1].min() < margin or corners[:, 1].max() > height - margin):
            raise RuntimeError("target is at the image border")
        self._last_range = float(np.linalg.norm(geom.translation_of(detection.transform)))
        return corners.mean(axis=0)

    def _ik(self, pose):
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group
        request.ik_request.ik_link_name = self.flange_frame
        request.ik_request.pose_stamped.header.frame_id = self.base_frame
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        message = Pose()
        message.position.x, message.position.y, message.position.z = (
            float(v) for v in geom.translation_of(pose)
        )
        quat = geom.matrix_to_quat(geom.rotation_of(pose))
        message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w = (
            float(v) for v in quat
        )
        request.ik_request.pose_stamped.pose = message
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = Duration(seconds=0.3).to_msg()

        if not self._ik_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"{self.ik_service} not available (is move_group running?)")
        future = self._ik_client.call_async(request)
        yield from self._wait_future(future, 5.0, "compute_ik")
        response = future.result()
        if response.error_code.val != 1:
            return None
        positions = dict(zip(response.solution.joint_state.name,
                             response.solution.joint_state.position))
        if any(name not in positions for name in self.joint_names):
            return None
        return [float(positions[name]) for name in self.joint_names]

    def _workspace_ok(self, pose):
        delta = np.linalg.norm(
            geom.translation_of(pose) - geom.translation_of(self._seed)
        )
        return delta <= self.max_translation

    def _goto(self, pose, label):
        self.get_logger().info(f"moving to {label}")
        if not self._workspace_ok(pose):
            distance = float(np.linalg.norm(
                geom.translation_of(pose) - geom.translation_of(self._seed)
            ))
            raise RuntimeError(
                f"{label}: target is {distance * 100:.0f} cm from the seed pose, "
                f"outside the {self.max_translation * 100:.0f} cm safety box"
            )
        positions = yield from self._ik(pose)
        if positions is None:
            raise RuntimeError(f"{label}: IK failed, skipping")

        current = [self._joints.get(name) for name in self.joint_names]
        if any(value is None for value in current):
            raise RuntimeError("joint states missing")
        travel = max(abs(a - b) for a, b in zip(current, positions))
        duration = max(2.0, min(8.0, 1.5 * self.move_duration
                                 * max(1.0, math.degrees(travel) / 20.0)))

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joint_names)
        start = JointTrajectoryPoint()
        start.positions = [float(v) for v in current]
        start.time_from_start = Duration(seconds=0.1).to_msg()
        end = JointTrajectoryPoint()
        end.positions = [float(v) for v in positions]
        end.time_from_start = Duration(seconds=duration).to_msg()
        goal.trajectory.points = [start, end]

        if not self._traj_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("joint trajectory controller not available")
        send_future = self._traj_client.send_goal_async(goal)
        yield from self._wait_future(send_future, 10.0, "trajectory goal")
        handle = send_future.result()
        if not handle.accepted:
            raise RuntimeError(f"{label}: trajectory rejected")
        self._goal_handle = handle
        result_future = handle.get_result_async()
        result = yield from self._wait_future(result_future, duration + 20.0, "trajectory result")
        self._goal_handle = None
        if result.status != 4:  # SUCCEEDED
            raise RuntimeError(f"{label}: trajectory ended with status {result.status}")
        self._pose = pose
        yield from self._sleep(self.settle)

    def _capture(self, label):
        for _ in range(5):
            if not self._capture_client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError("capture service not available (start the calibrate node)")
            future = self._capture_client.call_async(Trigger.Request())
            response = yield from self._wait_future(future, 15.0, "capture")
            if response.success:
                self.get_logger().info(f"captured {label}: {response.message}")
                return True
            message = response.message
            if "too close" in message:
                self.get_logger().warn(
                    f"{label}: duplicate of an existing pose, skipped ({message})"
                )
                return False
            if "no usable tag" in message:
                self.get_logger().warn(
                    f"{label}: target not usable ({message}). Bring it back into view, "
                    "then call ~/run to continue"
                )
                yield from self._wait_for_go()
                continue
            raise RuntimeError(f"capture failed at {label}: {message}")
        raise RuntimeError(f"capture kept failing at {label}")

    def _recentre(self, target_px, jacobian, label):
        for attempt in range(self.max_corrections):
            centre = yield from self._measure_or_wait(f"{label} recentre")
            error = centre - target_px
            distance = float(np.linalg.norm(error))
            if distance < self.correction_threshold_px:
                return
            self.get_logger().info(
                f"{label}: board {distance:.0f} px off, correcting ({attempt + 1})"
            )
            try:
                delta = -np.linalg.inv(jacobian) @ error
            except np.linalg.LinAlgError:
                raise RuntimeError("translation Jacobian is singular")
            pose = self._pose.copy()
            pose[:3, 3] += pose[:3, :3] @ np.array([delta[0], delta[1], 0.0])
            yield from self._goto(pose, f"{label} recentre")

    # ------------------------------------------------------------------ #
    def _sequence(self):
        seed = self._lookup_pose()
        if seed is None:
            raise RuntimeError(f"tf {self.base_frame} <- {self.flange_frame} unavailable")
        self._seed = seed
        self._pose = seed
        ok, message = self._preflight()
        if not ok:
            raise RuntimeError(message)
        self.get_logger().warn("sequence started - keep your hand on the e-stop")

        yield from self._capture("seed")
        target_px = yield from self._measure_or_wait("seed")

        # measure how a flange translation moves the board in the image
        jacobian = np.zeros((2, 2))
        for column, axis in enumerate(([1.0, 0.0, 0.0], [0.0, 1.0, 0.0])):
            probe = seed.copy()
            probe[:3, 3] += seed[:3, :3] @ (np.asarray(axis) * self.probe_translation)
            yield from self._goto(probe, "probe")
            moved = yield from self._measure_or_wait("probe")
            jacobian[:, column] = (moved - target_px) / self.probe_translation
            self.get_logger().info(
                f"probe {'x' if column == 0 else 'y'}: "
                f"{np.round(jacobian[:, column], 0).tolist()} px/m"
            )
            back = seed.copy()
            yield from self._goto(back, "probe return")

        plan = self._plan()
        for index, (label, (rx, ry, rz)) in enumerate(plan, start=1):
            self._pose_index = index
            steps = 1 if max(abs(rx), abs(ry), abs(rz)) <= 15.0 else 2
            for step in range(1, steps + 1):
                fraction = step / steps
                rotation = (
                    geom.rot_x(math.radians(rx * fraction))
                    @ geom.rot_y(math.radians(ry * fraction))
                    @ geom.rot_z(math.radians(rz * fraction))
                )
                target = geom.make_transform(rotation @ seed[:3, :3], seed[:3, 3])
                yield from self._goto(target, f"{index}/{len(plan)} {label} ({step}/{steps})")
                yield from self._recentre(target_px, jacobian, f"{index}/{len(plan)}")
            yield from self._capture(f"{index}/{len(plan)} {label}")
            if self.step_mode and index < len(plan):
                self.get_logger().info(
                    f"pose {index}/{len(plan)} done - call ~/run for the next pose"
                )
                yield from self._wait_for_go()
        # back to the seed before solving
        yield from self._goto(seed, "return to seed")
        yield from self._solve()

    def _solve(self):
        if not self._solve_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("solve service not available")
        future = self._solve_client.call_async(Trigger.Request())
        response = yield from self._wait_future(future, 180.0, "solve")
        self.get_logger().info("solve:\n" + response.message)


def main(args=None):
    rclpy.init(args=args)
    node = AutoMove()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
