"""Find the ball with the overhead camera, grab it from above, and lift it.

Needs the sim, MoveIt, spawn_overhead_camera, and ball_detector running:
    ros2 run basketball_moveit pick_ball
    ros2 run basketball_moveit pick_ball --speed 0.2 --lift 0.2

To test with a random ball position, move the ball first:
    ros2 run basketball_moveit check_ball --trials 1
"""

import argparse
import collections
import os
import signal
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped, Pose
from moveit_msgs.srv import GetPositionFK
from pymoveit2 import GripperCommand, MoveIt2
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
GRIPPER_JOINT = "right_finger_bottom_joint"
GRIPPER_OPEN = 0.0      # about 111 mm between the fingertips
GRIPPER_CLOSED = 0.8    # firm close with nothing in the way

# Distance from end_effector_link down to the fingertips (from the midterm code).
GRIPPER_LENGTH = 0.157
# Gripper pointing straight down, quaternion (x, y, z, w).
TOP_DOWN = (0.0, 1.0, 0.0, 0.0)
# Elbow-up pose above the table. Straight-line moves fail from the folded start pose.
WORK = [0.40, 0.02, 2.27, -1.57, -0.84, 1.97]


class Picker(Node):
    def __init__(self):
        super().__init__("pick_ball")
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ARM_JOINTS,
            base_link_name="base_link",
            end_effector_name="end_effector_link",
            group_name="arm",
        )
        self.gripper = GripperCommand(
            node=self,
            gripper_joint_names=[GRIPPER_JOINT],
            open_gripper_joint_positions=[GRIPPER_OPEN],
            closed_gripper_joint_positions=[GRIPPER_CLOSED],
            ignore_new_calls_while_executing=False,
            gripper_command_action_name="/gen3_lite_2f_gripper_controller/gripper_cmd",
        )
        self.history = collections.deque(maxlen=50)  # (time, x, y, z) of /ball_position
        self.fingers = None
        self.joint_stamp = 0.0
        self.create_subscription(PointStamped, "/ball_position", self.on_ball, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.fk_client = self.create_client(GetPositionFK, "compute_fk")

    # ---------------------------------------------------------------- inputs

    def on_ball(self, msg):
        self.history.append((time.monotonic(), msg.point.x, msg.point.y, msg.point.z))

    def on_joint_state(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name == GRIPPER_JOINT:
                self.fingers = position
        self.joint_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def wait_until_ready(self, timeout=30.0):
        if not self.fk_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("MoveIt did not start. Is kinova-sim-moveit running?")
            return False
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.moveit2.compute_fk() is not None:
                return True
            time.sleep(1.0)
        self.get_logger().error("MoveIt could not report the arm's pose.")
        return False

    def stable_ball(self, timeout=15.0, count=5, spread=0.01):
        """Average of several steady /ball_position readings, or None."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            now = time.monotonic()
            recent = [p for p in list(self.history) if p[0] > now - 1.5]
            if len(recent) >= count:
                xs = [p[1] for p in recent]
                ys = [p[2] for p in recent]
                if max(xs) - min(xs) < spread and max(ys) - min(ys) < spread:
                    n = len(recent)
                    return sum(xs) / n, sum(ys) / n, sum(p[3] for p in recent) / n
            time.sleep(0.1)
        return None

    # ---------------------------------------------------------------- motion

    def move_to_joints(self, joints, attempts=3):
        for attempt in range(1, attempts + 1):
            trajectory = self.moveit2.plan(joint_positions=joints)
            if trajectory is None:
                self.get_logger().warn(f"Joint plan attempt {attempt} of {attempts} failed.")
                continue
            self.moveit2.execute(trajectory)
            if self.moveit2.wait_until_executed():
                return True
        return False

    def move_to_pose(self, x, y, z, cartesian=False, speed=1.0, attempts=3):
        """Move end_effector_link to (x, y, z) in base_link, gripper pointing down."""
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = TOP_DOWN
        self.moveit2.max_velocity = speed
        self.moveit2.max_acceleration = speed
        try:
            for attempt in range(1, attempts + 1):
                trajectory = self.moveit2.plan(
                    pose=pose,
                    cartesian=cartesian,
                    max_step=0.005,
                    cartesian_fraction_threshold=0.9 if cartesian else 0.0,
                )
                if trajectory is None:
                    self.get_logger().warn(
                        f"Plan attempt {attempt} of {attempts} to "
                        f"({x:.3f}, {y:.3f}, {z:.3f}) failed.")
                    continue
                self.moveit2.execute(trajectory)
                if self.moveit2.wait_until_executed():
                    return True
        finally:
            self.moveit2.max_velocity = 0.0
            self.moveit2.max_acceleration = 0.0
        kind = "straight-line" if cartesian else "pose-goal"
        self.get_logger().error(f"Could not plan a {kind} move to ({x:.3f}, {y:.3f}, {z:.3f}).")
        return False

    def move_fingers(self, position, timeout=4.0):
        """Command the fingers, wait until they stop, return the measured position."""
        self.gripper.move_to_position(position)
        start = self.joint_stamp
        last = self.fingers
        still_since = start
        wall_limit = time.time() + 10.0 * timeout
        while time.time() < wall_limit:
            time.sleep(0.05)
            now, stamp = self.fingers, self.joint_stamp
            if stamp - start > timeout:
                break
            if last is None or now is None or abs(now - last) > 0.002:
                last, still_since = now, stamp
            elif stamp - still_since > 0.5 and stamp - start > 0.5:
                break
        return self.fingers


def pick(node, args):
    log = node.get_logger()
    log.info("Opening the gripper and moving to the work pose.")
    node.move_fingers(GRIPPER_OPEN)
    if not node.move_to_joints(WORK):
        log.error("Could not reach the work pose.")
        return False

    log.info("Waiting for a steady ball reading from the detector.")
    ball = node.stable_ball()
    if ball is None:
        log.error("No steady /ball_position. Is ball_detector running and seeing the ball?")
        return False
    x, y, z = ball
    log.info(f"Ball at x={x:.3f} y={y:.3f} z={z:.3f}")

    # Fingertips end up level with the ball's center (plus the optional offset).
    grasp_z = z + GRIPPER_LENGTH + args.grasp_offset
    above_z = grasp_z + args.approach

    # The planning-scene ball would block the open fingers, so take it out of the scene.
    node.moveit2.remove_collision_object("basketball")
    time.sleep(0.5)

    log.info("Moving above the ball.")
    if not node.move_to_pose(x, y, above_z):
        log.error("Cannot reach a spot above the ball. It may be out of the arm's reach.")
        return False
    log.info("Going down.")
    if not node.move_to_pose(x, y, grasp_z, cartesian=True, speed=args.speed):
        log.error("Could not go straight down to the ball.")
        return False

    finger = node.move_fingers(GRIPPER_CLOSED)
    log.info(f"Fingers stopped at {finger}. About 0.8 means nothing is in the gripper.")
    if finger is None or finger < args.closed_min:
        log.error(f"The fingers did not close (reading {finger}). Check the gripper controller.")
        node.move_to_pose(x, y, above_z, cartesian=True, speed=args.speed)
        return False
    if finger > args.empty_threshold:
        log.error("Missed: the fingers closed on nothing.")
        node.move_fingers(GRIPPER_OPEN)
        node.move_to_pose(x, y, above_z, cartesian=True, speed=args.speed)
        return False

    log.info("Got it. Lifting.")
    if not node.move_to_pose(x, y, grasp_z + args.lift, cartesian=True, speed=args.speed):
        log.error("Could not lift straight up.")
        return False
    log.info("Lifted the ball.")
    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--approach", type=float, default=0.12, help="height above grasp to approach from, m")
    parser.add_argument("--lift", type=float, default=0.15, help="how far to lift, m")
    parser.add_argument("--speed", type=float, default=0.3, help="speed scale for straight moves")
    parser.add_argument("--grasp-offset", type=float, default=0.0,
                        help="raise (+) or lower (-) the grasp relative to the ball center, m")
    parser.add_argument("--empty-threshold", type=float, default=0.75,
                        help="finger reading above this means nothing was grasped")
    parser.add_argument("--closed-min", type=float, default=0.05,
                        help="finger reading below this means the fingers did not close")
    args, _ = parser.parse_known_args()

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def stop(_signum, _frame):
        print("Interrupted. Stopping; the arm holds its position.", flush=True)
        os._exit(130)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    node = Picker()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    success = False
    try:
        if node.wait_until_ready():
            success = pick(node, args)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()