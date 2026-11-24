import argparse
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK
from pymoveit2 import MoveIt2
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
FRAME = "base_link"

# All locations are expressed in meters in the robot's base_link frame.
# Geometry is illustrative; verify it against the actual setup.

BOARD_SHIFT_X = 0.22
HOOP_RAISE = 0.08

# Mounting plane relative to base_link. Adjust ROBOT_MOUNT_Z if your robot's
# base mesh uses a different mounting-plane offset from the standard origin.
ROBOT_MOUNT_Z = 0.0
TABLE_SURFACE_CLEARANCE = 0.001  # 1 mm avoids touching the fixed base collider.
TABLE_TOP = ROBOT_MOUNT_Z - TABLE_SURFACE_CLEARANCE
TABLE_SIZE = (0.90 + BOARD_SHIFT_X, 0.65, 0.12)
TABLE_CENTER = (
    0.12 + BOARD_SHIFT_X / 2,
    0.0,
    TABLE_TOP - TABLE_SIZE[2] / 2,
)

BALL_DIAMETER = 0.127
BALL_CENTER_XY = (0.24, 0.0)

# Upper backboard is the reference. Its position and dimensions are unchanged.
BACKBOARD_SIZE = (0.015, 0.4572, 0.2667)
BACKBOARD_BOTTOM_Z = 0.32
BACKBOARD_CENTER = (
    0.535 + BOARD_SHIFT_X,
    0.0,
    BACKBOARD_BOTTOM_Z + BACKBOARD_SIZE[2] / 2,
)
BACKBOARD_FRONT_X = BACKBOARD_CENTER[0] - BACKBOARD_SIZE[0] / 2

# Extend the lower whiteboard from the tabletop to the upper backboard.
# Preserve its original width and thickness; make both front faces flush.
WHITEBOARD_THICKNESS = 0.02
WHITEBOARD_WIDTH = 0.55
WHITEBOARD_BOTTOM_Z = TABLE_TOP
WHITEBOARD_TOP = BACKBOARD_CENTER[2] - BACKBOARD_SIZE[2] / 2
WHITEBOARD_HEIGHT = WHITEBOARD_TOP - WHITEBOARD_BOTTOM_Z
if WHITEBOARD_HEIGHT <= 0:
    raise ValueError("The upper backboard must start above the tabletop")
WHITEBOARD_SIZE = (WHITEBOARD_THICKNESS, WHITEBOARD_WIDTH, WHITEBOARD_HEIGHT)
WHITEBOARD_CENTER = (
    BACKBOARD_FRONT_X + WHITEBOARD_THICKNESS / 2,
    BACKBOARD_CENTER[1],
    (WHITEBOARD_BOTTOM_Z + WHITEBOARD_TOP) / 2,
)

# Preserve the existing hoop location relative to the upper backboard assembly.
HOOP_CENTER = (0.37 + BOARD_SHIFT_X, 0.0, 0.39 + HOOP_RAISE)
HOOP_INNER_DIAMETER = 0.3
RIM_THICKNESS = 0.010
RIM_SEGMENTS = 12
APPROACH_CLEARANCE = 0.12


def yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class BasketballScene(Node):
    def __init__(self):
        super().__init__("simulation")
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ARM_JOINTS,
            base_link_name=FRAME,
            end_effector_name="end_effector_link",
            group_name="arm",
        )
        self.fk_client = self.create_client(GetPositionFK, "compute_fk")

    def wait_ready(self):
        if not self.fk_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("compute_fk not available. Start Kinova MoveIt simulation first.")
        for _ in range(30):
            result = self.moveit2.compute_fk()
            if result is not None:
                return result.pose
            time.sleep(1)
        raise RuntimeError("MoveIt did not provide a current end-effector pose.")

    def box(self, name, size, center, quat=(0., 0., 0., 1.)):
        self.moveit2.add_collision_box(
            id=name,
            size=tuple(size),
            position=tuple(center),
            quat_xyzw=tuple(quat),
            frame_id=FRAME,
        )
        self.get_logger().info(f"Added {name}: center={center}, size={size}")

    def add_environment(self):
        self.box("basketball_table", TABLE_SIZE, TABLE_CENTER)

        table_top = TABLE_CENTER[2] + TABLE_SIZE[2] / 2
        ball_center = (BALL_CENTER_XY[0], BALL_CENTER_XY[1], table_top + BALL_DIAMETER / 2)
        self.moveit2.add_collision_sphere(
            id="basketball",
            radius=BALL_DIAMETER / 2,
            position=ball_center,
            frame_id=FRAME,
        )
        self.get_logger().info(f"Added sphere basketball: center={ball_center}")

        self.box("whiteboard", WHITEBOARD_SIZE, WHITEBOARD_CENTER)
        self.box("backboard", BACKBOARD_SIZE, BACKBOARD_CENTER)

        hx, hy, hz = HOOP_CENTER
        centerline_radius = HOOP_INNER_DIAMETER / 2 + RIM_THICKNESS / 2
        segment_length = 2 * centerline_radius * math.sin(math.pi / RIM_SEGMENTS) * 1.02
        for i in range(RIM_SEGMENTS):
            angle = 2 * math.pi * i / RIM_SEGMENTS
            center = (
                hx + centerline_radius * math.cos(angle),
                hy + centerline_radius * math.sin(angle),
                hz,
            )
            self.box(
                f"hoop_rim_{i:02d}",
                (segment_length, RIM_THICKNESS, RIM_THICKNESS),
                center,
                yaw_quaternion(angle + math.pi / 2),
            )

        self.get_logger().info(
            "Planning scene added: lower whiteboard meets the tabletop and upper "
            "backboard, with flush front faces. Verify dimensions in RViz."
        )
        return ball_center

    def target_pose(self, xyz, orientation):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation = orientation
        return pose

    def plan_test(self, ball_center, orientation, execute=False):
        ball_approach = (
            ball_center[0], ball_center[1],
            ball_center[2] + BALL_DIAMETER / 2 + APPROACH_CLEARANCE,
        )
        hoop_approach = (
            HOOP_CENTER[0], HOOP_CENTER[1],
            HOOP_CENTER[2] + BALL_DIAMETER / 2 + APPROACH_CLEARANCE,
        )
        self.get_logger().warn(
            "Targets use the current gripper orientation, not a tested dunk orientation. "
            "This is only a preliminary reachability/planning check."
        )
        for name, target in (("ball approach", ball_approach), ("hoop approach", hoop_approach)):
            pose = self.target_pose(target, orientation)
            plan = self.moveit2.plan(pose=pose, cartesian=False)
            if plan is None:
                self.get_logger().error(f"Cannot plan to {name}: {target}")
                return False
            self.get_logger().info(f"Plan available for {name}: {target}")
            if execute:
                self.moveit2.execute(plan)
                if not self.moveit2.wait_until_executed():
                    self.get_logger().error(f"Execution failed at {name}")
                    return False
        if not execute:
            self.get_logger().info(
                "Planning checks passed from the current state individually. "
                "Use --execute in simulation to verify the actual ball-to-hoop sequence."
            )
        else:
            self.get_logger().info("Executed both moves in simulation; inspect RViz for clearance.")
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--test", action="store_true", help="Plan both goals without moving")
    parser.add_argument("--execute", action="store_true", help="SIMULATION ONLY: move ball approach then hoop approach")
    args = parser.parse_args()
    rclpy.init()
    node = BasketballScene()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        current_pose = node.wait_ready()
        ball = node.add_environment()
        time.sleep(2)
        if args.test or args.execute:
            node.plan_test(ball, current_pose.orientation, execute=args.execute)
        else:
            node.get_logger().info("Scene ready. Keep this process running while inspecting RViz (Ctrl+C to quit).")
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

