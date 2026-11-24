"""Dunk: carry the ball that is already in the gripper over the hoop and let go.

Start with the ball held in the gripper (for example right after pick_ball).
The script moves the ball to a spot behind and above the rim, slides it
straight over the hoop's center, opens the gripper, and backs away.

The hoop may be at the edge of the arm's reach, so the script tries several
gripper angles, from pointing forward (horizontal) to pointing straight down,
and uses the first one that MoveIt can plan.

Needs the sim (or the real arm) and MoveIt running. Run basketball_scene once
first so the rim and backboard are in the planning scene.

    ros2 run basketball_moveit dunk_ball --dry-run      # plan only, nothing moves
    ros2 run basketball_moveit dunk_ball                # do the dunk
    ros2 run basketball_moveit dunk_ball --pitch 90     # force one gripper angle
    ros2 run basketball_moveit dunk_ball --hoop 0.55 0.0 0.45

Pitch is the gripper's angle from straight down: 0 = pointing down,
90 = pointing forward (horizontal).

In simulation the Gazebo ball is static, so the arm cannot really carry it.
Use --skip-grip-check there; the run then shows only the arm's motion. With
--show-ball, a sphere rides with the gripper in the planning scene and drops
through the hoop in RViz when the gripper opens.
"""

import argparse
import math
import os
import signal
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Pose
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from basketball_moveit.scripts import vision
from basketball_moveit.scripts.basketball_scene import (
    HOOP_CENTER,
    HOOP_INNER_DIAMETER,
    RIM_THICKNESS,
)
from basketball_moveit.scripts.pick_ball import (
    GRIPPER_LENGTH,
    GRIPPER_OPEN,
    WORK,
    Picker,
)

# Gripper angles to try, from straight down. Forward first: it needs the least reach.
DEFAULT_PITCHES = [90, 75, 60, 45, 30, 0]
SHOW_BALL_ID = "held_ball"
# Gen3 Lite 2F finger links. The held ball may touch these without counting as a collision.
FINGER_LINKS = [
    "right_finger_prox_link", "right_finger_dist_link",
    "left_finger_prox_link", "left_finger_dist_link",
    "end_effector_link",
]


# ------------------------------------------------------------------ geometry

def quat_mul(a, b):
    """Hamilton product of two (x, y, z, w) quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def axis_quat(axis, angle):
    s = math.sin(angle / 2)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2))


def rotate(q, v):
    """Rotate vector v by quaternion q."""
    x, y, z, w = quat_mul(quat_mul(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return (x, y, z)


def gripper_quat(pitch_deg, yaw, roll_deg=0.0):
    """Orientation of end_effector_link.

    pitch_deg: 0 = gripper pointing straight down (pick_ball's TOP_DOWN),
               90 = pointing horizontally along `yaw`.
    yaw: direction (radians, in base_link) the gripper tips toward.
    roll_deg: turn about the gripper's own axis (changes which way the fingers open).
    """
    top_down = (0.0, 1.0, 0.0, 0.0)                    # 180 deg about y: tool z points down
    q = quat_mul(top_down, axis_quat((0, 0, 1), math.radians(roll_deg)))
    q = quat_mul(axis_quat((0, 1, 0), -math.radians(pitch_deg)), q)   # tip toward +x
    return quat_mul(axis_quat((0, 0, 1), yaw), q)        # then turn toward the hoop


def pointing(q):
    """Unit vector the gripper points along (its z axis)."""
    return rotate(q, (0.0, 0.0, 1.0))


def ee_for_ball(ball, q, length=GRIPPER_LENGTH):
    """end_effector_link position that puts the ball center at `ball`.

    Same grasp as pick_ball: the ball center sits GRIPPER_LENGTH along the gripper axis.
    """
    d = pointing(q)
    return (ball[0] - length * d[0], ball[1] - length * d[1], ball[2] - length * d[2])


def dunk_points(hoop, ball_radius, clearance, back, rise):
    """Ball-center positions: (start, over the hoop, release)."""
    hx, hy, hz = hoop
    yaw = math.atan2(hy, hx)                             # robot -> hoop direction
    over = (hx, hy, hz + RIM_THICKNESS / 2 + ball_radius + clearance)
    start = (hx - back * math.cos(yaw), hy - back * math.sin(yaw), over[2] + rise)
    return yaw, start, over


# ------------------------------------------------------------------ node

class Dunker(Picker):
    def plan_pose(self, xyz, q, cartesian=False, speed=1.0):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = q
        self.moveit2.max_velocity = speed
        self.moveit2.max_acceleration = speed
        try:
            return self.moveit2.plan(
                pose=pose,
                cartesian=cartesian,
                max_step=0.005,
                cartesian_fraction_threshold=0.9 if cartesian else 0.0,
            )
        finally:
            self.moveit2.max_velocity = 0.0
            self.moveit2.max_acceleration = 0.0

    def go(self, xyz, q, cartesian=False, speed=1.0, attempts=3):
        for attempt in range(1, attempts + 1):
            trajectory = self.plan_pose(xyz, q, cartesian=cartesian, speed=speed)
            if trajectory is None:
                self.get_logger().warn(f"Plan attempt {attempt} of {attempts} failed.")
                continue
            self.moveit2.execute(trajectory)
            if self.moveit2.wait_until_executed():
                return True
        return False

    def held_ball_position(self):
        """Ball-center position from the current gripper pose, or None."""
        fk = self.moveit2.compute_fk()
        if fk is None:
            return None
        p, o = fk.pose.position, fk.pose.orientation
        d = pointing((o.x, o.y, o.z, o.w))
        return (p.x + GRIPPER_LENGTH * d[0], p.y + GRIPPER_LENGTH * d[1],
                p.z + GRIPPER_LENGTH * d[2])


def choose_pitch(node, pitches, yaw, start, over, roll):
    """First gripper angle with a plan to both the start and the over-the-hoop pose."""
    log = node.get_logger()
    for pitch in pitches:
        q = gripper_quat(pitch, yaw, roll)
        ok = True
        for name, ball in (("start", start), ("over the hoop", over)):
            ee = ee_for_ball(ball, q)
            if node.plan_pose(ee, q) is None:
                log.info(f"pitch {pitch:3.0f} deg: no plan to {name} "
                         f"(gripper at {ee[0]:.3f}, {ee[1]:.3f}, {ee[2]:.3f})")
                ok = False
                break
        if ok:
            log.info(f"pitch {pitch:3.0f} deg: both poses reachable.")
            return pitch, q
    return None, None


def dunk(node, args):
    log = node.get_logger()
    ball_radius = args.ball_diameter / 2
    hoop = tuple(args.hoop)

    if args.ball_diameter >= HOOP_INNER_DIAMETER - RIM_THICKNESS:
        log.error("The ball is too big to fit through the hoop. Check --ball-diameter.")
        return False

    yaw, start, over = dunk_points(hoop, ball_radius, args.clearance, args.back, args.rise)
    log.info(f"Hoop center {hoop}; ball goes from "
             f"({start[0]:.3f}, {start[1]:.3f}, {start[2]:.3f}) to "
             f"({over[0]:.3f}, {over[1]:.3f}, {over[2]:.3f}), "
             f"its bottom {args.clearance * 100:.0f} cm above the rim.")

    # Is the ball in the gripper? About 0.8 means the fingers closed on nothing.
    time.sleep(0.5)
    finger = node.fingers
    if args.skip_grip_check:
        log.warn("Skipping the grip check.")
    elif finger is None:
        log.error("No gripper reading on /joint_states. Use --skip-grip-check to go anyway.")
        return False
    elif finger > args.empty_threshold or finger < args.closed_min:
        log.error(f"Gripper reading {finger:.2f} does not look like it is holding a ball "
                  f"(expected between {args.closed_min} and {args.empty_threshold}). "
                  "Pick the ball first, or use --skip-grip-check.")
        return False
    else:
        log.info(f"Gripper reading {finger:.2f}: holding the ball.")

    # The held ball would collide with itself in the scene; the table ball is not in the way.
    node.moveit2.remove_collision_object("basketball")
    if args.show_ball:
        held = node.held_ball_position()
        if held is not None:
            node.moveit2.add_collision_sphere(
                id=SHOW_BALL_ID, radius=ball_radius, position=held, frame_id="base_link")
            time.sleep(0.5)
            node.moveit2.attach_collision_object(
                id=SHOW_BALL_ID, link_name="end_effector_link", touch_links=FINGER_LINKS)
    time.sleep(0.5)

    pitches = [args.pitch] if args.pitch is not None else DEFAULT_PITCHES
    pitch, q = choose_pitch(node, pitches, yaw, start, over, args.roll)
    if pitch is None:
        log.error(
            f"No gripper angle reaches the hoop at x = {hoop[0]:.2f} m: it is out of the "
            "arm's reach. In basketball_scene.py, comment out the BOARD_SHIFT_X = 0.40 line "
            "and uncomment the line below it (BOARD_SHIFT_X = 0.65 - 0.37), restart "
            "basketball_scene, then run dunk_ball again. (Or test another spot with "
            "--hoop X Y Z.)")
        return False

    if args.dry_run:
        log.info(f"Dry run: the dunk is reachable with the gripper at {pitch:.0f} deg. "
                 "Nothing moved.")
        return True

    log.info("Moving to the start pose behind the rim.")
    if not node.go(ee_for_ball(start, q), q):
        log.error("Could not reach the start pose.")
        return False

    log.info("Sliding the ball over the hoop.")
    over_ee = ee_for_ball(over, q)
    if not node.go(over_ee, q, cartesian=True, speed=args.speed):
        log.warn("No straight-line path; trying a normal move instead.")
        if not node.go(over_ee, q, speed=args.speed):
            log.error("Could not get the ball over the hoop.")
            return False

    log.info("Releasing.")
    if args.show_ball:
        node.moveit2.detach_collision_object(SHOW_BALL_ID)
    node.move_fingers(GRIPPER_OPEN)
    if args.show_ball:
        drop_ball(node, over, hoop[2])
    time.sleep(args.wait)

    log.info("Backing away from the hoop.")
    if not node.go(ee_for_ball(start, q), q, cartesian=True, speed=args.speed):
        node.go(ee_for_ball(start, q), q)
    if args.home:
        node.move_to_joints(WORK)
    log.info("Dunk done.")
    return True


def drop_ball(node, top, rim_z, steps=25):
    """Let the planning-scene ball fall straight through the hoop, for RViz only."""
    bottom = rim_z - 0.25
    for i in range(1, steps + 1):
        z = top[2] - (top[2] - bottom) * (i / steps) ** 2
        node.moveit2.move_collision(
            id=SHOW_BALL_ID, position=(top[0], top[1], z),
            quat_xyzw=(0.0, 0.0, 0.0, 1.0), frame_id="base_link")
        time.sleep(0.02)


def main():
    parser = argparse.ArgumentParser(description="Dunk the held ball through the hoop.")
    parser.add_argument("--hoop", type=float, nargs=3, metavar=("X", "Y", "Z"),
                        default=HOOP_CENTER,
                        help="rim center in base_link, m (default: basketball_scene.HOOP_CENTER)")
    parser.add_argument("--ball-diameter", type=float, default=vision.BALL_DIAMETER, help="m")
    parser.add_argument("--clearance", type=float, default=0.03,
                        help="gap between the ball's bottom and the rim, m")
    parser.add_argument("--back", type=float, default=0.12,
                        help="start this far behind the hoop center (toward the robot), m")
    parser.add_argument("--rise", type=float, default=0.03,
                        help="start this much higher than the release height, m")
    parser.add_argument("--pitch", type=float, default=None,
                        help="use only this gripper angle from straight down, deg")
    parser.add_argument("--roll", type=float, default=0.0,
                        help="turn the gripper about its own axis, deg (try 90 if the fingers "
                             "open up/down and the ball sits on the lower finger)")
    parser.add_argument("--speed", type=float, default=0.2, help="speed scale near the hoop")
    parser.add_argument("--wait", type=float, default=1.0, help="seconds to wait after opening")
    parser.add_argument("--home", action="store_true", help="return to the work pose at the end")
    parser.add_argument("--dry-run", action="store_true", help="plan only, do not move")
    parser.add_argument("--show-ball", action="store_true",
                        help="show the held ball in RViz and drop it through the hoop")
    parser.add_argument("--skip-grip-check", action="store_true",
                        help="skip checking that the gripper holds a ball (use in sim)")
    parser.add_argument("--empty-threshold", type=float, default=0.75,
                        help="finger reading above this means nothing is held")
    parser.add_argument("--closed-min", type=float, default=0.05,
                        help="finger reading below this means the fingers are open")
    args, _ = parser.parse_known_args()

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def stop(_signum, _frame):
        print("Interrupted. Stopping; the arm holds its position.", flush=True)
        os._exit(130)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    node = Dunker()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    success = False
    try:
        if node.wait_until_ready():
            success = dunk(node, args)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()