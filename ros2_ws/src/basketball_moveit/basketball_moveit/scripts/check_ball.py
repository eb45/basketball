"""Move the Gazebo ball to random spots and check that the detector finds each one.

Needs the sim, spawn_overhead_camera, and ball_detector running:
    ros2 run basketball_moveit check_ball
    ros2 run basketball_moveit check_ball --trials 10 --seed 1 --tol 0.015

The ball is moved with Gazebo's set_pose service, so nothing has to be respawned.
The detector only publishes when it sees the ball, so a missing reading counts
as "not detected".
"""

import argparse
import math
import random
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

from basketball_moveit.scripts import vision
from basketball_moveit.scripts.spawn_overhead_camera import BASE_Z_WORLD


class Checker(Node):
    def __init__(self):
        super().__init__("check_ball")
        self.last = None  # (arrival time, x, y)
        self.create_subscription(PointStamped, "/ball_position", self.on_point, 10)

    def on_point(self, msg):
        self.last = (time.monotonic(), msg.point.x, msg.point.y)


def set_ball_pose(world, x, y, z):
    """Move the Gazebo ball. Returns True if Gazebo accepted the move."""
    request = (
        f'name: "gz_ball" position: {{x: {x}, y: {y}, z: {z}}} '
        "orientation: {x: 0, y: 0, z: 0, w: 1}"
    )
    cmd = [
        "gz", "service", "-s", f"/world/{world}/set_pose",
        "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000", "--req", request,
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    return "true" in result.stdout


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--seed", type=int, default=None, help="repeatable positions")
    parser.add_argument("--tol", type=float, default=0.02, help="allowed error, meters")
    parser.add_argument("--world", default="empty", help="Gazebo world name")
    parser.add_argument("--x-range", type=float, nargs=2, default=(0.15, 0.40))
    parser.add_argument("--y-range", type=float, nargs=2, default=(-0.20, 0.20))
    parser.add_argument("--settle", type=float, default=1.5,
                        help="seconds to wait after moving the ball")
    parser.add_argument("--timeout", type=float, default=5.0,
                        help="seconds to wait for a detection before giving up")
    args = parser.parse_args()

    rclpy.init()
    node = Checker()
    rng = random.Random(args.seed)
    errors = []
    misses = 0

    print(f"{'trial':>5} {'true x':>8} {'true y':>8} {'found x':>8} {'found y':>8} {'error':>8}  result")
    for trial in range(1, args.trials + 1):
        x = rng.uniform(*args.x_range)
        y = rng.uniform(*args.y_range)
        if not set_ball_pose(args.world, x, y, vision.BALL_Z + BASE_Z_WORLD):
            print("Gazebo rejected set_pose. Is gz_ball spawned? Is --world right "
                  "(gz model --list prints the world name)?")
            sys.exit(2)

        start = time.monotonic()
        found = None
        while time.monotonic() < start + args.settle + args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.last and node.last[0] >= start + args.settle:
                found = node.last
                break

        if found is None:
            misses += 1
            print(f"{trial:>5} {x:>8.3f} {y:>8.3f} {'-':>8} {'-':>8} {'-':>8}  NOT DETECTED")
            continue
        error = math.hypot(found[1] - x, found[2] - y)
        errors.append(error)
        verdict = "ok" if error <= args.tol else "TOO FAR OFF"
        print(f"{trial:>5} {x:>8.3f} {y:>8.3f} {found[1]:>8.3f} {found[2]:>8.3f} {error:>8.3f}  {verdict}")

    passed = sum(1 for e in errors if e <= args.tol)
    print(f"\n{passed}/{args.trials} within {args.tol * 100:.1f} cm, {misses} not detected", end="")
    if errors:
        print(f", mean error {sum(errors) / len(errors) * 100:.1f} cm, worst {max(errors) * 100:.1f} cm")
    else:
        print()
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if passed == args.trials else 1)


if __name__ == "__main__":
    main()