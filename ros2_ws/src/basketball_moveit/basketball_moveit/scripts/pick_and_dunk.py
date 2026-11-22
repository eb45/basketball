"""Full sequence in one node: detect the ball, pick it, dunk it.

In sim the Gazebo gripper does not move and the Gazebo ball is static, so --sim
pretends the grasp worked and skips the dunk's grip check. The sim run tests
detection, reach, and arm motion only. On the real arm, leave --sim off.
"""

import argparse
import os
import random
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from basketball_moveit.scripts import vision
from basketball_moveit.scripts.basketball_scene import HOOP_CENTER
from basketball_moveit.scripts.check_ball import set_ball_pose
from basketball_moveit.scripts.dunk_ball import Dunker, dunk
from basketball_moveit.scripts.pick_ball import GRIPPER_CLOSED, GRIPPER_OPEN, WORK, pick
from basketball_moveit.scripts.spawn_overhead_camera import BASE_Z_WORLD

SIM_GRASP_READING = 0.4


class SimDunker(Dunker):
    """Sim only: the Gazebo gripper does not move, so pretend the grasp worked."""

    def move_fingers(self, position, timeout=4.0):
        super().move_fingers(position, timeout=1.0)
        if position == GRIPPER_CLOSED:
            self.get_logger().warn(f"--sim: reporting a grasp (finger reading {SIM_GRASP_READING}).")
            return SIM_GRASP_READING
        return position


def parse():
    p = argparse.ArgumentParser(description="Pick the ball, then dunk it.")
    p.add_argument("--approach", type=float, default=0.12)
    p.add_argument("--lift", type=float, default=0.15)
    p.add_argument("--grasp-offset", type=float, default=0.0)
    p.add_argument("--hoop", type=float, nargs=3, default=HOOP_CENTER)
    p.add_argument("--ball-diameter", type=float, default=vision.BALL_DIAMETER)
    p.add_argument("--clearance", type=float, default=0.03)
    p.add_argument("--back", type=float, default=0.12)
    p.add_argument("--rise", type=float, default=0.03)
    p.add_argument("--pitch", type=float, default=None)
    p.add_argument("--roll", type=float, default=0.0)
    p.add_argument("--wait", type=float, default=1.0)
    p.add_argument("--show-ball", action="store_true")
    p.add_argument("--dry-run-dunk", action="store_true")
    p.add_argument("--speed", type=float, default=0.2)
    p.add_argument("--empty-threshold", type=float, default=0.75)
    p.add_argument("--closed-min", type=float, default=0.05)
    p.add_argument("--sim", action="store_true")
    p.add_argument("--trials", type=int, default=1)
    p.add_argument("--world", default="empty")
    p.add_argument("--seed", type=int, default=None)
    args, _ = p.parse_known_args()
    args.dry_run = args.dry_run_dunk
    args.skip_grip_check = args.sim
    args.home = True
    return args


def run_once(node, args):
    log = node.get_logger()
    log.info("=== PICK ===")
    if not pick(node, args):
        return "pick failed"
    log.info("=== DUNK ===")
    if not dunk(node, args):
        node.move_fingers(GRIPPER_OPEN)
        node.move_to_joints(WORK)
        return "dunk failed"
    return "ok"


def main():
    args = parse()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def stop(_signum, _frame):
        print("Interrupted. Stopping; the arm holds its position.", flush=True)
        os._exit(130)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    node = SimDunker() if args.sim else Dunker()
    node.get_logger().info(f"Mode: {'SIM (fake grasp)' if args.sim else 'REAL'}")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    rng = random.Random(args.seed)
    results = []
    try:
        if not node.wait_until_ready():
            sys.exit(1)
        for trial in range(1, args.trials + 1):
            if args.trials > 1 and args.sim:
                x, y = rng.uniform(0.15, 0.40), rng.uniform(-0.20, 0.20)
                set_ball_pose(args.world, x, y, vision.BALL_Z + BASE_Z_WORLD)
                node.history.clear()
                time.sleep(1.5)
            result = run_once(node, args)
            results.append(result)
            node.get_logger().info(f"Trial {trial}/{args.trials}: {result}")
    except subprocess.SubprocessError as exc:
        node.get_logger().error(f"Could not move the Gazebo ball: {exc}")
    finally:
        ok = sum(r == "ok" for r in results)
        print(f"\n{ok}/{len(results)} trials succeeded: {results}")
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if results and all(r == "ok" for r in results) else 1)


if __name__ == "__main__":
    main()
