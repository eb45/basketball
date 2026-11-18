"""Throw feasibility check for the basketball scene.

Computes the release speed and launch angle needed to put the ball through the
hoop from a given release point, checks the flight path against the backboard,
estimates how accurate the release speed must be, and compares the result with
what the arm can physically do. With --animate it also flies the "basketball"
collision sphere along the computed path in RViz.

Physics: ideal projectile only (no air drag, no spin, no bounce). The planning
scene has no physics engine, so the animation is a visualization, not a
simulation. Run basketball_scene first (and leave it running) if you use --animate.

Run (inside the container, after sourcing the workspace):
    python3 -m final_project.throw_check
    python3 -m final_project.throw_check --release 0.25 0.0 0.30
    python3 -m final_project.throw_check --animate --slow 0.25
"""
import argparse
import math
import threading
import time

from final_project.basketball_scene import (
    ARM_JOINTS,
    BACKBOARD_CENTER,
    BACKBOARD_SIZE,
    BALL_DIAMETER,
    FRAME,
    HOOP_CENTER,
    HOOP_INNER_DIAMETER,
    WHITEBOARD_CENTER,
    WHITEBOARD_SIZE,
)

G = 9.81
BALL_RADIUS = BALL_DIAMETER / 2
# Ball center must stay this close to the hoop axis at rim height to clear the rim.
CLEARANCE_MARGIN = HOOP_INNER_DIAMETER / 2 - BALL_RADIUS

DEFAULT_RELEASE = (0.25, 0.0, 0.95)   # base_link frame, meters. Change to your real release pose.
# Rated tool speed of the arm. ASSUMPTION: check the Gen3 Lite datasheet and your
# MoveIt joint limits, then put the real number here.
ARM_MAX_TIP_SPEED = 0.25              # m/s


def horizontal_unit(release, target):
    dx, dy = target[0] - release[0], target[1] - release[1]
    d = math.hypot(dx, dy)
    return d, (dx / d, dy / d)


def required_speed(d, dz, theta):
    """Speed that makes the ball pass through height dz at horizontal distance d."""
    denom = 2 * math.cos(theta) ** 2 * (d * math.tan(theta) - dz)
    if denom <= 0:
        return None
    return math.sqrt(G * d * d / denom)


def landing_distance(v, theta, dz):
    """Horizontal distance at which the ball is at height dz on its way DOWN."""
    vz = v * math.sin(theta)
    disc = vz * vz - 2 * G * dz
    if disc < 0:
        return None
    t = (vz + math.sqrt(disc)) / G
    return v * math.cos(theta) * t


def speed_tolerance(v, theta, d, dz):
    """Fractional release-speed error that still keeps the ball inside the rim."""
    tol = 0.0
    for k in range(1, 400):
        frac = k * 0.001
        ok = True
        for sign in (-1, 1):
            land = landing_distance(v * (1 + sign * frac), theta, dz)
            if land is None or abs(land - d) > CLEARANCE_MARGIN:
                ok = False
        if not ok:
            break
        tol = frac
    return tol


def trajectory(release, target, v, theta, dt=0.02):
    """List of (t, (x, y, z)) from release until the ball reaches rim height going down."""
    d, (ux, uy) = horizontal_unit(release, target)
    vh, vz = v * math.cos(theta), v * math.sin(theta)
    t_cross = d / vh
    points = []
    t = 0.0
    while t <= t_cross + 1e-9:
        points.append((t, (
            release[0] + ux * vh * t,
            release[1] + uy * vh * t,
            release[2] + vz * t - 0.5 * G * t * t,
        )))
        t += dt
    if points[-1][0] < t_cross:
        t = t_cross
        points.append((t, (
            release[0] + ux * vh * t,
            release[1] + uy * vh * t,
            release[2] + vz * t - 0.5 * G * t * t,
        )))
    return points, vz - G * t_cross   # path, vertical speed at the rim (must be < 0)


def hits_board(points):
    """True if the ball sphere touches the backboard or whiteboard before the rim."""
    boards = (
        (BACKBOARD_CENTER, BACKBOARD_SIZE),
        (WHITEBOARD_CENTER, WHITEBOARD_SIZE),
    )
    for _, (x, y, z) in points:
        for center, size in boards:
            front = center[0] - size[0] / 2
            if (x + BALL_RADIUS > front
                    and abs(y - center[1]) < size[1] / 2 + BALL_RADIUS
                    and abs(z - center[2]) < size[2] / 2 + BALL_RADIUS):
                return True
    return False


def analyze(release, max_speed):
    target = HOOP_CENTER
    d, _ = horizontal_unit(release, target)
    dz = target[2] - release[2]
    print(f"Release point : {tuple(round(c, 3) for c in release)} m")
    print(f"Hoop center   : {tuple(round(c, 3) for c in target)} m")
    print(f"Horizontal distance {d:.3f} m, height gain {dz:+.3f} m")
    print(f"Rim clearance margin for the ball center: +/-{CLEARANCE_MARGIN * 100:.1f} cm\n")
    print("angle   speed    notes")

    best = None
    for deg in range(30, 86, 5):
        theta = math.radians(deg)
        v = required_speed(d, dz, theta)
        if v is None:
            print(f"{deg:3d} deg   --     cannot reach the rim at this angle")
            continue
        points, vz_rim = trajectory(release, target, v, theta)
        notes = []
        if vz_rim >= 0:
            notes.append("rising at the rim")
        if hits_board(points):
            notes.append("hits the board")
        print(f"{deg:3d} deg {v:6.2f} m/s  {', '.join(notes) if notes else 'ok'}")
        if not notes and (best is None or v < best[0]):
            best = (v, theta, points)

    print()
    if best is None:
        print("No valid shot found from this release point. Try a different release point.")
        return None
    v, theta, points = best
    tol = speed_tolerance(v, theta, d, dz)
    flight = points[-1][0]
    apex = release[2] + (v * math.sin(theta)) ** 2 / (2 * G)
    print(f"Lowest-speed valid shot: {math.degrees(theta):.0f} deg at {v:.2f} m/s "
          f"(flight {flight:.2f} s, apex z = {apex:.2f} m)")
    print(f"Release speed must be within about +/-{tol * 100:.1f}% to stay inside the rim.")
    print(f"\nArm tool speed assumed: {max_speed:.2f} m/s")
    ratio = v / max_speed
    if ratio <= 1.0:
        print("Verdict: the required speed is within the arm's rated speed.")
    else:
        print(f"Verdict: NOT FEASIBLE as a throw. The ball needs {ratio:.1f}x the arm's tool speed.")
        print("At the arm's own speed the ball would rise only "
              f"{max_speed ** 2 / (2 * G) * 100:.1f} cm. Consider a dunk (carry the ball over\n"
              "the rim and open the gripper) or a very short drop shot instead of a throw.")
    return points


def animate(points, slow):
    import rclpy
    from pymoveit2 import MoveIt2
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    rclpy.init()
    node = Node(
        "throw_animation",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
    )
    moveit2 = MoveIt2(
        node=node,
        joint_names=ARM_JOINTS,
        base_link_name=FRAME,
        end_effector_name="end_effector_link",
        group_name="arm",
    )
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        time.sleep(2)   # let publishers connect
        print(f"Animating at {slow}x speed. Watch the planning scene in RViz.")
        previous_t = 0.0
        for t, pos in points:
            moveit2.move_collision(
                id="basketball",
                position=tuple(pos),
                quat_xyzw=(0.0, 0.0, 0.0, 1.0),
                frame_id=FRAME,
            )
            time.sleep(max(t - previous_t, 0.0) / slow)
            previous_t = t
        print("Animation done. The ball stays at the rim height point.")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--release", nargs=3, type=float, metavar=("X", "Y", "Z"),
                        default=DEFAULT_RELEASE, help="release point in base_link (m)")
    parser.add_argument("--max-speed", type=float, default=ARM_MAX_TIP_SPEED,
                        help="arm tool speed limit in m/s (check the datasheet)")
    parser.add_argument("--animate", action="store_true",
                        help="fly the basketball sphere along the path in RViz")
    parser.add_argument("--slow", type=float, default=0.25,
                        help="animation speed factor (0.25 = 4x slow motion)")
    args = parser.parse_args()

    points = analyze(tuple(args.release), args.max_speed)
    if args.animate and points:
        animate(points, args.slow)


if __name__ == "__main__":
    main()