import argparse
import copy
import json
import math
from pathlib import Path
import threading
import time

from .basketball_scene import (
    ARM_JOINTS, BACKBOARD_CENTER, BACKBOARD_SIZE, BALL_DIAMETER, FRAME,
    HOOP_CENTER, HOOP_INNER_DIAMETER, TABLE_CENTER, TABLE_SIZE,
    WHITEBOARD_CENTER, WHITEBOARD_SIZE,
)
from . import basketball_scene as scene_geometry

G = 9.81
BALL_RADIUS = BALL_DIAMETER / 2
CLEARANCE_MARGIN = HOOP_INNER_DIAMETER / 2 - BALL_RADIUS
TABLE_TOP = TABLE_CENTER[2] + TABLE_SIZE[2] / 2
DEFAULT_RELEASE = (0.25, 0.0, 0.55)
GRIPPER_GROUP = "gripper"
GRIPPER_JOINT = "right_finger_bottom_joint"
GRIPPER_OPEN = 0.0
EE_LINK = "end_effector_link"


# ----------------------------------------------------------------------------------
# Ballistics (pure Python)
# ----------------------------------------------------------------------------------
def horizontal_unit(release, target):
    dx, dy = target[0] - release[0], target[1] - release[1]
    d = math.hypot(dx, dy)
    if d < 1e-9:
        raise ValueError("Release needs a nonzero horizontal distance from the hoop")
    return d, (dx / d, dy / d)


def required_speed(d, dz, theta):
    """Speed that makes the ball pass through height dz at horizontal distance d."""
    denom = 2 * math.cos(theta) ** 2 * (d * math.tan(theta) - dz)
    if denom <= 0:
        return None
    return math.sqrt(G * d * d / denom)


def trajectory(release, target, v, theta, dt=0.02):
    """List of (t, (x, y, z)) from release until the ball reaches rim height going down."""
    d, (ux, uy) = horizontal_unit(release, target)
    vh, vz = v * math.cos(theta), v * math.sin(theta)
    t_cross = d / vh

    def point(t):
        return (t, (release[0] + ux * vh * t,
                    release[1] + uy * vh * t,
                    release[2] + vz * t - 0.5 * G * t * t))

    points = []
    t = 0.0
    while t <= t_cross + 1e-9:
        points.append(point(t))
        t += dt
    if points[-1][0] < t_cross:
        points.append(point(t_cross))
    return points, vz - G * t_cross


def hits_board(points):
    """True if the ball sphere touches the backboard or whiteboard."""
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


def analyze(release, verbose=True):
    """Ideal-arc analysis to the current hoop. Returns (speed, theta, points) or None."""
    say = print if verbose else (lambda *a, **k: None)
    target = HOOP_CENTER
    d, _ = horizontal_unit(release, target)
    dz = target[2] - release[2]
    say(f"Release point : {tuple(round(c, 3) for c in release)} m")
    say(f"Hoop center   : {tuple(round(c, 3) for c in target)} m")
    say(f"Horizontal distance {d:.3f} m, height gain {dz:+.3f} m")
    say(f"Rim clearance margin for the ball center: +/-{CLEARANCE_MARGIN * 100:.1f} cm\n")
    say("angle   speed    notes")

    best = None
    for deg in range(30, 86, 5):
        theta = math.radians(deg)
        v = required_speed(d, dz, theta)
        if v is None:
            say(f"{deg:3d} deg   --     cannot reach the rim at this angle")
            continue
        points, vz_rim = trajectory(release, target, v, theta)
        notes = []
        if vz_rim >= 0:
            notes.append("rising at the rim")
        if hits_board(points):
            notes.append("hits the board")
        say(f"{deg:3d} deg {v:6.2f} m/s  {', '.join(notes) if notes else 'ok'}")
        if not notes and (best is None or v < best[0]):
            best = (v, theta, points)

    say("")
    if best is None:
        say("No valid ideal shot found from this release point to the current hoop.")
        return None
    v, theta, points = best
    flight = points[-1][0]
    apex = release[2] + (v * math.sin(theta)) ** 2 / (2 * G)
    say(f"Lowest-speed ideal arc: {math.degrees(theta):.0f} deg at {v:.2f} m/s "
        f"(flight {flight:.2f} s, apex z = {apex:.2f} m)")
    return v, theta, points


def estimate_velocity(history, window=0.12):
    """Least-squares velocity of the ball from recent (time, position) samples."""
    if len(history) < 2:
        raise RuntimeError("Not enough joint-state samples to measure the release velocity")
    t_end = history[-1][0]
    recent = [h for h in history if t_end - h[0] <= window]
    if len(recent) < 3:
        recent = history[-3:]
    n = len(recent)
    tm = sum(t for t, _ in recent) / n
    denom = sum((t - tm) ** 2 for t, _ in recent)
    if denom <= 1e-12:
        raise RuntimeError("Joint-state timestamps did not advance; cannot measure velocity")
    pm = [sum(p[k] for _, p in recent) / n for k in range(3)]
    return tuple(sum((t - tm) * (p[k] - pm[k]) for t, p in recent) / denom for k in range(3))


def urdf_limits(xml_text, names):
    """Read position and speed limits, preserving unbounded continuous joints."""
    import xml.etree.ElementTree as ET
    found = {}
    for joint in ET.fromstring(xml_text).findall("joint"):
        name, lim = joint.attrib.get("name"), joint.find("limit")
        if name not in names or lim is None:
            continue
        if joint.attrib.get("type") == "continuous":
            # Keep measured multi-turn angles; do not wrap or command a full turn.
            lower, upper = -math.inf, math.inf
        else:
            if "lower" not in lim.attrib or "upper" not in lim.attrib:
                raise RuntimeError(f"Bounded joint {name} is missing explicit lower/upper URDF limits")
            lower, upper = float(lim.attrib["lower"]), float(lim.attrib["upper"])
            if not math.isfinite(lower) or not math.isfinite(upper) or lower > upper:
                raise RuntimeError(f"Invalid URDF position limits for {name}: [{lower}, {upper}]")
        found[name] = (lower, upper, float(lim.attrib.get("velocity", 0.0)))
    missing = [n for n in names if n not in found or not math.isfinite(found[n][2]) or found[n][2] <= 0]
    if missing:
        raise RuntimeError(f"No usable <limit velocity=...> in the URDF for: {missing}")
    return [found[n] for n in names]


def joint_limit_violations(positions, limits, joint_names=None):
    """Describe genuine limit violations without silently changing a pose."""
    if len(positions) != len(limits):
        raise ValueError("Joint positions and limits have different lengths")
    names = list(joint_names) if joint_names is not None else [f"joint_{i+1}" for i in range(len(positions))]
    if len(names) != len(positions):
        raise ValueError("Joint names and positions have different lengths")
    problems = []
    for name, q, (lo, hi, _) in zip(names, positions, limits):
        if not math.isfinite(q):
            problems.append(f"{name}: non-finite starting position {q}")
        elif not lo <= q <= hi:
            problems.append(f"{name}: {q:.6f} rad ({math.degrees(q):.2f} deg), "
                            f"allowed [{lo:.6f}, {hi:.6f}] rad "
                            f"([{math.degrees(lo):.2f}, {math.degrees(hi):.2f}] deg)")
    return problems


def simple_swing_candidates(home, limits, joint_index, windup_deg, follow_deg, joint_names=None):
    """Small one-joint swings; preserve all other joints exactly.

    Both signs are considered because the positive joint axis depends on the
    robot model. Forward kinematics selects the direction toward the hoop.
    """
    if len(home) != len(limits) or not 0 <= joint_index < len(home):
        raise ValueError("Invalid home pose or swing-joint index")
    if not 0 < windup_deg <= 60 or not 0 < follow_deg <= 45:
        raise ValueError("Windup must be in (0, 60] and follow-through in (0, 45] degrees")
    problems = joint_limit_violations(home, limits, joint_names)
    if problems:
        raise ValueError("Starting pose violates joint limits: "+"; ".join(problems))
    lo, hi, _ = limits[joint_index]
    for sign in (-1.0, 1.0):
        for back_factor in (1.0, 0.85, 0.7, 0.55, 0.4):
            for forward_factor in (1.0, 0.75, 0.5):
                qa, qb = list(home), list(home)
                qa[joint_index] -= sign * math.radians(windup_deg) * back_factor
                qb[joint_index] += sign * math.radians(follow_deg) * forward_factor
                if lo + 0.01 <= qa[joint_index] <= hi - 0.01 and lo + 0.01 <= qb[joint_index] <= hi - 0.01:
                    yield qa, qb


def elbow_swing_candidates(home, limits, shoulder_index, windup_deg, follow_deg,
                            elbow_index=None, elbow_deg=20.0, joint_names=None):
    """Include the working shoulder-only paths plus small coordinated elbow bends.

    elbow_deg is total elbow travel: half behind neutral and half beyond it.
    Both relative directions are evaluated with FK; no axis sign is assumed.
    """
    if elbow_index is not None:
        if not 0 <= elbow_index < len(home) or elbow_index == shoulder_index:
            raise ValueError("Shoulder and elbow must be different valid joint indices")
        if not math.isfinite(elbow_deg) or not 0 <= elbow_deg <= 45:
            raise ValueError("Elbow travel must be between 0 and 45 degrees")
    for qa, qb in simple_swing_candidates(home, limits, shoulder_index, windup_deg, follow_deg, joint_names):
        # Preserve a shoulder-only candidate for comparison and safe fallback.
        yield qa, qb
        if elbow_index is None or elbow_deg == 0:
            continue
        lo, hi, _ = limits[elbow_index]
        direction = math.copysign(1.0, qb[shoulder_index]-qa[shoulder_index])
        for scale in (1.0, 0.5):
            for relative in (1.0, -1.0):
                start, finish = list(qa), list(qb)
                bend = direction*relative*math.radians(elbow_deg)*0.5*scale
                start[elbow_index] = home[elbow_index]-bend
                finish[elbow_index] = home[elbow_index]+bend
                if lo+0.01 <= start[elbow_index] <= hi-0.01 and lo+0.01 <= finish[elbow_index] <= hi-0.01:
                    yield start, finish


def swing_profile(qa, qb, vmax, amax, s_rel, brake_mult, dt=0.01):
    """Synchronized joint-space swing qa -> qb: accelerate, cruise, brake.
    Path fraction s goes 0..1; the ball is released at s_rel, at the peak speed.
    Returns (samples[(t, s, ds/dt, d2s/dt2)], t_release, ds/dt at release, total time)."""
    delta = [b - a for a, b in zip(qa, qb)]
    moving = [(v, a, abs(d)) for v, a, d in zip(vmax, amax, delta) if abs(d) > 1e-6]
    if not moving:
        raise ValueError("Zero-length swing")
    s_dot_max = min(v / d for v, _, d in moving)
    s_acc = min(a / d for _, a, d in moving)
    s_brk = s_acc * brake_mult
    v = min(s_dot_max, math.sqrt(2 * s_acc * s_rel), math.sqrt(2 * s_brk * (1 - s_rel)))
    d1, d3 = v * v / (2 * s_acc), v * v / (2 * s_brk)
    t1, tc, t3 = v / s_acc, (1 - d1 - d3) / v, v / s_brk
    total = t1 + tc + t3

    def at(t):
        if t <= t1:
            return 0.5 * s_acc * t * t, s_acc * t, s_acc
        if t <= t1 + tc:
            return d1 + v * (t - t1), v, 0.0
        u = min(t - t1 - tc, t3)
        return 1 - d3 + v * u - 0.5 * s_brk * u * u, v - s_brk * u, -s_brk

    samples = []
    for k in range(int(math.ceil(total / dt)) + 1):
        t = min(k * dt, total)
        samples.append((t,) + at(t))
    return samples, t1 + (s_rel - d1) / v, v, total


def joint_trajectory_from_profile(start, finish, profile, joint_names):
    """Preserve the straight joint-space path in every trajectory sample."""
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    change = [b-a for a, b in zip(start, finish)]
    trajectory = JointTrajectory()
    trajectory.joint_names = list(joint_names)
    for t, fraction, rate, acceleration in profile:
        pt = JointTrajectoryPoint()
        pt.positions = [a+fraction*d for a, d in zip(start, change)]
        pt.velocities = [d*rate for d in change]
        pt.accelerations = [d*acceleration for d in change]
        nanoseconds = round(t * 1e9)
        pt.time_from_start = Duration(sec=nanoseconds // 1000000000,
                                      nanosec=nanoseconds % 1000000000)
        trajectory.points.append(pt)
    return trajectory


def landing_range(p0, vel, u):
    """(forward range along u, lateral offset, flight time) when the ball falls back
    to table-top height, or None."""
    z_land = TABLE_TOP + BALL_RADIUS
    disc = vel[2] ** 2 + 2 * G * (p0[2] - z_land)
    if disc < 0:
        return None
    t = (vel[2] + math.sqrt(disc)) / G
    if t <= 0:
        return None
    dx, dy = vel[0] * t, vel[1] * t
    return dx * u[0] + dy * u[1], -dx * u[1] + dy * u[0], t


def on_table(x, y):
    return (abs(x - TABLE_CENTER[0]) <= TABLE_SIZE[0] / 2
            and abs(y - TABLE_CENTER[1]) <= TABLE_SIZE[1] / 2)


def simulate_flight(p0, v, dt=0.01, t_max=3.0):
    """Ideal projectile from p0 with velocity v, until it lands, hits a board or times out."""
    pts = []
    t = 0.0
    while t <= t_max:
        pos = (p0[0] + v[0] * t, p0[1] + v[1] * t, p0[2] + v[2] * t - 0.5 * G * t * t)
        pts.append((t, pos))
        falling = v[2] - G * t < 0
        if falling and on_table(pos[0], pos[1]) and pos[2] <= TABLE_TOP + BALL_RADIUS:
            break
        if falling and pos[2] < TABLE_TOP - 0.5:
            break
        if t > 0 and hits_board([(t, pos)]):
            break
        t += dt
    return pts


def judge_flight(pts):
    """Print what the flight does at the hoop. Returns True for a score."""
    hx, hy, hz = HOOP_CENTER
    apex = max(p[2] for _, p in pts)
    for i, ((_, a), (_, b)) in enumerate(zip(pts, pts[1:])):
        if a[2] >= hz > b[2]:
            f = (a[2] - hz) / (a[2] - b[2])
            x = a[0] + f * (b[0] - a[0])
            y = a[1] + f * (b[1] - a[1])
            miss = math.hypot(x - hx, y - hy)
            if hits_board(pts[:i + 2]):
                print(f"Result: the ball hits the board. (Crosses rim height {miss * 100:.0f} cm "
                      "from the hoop center.)")
                return False
            if miss <= CLEARANCE_MARGIN:
                print(f"Result: SCORE. The ball crosses the rim {miss * 100:.1f} cm from the center "
                      f"(limit {CLEARANCE_MARGIN * 100:.1f} cm).")
                return True
            print(f"Result: MISS. The ball crosses rim height {miss * 100:.0f} cm from the hoop "
                  f"center (must be within {CLEARANCE_MARGIN * 100:.1f} cm).")
            return False
    last = pts[-1][1]
    if hits_board(pts):
        print("Result: MISS. The ball hits the board before reaching the rim.")
    elif apex < hz:
        print(f"Result: MISS. The ball only reaches z = {apex:.2f} m; the rim is at z = {hz:.2f} m. "
              f"It lands about {math.hypot(last[0] - hx, last[1] - hy) * 100:.0f} cm from the hoop axis.")
    else:
        print("Result: MISS. The ball never comes down through the rim.")
    return False


def crossing_at(pts, height):
    """(x, y) where the ball center passes `height` on the way down, or None."""
    for (_, a), (_, b) in zip(pts, pts[1:]):
        if a[2] >= height > b[2]:
            f = (a[2] - height) / (a[2] - b[2])
            return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))
    return None


def suggest_hoops(pts):
    """Print rim positions that would catch this shot, as scene settings."""
    start_z = pts[0][1][2]
    shift0 = getattr(scene_geometry, "BOARD_SHIFT_X", None)
    raise0 = getattr(scene_geometry, "HOOP_RAISE", None)
    print("\nWhere a rim would catch THIS shot (ball center passing through the rim center):")
    shown = False
    for drop in (0.05, 0.10, 0.15, 0.20, 0.30, 0.40):
        height = start_z - drop
        if height < TABLE_TOP + 0.12:
            continue
        hit = crossing_at(pts, height)
        if hit is None:
            continue
        shown = True
        line = f"  rim z = {height:.2f} m: x = {hit[0]:.3f} m, y = {hit[1]:+.3f} m"
        if shift0 is not None and raise0 is not None:
            line += (f"   ->  BOARD_SHIFT_X = {shift0 + hit[0] - HOOP_CENTER[0]:.3f}, "
                     f"HOOP_RAISE = {raise0 + height - HOOP_CENTER[2]:+.3f}")
        print(line)
    if not shown:
        print("  None found: the ball never passes down through a rim height above the table.")
    print("The rim ring is about "
          f"{HOOP_INNER_DIAMETER * 100:.0f} cm wide, so keep it clear of the arm itself.")


# ----------------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------------
def rotated_offset(v, q):
    """Rotate a tool-frame ball offset into base_link."""
    n = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if n < 1e-12:
        raise RuntimeError("Invalid end-effector quaternion")
    x, y, z, w = q.x/n, q.y/n, q.z/n, q.w/n
    a, b, c = v
    tx, ty, tz = 2*(y*c-z*b), 2*(z*a-x*c), 2*(x*b-y*a)
    return (a+w*tx+y*tz-z*ty, b+w*ty+z*tx-x*tz, c+w*tz+x*ty-y*tx)


def wait_for_condition(condition, timeout, description, alive=lambda: True):
    """Wait using wall time, so a paused/missing ROS clock cannot hang us."""
    deadline = time.monotonic() + timeout
    while True:
        if not alive():
            raise RuntimeError(f"ROS executor stopped while waiting for {description}")
        if condition():
            return
        if time.monotonic() >= deadline:
            raise RuntimeError(f"Timed out after {timeout:g}s waiting for {description}")
        time.sleep(0.01)


def load_home_positions(current, args):
    """Remember the first run's measured starting joints; never invent a pose."""
    if args.home_joints is not None:
        return list(args.home_joints)
    path = Path(args.home_file).expanduser()
    if args.set_home or not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps({"joint_names": ARM_JOINTS, "positions": current}, indent=2))
        temporary.replace(path)
        print(f"[reset] Saved current joint pose as the repeatable start: {path}", flush=True)
        return list(current)
    data = json.loads(path.read_text())
    if data.get("joint_names") != ARM_JOINTS:
        raise RuntimeError("Saved home belongs to different joints; use --set-home or --home-joints")
    positions = data.get("positions", [])
    if len(positions) != len(ARM_JOINTS) or not all(math.isfinite(x) for x in positions):
        raise RuntimeError("Invalid saved home; use --set-home or --home-joints")
    return positions


# ----------------------------------------------------------------------------------
# The throw
# ----------------------------------------------------------------------------------
def run_throw(release, args):
    # Direction of the swing: toward the hoop horizontally, tilted up by --angle.
    _, (hux, huy) = horizontal_unit(release, HOOP_CENTER)
    angle = math.radians(args.angle)
    direction = (hux * math.cos(angle), huy * math.cos(angle), math.sin(angle))

    print("[arm] Loading ROS interfaces...", flush=True)
    import rclpy
    from geometry_msgs.msg import Pose
    from moveit_msgs.action import ExecuteTrajectory
    from moveit_msgs.msg import MoveItErrorCodes
    from action_msgs.msg import GoalStatus
    from rclpy.action import ActionClient
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene, PlanningSceneComponents
    from moveit_msgs.srv import ApplyPlanningScene, GetPositionFK, GetMotionPlan, GetCartesianPath, GetPlanningScene, GetStateValidity
    from action_msgs.srv import CancelGoal
    from rcl_interfaces.srv import GetParameters
    from pymoveit2 import MoveIt2
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from shape_msgs.msg import SolidPrimitive

    ee = args.ee_link
    print("[arm] Initializing ROS node...", flush=True)
    rclpy.init()
    node = Node("illustrative_arm_throw", parameter_overrides=[
        Parameter("use_sim_time", Parameter.Type.BOOL, args.use_sim_time)])
    callback_group = ReentrantCallbackGroup()
    print(f"[arm] Creating MoveIt interface (use_sim_time={args.use_sim_time})...", flush=True)
    moveit = MoveIt2(node=node, joint_names=ARM_JOINTS,
                     base_link_name=FRAME, end_effector_name=ee, group_name="arm",
                     callback_group=callback_group)

    def set_speed(scale):
        moveit.max_velocity = scale
        moveit.max_acceleration = scale

    set_speed(args.arm_speed)
    apply_client = node.create_client(ApplyPlanningScene, "apply_planning_scene", callback_group=callback_group)
    fk_client = node.create_client(GetPositionFK, "compute_fk", callback_group=callback_group)
    plan_client = node.create_client(GetMotionPlan, "plan_kinematic_path", callback_group=callback_group)
    cartesian_client = node.create_client(GetCartesianPath, "compute_cartesian_path", callback_group=callback_group)
    scene_client = node.create_client(GetPlanningScene, "get_planning_scene", callback_group=callback_group)
    validity_client = node.create_client(GetStateValidity, "check_state_validity", callback_group=callback_group)
    cancel_client = node.create_client(CancelGoal, "execute_trajectory/_action/cancel_goal", callback_group=callback_group)
    arm_client = ActionClient(node, ExecuteTrajectory, "execute_trajectory",
                              callback_group=callback_group)
    gripper = None if args.no_gripper else MoveIt2(
        node=node, joint_names=[GRIPPER_JOINT], base_link_name=FRAME,
        end_effector_name=ee, group_name=GRIPPER_GROUP, callback_group=callback_group)
    gripper_thread, gripper_requested = None, False
    joint_lock, joint_state = threading.Lock(), [None]

    def receive_joints(msg):
        # Ignore unrelated joint messages; FK needs all six arm joints.
        if len(msg.name) != len(msg.position) or not set(ARM_JOINTS).issubset(msg.name):
            return
        with joint_lock:
            joint_state[0] = copy.deepcopy(msg)

    subscription = node.create_subscription(
        JointState, args.joint_states, receive_joints, qos_profile_sensor_data,
        callback_group=callback_group)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    executor_errors = []

    def spin_executor():
        try:
            executor.spin()
        except Exception as error:
            executor_errors.append(error)
            print(f"[arm] ROS executor failed: {error}", flush=True)

    spin = threading.Thread(target=spin_executor, daemon=True)
    spin.start()
    attached, swing_thread, swing_done = False, None, threading.Event()
    swing_ok, swing_errors = [False], []
    home_positions, scene_touched = None, False
    touch_links = list(dict.fromkeys([ee] + args.touch_links))
    execution_events = []
    arm_execution = [None]

    def alive():
        return rclpy.ok() and spin.is_alive() and not executor_errors

    def wait_future(future, description, timeout=10.0):
        if future is None:
            raise RuntimeError(f"No ROS request was sent for {description}")
        wait_for_condition(future.done, timeout, description, alive)
        result = future.result()
        if result is None:
            raise RuntimeError(f"Empty ROS response for {description}")
        return result

    def current_joints():
        with joint_lock:
            state = copy.deepcopy(joint_state[0])
        if state is None:
            raise RuntimeError(f"No complete arm joint state on {args.joint_states}")
        return state

    def fk(state=None):
        # Use the service directly. Some pymoveit2 versions wait indefinitely
        # inside compute_fk() when state updates or ROS time are unavailable.
        request = GetPositionFK.Request()
        request.header.frame_id = FRAME
        request.fk_link_names = [ee]
        request.robot_state.joint_state = current_joints() if state is None else state
        request.robot_state.is_diff = True
        result = wait_future(fk_client.call_async(request), "compute_fk pose response", 5.0)
        if result.error_code.val != 1 or not result.pose_stamped:
            raise RuntimeError(f"compute_fk failed with code {result.error_code.val}; check --ee-link and FRAME")
        return result.pose_stamped[0].pose

    def plan_pose(pose=None, cartesian=False, label=None, start_state=None, position=None):
        label = label or ("Cartesian swing plan" if cartesian else "windup plan")
        print(f"[arm] Requesting {label}...", flush=True)
        # Older pymoveit2 versions do not accept cartesian_max_step here.
        # Use the installed version's default Cartesian sampling step.
        kwargs = dict(start_joint_state=current_joints() if start_state is None else start_state,
                      cartesian=cartesian)
        if position is None:
            kwargs["pose"] = pose
        else:
            # A position goal lets IK choose the wrist orientation. Passing
            # the upright home quaternion here overconstrains a low target.
            kwargs["position"] = tuple(position)
        future = moveit.plan_async(**kwargs)
        response = wait_future(future, label, args.plan_timeout)
        planning_response = response if cartesian else getattr(response, "motion_plan_response", response)
        error = getattr(planning_response, "error_code", None)
        if error is not None and error.val != 1:
            print(f"[arm] {label} rejected: code {error.val}; "
                  f"{getattr(error, 'message', '')}", flush=True)
        return moveit.get_trajectory(
            future, cartesian=cartesian,
            cartesian_fraction_threshold=0.999 if cartesian else 0.0)

    def plan_end_state(plan, start_state):
        state = copy.deepcopy(start_state)
        values = dict(zip(state.name, state.position))
        if not plan.points:
            raise RuntimeError("MoveIt returned an empty trajectory")
        values.update(zip(plan.joint_names, plan.points[-1].positions))
        state.name, state.position = list(values), list(values.values())
        state.velocity, state.effort = [], []
        return state

    def check_start_state():
        if not validity_client.service_is_ready():
            print("[arm] State-validity service unavailable; planner will validate collisions.", flush=True)
            return
        request = GetStateValidity.Request()
        request.robot_state.joint_state = current_joints()
        request.robot_state.is_diff = True
        request.group_name = "arm"
        response = wait_future(validity_client.call_async(request), "start-state collision check")
        if not response.valid:
            contacts = sorted({f"{c.contact_body_1} / {c.contact_body_2}" for c in response.contacts})
            reason = "; ".join(contacts) or "joint bounds or state constraints (no contact pairs returned)"
            raise RuntimeError("MoveIt rejects the CURRENT starting state: "+reason+
                               ". Fix these collisions/bounds in basketball_scene or the robot model first")

    def start_arm_execution(trajectory, label):
        """Track acceptance and result of this specific ExecuteTrajectory goal.

        Some pymoveit2 versions report no motion immediately after execute().
        This uses the ROS action's futures instead of that wrapper state.
        """
        previous = arm_execution[0]
        if previous is not None and not previous["done"].is_set():
            raise RuntimeError("Previous arm execution is still outstanding")
        if trajectory is None:
            raise RuntimeError(f"{label}: no trajectory was provided")
        joint_trajectory = getattr(trajectory, "joint_trajectory", trajectory)
        if not joint_trajectory.joint_names or not joint_trajectory.points:
            raise RuntimeError(f"{label}: the trajectory is empty")
        wait_for_condition(arm_client.server_is_ready, args.startup_timeout,
                           "execute_trajectory action server", alive)
        goal = ExecuteTrajectory.Goal()
        if hasattr(trajectory, "joint_trajectory"):
            goal.trajectory = copy.deepcopy(trajectory)
        else:
            goal.trajectory.joint_trajectory = copy.deepcopy(trajectory)
        operation = {"done": threading.Event(), "handle": None,
                     "response": None, "result": None, "error": None}
        arm_execution[0] = operation
        # Register before submitting, so cleanup can see a pending acceptance.
        execution_events.append(operation["done"])

        def receive_result(future):
            try:
                operation["result"] = future.result()
                if operation["result"] is None:
                    raise RuntimeError("ExecuteTrajectory returned an empty result")
            except Exception as error:
                operation["error"] = error
            finally:
                operation["done"].set()

        def receive_acceptance(future):
            try:
                handle = future.result()
                if handle is None or not handle.accepted:
                    raise RuntimeError("MoveIt rejected the ExecuteTrajectory goal; check move_group logs")
                operation["handle"] = handle
                operation["response"] = handle.get_result_async()
                operation["response"].add_done_callback(receive_result)
            except Exception as error:
                operation["error"] = error
                operation["done"].set()

        print(f"[arm] Submitting {label}...", flush=True)
        try:
            operation["submission"] = arm_client.send_goal_async(goal)
        except Exception as error:
            operation["error"] = error
            operation["done"].set()
            raise
        operation["submission"].add_done_callback(receive_acceptance)
        # The externally running executor processes the action callbacks.
        wait_for_condition(lambda: operation["handle"] is not None or operation["done"].is_set(),
                           args.startup_timeout, f"{label} goal acceptance", alive)
        if operation["error"] is not None:
            raise RuntimeError(f"{label}: {operation['error']}")
        print(f"[arm] MoveIt accepted {label}; waiting for its execution result.", flush=True)

    def wait_execution(label):
        operation = arm_execution[0]
        if operation is None:
            raise RuntimeError(f"{label}: no arm goal was submitted")
        try:
            wait_for_condition(operation["done"].is_set, 90, label, alive)
        except RuntimeError:
            handle = operation["handle"]
            if handle is not None:
                handle.cancel_goal_async()
            raise
        if operation["error"] is not None:
            raise RuntimeError(f"{label}: {operation['error']}")
        response = operation["result"]
        if response is None:
            raise RuntimeError(f"{label}: no execution result was received")
        error_code = response.result.error_code
        code = error_code.val
        if response.status != GoalStatus.STATUS_SUCCEEDED or code != MoveItErrorCodes.SUCCESS:
            name = next((n for n in dir(MoveItErrorCodes)
                         if n.isupper() and getattr(MoveItErrorCodes, n) == code), "UNKNOWN")
            detail = getattr(error_code, "message", "")
            raise RuntimeError(f"{label}: ExecuteTrajectory status={response.status}, "
                               f"MoveIt error {name} ({code}). {detail} "
                               "Check the move_group and controller terminals")
        print(f"[arm] {label} completed successfully.", flush=True)
        return True

    def request_gripper_open():
        nonlocal gripper_thread, gripper_requested
        if gripper is None or gripper_requested:
            return
        gripper_requested = True

        def open_fingers():
            try:
                future = gripper.plan_async(
                    joint_positions=[GRIPPER_OPEN], joint_names=[GRIPPER_JOINT],
                    start_joint_state=current_joints())
                wait_future(future, "gripper opening plan", args.plan_timeout)
                plan = gripper.get_trajectory(future)
                if plan is None:
                    raise RuntimeError("No gripper opening plan; verify GRIPPER_GROUP/JOINT/OPEN")
                gripper.execute(plan)
                gripper.wait_until_executed()
            except Exception as error:
                print(f"[gripper] Opening command failed: {error}. "
                      "The ball release does not depend on finger motion.", flush=True)

        # Gripper planning must not delay release or pause the ball animation.
        gripper_thread = threading.Thread(target=open_fingers, daemon=True)
        gripper_thread.start()

    def ball_center(pose):
        offset = rotated_offset(args.ball_offset, pose.orientation)
        p = pose.position
        return (p.x+offset[0], p.y+offset[1], p.z+offset[2])

    def sphere(position, frame):
        obj = CollisionObject()
        obj.id, obj.header.frame_id = "basketball", frame
        obj.operation = CollisionObject.ADD
        if hasattr(obj, "pose"):
            obj.pose.orientation.w = 1.0
        primitive = SolidPrimitive()
        primitive.type, primitive.dimensions = SolidPrimitive.SPHERE, [BALL_RADIUS]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.w = 1.0
        obj.primitives, obj.primitive_poses = [primitive], [pose]
        return obj

    def apply(world=(), attachments=(), label="planning-scene update"):
        scene = PlanningScene()
        scene.is_diff, scene.robot_state.is_diff = True, True
        scene.world.collision_objects = list(world)
        scene.robot_state.attached_collision_objects = list(attachments)
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = apply_client.call_async(request)
        if not wait_future(future, label).success:
            raise RuntimeError(f"MoveIt rejected {label}; check move_group's terminal for its reason")

    def ball_state():
        request = GetPlanningScene.Request()
        request.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                                         PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        state = wait_future(scene_client.call_async(request), "ball scene state").scene
        world = [o for o in state.world.collision_objects if o.id == "basketball"]
        held = [o for o in state.robot_state.attached_collision_objects if o.object.id == "basketball"]
        return world, held

    def detach_if_present():
        nonlocal attached
        _, held = ball_state()
        if held:
            remove = AttachedCollisionObject()
            # Empty link name lets MoveIt detach this id from its actual link,
            # even if a previous script used a different gripper link.
            remove.object.id, remove.object.operation = "basketball", CollisionObject.REMOVE
            apply(attachments=[remove], label="detach previous basketball")
        if ball_state()[1]:
            raise RuntimeError("Basketball remained attached after detach request")
        attached = False

    def reset_ball():
        nonlocal scene_touched
        print("[reset] Clearing stale ball attachment and restoring the ball...", flush=True)
        scene_touched = True
        detach_if_present()
        xy = scene_geometry.BALL_CENTER_XY
        top = scene_geometry.TABLE_CENTER[2] + scene_geometry.TABLE_SIZE[2]/2
        apply(world=[sphere((xy[0], xy[1], top+BALL_RADIUS), FRAME)], label="restore basketball on table")

    def reset_arm():
        if home_positions is None:
            return
        values = dict(zip(current_joints().name, current_joints().position))
        if max(abs(values[name]-q) for name, q in zip(ARM_JOINTS, home_positions)) < 0.005:
            print("[reset] Arm is at the saved starting pose.", flush=True)
            return
        if args.max_throw:
            # Return only the selected shoulder/elbow joints directly, without
            # a general planner moving the base or wrist through another pose.
            index = ARM_JOINTS.index(args.swing_joint)
            start = [values[name] for name in ARM_JOINTS]
            allowed = {index}
            if not args.no_elbow and args.elbow_deg > 0:
                allowed.add(ARM_JOINTS.index(args.elbow_joint))
            if any(abs(a-b) > 0.02 for j, (a, b) in enumerate(zip(start, home_positions)) if j not in allowed):
                raise RuntimeError("Position the arm in the desired straight pose and run --set-home; "
                                   "the simple throw will not fold other joints to reach an old saved pose")
            if not validity_client.service_is_ready():
                raise RuntimeError("check_state_validity is required for the straight-arm reset")
            limits = urdf_limits(fetch_urdf(), ARM_JOINTS)
            problems = joint_limit_violations(home_positions, limits, ARM_JOINTS)
            if problems:
                raise RuntimeError("Saved starting pose violates joint limits: "+"; ".join(problems))
            steps = max(10, int(math.ceil(max(abs(a-b) for a, b in zip(start, home_positions)) / math.radians(2))))
            current_state = current_joints()
            for k in range(steps+1):
                request = GetStateValidity.Request()
                state = copy.deepcopy(current_state)
                positions = dict(zip(state.name, state.position))
                positions.update(zip(ARM_JOINTS, [a+(b-a)*k/steps for a, b in zip(start, home_positions)]))
                state.name, state.position = list(positions), list(positions.values())
                state.velocity, state.effort = [], []
                request.robot_state.joint_state = state
                request.robot_state.is_diff = True
                request.group_name = "arm"
                if not wait_future(validity_client.call_async(request), "straight-arm reset collision check").valid:
                    raise RuntimeError("The direct return to the saved straight pose is blocked; reposition the arm")
            vmax = [args.arm_speed*limit[2] for limit in limits]
            amax = [args.accel_factor*v for v in vmax]
            profile, _, _, _ = swing_profile(start, home_positions, vmax, amax, 0.5, 1.0)
            print("[reset] Returning directly to the saved straight pose...", flush=True)
            start_arm_execution(joint_trajectory_from_profile(start, home_positions, profile, ARM_JOINTS),
                                "straight-arm reset")
            wait_execution("straight-arm reset execution")
            return
        print("[reset] Returning arm to the saved starting pose...", flush=True)
        set_speed(args.arm_speed)
        future = moveit.plan_async(joint_positions=home_positions, joint_names=ARM_JOINTS,
                                   start_joint_state=current_joints())
        wait_future(future, "reset arm plan", args.plan_timeout)
        plan = moveit.get_trajectory(future)
        if plan is None:
            raise RuntimeError("Cannot plan a collision-free return to the saved start; reposition in RViz or set --home-joints")
        start_arm_execution(plan, "planned arm move")
        if not wait_execution("return to saved starting pose"):
            raise RuntimeError("Arm reset failed")

    def discover_gripper_links():
        import xml.etree.ElementTree as ET
        for owner in ("robot_state_publisher", "move_group"):
            client = node.create_client(GetParameters, owner+"/get_parameters", callback_group=callback_group)
            if not client.wait_for_service(timeout_sec=0.5):
                continue
            request = GetParameters.Request()
            request.names = ["robot_description"]
            try:
                response = wait_future(client.call_async(request), "robot link names", 2.0)
                if not response.values or not response.values[0].string_value:
                    continue
                robot = ET.fromstring(response.values[0].string_value)
                for link in robot.findall("link"):
                    name = link.attrib.get("name", "")
                    if any(word in name.lower() for word in ("finger", "gripper", "knuckle")):
                        if name not in touch_links:
                            touch_links.append(name)
                break
            except (RuntimeError, ET.ParseError) as error:
                print(f"[arm] Could not discover gripper links from {owner}: {error}", flush=True)
        print("[arm] Allowed held-ball contacts: "+", ".join(touch_links), flush=True)

    def load_ball():
        nonlocal attached
        # Separate messages are essential: attachment automatically removes the
        # world object. Removing it again in the same diff returns failure.
        apply(world=[sphere(ball_center(fk()), FRAME)], label="position ball at gripper")
        held = AttachedCollisionObject()
        held.link_name = ee
        held.touch_links = touch_links
        held.object.id, held.object.operation = "basketball", CollisionObject.ADD
        apply(attachments=[held], label="attach existing basketball to gripper")
        if not ball_state()[1]:
            raise RuntimeError("MoveIt did not attach basketball; check --ee-link and the move_group log")
        attached = True

    def release_ball(position):
        detach_if_present()
        apply(world=[sphere(position, FRAME)], label="release basketball")

    def finish_swing():
        try:
            swing_ok[0] = wait_execution("arm swing execution")
        except Exception as error:
            swing_errors.append(error)
        finally:
            swing_done.set()

    def fetch_urdf():
        for owner in ("robot_state_publisher", "move_group"):
            client = node.create_client(GetParameters, owner+"/get_parameters", callback_group=callback_group)
            if not client.wait_for_service(timeout_sec=0.5):
                continue
            request = GetParameters.Request()
            request.names = ["robot_description"]
            response = wait_future(client.call_async(request), "robot description", 3.0)
            if response.values and response.values[0].string_value:
                return response.values[0].string_value
        raise RuntimeError("Could not read robot_description for the joint velocity limits")

    def fly_and_report(launch_position, launch_velocity):
        speed = math.sqrt(sum(c*c for c in launch_velocity))
        elevation = math.degrees(math.asin(max(-1.0, min(1.0, launch_velocity[2]/speed)))) if speed > 1e-9 else 0.0
        print(f"\n[arm] Released at ({launch_position[0]:.3f}, {launch_position[1]:.3f}, "
              f"{launch_position[2]:.3f}) m.", flush=True)
        print(f"[arm] MEASURED ball launch velocity: {speed:.2f} m/s at {elevation:.0f} deg elevation "
              f"(vx {launch_velocity[0]:+.2f}, vy {launch_velocity[1]:+.2f}, vz {launch_velocity[2]:+.2f}).",
              flush=True)
        flight = simulate_flight(launch_position, launch_velocity)
        end = flight[-1][1]
        print(f"[arm] Ball travels {math.hypot(end[0]-launch_position[0], end[1]-launch_position[1]):.2f} m "
              f"horizontally (x = {end[0]:.3f}, y = {end[1]:+.3f}, z = {end[2]:.3f}).", flush=True)
        judge_flight(flight)
        suggest_hoops(flight)
        print(f"\n[arm] Playing the flight at {args.slow}x speed...", flush=True)
        launch = time.monotonic()
        for t, position in flight:
            while rclpy.ok() and launch+t/args.slow-time.monotonic() > 0:
                time.sleep(min(0.02, max(0.0, launch+t/args.slow-time.monotonic())))
            if not rclpy.ok():
                raise RuntimeError("ROS stopped during ball animation")
            moveit.move_collision(id="basketball", position=tuple(position),
                                  quat_xyzw=(0., 0., 0., 1.), frame_id=FRAME)
        if not swing_done.wait(timeout=90):
            raise RuntimeError("Arm follow-through did not finish")
        if swing_errors:
            raise RuntimeError(f"Arm follow-through failed: {swing_errors[0]}")
        if not swing_ok[0]:
            raise RuntimeError("Arm follow-through execution failed")
        print(f"[arm] Shot complete. Resetting scene after {args.finish_pause:g}s.", flush=True)
        time.sleep(args.finish_pause)

    def simple_throw_sequence():
        nonlocal swing_thread
        limits = urdf_limits(fetch_urdf(), ARM_JOINTS)
        vmax = [args.vel_scale*l[2] for l in limits]
        amax = [args.accel_factor*v for v in vmax]
        print("[max] Joint velocity limits (rad/s): "+", ".join(f"{v:.2f}" for v in vmax), flush=True)
        base = current_joints()
        current = dict(zip(base.name, base.position))
        home = list(home_positions)
        if max(abs(current[name] - q) for name, q in zip(ARM_JOINTS, home)) > 0.02:
            raise RuntimeError("Arm is not at its saved straight pose; use --set-home after positioning it")
        joint_index = ARM_JOINTS.index(args.swing_joint)
        elbow_index = None if args.no_elbow or args.elbow_deg == 0 else ARM_JOINTS.index(args.elbow_joint)
        if elbow_index is None:
            print(f"[max] Simple throw: only {args.swing_joint} moves; all other arm joints stay fixed.", flush=True)
        else:
            print(f"[max] Comparing {args.swing_joint} alone with {args.swing_joint} + {args.elbow_joint}; "
                  "all other arm joint targets stay fixed.", flush=True)
            print(f"[elbow] Maximum total travel {args.elbow_deg:g} deg "
                  f"(up to {args.elbow_deg/2:g} deg each side of neutral).", flush=True)
        print(f"[max] Windup at most {args.windup_deg:g} deg, follow-through at most {args.follow_deg:g} deg.", flush=True)
        _, (ux, uy) = horizontal_unit(release, HOOP_CENTER)
        s_rel = args.release_frac

        def state_with(q):
            state = copy.deepcopy(base)
            values = dict(zip(state.name, state.position))
            values.update(zip(ARM_JOINTS, q))
            state.name, state.position = list(values), list(values.values())
            state.velocity, state.effort = [], []
            return state

        def path_valid(qa, qb):
            if not validity_client.service_is_ready():
                raise RuntimeError("check_state_validity is required to check the simple swing")
            steps = max(10, int(math.ceil(max(abs(b-a) for a, b in zip(qa, qb)) / math.radians(2))))
            for k in range(steps + 1):
                s = k/steps
                request = GetStateValidity.Request()
                request.robot_state.joint_state = state_with([a+s*(b-a) for a, b in zip(qa, qb)])
                request.robot_state.is_diff = True
                request.group_name = "arm"
                if not wait_future(validity_client.call_async(request), "swing collision check").valid:
                    return False
            return True

        def evaluate(qa, qb):
            _, _, v_s, _ = swing_profile(qa, qb, vmax, amax, s_rel, args.brake_mult, dt=1.0)
            delta = [b-a for a, b in zip(qa, qb)]
            qr = [a+s_rel*d for a, d in zip(qa, delta)]
            qd = [d*v_s for d in delta]
            h = 0.002
            p1 = ball_center(fk(state_with([q+h*w for q, w in zip(qr, qd)])))
            p0 = ball_center(fk(state_with([q-h*w for q, w in zip(qr, qd)])))
            vel = tuple((a-b)/(2*h) for a, b in zip(p1, p0))
            pos = tuple((a+b)/2 for a, b in zip(p1, p0))
            if pos[2] < TABLE_TOP+0.05 or vel[2] < 0:
                return None
            hit = landing_range(pos, vel, (ux, uy))
            if hit is None or hit[0] <= 0 or abs(hit[1]) > hit[0]*math.tan(math.radians(args.max_yaw)):
                return None
            return hit[0], pos, vel

        best_by_mode = {False: None, True: None}
        candidates = list(elbow_swing_candidates(
            home, limits, joint_index, args.windup_deg, args.follow_deg,
            elbow_index, args.elbow_deg, ARM_JOINTS))
        print(f"[max] Checking {min(len(candidates), args.search_iters)} bounded shoulder/elbow candidates...", flush=True)
        for i, (qa, qb) in enumerate(candidates[:args.search_iters]):
            uses_elbow = elbow_index is not None and abs(qb[elbow_index]-qa[elbow_index]) > 1e-8
            result = evaluate(qa, qb)
            previous = best_by_mode[uses_elbow]
            if (result and (previous is None or result[0] > previous[0])
                    and path_valid(home, qa) and path_valid(qa, qb)):
                best_by_mode[uses_elbow] = (result[0], qa, qb, result[1], result[2])
                mode = "shoulder + elbow" if uses_elbow else "shoulder only"
                print(f"[max]   candidate {i} ({mode}): predicted range {result[0]:.2f} m", flush=True)
        available = [result for result in best_by_mode.values() if result is not None]
        if not available:
            raise RuntimeError("No upward collision-free throw toward the hoop fits these bounded movements. "
                               "Use --no-elbow to compare the original mode, or adjust --release-frac")
        best = max(available, key=lambda result: result[0])
        for uses_elbow, result in best_by_mode.items():
            if result is not None:
                mode = "shoulder + elbow" if uses_elbow else "shoulder only"
                print(f"[compare] Best {mode}: predicted range {result[0]:.2f} m.", flush=True)
        selected_elbow = elbow_index is not None and abs(best[2][elbow_index]-best[1][elbow_index]) > 1e-8
        if selected_elbow:
            print("[elbow] Selected coordinated shoulder/elbow throw.", flush=True)
        elif elbow_index is not None:
            print("[elbow] Selected shoulder-only fallback; the tested elbow paths did not improve predicted range.", flush=True)
        _, qa, qb, pos, vel = best
        samples, t_rel, v_s, total = swing_profile(qa, qb, vmax, amax, s_rel, args.brake_mult)
        speed = math.sqrt(sum(c*c for c in vel))
        print(f"[max] Best swing: predicted release speed {speed:.2f} m/s, release at "
              f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}), swing lasts {total:.2f} s.", flush=True)

        delta = [b-a for a, b in zip(qa, qb)]

        def make_trajectory(start, finish, profile):
            return joint_trajectory_from_profile(start, finish, profile, ARM_JOINTS)

        traj = make_trajectory(qa, qb, samples)
        # Coordinate the selected joint targets along one checked path while
        # holding all unselected joints at their starting angles.
        windup_vmax = [args.arm_speed*limit[2] for limit in limits]
        windup_amax = [args.accel_factor*v for v in windup_vmax]
        windup_samples, _, _, _ = swing_profile(home, qa, windup_vmax, windup_amax,
                                                0.5, 1.0)
        windup = make_trajectory(home, qa, windup_samples)
        if not path_valid(home, qa) or not path_valid(qa, qb):
            raise RuntimeError("The scene changed; the straight-arm windup or swing is now in collision")
        print("[max] Moving the selected shoulder/elbow joints to windup...", flush=True)
        start_arm_execution(windup, "bounded throw windup")
        if not wait_execution("windup execution"):
            raise RuntimeError("Windup move failed")
        time.sleep(args.windup_pause)

        denom = sum(d*d for d in delta)

        def progress(state):
            vals = dict(zip(state.name, state.position))
            return sum((vals[n]-a)*d for n, a, d in zip(ARM_JOINTS, qa, delta))/denom

        trigger = s_rel-v_s*args.release_latency
        records = []
        print("[max] SWING!", flush=True)
        start_arm_execution(traj, "coordinated throw swing")
        swing_thread = threading.Thread(target=finish_swing, daemon=True)
        swing_thread.start()
        deadline = time.monotonic()+30
        while time.monotonic() < deadline and alive():
            state = current_joints()
            stamp = state.header.stamp.sec+state.header.stamp.nanosec*1e-9
            stamp = stamp if stamp > 0 else time.monotonic()
            if not records or stamp > records[-1][0]:
                records.append((stamp, state))
                del records[:-15]
            s = progress(state)
            if s >= s_rel-v_s*args.lead:
                request_gripper_open()
            if s >= trigger:
                break
            if swing_done.is_set():
                raise RuntimeError("Swing ended before the release point; the controller may have "
                                   "rejected or aborted the fast trajectory (check its tolerances)")
            time.sleep(0.002)
        else:
            raise RuntimeError("Timed out waiting for the release point")

        launch_position = ball_center(fk(records[-1][1]))
        release_ball(launch_position)
        history = [(stamp, ball_center(fk(st))) for stamp, st in records[-8:]]
        fly_and_report(launch_position, estimate_velocity(history, window=0.1))

    try:
        print("[arm] ROS executor started.", flush=True)
        if args.use_sim_time:
            initial_clock = node.get_clock().now().nanoseconds
            print("[arm] Waiting for advancing /clock...", flush=True)
            wait_for_condition(lambda: node.get_clock().now().nanoseconds > initial_clock,
                               args.startup_timeout, "advancing /clock; unpause simulation or omit --use-sim-time", alive)
        for client, name in ((apply_client, "apply_planning_scene"),
                             (fk_client, "compute_fk"),
                             (plan_client, "plan_kinematic_path"),
                             (cartesian_client, "compute_cartesian_path"),
                             (scene_client, "get_planning_scene"),
                             (cancel_client, "execute_trajectory cancellation")):
            print(f"[arm] Waiting for {name} service...", flush=True)
            wait_for_condition(client.service_is_ready, args.startup_timeout,
                               f"{name} service; start MoveIt simulation", alive)
        print(f"[arm] Waiting for all six arm joints on {args.joint_states}...", flush=True)
        wait_for_condition(lambda: joint_state[0] is not None, args.startup_timeout,
                           f"arm joint states on {args.joint_states}; check controller/joint-state publisher", alive)
        print("[reset] Cancelling any previous MoveIt execution...", flush=True)
        wait_future(cancel_client.call_async(CancelGoal.Request()), "cancel previous trajectory")
        time.sleep(0.3)
        reset_ball()
        values = dict(zip(current_joints().name, current_joints().position))
        home_positions = load_home_positions([values[name] for name in ARM_JOINTS], args)
        reset_arm()
        discover_gripper_links()
        print("[arm] Requesting current gripper pose...", flush=True)
        current = fk()
        print(f"[arm] Current tool position: ({current.position.x:.3f}, "
              f"{current.position.y:.3f}, {current.position.z:.3f})", flush=True)
        orientation = current.orientation
        start_center = tuple(release[i]-args.stroke*direction[i] for i in range(3))
        end_center = tuple(release[i]+args.follow*direction[i] for i in range(3))

        def tool_pose(center, quat):
            pose = Pose()
            local_offset = rotated_offset(args.ball_offset, quat)
            pose.position.x, pose.position.y, pose.position.z = (
                center[i]-local_offset[i] for i in range(3))
            pose.orientation = quat
            return pose

        # Hold the ball during planning too, so every candidate includes its
        # collision geometry. The ready pose is position-only by default.
        check_start_state()
        load_ball()
        check_start_state()

        if args.max_throw:
            simple_throw_sequence()
            return

        print("[arm] Checking ready pose, backswing and swing BEFORE moving...", flush=True)
        base_state = current_joints()
        choice = None
        # First ask IK to choose a natural wrist orientation. If that produces
        # no full stroke, try two tool-axis directions; all are collision-checked.
        candidates = [("free wrist", None)]
        for sign in (1.0, -1.0):
            dx, dy, dz = (sign*x for x in direction)
            yaw, pitch = math.atan2(dy, dx), math.acos(max(-1., min(1., dz)))
            quat = copy.deepcopy(orientation)
            quat.x = -math.sin(yaw/2)*math.sin(pitch/2)
            quat.y = math.cos(yaw/2)*math.sin(pitch/2)
            quat.z = math.sin(yaw/2)*math.cos(pitch/2)
            quat.w = math.cos(yaw/2)*math.cos(pitch/2)
            candidates.append(("tool axis forward" if sign > 0 else "tool axis backward", quat))
        for name, quat in candidates:
            print(f"[arm] Trying {name}...", flush=True)
            set_speed(args.arm_speed)
            target = tool_pose(end_center, orientation if quat is None else quat)
            target_xyz = (target.position.x, target.position.y, target.position.z)
            ready = plan_pose(pose=target if quat is not None else None,
                              position=target_xyz if quat is None else None,
                              start_state=base_state, label=f"ready-pose plan ({name})")
            if ready is None:
                continue
            ready_state = plan_end_state(ready, base_state)
            natural = fk(ready_state).orientation
            # Re-align a nonzero ball offset after position-only IK selects
            # its orientation; the requested BALL release coordinate is fixed.
            ready_ball = ball_center(fk(ready_state))
            if math.dist(ready_ball, end_center) > 0.003:
                ready = plan_pose(tool_pose(end_center, natural), start_state=base_state,
                                  label=f"offset-corrected ready plan ({name})")
                if ready is None:
                    continue
                ready_state = plan_end_state(ready, base_state)
                natural = fk(ready_state).orientation
            back = plan_pose(tool_pose(start_center, natural), cartesian=True,
                             start_state=ready_state, label=f"backswing plan ({name})")
            if back is None:
                continue
            back_state = plan_end_state(back, ready_state)
            # The swing is the only fast move: it carries the ball.
            set_speed(args.swing_speed)
            forward = plan_pose(tool_pose(end_center, natural), cartesian=True,
                                start_state=back_state, label=f"swing plan ({name})")
            set_speed(args.arm_speed)
            if forward is None:
                continue
            choice = ready, back, forward
            print(f"[arm] Complete motion is planned using {name}.", flush=True)
            break
        if choice is None:
            raise RuntimeError("No complete ready/backswing/swing plan for this release. "
                               "Try a lower --release, a shorter --stroke/--follow, or a different --angle. "
                               "Send the matching move_group terminal error for the rejected plan")
        ready, back, swing = choice
        print("[arm] Moving to ready pose before the backswing...", flush=True)
        start_arm_execution(ready, "ready-pose move")
        if not wait_execution("ready-pose execution"):
            raise RuntimeError("Ready-pose move failed")
        print("[arm] Ball attached. Drawing the arm back for the windup...", flush=True)
        start_arm_execution(back, "windup move")
        if not wait_execution("windup execution"):
            raise RuntimeError("Windup execution failed")
        print(f"[arm] Holding windup for {args.windup_pause:g}s, then swinging...", flush=True)
        time.sleep(args.windup_pause)

        # One straight swing passes THROUGH the release plane without stopping. The ball
        # leaves the gripper when the plane is crossed, with the velocity the arm
        # actually has at that moment (measured from its joint motion).
        p = ball_center(fk())
        progress = lambda q: sum((q[i]-release[i])*direction[i] for i in range(3))
        if progress(p) >= 0:
            raise RuntimeError("Windup is not behind the release plane")
        print("[arm] Swinging toward release...", flush=True)
        start_arm_execution(swing, "throw swing")
        swing_thread = threading.Thread(target=finish_swing, daemon=True)
        swing_thread.start()
        history = []
        launch_position, launch_velocity = None, None
        deadline = time.monotonic()+90
        previous_progress, previous_time = progress(p), time.monotonic()
        while rclpy.ok() and time.monotonic() < deadline:
            state = current_joints()
            stamp = state.header.stamp.sec + state.header.stamp.nanosec*1e-9
            sample_time = stamp if stamp > 0 else time.monotonic()
            p = ball_center(fk(state))
            if not history or sample_time > history[-1][0]:
                history.append((sample_time, p))
                del history[:-20]
            along = progress(p)
            now = time.monotonic()
            elapsed = now-previous_time
            forward_rate = (along-previous_progress)/elapsed if elapsed > 0 else 0.0
            if along >= 0 or (forward_rate > 0 and -along/forward_rate <= args.lead):
                request_gripper_open()
            previous_progress, previous_time = along, now
            if along >= 0:
                launch_velocity = estimate_velocity(history)
                launch_position = p
                release_ball(p)
                break
            if swing_done.is_set():
                raise RuntimeError("Swing ended before the release plane was crossed")
            time.sleep(0.005)
        else:
            raise RuntimeError("Timed out waiting for arm release crossing")

        speed = math.sqrt(sum(c*c for c in launch_velocity))
        elevation = math.degrees(math.asin(max(-1.0, min(1.0, launch_velocity[2]/speed)))) if speed > 1e-9 else 0.0
        print(f"\n[arm] Released at ({launch_position[0]:.3f}, {launch_position[1]:.3f}, "
              f"{launch_position[2]:.3f}) m.", flush=True)
        print(f"[arm] MEASURED ball launch velocity: {speed:.2f} m/s at {elevation:.0f} deg elevation "
              f"(vx {launch_velocity[0]:+.2f}, vy {launch_velocity[1]:+.2f}, vz {launch_velocity[2]:+.2f}).",
              flush=True)
        if speed < 0.05:
            print("[arm] The arm was almost stationary at release; the ball will simply drop. "
                  "Check --swing-speed and that the swing plan has a real stroke.", flush=True)
        flight = simulate_flight(launch_position, launch_velocity)
        judge_flight(flight)
        suggest_hoops(flight)
        print(f"\n[arm] Playing the flight at {args.slow}x speed...", flush=True)

        # The ball flies from where and how the arm released it. Absolute deadlines
        # avoid accumulating publisher delay on every animation frame.
        launch = time.monotonic()
        for t, position in flight:
            deadline = launch+t/args.slow
            while rclpy.ok():
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(remaining, 0.02))
            if not rclpy.ok():
                raise RuntimeError("ROS stopped during ball animation")
            moveit.move_collision(id="basketball", position=tuple(position),
                                  quat_xyzw=(0., 0., 0., 1.), frame_id=FRAME)
        if not swing_done.wait(timeout=90):
            raise RuntimeError("Arm follow-through did not finish")
        if swing_errors:
            raise RuntimeError(f"Arm follow-through failed: {swing_errors[0]}")
        if not swing_ok[0]:
            raise RuntimeError("Arm follow-through execution failed")
        print(f"[arm] Shot complete. Resetting scene after {args.finish_pause:g}s.", flush=True)
        time.sleep(args.finish_pause)
    finally:
        # On a failed plan/release, keep scene coordinates coherent. Do not
        # detach the held ball while an outstanding arm motion is running.
        try:
            if scene_touched:
                try:
                    pending = [event for event in execution_events if not event.is_set()]
                    if pending:
                        wait_future(cancel_client.call_async(CancelGoal.Request()), "cancel unfinished arm move")
                        for event in pending:
                            if not event.wait(timeout=10):
                                raise RuntimeError("Arm move did not confirm cancellation; skipping scene reset")
                    if swing_thread is not None and not swing_done.is_set():
                        wait_future(cancel_client.call_async(CancelGoal.Request()), "cancel unfinished swing")
                        if not swing_done.wait(timeout=10):
                            raise RuntimeError("Swing did not confirm cancellation; leaving scene untouched")
                    if gripper_thread is not None and gripper_thread.is_alive():
                        gripper_thread.join(timeout=args.plan_timeout+2)
                        if gripper_thread.is_alive():
                            wait_future(cancel_client.call_async(CancelGoal.Request()), "cancel unfinished gripper action")
                            gripper_thread.join(timeout=5)
                    reset_ball()
                    if not args.leave_at_finish:
                        reset_arm()
                except Exception as error:
                    # Do not hide the original planning/attachment failure.
                    print(f"[reset] Cleanup could not finish: {error}", flush=True)
        finally:
            if gripper_thread is not None:
                gripper_thread.join(timeout=2.0)
            executor.shutdown(timeout_sec=10.0)
            spin.join(timeout=5)
            if not spin.is_alive():
                arm_client.destroy()
                node.destroy_node()
            rclpy.try_shutdown()


def animate(points, slow, use_sim_time=False):
    """Ideal-arc ball-only animation; no arm motion."""
    import rclpy
    from pymoveit2 import MoveIt2
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    rclpy.init()
    node = Node("throw_animation", parameter_overrides=[
        Parameter("use_sim_time", Parameter.Type.BOOL, use_sim_time)])
    arm = MoveIt2(node=node, joint_names=ARM_JOINTS, base_link_name=FRAME,
                  end_effector_name=EE_LINK, group_name="arm",
                  callback_group=ReentrantCallbackGroup())
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        time.sleep(2)
        print("Animating the ideal ball arc (no arm motion).", flush=True)
        launch = time.monotonic()
        for t, position in points:
            while rclpy.ok() and time.monotonic() < launch+t/slow:
                time.sleep(min(0.02, max(0.0, launch+t/slow-time.monotonic())))
            if not rclpy.ok():
                raise RuntimeError("ROS stopped during ball animation")
            arm.move_collision(id="basketball", position=tuple(position),
                               quat_xyzw=(0., 0., 0., 1.), frame_id=FRAME)
    finally:
        executor.shutdown(timeout_sec=5.0)
        spin.join(timeout=2)
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--release", nargs=3, type=float, default=DEFAULT_RELEASE, metavar=("X", "Y", "Z"),
                        help="Ball-center release point in base_link (m)")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--throw", action="store_true", help="The arm throws the ball; its motion sets the launch velocity")
    mode.add_argument("--simple-throw", "--max-throw", dest="max_throw", action="store_true",
                      help="Bounded shoulder throw with a small elbow assist; --no-elbow uses the original candidate set")
    mode.add_argument("--animate", action="store_true", help="Animate only the IDEAL arc to the current hoop; no arm motion")
    parser.add_argument("--angle", type=float, default=30.0, help="Swing elevation in degrees above horizontal (5 to 85)")
    parser.add_argument("--swing-speed", type=float, default=1.0, help="Velocity/acceleration scaling for the swing (0 to 1)")
    parser.add_argument("--arm-speed", type=float, default=1.0, help="Scaling for the slow ready, windup and reset moves")
    parser.add_argument("--slow", type=float, default=0.25, help="Ball playback factor")
    parser.add_argument("--stroke", type=float, default=0.30, help="Windup distance before release in meters")
    parser.add_argument("--follow", type=float, default=0.30, help="Follow-through distance after release in meters")
    parser.add_argument("--windup-pause", type=float, default=0.6, help="Pause at the visible backswing in seconds")
    parser.add_argument("--finish-pause", type=float, default=2.0, help="Show the final ball position before resetting")
    parser.add_argument("--home-file", default=str(Path.home()/".ros"/"throw_check_start.json"), help="Saved starting joint pose")
    parser.add_argument("--set-home", action="store_true", help="Replace saved starting pose with current measured joints")
    parser.add_argument("--home-joints", nargs=6, type=float, metavar=("J1", "J2", "J3", "J4", "J5", "J6"), help="Explicit reset joint angles in radians")
    parser.add_argument("--leave-at-finish", action="store_true", help="Skip the final arm return; next run still resets first")
    parser.add_argument("--lead", type=float, default=0.15, help="Approximate lead time for finger opening in seconds")
    parser.add_argument("--no-gripper", action="store_true", help="Skip finger commands; still detach and release the ball")
    parser.add_argument("--ball-offset", nargs=3, type=float, default=(0., 0., 0.10), metavar=("X", "Y", "Z"), help="Ball center relative to tool frame")
    parser.add_argument("--touch-links", nargs="*", default=[], help="Gripper link names allowed to contact held ball")
    parser.add_argument("--ee-link", default=EE_LINK)
    parser.add_argument("--use-sim-time", action="store_true", help="Use an advancing /clock")
    parser.add_argument("--joint-states", default="/joint_states")
    parser.add_argument("--startup-timeout", type=float, default=15.0)
    parser.add_argument("--plan-timeout", type=float, default=30.0)
    # Small straight-arm swing options (--max-throw remains an alias)
    parser.add_argument("--swing-joint", choices=ARM_JOINTS, default="joint_2",
                        help="Joint to pivot for the throw; joint_2 is the default shoulder assumption")
    parser.add_argument("--windup-deg", type=float, default=60.0, help="Maximum backswing from the saved straight pose (degrees, up to 60)")
    parser.add_argument("--follow-deg", type=float, default=15.0, help="Maximum follow-through beyond the saved straight pose (degrees, up to 45)")
    parser.add_argument("--elbow-joint", choices=ARM_JOINTS, default="joint_3",
                        help="Additional elbow joint used during the shoulder throw")
    parser.add_argument("--elbow-deg", type=float, default=40.0,
                        help="Maximum total elbow travel, split equally around neutral (0 to 45 degrees)")
    parser.add_argument("--no-elbow", action="store_true", help="Use the original shoulder-only candidate set")
    parser.add_argument("--search-iters", type=int, default=150,
                        help="Maximum candidate paths; 150 includes both shoulder-only and elbow-assisted comparisons")
    parser.add_argument("--vel-scale", type=float, default=1.0, help="Multiplier on URDF joint velocity limits")
    parser.add_argument("--accel-factor", type=float, default=8.0, help="Joint accel limit = factor x velocity limit (rad/s^2)")
    parser.add_argument("--release-frac", type=float, default=0.60, help="Where along the swing the ball is released (0-1)")
    parser.add_argument("--brake-mult", type=float, default=2.0, help="Follow-through braking vs. acceleration")
    parser.add_argument("--max-yaw", type=float, default=25.0, help="Max sideways throw angle from the hoop direction (deg)")
    parser.add_argument("--release-latency", type=float, default=0.02, help="Release this many seconds early to offset latency")
    args = parser.parse_args()
    values = [*args.release, *args.ball_offset, args.slow, args.stroke, args.follow,
              args.lead, args.arm_speed, args.swing_speed, args.angle, args.startup_timeout,
              args.plan_timeout, args.windup_pause, args.finish_pause,
              args.vel_scale, args.accel_factor, args.release_frac, args.brake_mult,
              args.max_yaw, args.release_latency, args.windup_deg, args.follow_deg, args.elbow_deg,
              *(args.home_joints or [])]
    if not all(math.isfinite(x) for x in values):
        parser.error("All numeric motion arguments must be finite")
    if min(args.slow, args.stroke, args.follow, args.startup_timeout,
           args.plan_timeout) <= 0 or min(args.lead, args.windup_pause, args.finish_pause) < 0:
        parser.error("Playback, distances and timeouts must be positive; lead and pauses must be nonnegative")
    if not 0 < args.arm_speed <= 1 or not 0 < args.swing_speed <= 1:
        parser.error("arm-speed and swing-speed must be in (0, 1]")
    if not 5 <= args.angle <= 85:
        parser.error("angle must be between 5 and 85 degrees")
    if not 0 < args.vel_scale <= 1:
        parser.error("vel-scale must be in (0, 1]")
    if not 0 < args.windup_deg <= 60 or not 0 < args.follow_deg <= 45:
        parser.error("windup-deg must be in (0, 60] and follow-deg in (0, 45]")
    if not 0 <= args.elbow_deg <= 45:
        parser.error("elbow-deg must be between 0 and 45")
    if args.max_throw and not args.no_elbow and args.elbow_deg > 0 and args.elbow_joint == args.swing_joint:
        parser.error("elbow-joint and swing-joint must be different")
    if args.accel_factor <= 0 or args.brake_mult <= 0 or args.search_iters < 1:
        parser.error("vel-scale, accel-factor, brake-mult and search-iters must be positive")
    if not 0.2 <= args.release_frac <= 0.9:
        parser.error("release-frac must be between 0.2 and 0.9")
    if not 0 < args.max_yaw <= 89 or args.release_latency < 0:
        parser.error("max-yaw must be in (0, 89] and release-latency nonnegative")
    release = tuple(args.release)
    try:
        if args.throw or args.max_throw:
            reference = analyze(release, verbose=False)
            if reference:
                print(f"For reference: an ideal shot to the CURRENT hoop needs {reference[0]:.2f} m/s "
                      f"at {math.degrees(reference[1]):.0f} deg. The arm's measured release speed decides the real shot.")
            if args.max_throw:
                print("\n--- Shoulder throw: compare bounded elbow assist, swing, release ---", flush=True)
            else:
                print("\n--- Arm throw: the ball's launch velocity is measured from the arm ---", flush=True)
            run_throw(release, args)
        else:
            result = analyze(release)
            if result is None:
                return
            if args.animate:
                animate(result[2], args.slow, args.use_sim_time)
    except KeyboardInterrupt:
        print("Interrupted.")
    except (RuntimeError, ValueError) as error:
        parser.exit(1, f"throw_check: {error}\n")


if __name__ == "__main__":
    main()