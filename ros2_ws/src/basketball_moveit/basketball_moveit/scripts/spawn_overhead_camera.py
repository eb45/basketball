"""Spawn an overhead camera and an orange ball in Gazebo, and bridge the image to ROS.

MoveIt's planning-scene ball is not a Gazebo object, so no camera can see it.
This puts a real (static) ball in the simulation for the camera to look at.

Positions in vision.py are in base_link. The arm sits BASE_Z_WORLD above the
Gazebo world origin, so heights are shifted by that amount when spawning.

Run after kinova-midterm has the arm up:
    ros2 run basketball_moveit spawn_overhead_camera
Keep it running: it also runs the Gazebo-to-ROS image bridge.
"""

import argparse
import subprocess
import sys

from basketball_moveit.scripts import vision

# World height of base_link (from: gz model -m gen3_lite -p).
BASE_Z_WORLD = 0.3

CAMERA_SDF = """<sdf version="1.9">
  <model name="overhead_camera">
    <static>true</static>
    <link name="link">
      <sensor name="camera" type="camera">
        <always_on>true</always_on>
        <update_rate>10</update_rate>
        <topic>overhead_camera/image</topic>
        <camera>
          <horizontal_fov>{hfov}</horizontal_fov>
          <image>
            <width>{w}</width>
            <height>{h}</height>
            <format>R8G8B8</format>
          </image>
          <clip><near>0.05</near><far>5</far></clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>"""

BALL_SDF = """<sdf version="1.9">
  <model name="gz_ball">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><sphere><radius>{r}</radius></sphere></geometry>
        <material>
          <ambient>1 0.45 0 1</ambient>
          <diffuse>1 0.45 0 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry><sphere><radius>{r}</radius></sphere></geometry>
      </collision>
    </link>
  </model>
</sdf>"""


def spawn(sdf, name, x, y, z, pitch=0.0):
    cmd = [
        "ros2", "run", "ros_gz_sim", "create",
        "-string", sdf, "-name", name,
        "-x", str(x), "-y", str(y), "-z", str(z), "-P", str(pitch),
    ]
    print(f"Spawning {name} at world ({x:.3f}, {y:.3f}, {z:.3f}) ...")
    return subprocess.run(cmd).returncode == 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ball-x", type=float, default=vision.BALL_XY[0])
    parser.add_argument("--ball-y", type=float, default=vision.BALL_XY[1])
    parser.add_argument("--no-ball", action="store_true", help="only spawn the camera")
    parser.add_argument("--no-bridge", action="store_true", help="do not start the image bridge")
    args = parser.parse_args()

    # The camera hangs at CAM_POS (base_link), which is also what the detector assumes.
    cx, cy, cz = vision.CAM_POS
    camera = CAMERA_SDF.format(hfov=vision.HFOV, w=vision.IMAGE_W, h=vision.IMAGE_H)
    # Pitch 90 degrees points the camera straight down.
    if not spawn(camera, "overhead_camera", cx, cy, cz + BASE_Z_WORLD, 1.5708):
        sys.exit("Could not spawn the camera. Is kinova-midterm running?")
    if not args.no_ball:
        ball = BALL_SDF.format(r=vision.BALL_DIAMETER / 2)
        if not spawn(ball, "gz_ball", args.ball_x, args.ball_y, vision.BALL_Z + BASE_Z_WORLD):
            sys.exit("Could not spawn the ball.")
    if args.no_bridge:
        return

    print("Bridging /overhead_camera/image to ROS. Ctrl+C to stop.")
    bridge = subprocess.Popen([
        "ros2", "run", "ros_gz_bridge", "parameter_bridge",
        "/overhead_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
    ])
    try:
        bridge.wait()
    except KeyboardInterrupt:
        bridge.terminate()


if __name__ == "__main__":
    main()