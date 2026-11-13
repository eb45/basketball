# Basketball Scene (Kinova Gen3 Lite, MoveIt)

Separate from the midterm repo. Adds a table, ball, whiteboard, backboard, and
hoop to MoveIt's planning scene and checks that the arm can plan to the ball and
the hoop.

## Layout

- `ros2_ws/src/basketball_moveit/`: ROS 2 Python package
  - `basketball_moveit/scripts/basketball_scene.py`: the script (entry point `basketball_scene`)

## Build (inside the course container)

```bash
cd ~/workspaces/basketball/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

Terminal 1: `kinova-midterm` (wait for the arm to appear in Gazebo)
Terminal 2: `kinova-sim-moveit`
Terminal 3:

```bash
cd ~/workspaces/basketball/ros2_ws
source install/setup.bash
ros2 run basketball_moveit basketball_scene             # add scene only
ros2 run basketball_moveit basketball_scene --test      # plan to ball and hoop, no motion
ros2 run basketball_moveit basketball_scene --execute   # SIMULATION ONLY, moves the arm
```

Positions in the script are examples. Measure and check them in RViz.
