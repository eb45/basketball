# Basketball Scene (Kinova Gen3 Lite, ROS 2 Jazzy)

Overhead camera + ball detector + MoveIt planning scene. Runs in the course Docker image.

## Setup (once)

On the VM:
```bash
cd ~/workspaces && git clone https://github.com/eb45/basketball.git
xhost +local:
docker run --rm -it --name kinova --net=host --gpus all -e DISPLAY=$DISPLAY -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v ~/workspaces:/root/workspaces ghcr.io/mems-intro-to-robotics/mems-robotics-toolkit:kinova-jazzy-latest
```

In the container:
```bash
cd /root/workspaces/basketball/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

Terminal 1 is the container from `docker run`. For terminals 2 to 7, run this on the VM first:
```bash
docker exec -it kinova bash
cd /root/workspaces/basketball/ros2_ws && source install/setup.bash
```

| # | Command |
|---|---|
| 1 | `ros2 launch basketball_moveit basketball_sim.launch.py` |
| 2 | `kinova-sim-moveit` |
| 3 | `ros2 run basketball_moveit spawn_overhead_camera` |
| 4 | `ros2 run basketball_moveit ball_detector` |
| 5 | `ros2 run rqt_image_view rqt_image_view /ball_detector/debug` |
| 6 | `ros2 run basketball_moveit check_ball --trials 5` |
| 7 | `ros2 run basketball_moveit basketball_scene`|

Start them in order, and wait for Gazebo to finish loading before terminal 3.

In a new terminal, to run throw_checker.py and simulate the launch:

```bash
cd ~/workspaces/basketball/ros2_ws/src/basketball_moveit/basketball_moveit
```

```bash
python3 -m final_project.throw_check --simple-throw
```
The motion of the ball is simulated to be as realistic as possible using The ball's position when released, its velocity and direction (calculated from the simulated arm's movement), and gavity to calculate the resulting arc. 

## RViz

Set Fixed Frame to `base_link`. Then Add > By topic > `/ball_marker` as Marker.

The camera image is shown in terminal 5 (`rqt_image_view`) instead of RViz, because RViz closes when an Image display is added. The marker display is lost when RViz closes. Use File > Save Config As to keep it.

## Topics

| Topic | Content |
|---|---|
| `/overhead_camera/image` | Raw camera image |
| `/ball_position` | Ball position in `base_link`, meters |
| `/ball_marker` | Orange sphere for RViz |
| `/ball_detector/debug` | Annotated image |