# project_cameras

A ROS 2 package for using the course's cameras in your final project: a USB webcam,
an Intel RealSense D435i, or an Intel RealSense D421. It has:

- `webcam.launch.py` and `realsense.launch.py`, which start the camera driver with the
  course's image sizes and topic names and publish the camera's position on your robot;
- `Camera`, a Python class you use inside your own node to get the latest image and
  depth as NumPy arrays and to turn a pixel into a 3D point in a robot frame such as
  `base_link`;
- `point_at_pixel`, a small node that uses `Camera` to print the depth and 3D position at
  one pixel, for checking that your camera works.

| Camera | Color | Depth | IMU |
|---|---|---|---|
| USB webcam | yes | no | no |
| RealSense D435i | yes | yes | yes |
| RealSense D421 | no (gray infrared image instead) | yes | no |

The D421 has no color camera. If your project needs color and depth together, use
the D435i. Its infrared image shows a pattern of dots: the camera projects them to
measure depth on plain surfaces.

## Where the camera plugs in

Course VMs have no USB access. Plug the camera into a lab computer and run the
camera, your code, and the Kinova driver there. The site guides cover the camera
side in more detail: [USB Webcam in ROS 2](https://mems-intro-to-robotics.github.io/guides/usb_webcam_ros2/)
and [RealSense D435i and D421 in ROS 2](https://mems-intro-to-robotics.github.io/guides/realsense_ros2/).

## 1. Add the package to your workspace

Clone it into your project's `ros2_ws/src/`, next to your own packages:

```bash
cd ~/workspaces/<your project>/ros2_ws/src
git clone https://github.com/MEMS-Intro-to-Robotics/project_cameras.git
```

To get later fixes, run `git pull` in `ros2_ws/src/project_cameras` and build again.

If your project folder is itself a Git repository, a clone inside it is a separate
repository, and `git add` does not commit its files (Git warns `adding embedded git
repository`). Use one of these instead, from the root of your project repository:

- Add it as a submodule, so teammates get it when they clone your project with
  `git clone --recurse-submodules`:

    ```bash
    git submodule add https://github.com/MEMS-Intro-to-Robotics/project_cameras.git ros2_ws/src/project_cameras
    ```

- Or clone it as above, delete its `.git` folder (`rm -rf ros2_ws/src/project_cameras/.git`),
  and commit the files as part of your project. Later fixes then have to be copied in by hand.

## 2. Start the container with the camera

Plug the camera in first. Add these options to the `docker run` command you use for
the Kinova:

| Camera | Options |
|---|---|
| Webcam | `--device /dev/video0` (use the camera's own `/dev/video` entry if it is not `video0`) |
| RealSense | `-v /dev:/dev --device-cgroup-rule "c 81:* rmw" --device-cgroup-rule "c 189:* rmw"` |

Run the camera, your nodes, and the Kinova driver in the same container. In testing,
a node in a second container on the same computer did not receive camera images.

## 3. Install the drivers and build

The course images do not include the camera drivers yet. In each new container, run:

```bash
apt-get update && apt-get install -y ros-jazzy-usb-cam ros-jazzy-realsense2-camera
```

Then build your workspace as usual:

```bash
cd ~/workspaces/<your project>/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 4. Start the camera

```bash
ros2 launch project_cameras webcam.launch.py
ros2 launch project_cameras realsense.launch.py model:=d435i
ros2 launch project_cameras realsense.launch.py model:=d421
```

Run one of these, in its own terminal. `ros2 launch project_cameras realsense.launch.py --show-args`
lists every argument.

### Topics

| Camera | Image | Depth (16UC1, millimeters) | Calibration for the depth |
|---|---|---|---|
| Webcam | `/camera/color/image_raw` (rgb8) | none | `/camera/color/camera_info` |
| D435i | `/camera/color/image_raw` (rgb8) | `/camera/aligned_depth_to_color/image_raw` | `/camera/aligned_depth_to_color/camera_info` |
| D421 | `/camera/infra1/image_rect_raw` (mono8) | `/camera/depth/image_rect_raw` | `/camera/depth/camera_info` |

For the D435i, the depth image is aligned to the color image: pixel (u, v) in one is
the same point as pixel (u, v) in the other. For the D421, the infrared image is
taken from the same viewpoint as the depth image and has the same size, so pixel (u, v)
in the infrared image is also pixel (u, v) in the depth image.

Other arguments for `realsense.launch.py`: `depth_profile` sets the depth image size
and frame rate (default `848x480x30`; see [On a Windows computer](#on-a-windows-computer));
`pointcloud:=true` adds a point cloud on
`/camera/depth/color/points`; `imu:=true` (D435i only) adds `/camera/imu`;
`initial_reset:=true` resets the camera at startup, which can help when no images
arrive.

## 5. Tell ROS where the camera is

To convert a pixel to a point in a robot frame, `Camera` uses the transform from that
frame to the camera. Both launch files publish it from the camera's pose on the robot,
which they take as arguments: `parent_frame` (the robot frame you
measure from), the position `x`, `y`, `z` in meters, and the rotation `roll`, `pitch`,
`yaw` in radians. They describe the camera body frame `camera_link`, whose x axis
points out of the lens, y to the camera's left, and z up.

A camera on a stand 0.5 m in front of the arm's base and 0.8 m up, tilted down by
0.6 rad (about 34 degrees) to look at the table:

```bash
ros2 launch project_cameras realsense.launch.py model:=d435i parent_frame:=base_link x:=0.5 z:=0.8 pitch:=0.6
```

Positive `pitch` tilts the lens down. To look back toward the arm, add `yaw:=3.1416`.
A camera mounted on the gripper uses `parent_frame:=end_effector_link` and its offset
from that frame; the Kinova driver must be running so that frame moves with the arm.

Check the pose in RViz: set the Fixed Frame to `base_link` and add a TF display;
`camera_link` should appear where the camera is. Calibrating the camera's pose against
the arm gives a more accurate pose than measuring it with a ruler.

## 6. Check the camera

With the camera's launch file running, in a second terminal:

```bash
ros2 run project_cameras point_at_pixel --ros-args -p model:=d435i
```

Use `model:=d421` or `model:=webcam` for the other cameras. Once a second, it prints
the depth at the center of the image and that point's position in `base_link`, for
example `pixel (320, 240): depth 0.742 m, point (0.912, -0.013, 0.154) m in base_link`.
Add `-p u:=100 -p v:=200` to choose another pixel. Move an object in front of the
camera and check that the numbers change the way you expect.

## 7. Use the camera in your node

```python
import rclpy
from rclpy.node import Node

from project_cameras.camera import Camera


class FindObject(Node):
    def __init__(self):
        super().__init__("find_object")
        self.camera = Camera.d435i(self)   # or Camera.d421(self), Camera.webcam(self)
        self.create_timer(0.5, self.step)

    def step(self):
        if self.camera.image is None or self.camera.depth is None:
            return                         # nothing received yet
        u, v = 320, 240                    # replace with the pixel your detection finds
        point = self.camera.pixel_to_frame(u, v, "base_link")
        if point is not None:
            self.get_logger().info(f"object at {point} in base_link")


def main():
    rclpy.init()
    rclpy.spin(FindObject())
```

Add `project_cameras` as an `<exec_depend>` in your package's `package.xml`.

What `Camera` gives you:

| Name | What it is |
|---|---|
| `camera.image` | Latest image: RGB `(rows, columns, 3)` uint8, or for the D421 the infrared image `(rows, columns)` uint8; `None` until the first image arrives |
| `camera.depth` | Latest depth in meters, `(rows, columns)` float32, NaN where there is no reading; `None` for a webcam or before the first image |
| `camera.info` | Latest calibration (`sensor_msgs/CameraInfo`) |
| `camera.depth_at(u, v)` | Depth (m) at pixel (u, v), the median of valid readings within 2 pixels; `None` if there are none |
| `camera.pixel_to_point(u, v)` | 3D point (m) in the camera's optical frame (z out of the lens, x right, y down) |
| `camera.pixel_to_frame(u, v, frame)` | 3D point (m) in `frame`, such as `base_link`; `None` if there is no depth there or no transform |
| `camera.ray(u, v)` | Direction (unit vector, optical frame) of the ray through pixel (u, v); `None` until the camera has a calibration. The RealSense cameras have a factory calibration. A webcam needs a calibration file, passed to `webcam.launch.py` as `camera_info_url:=file:///path/to/file.yaml` |

These update only while your node is spinning. Pixel coordinates follow the image:
`u` is the column (0 at the left) and `v` is the row (0 at the top), so
`camera.image[v, u]` is the pixel at (u, v).

## On a Windows computer

The RealSense also works on a Windows computer through Docker Desktop and
`usbipd-win`, with two extra steps. Attach the camera to WSL as described in the site's
webcam guide, then:

1. Load the Linux camera driver if the container has no `/dev/video*` entries. Without
   it, the RealSense driver prints `No RealSense devices were found!`. In testing it had
   to be loaded again each time Docker Desktop's WSL2 system restarted. In PowerShell:

    ```powershell
    docker run --rm --privileged -v /lib/modules:/lib/modules alpine sh -c "apk add -q kmod && modprobe uvcvideo"
    ```

2. Launch with a smaller depth image. In testing over this connection, depth images of
   640 x 480 and larger arrived incomplete and were dropped, and 480 x 270 arrived
   complete at 30 frames per second alongside the color image:

    ```bash
    ros2 launch project_cameras realsense.launch.py model:=d435i depth_profile:=480x270x30
    ```

   The aligned depth image is still 640 x 480, the size of the color image.

## Common problems

- `Package 'usb_cam' not found` or `Package 'realsense2_camera' not found`: install the drivers (step 3).
- `No RealSense devices were found!`, repeated: the driver has not detected the camera.
  Check the cable (USB 3 for the RealSense), the `docker run` options in step 2, and
  that no other program is using the camera. See the RealSense site guide.
- `point_at_pixel` keeps printing `Waiting for images`: the camera's launch file is
  not running, or `model` does not match the camera.
- `no transform to base_link`: nothing connects the camera's `parent_frame` to
  `base_link`. With the default `parent_frame:=base_link` this does not happen. With
  another frame, such as `end_effector_link`, the Kinova driver publishes the
  connection, so start it, or use `-p frame:=camera_link` to check the camera on its own.
- `pixel (...): no depth reading`: the surface may be too close to the camera, too
  far, shiny, or black. Try another pixel or object.
