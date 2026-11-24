"""USB webcam: the usb_cam driver reading the camera's MJPEG stream, plus the camera's mount on the robot.

    ros2 launch project_cameras webcam.launch.py
    ros2 launch project_cameras webcam.launch.py video_device:=/dev/video2 parent_frame:=base_link x:=0.5 z:=0.8 pitch:=0.6

Topics: /camera/color/image_raw (rgb8) and /camera/color/camera_info.
Frames: parent_frame -> camera_link -> camera_color_optical_frame (z out of the lens, x right, y down).

usb_cam with MJPEG is used because v4l2_camera cannot read MJPEG, and its YUYV images
arrived incomplete through WSL2 on the course's test laptop (site guide, USB Webcam in ROS 2).
"""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from project_cameras.mount import mount_arguments, mount_transform


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("video_device", default_value="/dev/video0"),
        DeclareLaunchArgument("width", default_value="640"),
        DeclareLaunchArgument("height", default_value="480"),
        DeclareLaunchArgument("camera_info_url", default_value="",
                              description="Calibration file, such as file:///root/workspaces/webcam.yaml"),
        *mount_arguments(),
        Node(
            package="usb_cam", executable="usb_cam_node_exe", name="webcam", namespace="camera/color",
            output="screen",
            parameters=[{
                "video_device": LaunchConfiguration("video_device"),
                "pixel_format": "mjpeg2rgb",
                "image_width": LaunchConfiguration("width"),
                "image_height": LaunchConfiguration("height"),
                "framerate": 30.0,
                "frame_id": "camera_color_optical_frame",
                "camera_name": "webcam",
                "camera_info_url": LaunchConfiguration("camera_info_url"),
            }],
        ),
        mount_transform(),
        # camera_link (x forward) to the optical frame (z forward, x right, y down).
        Node(
            package="tf2_ros", executable="static_transform_publisher", name="webcam_optical", output="log",
            arguments=["--roll", str(-math.pi / 2), "--yaw", str(-math.pi / 2),
                       "--frame-id", "camera_link", "--child-frame-id", "camera_color_optical_frame"],
        ),
    ])
