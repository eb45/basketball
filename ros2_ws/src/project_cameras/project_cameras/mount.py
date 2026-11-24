"""Launch arguments and the static transform that place a camera on the robot.

Both launch files use these. The camera's position is given by the launch arguments
parent_frame, x, y, z (meters) and roll, pitch, yaw (radians): the pose of the camera's
body frame, camera_link, in parent_frame. camera_link has x out of the lens, y to the
camera's left, and z up, the same as a robot's base_link.
"""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MOUNT_ARGS = {
    "parent_frame": ("base_link", "Robot frame the camera is mounted relative to, such as base_link or end_effector_link"),
    "x": ("0.0", "camera_link position in parent_frame (m)"),
    "y": ("0.0", "camera_link position in parent_frame (m)"),
    "z": ("0.0", "camera_link position in parent_frame (m)"),
    "roll": ("0.0", "camera_link rotation about parent_frame x (rad)"),
    "pitch": ("0.0", "camera_link rotation about y (rad); positive tilts the lens down"),
    "yaw": ("0.0", "camera_link rotation about z (rad)"),
}


def mount_arguments() -> list[DeclareLaunchArgument]:
    return [DeclareLaunchArgument(name, default_value=default, description=text)
            for name, (default, text) in MOUNT_ARGS.items()]


def mount_transform() -> Node:
    """Publish parent_frame -> camera_link from the launch arguments."""
    args = []
    for name in ("x", "y", "z", "roll", "pitch", "yaw"):
        args += [f"--{name}", LaunchConfiguration(name)]
    return Node(
        package="tf2_ros", executable="static_transform_publisher", name="camera_mount", output="log",
        arguments=[*args, "--frame-id", LaunchConfiguration("parent_frame"), "--child-frame-id", "camera_link"],
    )
