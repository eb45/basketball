"""Real camera: RealSense D435i, its mount on the robot, and the ball detector.

    ros2 launch basketball_moveit realsense_ball.launch.py \
        x:=0.24 y:=0.0 z:=1.0 pitch:=1.5708 ball_diameter:=0.08

x, y, z, roll, pitch, yaw are the pose of camera_link in parent_frame (meters and
radians). camera_link has x out of the lens, so pitch:=1.5708 looks straight down.
The numbers above are only an example: measure yours. x, y, z, pitch and
ball_diameter have no defaults on purpose, because a wrong value would still give
a believable but wrong ball position.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

MOUNT_REQUIRED = ("x", "y", "z", "pitch")
PASSED_TO_CAMERA = ("parent_frame", *MOUNT_REQUIRED, "roll", "yaw", "depth_profile", "initial_reset")


def generate_launch_description() -> LaunchDescription:
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("project_cameras"), "/launch/realsense.launch.py"]),
        launch_arguments={
            "model": "d435i",
            **{name: LaunchConfiguration(name) for name in PASSED_TO_CAMERA},
        }.items(),
    )
    detector = Node(
        package="basketball_moveit",
        executable="ball_detector",
        output="screen",
        parameters=[{
            "camera_source": "realsense",
            "ball_diameter": ParameterValue(LaunchConfiguration("ball_diameter"), value_type=float),
            "update_scene": ParameterValue(LaunchConfiguration("update_scene"), value_type=bool),
        }],
    )
    return LaunchDescription([
        *[DeclareLaunchArgument(name, description="camera_link pose in parent_frame (m, rad)")
          for name in MOUNT_REQUIRED],
        DeclareLaunchArgument("ball_diameter", description="Measured ball diameter (m)"),
        DeclareLaunchArgument("parent_frame", default_value="base_link"),
        DeclareLaunchArgument("roll", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("depth_profile", default_value="848x480x30"),
        DeclareLaunchArgument("initial_reset", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("update_scene", default_value="false", choices=["true", "false"]),
        camera,
        detector,
    ])