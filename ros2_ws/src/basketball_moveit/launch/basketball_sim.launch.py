"""Basketball simulation: the midterm station and arm, without the pen."""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from whiteboard_setup.station import STATION_MODEL, station_sdf

CAMERA_POSE = ("pose: {position: {x: -0.75, y: -1.05, z: 1.05}, "
               "orientation: {x: -0.0588, y: 0.1468, z: 0.3672, w: 0.9166}}")


def generate_launch_description() -> LaunchDescription:
    kinova = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("kortex_bringup"), "/launch/kortex_sim_control.launch.py"]),
        launch_arguments={
            "sim_gazebo": "true",
            "robot_type": "gen3_lite",
            "gripper": "gen3_lite_2f",
            "robot_name": "gen3_lite",
            "dof": "6",
            "launch_rviz": "false",
            "use_sim_time": "true",
            "robot_controller": "joint_trajectory_controller",
        }.items(),
    )
    station = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-string", station_sdf(), "-name", STATION_MODEL, "-allow_renaming", "false"],
        output="screen",
    )
    scene = Node(package="whiteboard_setup", executable="whiteboard_scene", output="screen",
                 parameters=[{"watch": True, "use_sim_time": True}])
    camera = ExecuteProcess(
        cmd=["bash", "-c",
             "for i in $(seq 30); do gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera "
             "--reptype gz.msgs.Boolean --timeout 2000 "
             f"--req '{CAMERA_POSE}' 2>/dev/null | grep -q true && exit 0; sleep 2; done"],
    )
    return LaunchDescription([
        kinova,
        TimerAction(period=5.0, actions=[station]),
        TimerAction(period=8.0, actions=[camera, scene]),
    ])