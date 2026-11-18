"""RealSense D435i or D421: Intel's realsense2_camera driver with the course's settings, plus the camera's mount on the robot.

    ros2 launch project_cameras realsense.launch.py model:=d435i
    ros2 launch project_cameras realsense.launch.py model:=d421 parent_frame:=base_link x:=0.5 z:=0.8 pitch:=0.6

D435i (color, depth, IMU):
    /camera/color/image_raw (rgb8), /camera/color/camera_info
    /camera/aligned_depth_to_color/image_raw (16UC1, mm), /camera/aligned_depth_to_color/camera_info
        depth resampled to the color image, so pixel (u, v) in one matches (u, v) in the other
    /camera/imu (with imu:=true)
D421 (depth and infrared only; it has no color camera and no IMU):
    /camera/depth/image_rect_raw (16UC1, mm), /camera/depth/camera_info
    /camera/infra1/image_rect_raw (mono8), /camera/infra1/camera_info
        the left infrared image, from the same viewpoint as the depth image
Both, with pointcloud:=true: /camera/depth/color/points (D421: without color)

The driver publishes camera_link -> its optical frames; this launch adds parent_frame -> camera_link.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from project_cameras.mount import mount_arguments, mount_transform

MODELS = {
    "d435i": {
        "enable_color": "true",
        "rgb_camera.color_profile": "640x480x30",
        "align_depth.enable": "true",
        "enable_infra1": "false",
    },
    "d421": {
        "enable_color": "false",
        "align_depth.enable": "false",
        "enable_infra1": "true",
    },
}


def driver(context) -> list:
    model = LaunchConfiguration("model").perform(context)
    imu = LaunchConfiguration("imu").perform(context) == "true"
    if imu and model != "d435i":
        raise RuntimeError("imu:=true needs model:=d435i; the D421 has no IMU.")
    depth_profile = LaunchConfiguration("depth_profile").perform(context)
    args = {
        **MODELS[model],
        "depth_module.depth_profile": depth_profile,
        "depth_module.infra_profile": depth_profile,
        "camera_namespace": "",
        "camera_name": "camera",
        "enable_depth": "true",
        "pointcloud.enable": LaunchConfiguration("pointcloud").perform(context),
        "initial_reset": LaunchConfiguration("initial_reset").perform(context),
        "enable_gyro": str(imu).lower(),
        "enable_accel": str(imu).lower(),
        "unite_imu_method": "2" if imu else "0",
    }
    rs_launch = Path(get_package_share_directory("realsense2_camera")) / "launch" / "rs_launch.py"
    # rs_launch.py passes every launch configuration it receives to the driver as a parameter, and warns
    # about each one it does not know, so it gets only its own arguments (forwarding=False).
    return [GroupAction(forwarding=False, launch_configurations=args,
                        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(str(rs_launch)))])]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("model", choices=list(MODELS), description="d435i or d421"),
        # Over WSL2 and usbipd (a Windows laptop), depth frames of 848x480 and 640x480 arrived
        # incomplete and were dropped; 480x270x30 arrived complete at 29 Hz with color running (2026-11-12).
        DeclareLaunchArgument("depth_profile", default_value="848x480x30",
                              description="Depth (and D421 infrared) width x height x fps; use 480x270x30 on Windows"),
        DeclareLaunchArgument("pointcloud", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("imu", default_value="false", choices=["true", "false"], description="D435i only"),
        DeclareLaunchArgument("initial_reset", default_value="false", choices=["true", "false"],
                              description="Reset the camera before starting; try it if no images arrive"),
        *mount_arguments(),
        OpaqueFunction(function=driver),
        mount_transform(),
    ])
