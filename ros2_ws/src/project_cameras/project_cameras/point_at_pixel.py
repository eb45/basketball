"""Example and camera check: once a second, print the depth and 3D position at one pixel.

    ros2 run project_cameras point_at_pixel --ros-args -p model:=d435i
    ros2 run project_cameras point_at_pixel --ros-args -p model:=d421 -p frame:=base_link -p u:=100 -p v:=200

With a depth camera it prints the pixel's depth and its 3D point in `frame`. With a
webcam it prints the image size and the pixel's color. u and v default to the image center.
Start the camera's launch file first.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from project_cameras.camera import Camera


class PointAtPixel(Node):
    def __init__(self) -> None:
        super().__init__("point_at_pixel")
        self.model = self.declare_parameter("model", "d435i").value
        self.frame = self.declare_parameter("frame", "base_link").value
        self.u = self.declare_parameter("u", -1).value
        self.v = self.declare_parameter("v", -1).value
        presets = {"d435i": Camera.d435i, "d421": Camera.d421, "webcam": Camera.webcam}
        if self.model not in presets:
            raise SystemExit(f"model must be one of {', '.join(presets)}, not {self.model!r}.")
        self.camera = presets[self.model](self)
        self.create_timer(1.0, self.report)

    def report(self) -> None:
        cam = self.camera
        if cam.image is None or (self.model != "webcam" and cam.depth is None):
            self.get_logger().info("Waiting for images. Is the camera's launch file running?")
            return
        rows, cols = cam.image.shape[:2]
        u = self.u if self.u >= 0 else cols // 2
        v = self.v if self.v >= 0 else rows // 2
        if not (0 <= u < cols and 0 <= v < rows):
            self.get_logger().info(f"pixel ({u}, {v}) is outside the {cols}x{rows} image")
            return
        if self.model == "webcam":
            self.get_logger().info(f"image {cols}x{rows}; pixel ({u}, {v}) RGB {tuple(int(c) for c in cam.image[v, u])}")
            return
        d = cam.depth_at(u, v)
        if d is None:
            self.get_logger().info(f"pixel ({u}, {v}): no depth reading")
            return
        p = cam.pixel_to_frame(u, v, self.frame)
        where = f"({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) m in {self.frame}" if p else f"(no transform to {self.frame})"
        self.get_logger().info(f"pixel ({u}, {v}): depth {d:.3f} m, point {where}")


def main() -> None:
    rclpy.init()
    try:
        rclpy.spin(PointAtPixel())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()
