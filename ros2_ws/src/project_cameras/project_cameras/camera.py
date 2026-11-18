"""Camera: the latest image and depth from one camera as NumPy arrays, and pixels as 3D points on the robot.

Use it inside your own node:

    from project_cameras.camera import Camera

    class MyNode(Node):
        def __init__(self):
            super().__init__("my_node")
            self.camera = Camera.d435i(self)      # or Camera.d421(self), Camera.webcam(self)
            self.create_timer(0.5, self.step)

        def step(self):
            if self.camera.image is None:
                return                            # nothing received yet
            point = self.camera.pixel_to_frame(320, 240, "base_link")
            if point is not None:
                x, y, z = point                   # meters, in base_link

`image` is the latest color image as RGB (rows, columns, 3) uint8, or for the D421 the
infrared image as (rows, columns) uint8. `depth` is the latest depth image in meters as
float32, NaN where the camera has no reading. Both are None until the first message.
The D435i preset uses the depth image aligned to the color image, so depth[v, u] is the
depth at image pixel (u, v); the D421's infrared image is from the depth image's viewpoint.

The node must be spinning (rclpy.spin) for these to update.
"""

from __future__ import annotations

import math

import numpy as np
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs  # noqa: F401  registers PointStamped with Buffer.transform

# Image encodings this helper reads: NumPy element type and channels per pixel. The
# conversion is done here rather than with cv_bridge, because the Kinova course image's
# Python environment has NumPy 2, which its cv_bridge cannot be imported with (2026-11-12).
ENCODINGS = {
    "rgb8": (np.uint8, 3), "bgr8": (np.uint8, 3), "mono8": (np.uint8, 1),
    "16UC1": (np.uint16, 1), "mono16": (np.uint16, 1), "32FC1": (np.float32, 1),
}


def image_to_array(msg: Image) -> np.ndarray:
    """An Image message as a (rows, columns) or (rows, columns, 3) array; bgr8 is returned as RGB."""
    if msg.encoding not in ENCODINGS:
        raise ValueError(f"Image encoding {msg.encoding!r} is not one of {', '.join(ENCODINGS)}.")
    dtype, channels = ENCODINGS[msg.encoding]
    dtype = np.dtype(dtype).newbyteorder(">" if msg.is_bigendian else "<")
    row = np.frombuffer(bytes(msg.data), dtype=dtype).reshape(msg.height, msg.step // dtype.itemsize)
    pixels = row[:, :msg.width * channels]
    array = pixels.reshape(msg.height, msg.width, channels) if channels > 1 else pixels
    if msg.encoding == "bgr8":
        array = array[..., ::-1]
    return np.ascontiguousarray(array, dtype=dtype.newbyteorder("="))


class Camera:
    def __init__(self, node: Node, image_topic: str | None, depth_topic: str | None, info_topic: str) -> None:
        """Subscribe to one camera's topics. info_topic is the calibration for the depth image (or the image, without depth)."""
        self.node = node
        self.image: np.ndarray | None = None
        self.depth: np.ndarray | None = None
        self.info: CameraInfo | None = None
        self._depth_stamp = None
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, node)
        if image_topic:
            node.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)
        if depth_topic:
            node.create_subscription(Image, depth_topic, self._on_depth, qos_profile_sensor_data)
        node.create_subscription(CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)

    @classmethod
    def d435i(cls, node: Node) -> Camera:
        """RealSense D435i from realsense.launch.py model:=d435i: color image and depth aligned to it."""
        return cls(node, "/camera/color/image_raw", "/camera/aligned_depth_to_color/image_raw",
                   "/camera/aligned_depth_to_color/camera_info")

    @classmethod
    def d421(cls, node: Node) -> Camera:
        """RealSense D421 from realsense.launch.py model:=d421: infrared image and depth (no color)."""
        return cls(node, "/camera/infra1/image_rect_raw", "/camera/depth/image_rect_raw",
                   "/camera/depth/camera_info")

    @classmethod
    def webcam(cls, node: Node) -> Camera:
        """USB webcam from webcam.launch.py: color image only, no depth."""
        return cls(node, "/camera/color/image_raw", None, "/camera/color/camera_info")

    def _on_image(self, msg: Image) -> None:
        self.image = image_to_array(msg)

    def _on_depth(self, msg: Image) -> None:
        depth = image_to_array(msg).astype(np.float32)
        if msg.encoding in ("16UC1", "mono16"):
            depth[depth == 0] = np.nan  # 0 means no reading
            depth /= 1000.0             # millimeters to meters
        depth[~np.isfinite(depth)] = np.nan
        self.depth = depth
        self._depth_stamp = msg.header.stamp

    def _on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def depth_at(self, u: int, v: int, window: int = 2) -> float | None:
        """Depth (m) at pixel (u, v): the median of the valid readings within `window` pixels, or None if there are none."""
        if self.depth is None:
            return None
        rows, cols = self.depth.shape
        if not (0 <= u < cols and 0 <= v < rows):
            return None
        patch = self.depth[max(v - window, 0):v + window + 1, max(u - window, 0):u + window + 1]
        valid = patch[np.isfinite(patch)]
        return float(np.median(valid)) if valid.size else None

    def pixel_to_point(self, u: int, v: int) -> tuple[float, float, float] | None:
        """The 3D point (m) that pixel (u, v) sees, in the camera's optical frame (z out of the lens, x right, y down).

        None if there is no depth reading there or no calibration has arrived yet.
        """
        d = self.depth_at(u, v)
        if d is None or self.info is None:
            return None
        fx, fy, cx, cy = self.info.k[0], self.info.k[4], self.info.k[2], self.info.k[5]
        return (float((u - cx) * d / fx), float((v - cy) * d / fy), d)

    def pixel_to_frame(self, u: int, v: int, frame: str = "base_link",
                       timeout_s: float = 0.1) -> tuple[float, float, float] | None:
        """The 3D point (m) that pixel (u, v) sees, in `frame`, at the time the depth image was taken.

        None if there is no depth there, or if the transform is not available (check that the
        mount transform and the robot's transforms are being published).
        """
        point = self.pixel_to_point(u, v)
        if point is None:
            return None
        msg = PointStamped()
        msg.header.frame_id = self.info.header.frame_id
        msg.header.stamp = self._depth_stamp
        msg.point.x, msg.point.y, msg.point.z = point
        try:
            out = self._tf.transform(msg, frame, timeout=Duration(seconds=timeout_s))
        except TransformException as e:
            self.node.get_logger().warn(f"No transform from {msg.header.frame_id} to {frame}: {e}",
                                        throttle_duration_sec=2.0)
            return None
        return (float(out.point.x), float(out.point.y), float(out.point.z))

    def ray(self, u: int, v: int) -> tuple[float, float, float] | None:
        """Unit vector, in the camera's optical frame, of the ray through pixel (u, v). Works without depth (webcam).

        Needs a calibrated camera_info (a webcam without a calibration file has no focal length).
        """
        if self.info is None or self.info.k[0] == 0.0:
            return None
        fx, fy, cx, cy = self.info.k[0], self.info.k[4], self.info.k[2], self.info.k[5]
        x, y = (u - cx) / fx, (v - cy) / fy
        n = math.sqrt(x * x + y * y + 1.0)
        return (x / n, y / n, 1.0 / n)
