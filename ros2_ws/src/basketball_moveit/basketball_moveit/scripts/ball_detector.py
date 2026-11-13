"""ROS node: find the ball in the overhead camera image and publish where it is.

Publishes
  /ball_position        geometry_msgs/PointStamped, base_link, meters
  /ball_marker          visualization_msgs/Marker, orange sphere for RViz
  /ball_detector/debug  sensor_msgs/Image, camera image with the detection drawn

Optional: --ros-args -p update_scene:=true moves the "basketball" collision
sphere in MoveIt's planning scene to the detected position.
"""

import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker

from basketball_moveit.scripts import vision

FRAME = "base_link"


class BallDetector(Node):
    def __init__(self):
        super().__init__("ball_detector")
        self.declare_parameter("image_topic", "/overhead_camera/image")
        self.declare_parameter("hsv_low", list(vision.HSV_LOW))
        self.declare_parameter("hsv_high", list(vision.HSV_HIGH))
        self.declare_parameter("cam_pos", list(vision.CAM_POS))
        self.declare_parameter("ball_z", vision.BALL_Z)
        self.declare_parameter("hfov", vision.HFOV)
        self.declare_parameter("update_scene", False)

        self.pos_pub = self.create_publisher(PointStamped, "/ball_position", 10)
        self.marker_pub = self.create_publisher(Marker, "/ball_marker", 10)
        self.debug_pub = self.create_publisher(Image, "/ball_detector/debug", 1)
        self.create_subscription(
            Image, self.get_parameter("image_topic").value, self.on_image,
            qos_profile_sensor_data,
        )

        self.moveit2 = None
        self.last_scene_xy = None
        if self.get_parameter("update_scene").value:
            from pymoveit2 import MoveIt2
            self.moveit2 = MoveIt2(
                node=self,
                joint_names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
                base_link_name=FRAME,
                end_effector_name="end_effector_link",
                group_name="arm",
            )
        self.get_logger().info(
            f"Listening on {self.get_parameter('image_topic').value}. "
            "Publishing /ball_position and /ball_marker."
        )

    def to_bgr(self, msg):
        """Convert a ROS Image to a BGR numpy array without cv_bridge."""
        if msg.encoding not in ("rgb8", "bgr8"):
            raise ValueError(f"Unsupported image encoding {msg.encoding}")
        rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        img = rows[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else img.copy()

    def on_image(self, msg):
        try:
            bgr = self.to_bgr(msg)
        except ValueError as err:
            self.get_logger().error(str(err), throttle_duration_sec=5.0)
            return

        u, v, radius, _ = vision.find_ball(
            bgr,
            tuple(self.get_parameter("hsv_low").value),
            tuple(self.get_parameter("hsv_high").value),
        )
        if u is None:
            self.get_logger().warn(
                "No ball found. Check the color range (hsv_low / hsv_high).",
                throttle_duration_sec=3.0,
            )
            self.publish_debug(bgr, msg.header)
            return

        intr = vision.intrinsics(msg.width, msg.height, self.get_parameter("hfov").value)
        x, y, z = vision.pixel_to_world(
            u, v,
            cam_pos=tuple(self.get_parameter("cam_pos").value),
            plane_z=self.get_parameter("ball_z").value,
            intr=intr,
        )

        point = PointStamped()
        point.header.stamp = msg.header.stamp
        point.header.frame_id = FRAME
        point.point.x, point.point.y, point.point.z = x, y, z
        self.pos_pub.publish(point)
        self.publish_marker(point)
        self.get_logger().info(
            f"Ball at x={x:.3f} y={y:.3f} z={z:.3f} m (pixel {u:.0f}, {v:.0f})",
            throttle_duration_sec=1.0,
        )

        cv2.circle(bgr, (int(u), int(v)), int(radius), (0, 255, 0), 2)
        cv2.drawMarker(bgr, (int(u), int(v)), (0, 255, 0), cv2.MARKER_CROSS, 16, 2)
        cv2.putText(bgr, f"x={x:.2f} y={y:.2f}", (int(u) + 10, int(v) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        self.publish_debug(bgr, msg.header)
        self.update_scene(x, y, z)

    def publish_debug(self, bgr, header):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        out = Image()
        out.header = header
        out.height, out.width = rgb.shape[:2]
        out.encoding = "rgb8"
        out.step = out.width * 3
        out.data = rgb.tobytes()
        self.debug_pub.publish(out)

    def publish_marker(self, point):
        marker = Marker()
        marker.header = point.header
        marker.ns = "ball"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = vision.BALL_DIAMETER
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.45, 0.0, 0.8
        self.marker_pub.publish(marker)

    def update_scene(self, x, y, z):
        """Move the planning-scene ball to the detection if it moved over 1 cm."""
        if self.moveit2 is None:
            return
        if self.last_scene_xy and math.dist(self.last_scene_xy, (x, y)) < 0.01:
            return
        self.moveit2.add_collision_sphere(
            id="basketball",
            radius=vision.BALL_DIAMETER / 2,
            position=(x, y, z),
            frame_id=FRAME,
        )
        self.last_scene_xy = (x, y)
        self.get_logger().info("Updated 'basketball' in the planning scene.")


def main():
    rclpy.init()
    node = BallDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()