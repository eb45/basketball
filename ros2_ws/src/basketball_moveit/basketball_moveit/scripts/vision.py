"""Ball detection and pixel-to-world math for the overhead camera.

Everything here is plain numpy/OpenCV, no ROS, so it can be tested on its own.

Frame convention: the camera hangs above the table looking straight down, with
the SDF pose (x, y, z, roll=0, pitch=pi/2, yaw=0). That makes
    image up    = world +x
    image right = world -y
World coordinates are assumed to match base_link (check in Gazebo and RViz).
"""

import math

import cv2
import numpy as np

# Camera (meters, radians). Shared by the spawner and the detector.
IMAGE_W = 640
IMAGE_H = 480
HFOV = 1.047  # horizontal field of view, about 60 degrees
CAM_POS = (0.24, 0.0, 1.0)

# Ball (meters). The arm base sits 0.0627 m above the tabletop.
BALL_DIAMETER = 0.08
TABLE_TOP_Z = -0.0627
BALL_Z = TABLE_TOP_Z + BALL_DIAMETER / 2
BALL_XY = (0.24, 0.0)

# OpenCV HSV (H 0-179). Defaults match an orange ball: RGB (1, 0.45, 0).
HSV_LOW = (5, 150, 100)
HSV_HIGH = (22, 255, 255)
MIN_AREA_PX = 80


def intrinsics(width=IMAGE_W, height=IMAGE_H, hfov=HFOV):
    """Return fx, fy, cx, cy for a pinhole camera with square pixels."""
    fx = (width / 2.0) / math.tan(hfov / 2.0)
    return fx, fx, width / 2.0, height / 2.0


def find_ball(bgr, hsv_low=HSV_LOW, hsv_high=HSV_HIGH, min_area=MIN_AREA_PX):
    """Find the biggest blob in the color range.

    Returns (u, v, radius_px, mask), or (None, None, None, mask) if nothing
    big enough is found.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(hsv_low, np.uint8), np.array(hsv_high, np.uint8))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) >= min_area]
    if not contours:
        return None, None, None, mask
    biggest = max(contours, key=cv2.contourArea)
    (u, v), radius = cv2.minEnclosingCircle(biggest)
    return float(u), float(v), float(radius), mask


def pixel_to_world(u, v, cam_pos=CAM_POS, plane_z=BALL_Z, intr=None):
    """Back-project pixel (u, v) onto the horizontal plane z = plane_z."""
    fx, fy, cx, cy = intr or intrinsics()
    depth = cam_pos[2] - plane_z
    xn = (u - cx) / fx   # right in the image
    yn = (v - cy) / fy   # down in the image
    x = cam_pos[0] - yn * depth
    y = cam_pos[1] - xn * depth
    return x, y, plane_z

def ball_center_from_depth(depth, mask, u, v, radius_px, ball_radius, intr,
                           inner=0.6, min_valid=0.3):
    """Ball center (x, y, z) in the camera's optical frame, from a depth image in meters.

    The depth camera sees the front of the ball, so each reading is pushed back
    to the center using the sphere's shape. Only the middle of the blob is used,
    because depth is unreliable near the ball's edge. Returns None if too few
    pixels there have a depth reading.
    """
    fx, fy, cx, cy = intr
    rows, cols = depth.shape
    reach = int(math.ceil(radius_px * inner))
    u0, u1 = max(int(u) - reach, 0), min(int(u) + reach + 1, cols)
    v0, v1 = max(int(v) - reach, 0), min(int(v) + reach + 1, rows)
    if u0 >= u1 or v0 >= v1:
        return None
    vv, uu = np.mgrid[v0:v1, u0:u1]
    rho2 = ((uu - u) ** 2 + (vv - v) ** 2) / radius_px ** 2  # 0 at center, 1 at the edge
    patch = depth[v0:v1, u0:u1]
    on_ball = (rho2 <= inner ** 2) & (mask[v0:v1, u0:u1] > 0)
    valid = on_ball & np.isfinite(patch) & (patch > 0)
    if not valid.any() or valid.sum() < min_valid * on_ball.sum():
        return None
    z = float(np.median(patch[valid] + ball_radius * np.sqrt(1.0 - rho2[valid])))
    return (u - cx) / fx * z, (v - cy) / fy * z, z


def world_to_pixel(x, y, z, cam_pos=CAM_POS, intr=None):
    """Project a world point into the image (inverse of pixel_to_world)."""
    fx, fy, cx, cy = intr or intrinsics()
    depth = cam_pos[2] - z
    xn = -(y - cam_pos[1]) / depth
    yn = -(x - cam_pos[0]) / depth
    return cx + fx * xn, cy + fy * yn