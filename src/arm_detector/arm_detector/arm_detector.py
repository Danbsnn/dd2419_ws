#!/usr/bin/env python3
"""
CubeDetector — arm camera, top-down, fisheye calibrated
========================================================
Outputs X,Y position in camera_link frame (metres).
A separate node can then TF-transform to base_link.

Camera: fisheye, 640x480, mounted 20.1cm above ground facing down.
Intrinsics from YAML calibration file.

Key improvements in this version:
  - Stable centroid via image moments on the colour/edge mask
    (not bounding rect centre — that jitters with contour shape changes)
  - Fisheye undistortion applied to the centroid pixel before back-projection
  - Real X,Y in metres published (not pixel offsets)
  - Bounding box drawn from the stable mask, not the raw contour
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import defaultdict

# ─────────────────────────────────────────────────────────────────────────────
# CAMERA CALIBRATION  (from YAML)
# ─────────────────────────────────────────────────────────────────────────────
K = np.array([
    [404.23923175620627, 2.188762593807254, 319.5],
    [0.0,               402.195373124321,   239.5],
    [0.0,               0.0,                1.0  ]
], dtype=np.float64)

D = np.array([
    -0.4458703412524673,
     2.7816594156519177,
    -2.6362887625271108,
    -0.7973909844908826
], dtype=np.float64)

CAMERA_HEIGHT = 0.201   # metres — camera_link Z above ground

# Undistortion maps precomputed once at startup
_map1, _map2 = cv2.fisheye.initUndistortRectifyMap(
    K, D, np.eye(3), K, (640, 480), cv2.CV_16SC2)

# ─────────────────────────────────────────────────────────────────────────────
# TUNING
# ─────────────────────────────────────────────────────────────────────────────
MIN_AREA         = 800
ASPECT_LO        = 0.58
ASPECT_HI        = 1.58
SOLIDITY_MIN     = 0.72
COLOUR_MIN_FRAC  = 0.15
CONFIRM_FRAMES   = 3
CELL_SIZE        = 30

# Wood
WOOD_TOP_V_MIN   = 185
WOOD_TOP_S_MAX   = 40
WOOD_ASPECT_LO   = 0.65
WOOD_ASPECT_HI   = 1.45
WOOD_EDGE_DENSITY= 0.04
WOOD_SOLIDITY    = 0.75
WOOD_HULL_RATIO  = 0.82

# ─────────────────────────────────────────────────────────────────────────────
# HSV RANGES
# ─────────────────────────────────────────────────────────────────────────────
COLOR_HSV_RANGES = {
    "red": [
        (np.array([0,   45, 100], np.uint8), np.array([8,   255, 255], np.uint8)),
        (np.array([155, 45, 100], np.uint8), np.array([180, 255, 255], np.uint8)),
    ],
    "green": [
        (np.array([28, 90, 80], np.uint8), np.array([105, 255, 255], np.uint8)),
    ],
    "blue": [
        (np.array([82, 28, 100], np.uint8), np.array([108, 160, 255], np.uint8)),
        (np.array([88, 80, 100], np.uint8), np.array([115, 255, 255], np.uint8)),
    ],
    "brown": [
        (np.array([0, 120, 70], np.uint8), np.array([18, 255, 168], np.uint8)),
    ],
}

BLOB_COLOURS = {"red", "green", "blue", "brown"}

_DRAW_BGR = {
    "red":   (0,   0,   255),
    "green": (0,   210,   0),
    "blue":  (255, 160,   0),
    "wood":  (0,   180, 255),
    "brown": (30,   80, 160),
}

# ─────────────────────────────────────────────────────────────────────────────

def pixel_to_camera_frame(u, v):
    """
    Convert a distorted pixel (u,v) to X,Y in camera_link frame (metres).

    Steps:
      1. Undistort the point using fisheye model
      2. Normalise using K (subtract principal point, divide by focal length)
      3. Scale by camera height (flat-floor assumption: Z = CAMERA_HEIGHT)

    Returns (X, Y) in metres in camera_link frame.
    X = right,  Y = down  (optical convention, matching camera_link Z-down mount)
    """
    pt      = np.array([[[float(u), float(v)]]], dtype=np.float32)
    undist  = cv2.fisheye.undistortPoints(pt, K, D, P=K)
    u2, v2  = undist[0, 0]
    X = (u2 - K[0, 2]) / K[0, 0] * CAMERA_HEIGHT
    Y = (v2 - K[1, 2]) / K[1, 1] * CAMERA_HEIGHT
    return float(X), float(Y)


def mask_centroid(mask):
    """
    Return the centroid of a binary mask using image moments.
    Much more stable than bounding-rect centre — not affected by
    contour vertex jitter or partial edge detections.
    Returns (cx, cy) or None if mask is empty.
    """
    M = cv2.moments(mask, binaryImage=True)
    if M["m00"] < 1:
        return None
    return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])


def stable_bbox(mask):
    """
    Bounding box of the non-zero region in a mask.
    Using the mask directly gives a box that matches what was actually
    detected, not a contour approximation that can grow/shrink per frame.
    """
    pts = cv2.findNonZero(mask)
    if pts is None:
        return None
    return cv2.boundingRect(pts)   # x, y, w, h


# ─────────────────────────────────────────────────────────────────────────────

class CubeDetector(Node):
    def __init__(self):
        super().__init__('cube_detector')

        self.declare_parameter('camera_topic', '/arm/camera/image_raw')
        self.declare_parameter('debug',        True)

        cam_topic  = self.get_parameter('camera_topic').value
        self.debug = self.get_parameter('debug').value
        self.tf_broadcaster_ = TransformBroadcaster(self)

        self.sub    = self.create_subscription(
            Image, cam_topic, self.image_callback, 10)
        self.bridge = CvBridge()

        self._morph_k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        self._tracker = defaultdict(lambda: [0, "unknown", None])

        self.get_logger().info(
            f"CubeDetector ready | {cam_topic} | "
            f"height={CAMERA_HEIGHT*100:.1f}cm | "
            f"scale={CAMERA_HEIGHT/K[0,0]*1000:.2f}mm/px")

    # ══════════════════════════════════════════════════════════════════════════
    # SHAPE GATE
    # ══════════════════════════════════════════════════════════════════════════

    def _shape_ok(self, cnt, x, y, w, h):
        if not (ASPECT_LO < w / h < ASPECT_HI):
            return False
        return (cv2.contourArea(cnt) / (w * h)) >= SOLIDITY_MIN

    # ══════════════════════════════════════════════════════════════════════════
    # WOOD DETECTION
    # ══════════════════════════════════════════════════════════════════════════

    def _wood_candidates(self, hsv, gray):
        bright     = cv2.inRange(hsv[:, :, 2], WOOD_TOP_V_MIN, 255)
        low_sat    = cv2.inRange(hsv[:, :, 1], 0, WOOD_TOP_S_MAX)
        white_mask = cv2.bitwise_and(bright, low_sat)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN,
                                      self._morph_k, iterations=2)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE,
                                      self._morph_k, iterations=3)

        cnts, _ = cv2.findContours(
            white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cands = []
        for cnt in cnts:
            area = cv2.contourArea(cnt)
            if area < MIN_AREA * 0.4:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            if not (WOOD_ASPECT_LO < w / h < WOOD_ASPECT_HI):
                continue
            if area / (w * h) < WOOD_SOLIDITY:
                continue
            hull      = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            if hull_area == 0 or (area / hull_area) < WOOD_HULL_RATIO:
                continue
            pad = max(6, int(min(w, h) * 0.18))
            x2  = max(0, x - pad)
            y2  = max(0, y - pad)
            w2  = min(gray.shape[1] - x2, w + 2 * pad)
            h2  = min(gray.shape[0] - y2, h + 2 * pad)
            # return the white_mask ROI so centroid is computed from it
            cands.append((x2, y2, w2, h2, white_mask))
        return white_mask, cands

    def _is_wood(self, hsv, gray, x, y, w, h):
        if not (WOOD_ASPECT_LO < w / h < WOOD_ASPECT_HI):
            return False
        roi_hsv  = hsv[y:y+h, x:x+w]
        roi_gray = gray[y:y+h, x:x+w]
        margin   = max(4, int(min(w, h) * 0.20))
        inner    = roi_hsv[margin:h-margin, margin:w-margin]
        if inner.size == 0:
            return False
        if np.median(inner[:, :, 2]) < WOOD_TOP_V_MIN:
            return False
        if np.median(inner[:, :, 1]) > WOOD_TOP_S_MAX:
            return False
        border_mask = np.ones((h, w), dtype=np.uint8)
        border_mask[margin:h-margin, margin:w-margin] = 0
        border_v = roi_hsv[:, :, 2][border_mask == 1]
        if border_v.size == 0:
            return False
        if np.median(border_v) > np.median(inner[:, :, 2]) - 20:
            return False
        edge_density = cv2.countNonZero(
            cv2.Canny(roi_gray, 30, 100)) / (w * h)
        return edge_density >= WOOD_EDGE_DENSITY

    # ══════════════════════════════════════════════════════════════════════════
    # PATH A — edges
    # ══════════════════════════════════════════════════════════════════════════

    def _edge_candidates(self, gray):
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges   = cv2.Canny(blurred, 35, 115)
        edges   = cv2.dilate(edges, self._morph_k, iterations=1)
        cnts, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cands = []
        for cnt in cnts:
            if cv2.contourArea(cnt) < MIN_AREA:
                continue
            peri   = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
            elif 4 < len(approx) <= 8:
                x, y, w, h = cv2.boundingRect(cnt)
            else:
                continue
            if self._shape_ok(cnt, x, y, w, h):
                cands.append((x, y, w, h, cnt))
        return edges, cands

    # ══════════════════════════════════════════════════════════════════════════
    # PATH B — largest colour blob per colour
    # ══════════════════════════════════════════════════════════════════════════

    def _colour_candidates(self, hsv):
        """
        Returns per-colour masks too — centroid computed from the mask
        directly for stability, not from the bounding rect.
        """
        combined = np.zeros(hsv.shape[:2], dtype=np.uint8)
        cands    = []

        for color in BLOB_COLOURS:
            cmask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lo, hi in COLOR_HSV_RANGES[color]:
                cmask |= cv2.inRange(hsv, lo, hi)
            cmask = cv2.morphologyEx(cmask, cv2.MORPH_OPEN,
                                     self._morph_k, iterations=2)
            cmask = cv2.morphologyEx(cmask, cv2.MORPH_CLOSE,
                                     self._morph_k, iterations=3)

            cnts, _ = cv2.findContours(
                cmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                continue
            cnt = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(cnt) < MIN_AREA:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            if not (ASPECT_LO < w / h < ASPECT_HI):
                continue

            # build a clean single-object mask for stable centroid
            obj_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            cv2.drawContours(obj_mask, [cnt], -1, 255, -1)

            combined |= cmask
            cands.append((x, y, w, h, color, obj_mask))

        return combined, cands

    # ══════════════════════════════════════════════════════════════════════════
    # TEMPORAL GATE
    # ══════════════════════════════════════════════════════════════════════════

    def _tick(self, cell, color, bbox):
        e = self._tracker[cell]
        e[0] += 1; e[1] = color; e[2] = bbox
        return e[0] >= CONFIRM_FRAMES

    def _age(self, active):
        for cell in list(self._tracker):
            if cell not in active:
                self._tracker[cell][0] -= 1
                if self._tracker[cell][0] <= 0:
                    del self._tracker[cell]

    # ══════════════════════════════════════════════════════════════════════════
    # MAIN CALLBACK
    # ══════════════════════════════════════════════════════════════════════════

    def image_callback(self, msg):
        frame          = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h_img, w_img   = frame.shape[:2]
        cx_img, cy_img = w_img // 2, h_img // 2

        # undistort full frame for display only
        frame_undist = cv2.remap(frame, _map1, _map2,
                                 interpolation=cv2.INTER_LINEAR)

        gray = cv2.cvtColor(frame_undist, cv2.COLOR_BGR2GRAY)
        hsv  = cv2.cvtColor(frame_undist, cv2.COLOR_BGR2HSV)

        edges, edge_cands   = self._edge_candidates(gray)
        cmask, colour_cands = self._colour_candidates(hsv)
        wmask, wood_cands   = self._wood_candidates(hsv, gray)

        # ── merge ─────────────────────────────────────────────────────────
        # cell -> (x,y,w,h, cnt_or_None, path, known_color_or_None, obj_mask_or_None)
        all_cands = {}

        for (x, y, w, h, cnt) in edge_cands:
            cell = (x // CELL_SIZE, y // CELL_SIZE)
            all_cands[cell] = (x, y, w, h, cnt, "edge", None, None)

        for (x, y, w, h, known_color, obj_mask) in colour_cands:
            cell = (x // CELL_SIZE, y // CELL_SIZE)
            if cell not in all_cands:
                all_cands[cell] = (x, y, w, h, None, "colour",
                                   known_color, obj_mask)

        for (x, y, w, h, wmask_ref) in wood_cands:
            cell = (x // CELL_SIZE, y // CELL_SIZE)
            if cell not in all_cands:
                all_cands[cell] = (x, y, w, h, None, "wood", None, wmask_ref)

        vis          = frame_undist.copy()
        active_cells = set()
        cv2.drawMarker(vis, (cx_img, cy_img),
                       (200, 200, 200), cv2.MARKER_CROSS, 20, 1)

        for cell, (x, y, w, h, cnt, path, known_color, obj_mask) in all_cands.items():

            # ── determine colour ──────────────────────────────────────────
            if known_color is not None:
                # PATH B: colour already known from blob detection
                color = known_color
            else:
                # PATH A / C: run colour vote
                color, _ = self._classify_roi(hsv, x, y, w, h)
                if color == "unknown":
                    if self._is_wood(hsv, gray, x, y, w, h):
                        color = "wood"

            if color == "unknown":
                continue

            # ── stable centroid from mask ─────────────────────────────────
            # Build a mask for this detection to get a stable centroid.
            # PATH B has obj_mask already. Others: build from colour mask
            # within the bbox, or edge mask for wood.
            if obj_mask is not None:
                det_mask = obj_mask
            elif color == "wood":
                # use white blob mask clipped to bbox
                det_mask = wmask[y:y+h, x:x+w]
                # re-expand to full image coords for moments
                full_mask = np.zeros((h_img, w_img), dtype=np.uint8)
                full_mask[y:y+h, x:x+w] = det_mask
                det_mask = full_mask
            else:
                # build colour mask within bbox for this colour
                roi_hsv   = hsv[y:y+h, x:x+w]
                roi_mask  = np.zeros((h, w), dtype=np.uint8)
                for lo, hi in COLOR_HSV_RANGES[color]:
                    roi_mask |= cv2.inRange(roi_hsv, lo, hi)
                full_mask = np.zeros((h_img, w_img), dtype=np.uint8)
                full_mask[y:y+h, x:x+w] = roi_mask
                det_mask = full_mask

            centroid = mask_centroid(det_mask)
            if centroid is None:
                centroid = (x + w // 2, y + h // 2)   # fallback

            u_px, v_px = centroid

            # stable bbox from the mask (not from raw contour)
            bbox = stable_bbox(det_mask)
            if bbox is not None:
                bx, by, bw, bh = bbox
            else:
                bx, by, bw, bh = x, y, w, h

            # ── temporal gate ─────────────────────────────────────────────
            active_cells.add(cell)
            if not self._tick(cell, color, (bx, by, bw, bh)):
                cv2.rectangle(vis, (bx, by), (bx+bw, by+bh), (80, 80, 80), 1)
                continue

            # ── back-project to camera_link frame (metres) ────────────────
            X_m, Y_m = pixel_to_camera_frame(u_px, v_px)

            # ── broadcast ───────────────────────────────────────────────────
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'camera_link'
            t.child_frame_id = 'object_detected'

            # Position from command line
            t.transform.translation.x = X_m
            t.transform.translation.y = Y_m
            t.transform.translation.z = CAMERA_HEIGHT

            # Use parent's orientation (identity quaternion)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.tf_broadcaster_.sendTransform(t)

            

            self.get_logger().info(
                f"{color}_cube  "
                f"px=({u_px},{v_px})  "
                f"cam=({X_m:+.3f},{Y_m:+.3f})m  "
                f"path={path}")

            # ── draw — use stable mask bbox + moment centroid ─────────────
            dc = _DRAW_BGR[color]
            cv2.rectangle(vis, (bx, by), (bx+bw, by+bh), dc, 2)
            cv2.circle(vis, (u_px, v_px), 5, (255, 255, 255), -1)
            cv2.line(vis, (cx_img, cy_img), (u_px, v_px), dc, 1)
            cv2.putText(vis,
                        f"{color} ({X_m:+.3f},{Y_m:+.3f})m",
                        (bx, max(by - 8, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, dc, 2)

        self._age(active_cells)

        if self.debug:
            try:
                cv2.imshow("Detection",   vis)
                cv2.imshow("Edges",       edges)
                cv2.imshow("Colour mask", cmask)
                cv2.imshow("Wood mask",   wmask)
                cv2.waitKey(1)
            except Exception:
                pass

    def _classify_roi(self, hsv, x, y, w, h):
        roi      = hsv[y:y+h, x:x+w]
        roi_area = w * h
        best_color, best_frac = "unknown", 0.0
        for color, bands in COLOR_HSV_RANGES.items():
            mask = np.zeros((h, w), dtype=np.uint8)
            for lo, hi in bands:
                mask |= cv2.inRange(roi, lo, hi)
            frac = cv2.countNonZero(mask) / roi_area
            if frac > best_frac and frac >= COLOUR_MIN_FRAC:
                best_frac, best_color = frac, color
        return best_color, best_frac


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = CubeDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()