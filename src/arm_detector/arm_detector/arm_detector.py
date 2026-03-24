#!/usr/bin/env python3
"""
CubeDetector — dual-surface, image-calibrated
==============================================
Calibrated from actual camera images on both table and floor.

Key findings from pixel sampling:
  red  : table H=172 S=154 | floor H=167 S=56  → unify: H=155-180+0-8, S>=45
  green: table H=35  S=163 | floor H=78  S=105 → unify: H=28-100,       S>=90
  blue : table H=17  S=43  | floor H=94  S=92  → TWO separate ranges needed
  wood : table S=84        | floor S=24         → floor S≈floor, EDGE-ONLY on floor
  floor background: S=19-46  → use S>50 to separate most colours from floor

Wood cube on floor WARNING:
  Wood top S=24, floor S=29 — no colour separation possible.
  Wood is detected via edges only (shape gate). Colour confirms on table,
  shape-only fallback on floor with stricter solidity.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import defaultdict

# ─────────────────────────────────────────────────────────────────────────────
# TUNING
# ─────────────────────────────────────────────────────────────────────────────
MIN_AREA        = 800
ASPECT_LO       = 0.58
ASPECT_HI       = 1.58
SOLIDITY_MIN    = 0.72
COLOUR_MIN_FRAC = 0.15   # lower — floor lighting desaturates cubes
CONFIRM_FRAMES  = 3
CELL_SIZE       = 30

# ─────────────────────────────────────────────────────────────────────────────
# HSV RANGES — unified across table + floor from pixel sampling
#
#  red  : H wraps near 180. Table S=154, floor S=56 → floor at S=45
#  green: H=35 on table, H=78 on floor → cover H=28-105
#  blue : table H=17 S=43 (teal), floor H=94 S=92 (real blue) → two ranges
#  wood : only reliable via edges. Colour range kept tight to avoid
#         false-positives on floor (floor S≈29, wood floor S≈24 — overlap)
#  brown: H=9 S=149 on table — higher sat than wood
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
        # teal on table (low sat)
        (np.array([82,  28, 100], np.uint8), np.array([108, 160, 255], np.uint8)),
        # real blue on floor (higher sat)
        (np.array([88,  80, 100], np.uint8), np.array([115, 255, 255], np.uint8)),
    ],
    # wood: tight sat window — avoids floor (S≈29) and avoids brown cube (S≈149)
    # Only used to CONFIRM edge detections, not to find blobs
    "wood": [
        (np.array([5, 55, 140], np.uint8), np.array([22, 145, 235], np.uint8)),
    ],
    "brown": [
        (np.array([0, 120, 70], np.uint8), np.array([18, 255, 168], np.uint8)),
    ],
}

# Wood is too close to floor background for blob detection — skip it in PATH B
BLOB_COLOURS = {"red", "green", "blue", "brown"}

_DRAW_BGR = {
    "red":   (0,   0,   255),
    "green": (0,   210,   0),
    "blue":  (255, 160,   0),
    "wood":  (0,   180, 255),
    "brown": (30,   80, 160),
}

# ─────────────────────────────────────────────────────────────────────────────

class CubeDetector(Node):
    def __init__(self):
        super().__init__('cube_detector')

        self.declare_parameter('camera_topic', '/arm/camera/image_raw')
        self.declare_parameter('pose_topic',   '/cube_pose')
        self.declare_parameter('debug',        True)

        cam_topic  = self.get_parameter('camera_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        self.debug = self.get_parameter('debug').value

        self.sub    = self.create_subscription(
            Image, cam_topic, self.image_callback, 10)
        self.pub    = self.create_publisher(PoseStamped, pose_topic, 10)
        self.bridge = CvBridge()
        self._morph_k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        self._tracker = defaultdict(lambda: [0, "unknown", None])
        self.get_logger().info(f"CubeDetector ready | {cam_topic}")

    # ── shape ─────────────────────────────────────────────────────────────────
    def _shape_ok(self, cnt, x, y, w, h):
        if not (ASPECT_LO < w / h < ASPECT_HI):
            return False
        return (cv2.contourArea(cnt) / (w * h)) >= SOLIDITY_MIN

    # ── colour classify ───────────────────────────────────────────────────────
    def _classify_roi(self, hsv, x, y, w, h, allowed=None):
        roi      = hsv[y:y+h, x:x+w]
        roi_area = w * h
        best_color, best_frac = "unknown", 0.0
        for color, bands in COLOR_HSV_RANGES.items():
            if allowed and color not in allowed:
                continue
            mask = np.zeros((h, w), dtype=np.uint8)
            for lo, hi in bands:
                mask |= cv2.inRange(roi, lo, hi)
            frac = cv2.countNonZero(mask) / roi_area
            if frac > best_frac and frac >= COLOUR_MIN_FRAC:
                best_frac, best_color = frac, color
        return best_color, best_frac

    # ── temporal ──────────────────────────────────────────────────────────────
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

    # ── PATH A: edges (works on table + finds wood on floor) ──────────────────
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
                cands.append((x, y, w, h))
        return edges, cands

    # ── PATH B: colour blobs (works on grey floor for non-wood cubes) ─────────
    def _colour_candidates(self, hsv):
        combined = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for color, bands in COLOR_HSV_RANGES.items():
            if color not in BLOB_COLOURS:
                continue
            for lo, hi in bands:
                combined |= cv2.inRange(hsv, lo, hi)

        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN,
                                    self._morph_k, iterations=2)
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE,
                                    self._morph_k, iterations=3)
        cnts, _ = cv2.findContours(
            combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cands = []
        for cnt in cnts:
            if cv2.contourArea(cnt) < MIN_AREA:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            if ASPECT_LO < w / h < ASPECT_HI:
                cands.append((x, y, w, h))
        return combined, cands

    # ── main callback ─────────────────────────────────────────────────────────
    def image_callback(self, msg):
        frame          = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h_img, w_img   = frame.shape[:2]
        cx_img, cy_img = w_img // 2, h_img // 2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        edges, edge_cands   = self._edge_candidates(gray)
        cmask, colour_cands = self._colour_candidates(hsv)

        # merge paths — edge candidates searched for ALL colours including wood,
        # colour-blob candidates skip wood (unreliable on floor background)
        all_cands = {}
        for (x, y, w, h) in edge_cands:
            cell = (x // CELL_SIZE, y // CELL_SIZE)
            all_cands[cell] = (x, y, w, h, None)        # None = check all colours
        for (x, y, w, h) in colour_cands:
            cell = (x // CELL_SIZE, y // CELL_SIZE)
            if cell not in all_cands:
                all_cands[cell] = (x, y, w, h, BLOB_COLOURS)  # restrict to blob colours

        vis          = frame.copy()
        active_cells = set()
        cv2.drawMarker(vis, (cx_img, cy_img),
                       (200, 200, 200), cv2.MARKER_CROSS, 20, 1)

        for cell, (x, y, w, h, allowed) in all_cands.items():
            color, frac = self._classify_roi(hsv, x, y, w, h, allowed)
            if color == "unknown":
                continue

            active_cells.add(cell)
            if not self._tick(cell, color, (x, y, w, h)):
                cv2.rectangle(vis, (x, y), (x+w, y+h), (80, 80, 80), 1)
                continue

            obj_cx = x + w // 2
            obj_cy = y + h // 2
            dx_px  = obj_cx - cx_img
            dy_px  = obj_cy - cy_img

            out = PoseStamped()
            out.header.stamp    = self.get_clock().now().to_msg()
            out.header.frame_id = "camera_frame"
            out.pose.position.x = float(dx_px)
            out.pose.position.y = float(dy_px)
            out.pose.position.z = 0.0
            self.pub.publish(out)

            self.get_logger().info(
                f"{color}_cube  offset=({dx_px:+d},{dy_px:+d})px  "
                f"colour={frac*100:.0f}%")

            dc = _DRAW_BGR[color]
            cv2.rectangle(vis, (x, y), (x+w, y+h), dc, 2)
            cv2.circle(vis, (obj_cx, obj_cy), 5, (255, 255, 255), -1)
            cv2.line(vis, (cx_img, cy_img), (obj_cx, obj_cy), dc, 1)
            cv2.putText(vis, f"{color} ({dx_px:+d},{dy_px:+d})px",
                        (x, max(y - 8, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, dc, 2)

        self._age(active_cells)

        if self.debug:
            try:
                cv2.imshow("Detection",   vis)
                cv2.imshow("Edges",       edges)
                cv2.imshow("Colour mask", cmask)
                cv2.waitKey(1)
            except Exception:
                pass


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