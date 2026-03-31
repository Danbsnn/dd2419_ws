#!/usr/bin/env python3
"""
CubeDetector — dual-surface, wood via white-top + brown-border
==============================================================
Wood cube detection strategy (from pixel analysis):
  - Top face:  S≈0,  V≈207  (near-white, very bright)
   - Side/border: S≈33, V≈141  (warm brown, darker)
   - Floor:    S≈10, V≈185  (grey, mid-bright)

  The cube has a BRIGHT WHITE rectangle surrounded by a DARKER BROWN border.
  This relative contrast structure is unique vs the floor.
  Detection: find bright blobs → check darker surround → confirm square shape.

  Floor texture edge density ≈ 0.005 vs cube edge density ≈ 0.078
  → edge density inside ROI is a strong secondary gate.
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
COLOUR_MIN_FRAC = 0.15
CONFIRM_FRAMES  = 3
CELL_SIZE       = 30

# Wood detection thresholds (from pixel sampling)
WOOD_TOP_V_MIN    = 185   # white top face brightness floor
WOOD_TOP_S_MAX    = 40    # white top face max saturation
WOOD_BORDER_V_MAX = 175   # brown border must be darker than this
WOOD_BORDER_S_MIN = 15    # brown border has some warmth
WOOD_ASPECT_LO    = 0.65
WOOD_ASPECT_HI    = 1.45
WOOD_EDGE_DENSITY = 0.04  # min edge density inside ROI (floor≈0.005, cube≈0.078)

# ─────────────────────────────────────────────────────────────────────────────
# HSV RANGES — wood excluded, detected by structure instead
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
        (np.array([82,  28, 100], np.uint8), np.array([108, 160, 255], np.uint8)),
        (np.array([88,  80, 100], np.uint8), np.array([115, 255, 255], np.uint8)),
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

    # ── standard shape gate ───────────────────────────────────────────────────
    def _shape_ok(self, cnt, x, y, w, h):
        if not (ASPECT_LO < w / h < ASPECT_HI):
            return False
        return (cv2.contourArea(cnt) / (w * h)) >= SOLIDITY_MIN

    # ── wood detection: white-top + brown-border + edge density ──────────────
    def _is_wood(self, hsv, gray, x, y, w, h):
        """
        Three-part test unique to the wood cube:

        1. BRIGHT CENTRE: inner 50% of ROI must have a bright, low-sat region
           (the white painted top face). V>185, S<40.

        2. DARKER SURROUND: the border ring of the ROI must be noticeably
           darker and warmer than the centre (brown wooden sides).
           Border median V < centre median V - 20.

        3. EDGE DENSITY: Canny edge pixel ratio inside the full ROI must
           exceed WOOD_EDGE_DENSITY. Floor texture is near-zero; cube edges
           are strong.
        """
        if not (WOOD_ASPECT_LO < w / h < WOOD_ASPECT_HI):
            return False

        roi_hsv  = hsv[y:y+h, x:x+w]
        roi_gray = gray[y:y+h, x:x+w]

        # ── test 1: bright white centre ──────────────────────────────────
        margin  = max(4, int(min(w, h) * 0.20))   # 20% border ring
        inner_hsv = roi_hsv[margin:h-margin, margin:w-margin]
        if inner_hsv.size == 0:
            return False

        inner_v = np.median(inner_hsv[:, :, 2])
        inner_s = np.median(inner_hsv[:, :, 1])

        if inner_v < WOOD_TOP_V_MIN or inner_s > WOOD_TOP_S_MAX:
            return False

        # ── test 2: darker brown surround ────────────────────────────────
        # build border mask
        border_mask = np.ones((h, w), dtype=np.uint8)
        border_mask[margin:h-margin, margin:w-margin] = 0
        border_v_vals = roi_hsv[:, :, 2][border_mask == 1]
        if border_v_vals.size == 0:
            return False

        border_v = np.median(border_v_vals)
        # border must be meaningfully darker than the white top
        if border_v > inner_v - 20:
            return False

        # ── test 3: edge density ─────────────────────────────────────────
        roi_edges   = cv2.Canny(roi_gray, 30, 100)
        edge_density = cv2.countNonZero(roi_edges) / (w * h)
        if edge_density < WOOD_EDGE_DENSITY:
            return False

        return True

    # ── PATH C: bright-blob candidates for wood ───────────────────────────────
    def _wood_candidates(self, hsv, gray):
        """
        Finds the wood cube white top face.
        Key filters that separate cube from floor reflections:
          - solidity  >= 0.75  (cube is solid rectangle; reflection is ragged)
          - hull_ratio >= 0.82 (convex hull tightly wraps a cube; not a blob)
          - aspect    0.65-1.45 (square-ish)
        Floor reflection measured: solidity=0.43, hull_ratio=0.59 → both fail.
        """
        v_channel  = hsv[:, :, 2]
        s_channel  = hsv[:, :, 1]
        bright     = cv2.inRange(v_channel, 185, 255)
        low_sat    = cv2.inRange(s_channel, 0, 40)
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

            # aspect ratio gate
            if not (WOOD_ASPECT_LO < w / h < WOOD_ASPECT_HI):
                continue

            # solidity gate — rejects ragged floor reflections
            solidity = area / (w * h)
            if solidity < 0.75:
                continue

            # convex hull ratio — rejects irregular blobs
            hull      = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            if hull_area == 0 or (area / hull_area) < 0.82:
                continue

            # passed — expand bbox to include brown border
            pad = max(6, int(min(w, h) * 0.18))
            x2  = max(0, x - pad)
            y2  = max(0, y - pad)
            w2  = min(gray.shape[1] - x2, w + 2 * pad)
            h2  = min(gray.shape[0] - y2, h + 2 * pad)
            cands.append((x2, y2, w2, h2))

        return white_mask, cands

    # ── colour classify ───────────────────────────────────────────────────────
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

    # ── PATH A: edges ─────────────────────────────────────────────────────────
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

    # ── PATH B: colour blobs ──────────────────────────────────────────────────
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
                cands.append((x, y, w, h, None))
        return combined, cands

    # ── main callback ─────────────────────────────────────────────────────────
    def image_callback(self, msg):
        frame          = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h_img, w_img   = frame.shape[:2]
        cx_img, cy_img = w_img // 2, h_img // 2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        edges,  edge_cands   = self._edge_candidates(gray)
        cmask,  colour_cands = self._colour_candidates(hsv)
        wmask,  wood_cands   = self._wood_candidates(hsv, gray)

        # merge all three paths — edge path has priority (keeps cnt)
        all_cands = {}
        for (x, y, w, h, cnt) in edge_cands:
            cell = (x // CELL_SIZE, y // CELL_SIZE)
            all_cands[cell] = (x, y, w, h, cnt, "edge")
        for (x, y, w, h, _) in colour_cands:
            cell = (x // CELL_SIZE, y // CELL_SIZE)
            if cell not in all_cands:
                all_cands[cell] = (x, y, w, h, None, "colour")
        for (x, y, w, h) in wood_cands:
            cell = (x // CELL_SIZE, y // CELL_SIZE)
            if cell not in all_cands:
                all_cands[cell] = (x, y, w, h, None, "wood")

        vis          = frame.copy()
        active_cells = set()
        cv2.drawMarker(vis, (cx_img, cy_img),
                       (200, 200, 200), cv2.MARKER_CROSS, 20, 1)

        for cell, (x, y, w, h, cnt, path) in all_cands.items():

            # colour vote (red/green/blue/brown)
            color, frac = self._classify_roi(hsv, x, y, w, h)

            # wood fallback: white-top + brown-border + edge-density test
            if color == "unknown":
                if self._is_wood(hsv, gray, x, y, w, h):
                    color, frac = "wood", 0.0

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
                f"path={path}  colour={frac*100:.0f}%")

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
                cv2.imshow("Wood mask",   wmask)
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