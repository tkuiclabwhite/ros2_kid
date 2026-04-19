"""
Navigation Node (ROS + TUI 版)
整合 AprilTag + YOLO + 幾何箭頭驗證

影像來源切換:
  USE_ROS_IMAGE = True  → 訂閱 /camera1/image_raw  (小機器人用)
  USE_ROS_IMAGE = False → 從 /dev/video0 擷取     (測試用)
"""
import os
import time
import threading
import curses
from collections import deque

import cv2
import numpy as np
from pupil_apriltags import Detector
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

# ── 影像來源設定 ────────────────────────────────────────────
USE_ROS_IMAGE   = True
CAMERA_DEVICE   = "/dev/video0"
ROS_IMAGE_TOPIC = "/camera1/image_raw"

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# ── AprilTag 設定 ─────────────────────────────────────────
april_detector = Detector(
    families="tag36h11", nthreads=2, quad_decimate=1.0, quad_sigma=0.5,
    refine_edges=True, decode_sharpening=0.25,
)
TAG_ACTION = {1: "STRAIGHT", 2: "RIGHT", 3: "LEFT"}

# ── 投票參數 ──────────────────────────────────────────────
WINDOW_SECONDS     = 1.0
VOTE_THRESHOLD     = 0.5
MIN_VOTES          = 3
REPUBLISH_INTERVAL = 0.3
YOLO_CONF_THRESH   = 0.85


def find_arrow_direction(roi_bgr, yolo_is_forward=False):
    """幾何箭頭方向驗證,回傳 "STRAIGHT"/"LEFT"/"RIGHT"/None"""
    MIN_SIDE = 200
    h0, w0 = roi_bgr.shape[:2]
    short = min(h0, w0)
    if short < MIN_SIDE and short > 0:
        roi_bgr = cv2.resize(roi_bgr, None, fx=MIN_SIDE/short, fy=MIN_SIDE/short)

    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    h_roi, w_roi = gray.shape

    _, dark = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
    k5 = np.ones((5, 5), np.uint8)
    card = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, k5, iterations=5)
    dc, _ = cv2.findContours(card, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if dc:
        mask = np.zeros_like(gray)
        cv2.drawContours(mask, [max(dc, key=cv2.contourArea)], -1, 255, -1)
    else:
        mask = np.ones_like(gray) * 255

    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(4, 4))
    ge = clahe.apply(gray)
    px = ge[mask > 0]
    if len(px) < 100:
        return None
    thresh = (np.percentile(px, 95) + np.percentile(px, 5)) / 2
    _, b = cv2.threshold(ge, thresh, 255, cv2.THRESH_BINARY)
    binary = cv2.bitwise_and(b, mask)
    k3 = np.ones((3, 3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, k3)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, k3)
    if cv2.countNonZero(binary) < 800:
        return None

    cts, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not cts:
        return None
    largest = max(cts, key=cv2.contourArea)
    if cv2.contourArea(largest) < 800:
        return None
    contour = largest.reshape(-1, 2).astype(np.float32)
    if len(contour) < 5:
        return None

    M = cv2.moments(largest)
    if M["m00"] < 1e-6:
        return None
    centroid = np.array([M["m10"]/M["m00"], M["m01"]/M["m00"]], dtype=np.float32)

    hull_pts = cv2.convexHull(largest, returnPoints=True).reshape(-1, 2).astype(np.float32)
    if len(hull_pts) < 3:
        return None

    if yolo_is_forward:
        upper = hull_pts[hull_pts[:, 1] < h_roi * 0.6]
        if len(upper) == 0:
            upper = hull_pts
    else:
        upper = hull_pts

    tip = upper[int(np.argmax(np.linalg.norm(upper - centroid, axis=1)))]

    hi = cv2.convexHull(largest, returnPoints=False)
    if hi is None or len(hi) < 3:
        return None
    try:
        defects = cv2.convexityDefects(largest, hi)
    except cv2.error:
        return None
    if defects is None or len(defects) < 2:
        return None

    md = max(8, int(np.sqrt(cv2.contourArea(largest)) * 0.08)) * 256
    deep = [d for d in defects[:, 0, :] if d[3] >= md]
    if len(deep) < 2:
        return None

    pts = sorted(
        ((d, contour[int(d[2])], np.linalg.norm(contour[int(d[2])] - tip)) for d in deep),
        key=lambda x: x[2])
    sm = (pts[0][1] + pts[1][1]) / 2.0

    av = tip - sm
    if np.linalg.norm(av) < 1e-6:
        return None
    ad = av / np.linalg.norm(av)
    angle_deg = np.degrees(np.arctan2(ad[0], -ad[1]))
    if abs(angle_deg) < 45:
        return "STRAIGHT"
    return "RIGHT" if angle_deg > 0 else "LEFT"


class TUIState:
    def __init__(self):
        self.lock = threading.Lock()
        self.frame_count = 0; self.fps = 0.0; self.frame_size = (0, 0)
        self.april_action = None; self.april_tag_ids = []
        self.yolo_action = None; self.yolo_name = None; self.yolo_conf = 0.0
        self.yolo_center = None; self.yolo_area = 0; self.yolo_num_candidates = 0
        self.geo_action = None; self.final_action = None
        self.current_action = None; self.last_confirmed_action = None
        self.vote_counts = {}; self.vote_leader = None
        self.vote_leader_ratio = 0.0; self.window_votes = 0
        self.published_count = 0; self.last_publish_time = 0.0
        self.last_source = "—"; self.publish_log = deque(maxlen=5)
        self.last_error = ""

    def snapshot(self):
        with self.lock:
            snap = {k: v for k, v in self.__dict__.items() if k != "lock"}
            snap['publish_log'] = list(self.publish_log)
            return snap


def tui_loop(stdscr, state, stop_event):
    curses.curs_set(0); stdscr.nodelay(True)
    curses.start_color(); curses.use_default_colors()
    for i, c in enumerate([curses.COLOR_GREEN, curses.COLOR_YELLOW, curses.COLOR_CYAN,
                           curses.COLOR_RED, curses.COLOR_MAGENTA], 1):
        curses.init_pair(i, c, -1)
    GREEN, YELLOW, CYAN, RED, MAGENTA = (curses.color_pair(i) for i in range(1, 6))

    while not stop_event.is_set():
        s = state.snapshot()
        stdscr.erase()
        try:
            r = 0
            stdscr.addstr(r, 2, "═══ Navigation Node Monitor ═══", CYAN | curses.A_BOLD); r += 2
            stdscr.addstr(r, 2, f"Frames: {s['frame_count']:>6}   FPS: {s['fps']:>5.1f}   "
                                f"Size: {s['frame_size'][0]}x{s['frame_size'][1]}"); r += 2

            stdscr.addstr(r, 2, "── AprilTag ──", YELLOW | curses.A_BOLD); r += 1
            stdscr.addstr(r, 4, f"Action : {s['april_action'] or '—'}",
                          GREEN if s['april_action'] else curses.A_DIM); r += 1
            stdscr.addstr(r, 4, f"Tag IDs: {s['april_tag_ids'] or '—'}"); r += 2

            stdscr.addstr(r, 2, "── YOLO + GEO ──", YELLOW | curses.A_BOLD); r += 1
            stdscr.addstr(r, 4, f"YOLO  : {s['yolo_action'] or '—'}  "
                                f"({s['yolo_name'] or '—'} conf={s['yolo_conf']:.2f})",
                          GREEN if s['yolo_action'] else curses.A_DIM); r += 1
            stdscr.addstr(r, 4, f"GEO   : {s['geo_action'] or '—'}",
                          GREEN if s['geo_action'] else curses.A_DIM); r += 1
            stdscr.addstr(r, 4, f"Final : {s['final_action'] or '—'}",
                          MAGENTA if s['final_action'] else curses.A_DIM); r += 1
            stdscr.addstr(r, 4, f"Center: {s['yolo_center'] or '—'}  area={s['yolo_area']}  "
                                f"candi={s['yolo_num_candidates']}"); r += 2

            stdscr.addstr(r, 2, "── Decision ──", YELLOW | curses.A_BOLD); r += 1
            stdscr.addstr(r, 4, f"Current  : {s['current_action'] or 'NONE'}",
                          GREEN if s['current_action'] else RED); r += 1
            ratio = s['vote_leader_ratio']
            bar = "█" * int(20 * ratio) + "░" * (20 - int(20 * ratio))
            rc = GREEN if ratio >= VOTE_THRESHOLD else YELLOW
            stdscr.addstr(r, 4, f"Window   : {s['window_votes']} votes / {WINDOW_SECONDS:.1f}s"); r += 1
            stdscr.addstr(r, 4, f"Leader   : {s['vote_leader'] or '—':<10} [{bar}] {ratio*100:5.1f}%", rc); r += 1
            cs = "  ".join(f"{k}:{v}" for k, v in s['vote_counts'].items()) or "—"
            stdscr.addstr(r, 4, f"Counts   : {cs}"); r += 1
            stdscr.addstr(r, 4, f"Confirmed: {s['last_confirmed_action'] or '—'}",
                          MAGENTA | curses.A_BOLD); r += 2

            stdscr.addstr(r, 2, "── Publish ──", YELLOW | curses.A_BOLD); r += 1
            stdscr.addstr(r, 4, f"Count : {s['published_count']}   Source: {s['last_source']}"); r += 1
            if s['last_publish_time']:
                stdscr.addstr(r, 4, f"Last  : {time.time() - s['last_publish_time']:5.1f}s ago"); r += 1
            r += 1

            stdscr.addstr(r, 2, "── Recent Publishes (last 5) ──", YELLOW | curses.A_BOLD); r += 1
            if not s['publish_log']:
                stdscr.addstr(r, 4, "(no publishes yet)", curses.A_DIM); r += 1
            else:
                stdscr.addstr(r, 4, f"{'#':<3}{'time':<11}{'action':<10}{'src':<6}data",
                              curses.A_UNDERLINE); r += 1
                for e in reversed(s['publish_log']):
                    color = GREEN if e['src'] == 'TAG' else CYAN
                    line = (f"{e['idx']:<3}{time.time() - e['t']:>5.1f}s ago "
                            f"{e['action']:<10}{e['src']:<6}{e['data']}")
                    stdscr.addstr(r, 4, line[:curses.COLS - 6], color); r += 1
            r += 1

            if s['last_error']:
                stdscr.addstr(r, 2, f"ERROR: {s['last_error']}", RED); r += 1
            stdscr.addstr(r + 1, 2, "Press Ctrl+C to quit", curses.A_DIM)
        except curses.error:
            pass
        stdscr.refresh()
        time.sleep(0.1)


class NavigationNode(Node):
    def __init__(self, state):
        super().__init__('navigation_node')
        self.state = state
        self.bridge = CvBridge()
        self.vote_window = deque()
        self.last_confirmed_action = None
        self.last_publish_time = 0.0
        self._fps_t0 = time.time()
        self._fps_count = 0

        print("[Nav] Loading YOLO model...")
        # self.yolo_model = YOLO(os.path.join(BASE_DIR, 'best1.onnx'))
        # self.yolo_model = YOLO('strategy/strategy/mar/best1.onnx')
        self.yolo_model = YOLO('/home/iclab/ros2_kid/src/strategy/strategy/mar/best1.onnx')
        self.yolo_class_names = {1: "left", 2: "right", 0: "straight"}
        self.yolo_to_action = {"left": "LEFT", "right": "RIGHT", "straight": "STRAIGHT"}
        print("[Nav] YOLO ready.")

        self.class_id_pub = self.create_publisher(String, 'class_id_topic', 1)
        self.coord_pub = self.create_publisher(Point, 'sign_coordinates', 1)

        self.cap = None
        if USE_ROS_IMAGE:
            self.image_sub = self.create_subscription(
                Image, ROS_IMAGE_TOPIC, self._ros_image_callback, 10)
            print(f"[Nav] Subscribing: {ROS_IMAGE_TOPIC}")
        else:
            self.cap = cv2.VideoCapture(CAMERA_DEVICE)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            threading.Thread(target=self._camera_loop, daemon=True).start()
            print(f"[Nav] Capturing from {CAMERA_DEVICE}")

    def _ros_image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_frame(frame)
        except Exception as e:
            with self.state.lock:
                self.state.last_error = f'cv_bridge: {e}'

    def _camera_loop(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue
            self.process_frame(frame)

    def process_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = frame.shape[:2]
        cx_frame, cy_frame = w // 2, h // 2

        self._fps_count += 1
        now = time.time()
        fps = None
        if now - self._fps_t0 >= 1.0:
            fps = self._fps_count / (now - self._fps_t0)
            self._fps_count = 0; self._fps_t0 = now

        # AprilTag
        april_action = None; april_tag_ids = []
        april_center = None; april_bottom_y = None
        for r in april_detector.detect(gray):
            april_tag_ids.append(r.tag_id)
            act = TAG_ACTION.get(r.tag_id)
            if act:
                april_action = act
                corners = r.corners
                april_center = (int(corners[:, 0].mean()), int(corners[:, 1].mean()))
                april_bottom_y = int(corners[:, 1].max())

        # YOLO + GEO
        yolo_action = yolo_name = yolo_center = None
        yolo_conf = 0.0; yolo_area = 0; yolo_bbox_bottom = None
        num_candidates = 0; geo_action = None; final_action = None

        if not april_action:
            results = self.yolo_model(frame, imgsz=640, verbose=False)
            candidates = []
            for box in results[0].boxes:
                conf = float(box.conf[0]); cls = int(box.cls[0])
                if cls not in self.yolo_class_names or conf < YOLO_CONF_THRESH:
                    continue
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                candidates.append({
                    'name': self.yolo_class_names[cls], 'conf': conf,
                    'center': ((x1+x2)//2, (y1+y2)//2),
                    'area': (x2-x1)*(y2-y1), 'bbox_bottom': y2,
                    'bbox': (x1, y1, x2, y2),
                })
            num_candidates = len(candidates)
            if candidates:
                best = max(candidates, key=lambda x: x['area'])
                yolo_name = best['name']; yolo_conf = best['conf']
                yolo_action = self.yolo_to_action.get(best['name'])
                yolo_center = best['center']; yolo_area = best['area']
                yolo_bbox_bottom = best['bbox_bottom']

                x1, y1, x2, y2 = best['bbox']
                pad = 20
                ax1, ay1 = max(0, x1-pad), max(0, y1-pad)
                ax2, ay2 = min(w, x2+pad), min(h, y2+pad)
                roi = frame[ay1:ay2, ax1:ax2]
                if roi.size > 0:
                    geo_action = find_arrow_direction(
                        roi, yolo_is_forward=(yolo_action == "STRAIGHT"))

                if yolo_action == "STRAIGHT" and geo_action != "STRAIGHT":
                    final_action = "STRAIGHT"
                elif yolo_action in ("LEFT", "RIGHT") and geo_action == "STRAIGHT":
                    final_action = yolo_action
                else:
                    final_action = geo_action or yolo_action

        current_action = april_action or final_action

        # 時間視窗投票
        now_t = time.time()
        self.vote_window.append((now_t, current_action))
        cutoff = now_t - WINDOW_SECONDS
        while self.vote_window and self.vote_window[0][0] < cutoff:
            self.vote_window.popleft()

        vote_counts = {}
        for _, a in self.vote_window:
            if a is None:
                continue
            vote_counts[a] = vote_counts.get(a, 0) + 1

        total_votes = len(self.vote_window)
        leader = None; leader_ratio = 0.0
        if vote_counts:
            leader = max(vote_counts, key=vote_counts.get)
            leader_ratio = vote_counts[leader] / total_votes if total_votes else 0.0
        elif self.last_confirmed_action is not None:
            self.last_confirmed_action = None

        # 發布
        published_now = False; source = None; data = None
        time_since_pub = now_t - self.last_publish_time
        if (total_votes >= MIN_VOTES and leader is not None and
                leader_ratio >= VOTE_THRESHOLD and
                (leader != self.last_confirmed_action or
                 time_since_pub >= REPUBLISH_INTERVAL)):
            confirmed = leader
            is_new_action = (leader != self.last_confirmed_action)
            self.last_confirmed_action = confirmed
            self.last_publish_time = now_t

            if april_action == confirmed:
                source = "TAG"
            elif yolo_action == confirmed or final_action == confirmed:
                source = "YOLO"
            else:
                source = "VOTE"

            if april_center is not None:
                data = f"{confirmed.lower()},{april_center[0]},{april_bottom_y},0"
            elif yolo_center and yolo_area:
                data = f"{confirmed.lower()},{yolo_center[0]},{yolo_bbox_bottom},{yolo_area}"
            else:
                data = f"{confirmed.lower()},{cx_frame},{cy_frame},0"

            ms = String(); ms.data = data
            self.class_id_pub.publish(ms)

            mp = Point()
            if april_center is not None:
                mp.x, mp.y = float(april_center[0]), float(april_center[1])
            elif yolo_center:
                mp.x, mp.y = float(yolo_center[0]), float(yolo_center[1])
            else:
                mp.x, mp.y = float(cx_frame), float(cy_frame)
            mp.z = 0.0
            self.coord_pub.publish(mp)
            published_now = True

        with self.state.lock:
            self.state.frame_count += 1
            if fps is not None:
                self.state.fps = fps
            self.state.frame_size = (w, h)
            self.state.april_action = april_action
            self.state.april_tag_ids = april_tag_ids
            self.state.yolo_action = yolo_action; self.state.yolo_name = yolo_name
            self.state.yolo_conf = yolo_conf; self.state.yolo_center = yolo_center
            self.state.yolo_area = yolo_area
            self.state.yolo_num_candidates = num_candidates
            self.state.geo_action = geo_action; self.state.final_action = final_action
            self.state.current_action = current_action
            self.state.vote_counts = vote_counts
            self.state.vote_leader = leader
            self.state.vote_leader_ratio = leader_ratio
            self.state.window_votes = total_votes
            self.state.last_confirmed_action = self.last_confirmed_action
            if published_now:
                self.state.published_count += 1
                self.state.last_publish_time = time.time()
                self.state.last_source = source
                if is_new_action:
                    self.state.publish_log.append({
                        'idx': self.state.published_count,
                        't': time.time(),
                        'action': self.last_confirmed_action,
                        'src': source, 'data': data,
                    })


def main(args=None):
    rclpy.init(args=args)
    state = TUIState()
    stop_event = threading.Event()
    node = NavigationNode(state)

    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    try:
        curses.wrapper(lambda scr: tui_loop(scr, state, stop_event))
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        if node.cap is not None:
            node.cap.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()