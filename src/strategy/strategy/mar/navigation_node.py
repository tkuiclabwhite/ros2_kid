import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading
import curses
from collections import deque
from pupil_apriltags import Detector
from ultralytics import YOLO

april_detector = Detector(
    families="tag36h11",
    nthreads=2,
    quad_decimate=1.0,
    quad_sigma=0.5,
    refine_edges=True,
    decode_sharpening=0.25,
)
TAG_ACTION = {1: "STRAIGHT", 2: "RIGHT", 3: "LEFT"}
WINDOW_SECONDS  = 1.0    # 投票時間視窗(秒)
VOTE_THRESHOLD  = 0.8    # 同一動作需占比 ≥ 80% 才確認
MIN_VOTES       = 3      # 視窗內至少要有幾幀才允許發布
REPUBLISH_INTERVAL = 0.3 # 同動作每 0.3s 重發一次,讓策略端持續收到最新座標


class TUIState:
    """執行緒安全的狀態容器,給 TUI 顯示用"""
    def __init__(self):
        self.lock = threading.Lock()
        self.frame_count = 0
        self.fps = 0.0
        self.frame_size = (0, 0)

        self.april_action = None
        self.april_tag_ids = []

        self.yolo_action = None
        self.yolo_name = None
        self.yolo_conf = 0.0
        self.yolo_center = None
        self.yolo_area = 0
        self.yolo_num_candidates = 0

        self.current_action = None
        self.last_confirmed_action = None
        self.vote_counts = {}            # 視窗內各動作票數
        self.vote_leader = None          # 票數最多的動作
        self.vote_leader_ratio = 0.0     # 領先者占比
        self.window_votes = 0            # 視窗內總票數
        self.published_count = 0
        self.last_publish_time = 0.0
        self.last_source = "—"
        self.publish_log = deque(maxlen=5)  # 最近 5 次發布記錄

        self.last_error = ""

    def snapshot(self):
        with self.lock:
            snap = {k: v for k, v in self.__dict__.items() if k != "lock"}
            snap['publish_log'] = list(self.publish_log)
            return snap


def tui_loop(stdscr, state: TUIState, stop_event: threading.Event):
    curses.curs_set(0)
    stdscr.nodelay(True)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN,  -1)
    curses.init_pair(2, curses.COLOR_YELLOW, -1)
    curses.init_pair(3, curses.COLOR_CYAN,   -1)
    curses.init_pair(4, curses.COLOR_RED,    -1)
    curses.init_pair(5, curses.COLOR_MAGENTA, -1)

    GREEN, YELLOW, CYAN, RED, MAGENTA = (curses.color_pair(i) for i in range(1, 6))

    while not stop_event.is_set():
        s = state.snapshot()
        stdscr.erase()
        try:
            row = 0
            stdscr.addstr(row, 2, "═══ Navigation Node Monitor ═══", CYAN | curses.A_BOLD); row += 2

            stdscr.addstr(row, 2, f"Frames : {s['frame_count']:>8}    "
                                  f"FPS: {s['fps']:>5.1f}    "
                                  f"Size: {s['frame_size'][0]}x{s['frame_size'][1]}")
            row += 2

            stdscr.addstr(row, 2, "── AprilTag ──", YELLOW | curses.A_BOLD); row += 1
            tag_color = GREEN if s['april_action'] else curses.A_DIM
            stdscr.addstr(row, 4, f"Action : {s['april_action'] or '—'}", tag_color); row += 1
            stdscr.addstr(row, 4, f"Tag IDs: {s['april_tag_ids'] or '—'}"); row += 2

            stdscr.addstr(row, 2, "── YOLO ──", YELLOW | curses.A_BOLD); row += 1
            yolo_color = GREEN if s['yolo_action'] else curses.A_DIM
            stdscr.addstr(row, 4, f"Action : {s['yolo_action'] or '—'}", yolo_color); row += 1
            stdscr.addstr(row, 4, f"Class  : {s['yolo_name'] or '—'}  conf={s['yolo_conf']:.2f}"); row += 1
            stdscr.addstr(row, 4, f"Center : {s['yolo_center'] or '—'}  area={s['yolo_area']}"); row += 1
            stdscr.addstr(row, 4, f"Candi. : {s['yolo_num_candidates']}"); row += 2

            stdscr.addstr(row, 2, "── Decision ──", YELLOW | curses.A_BOLD); row += 1
            cur_color = GREEN if s['current_action'] else RED
            stdscr.addstr(row, 4, f"Current  : {s['current_action'] or 'NONE'}", cur_color); row += 1

            # 投票進度條
            ratio = s['vote_leader_ratio']
            bar_w = 20
            filled = int(bar_w * ratio)
            bar = "█" * filled + "░" * (bar_w - filled)
            ratio_color = GREEN if ratio >= VOTE_THRESHOLD else YELLOW
            stdscr.addstr(row, 4, f"Window   : {s['window_votes']} votes / {WINDOW_SECONDS:.1f}s")
            row += 1
            stdscr.addstr(row, 4,
                f"Leader   : {s['vote_leader'] or '—':<8} [{bar}] {ratio*100:5.1f}%",
                ratio_color); row += 1
            # 顯示各動作票數
            counts_str = "  ".join(f"{k}:{v}" for k, v in s['vote_counts'].items()) or "—"
            stdscr.addstr(row, 4, f"Counts   : {counts_str}"); row += 1
            stdscr.addstr(row, 4, f"Confirmed: {s['last_confirmed_action'] or '—'}",
                          MAGENTA | curses.A_BOLD); row += 2

            stdscr.addstr(row, 2, "── Publish ──", YELLOW | curses.A_BOLD); row += 1
            stdscr.addstr(row, 4, f"Count    : {s['published_count']}"); row += 1
            stdscr.addstr(row, 4, f"Source   : {s['last_source']}"); row += 1
            if s['last_publish_time']:
                ago = time.time() - s['last_publish_time']
                stdscr.addstr(row, 4, f"Last pub : {ago:5.1f}s ago"); row += 1
            row += 1

            stdscr.addstr(row, 2, "── Recent Publishes (last 5) ──", YELLOW | curses.A_BOLD); row += 1
            if not s['publish_log']:
                stdscr.addstr(row, 4, "(no publishes yet)", curses.A_DIM); row += 1
            else:
                stdscr.addstr(row, 4,
                    f"{'#':<3}{'time':<10}{'action':<10}{'src':<6}{'data'}",
                    curses.A_UNDERLINE); row += 1
                # 最新的放最上面
                for entry in reversed(s['publish_log']):
                    ago = time.time() - entry['t']
                    color = GREEN if entry['src'] == 'TAG' else CYAN
                    line = (f"{entry['idx']:<3}"
                            f"{ago:>5.1f}s ago "
                            f"{entry['action']:<10}"
                            f"{entry['src']:<6}"
                            f"{entry['data']}")
                    stdscr.addstr(row, 4, line[:curses.COLS - 6], color); row += 1
            row += 1

            if s['last_error']:
                stdscr.addstr(row, 2, f"ERROR: {s['last_error']}", RED); row += 1

            stdscr.addstr(row + 1, 2, "Press Ctrl+C to quit", curses.A_DIM)
        except curses.error:
            pass  # 視窗太小

        stdscr.refresh()
        time.sleep(0.1)


class NavigationNode(Node):
    def __init__(self, state: TUIState):
        super().__init__('navigation_node')
        self.state = state
        self.bridge = CvBridge()
        # 每筆 = (timestamp, action)。action 可能為 None
        self.vote_window = deque()
        self.last_confirmed_action = None
        self.last_publish_time = 0.0

        self._fps_t0 = time.time()
        self._fps_count = 0

        self.yolo_model = YOLO('strategy/strategy/mar/best1.onnx')
        self.yolo_class_names = {0: "left", 1: "right", 2: "straight"}
        self.yolo_to_action  = {"left": "LEFT", "right": "RIGHT", "straight": "STRAIGHT"}
        self.conf_threshold  = 0.5

        self.image_sub = self.create_subscription(
            Image, '/camera1/image_raw', self.image_callback, 10)
        self.class_id_pub = self.create_publisher(String, 'class_id_topic', 1)
        self.coord_pub    = self.create_publisher(Point,  'sign_coordinates', 1)

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            with self.state.lock:
                self.state.last_error = f'cv_bridge: {e}'
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = frame.shape[:2]
        cx_frame, cy_frame = w // 2, h // 2

        # FPS
        self._fps_count += 1
        now = time.time()
        if now - self._fps_t0 >= 1.0:
            fps = self._fps_count / (now - self._fps_t0)
            self._fps_count = 0
            self._fps_t0 = now
        else:
            fps = None

        # AprilTag
        april_results = april_detector.detect(gray)
        april_action = None
        april_tag_ids = []
        april_center = None       # TAG 的中心 X
        april_bottom_y = None     # TAG 的底部 Y
        for r in april_results:
            april_tag_ids.append(r.tag_id)
            action = TAG_ACTION.get(r.tag_id)
            if action:
                april_action = action
                # 從 4 個角點計算中心 X 和底部 Y
                corners = r.corners  # shape (4, 2),每列是 (x, y)
                cx_tag = int(corners[:, 0].mean())
                bottom_y_tag = int(corners[:, 1].max())
                april_center = (cx_tag, int(corners[:, 1].mean()))
                april_bottom_y = bottom_y_tag

        # YOLO
        yolo_action = yolo_name = yolo_center = None
        yolo_conf = 0.0
        yolo_area = 0
        yolo_bbox_bottom = None
        num_candidates = 0
        if not april_action:
            results = self.yolo_model(frame, imgsz=640, verbose=False)
            candidates = []
            for box in results[0].boxes:
                conf = float(box.conf[0])
                cls  = int(box.cls[0])
                if cls not in self.yolo_class_names or conf < self.conf_threshold:
                    continue
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                area = (x2 - x1) * (y2 - y1)
                candidates.append({
                    'name': self.yolo_class_names[cls],
                    'conf': conf,
                    'center': ((x1+x2)//2, (y1+y2)//2),
                    'area': area,
                    'bbox_bottom': y2,
                })
            num_candidates = len(candidates)
            if candidates:
                best = max(candidates, key=lambda x: x['area'])
                yolo_name        = best['name']
                yolo_conf        = best['conf']
                yolo_action      = self.yolo_to_action.get(best['name'])
                yolo_center      = best['center']
                yolo_area        = best['area']
                yolo_bbox_bottom = best['bbox_bottom']

        current_action = april_action or yolo_action

        # 將本幀加入投票視窗,並丟掉超出時間範圍的舊資料
        now_t = time.time()
        self.vote_window.append((now_t, current_action))
        cutoff = now_t - WINDOW_SECONDS
        while self.vote_window and self.vote_window[0][0] < cutoff:
            self.vote_window.popleft()

        # 統計視窗內各動作票數(忽略 None)
        vote_counts = {}
        for _, a in self.vote_window:
            if a is None:
                continue
            vote_counts[a] = vote_counts.get(a, 0) + 1

        total_votes = len(self.vote_window)  # 含 None,當分母才公平
        leader = None
        leader_ratio = 0.0
        if vote_counts:
            leader = max(vote_counts, key=vote_counts.get)
            leader_ratio = vote_counts[leader] / total_votes if total_votes else 0.0
        else:
            # 視窗內完全沒看到任何號誌 → 重置確認狀態,
            # 這樣下次再看到同樣的號誌會被視為新指令
            if self.last_confirmed_action is not None:
                self.last_confirmed_action = None

        published_now = False
        source = None
        # 發布條件:(投票通過) 且 (動作改變 或 距上次發布超過重發間隔)
        time_since_pub = now_t - self.last_publish_time
        should_publish = (
            total_votes >= MIN_VOTES and
            leader is not None and
            leader_ratio >= VOTE_THRESHOLD and
            (leader != self.last_confirmed_action or
             time_since_pub >= REPUBLISH_INTERVAL)
        )
        if should_publish:
            confirmed = leader
            is_new_action = (leader != self.last_confirmed_action)
            self.last_confirmed_action = confirmed
            self.last_publish_time = now_t
            # 來源以「本幀是誰偵測到的」為準;若本幀沒偵測到 leader,標 VOTE
            if april_action == confirmed:
                source = "TAG"
            elif yolo_action == confirmed:
                source = "YOLO"
            else:
                source = "VOTE"

            if april_center is not None:
                # TAG 路徑:用 tag 自己的座標
                cx, cy = april_center
                data = f"{confirmed.lower()},{cx},{april_bottom_y},0"
            elif yolo_center and yolo_area:
                cx, cy = yolo_center
                data = f"{confirmed.lower()},{cx},{yolo_bbox_bottom},{yolo_area}"
            else:
                data = f"{confirmed.lower()},{cx_frame},{cy_frame},0"

            msg_str = String(); msg_str.data = data
            self.class_id_pub.publish(msg_str)

            msg_pt = Point()
            if april_center is not None:
                msg_pt.x = float(april_center[0])
                msg_pt.y = float(april_center[1])
            elif yolo_center:
                msg_pt.x = float(yolo_center[0])
                msg_pt.y = float(yolo_center[1])
            else:
                msg_pt.x = float(cx_frame)
                msg_pt.y = float(cy_frame)
            msg_pt.z = 0.0
            self.coord_pub.publish(msg_pt)
            published_now = True

        # 更新 TUI 狀態
        with self.state.lock:
            self.state.frame_count += 1
            if fps is not None:
                self.state.fps = fps
            self.state.frame_size = (w, h)
            self.state.april_action = april_action
            self.state.april_tag_ids = april_tag_ids
            self.state.yolo_action = yolo_action
            self.state.yolo_name = yolo_name
            self.state.yolo_conf = yolo_conf
            self.state.yolo_center = yolo_center
            self.state.yolo_area = yolo_area
            self.state.yolo_num_candidates = num_candidates
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
                # 只記錄「新動作」到 log,避免被重發灌爆
                if is_new_action:
                    self.state.publish_log.append({
                        'idx': self.state.published_count,
                        't': time.time(),
                        'action': self.last_confirmed_action,
                        'src': source,
                        'data': data,
                    })


def main(args=None):
    rclpy.init(args=args)
    state = TUIState()
    stop_event = threading.Event()
    node = NavigationNode(state)

    # ROS spin 放到背景執行緒
    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    try:
        curses.wrapper(lambda scr: tui_loop(scr, state, stop_event))
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import String
# from geometry_msgs.msg import Point
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# import time
# import threading
# import curses
# from collections import deque
# from pupil_apriltags import Detector
# from ultralytics import YOLO

# april_detector = Detector(
#     families="tag36h11",
#     nthreads=2,
#     quad_decimate=1.0,
#     quad_sigma=0.5,
#     refine_edges=True,
#     decode_sharpening=0.25,
# )
# TAG_ACTION = {1: "FORWARD", 2: "RIGHT", 3: "LEFT"}
# WINDOW_SECONDS  = 1.0    # 投票時間視窗(秒)
# VOTE_THRESHOLD  = 0.8    # 同一動作需占比 ≥ 80% 才確認
# MIN_VOTES       = 3      # 視窗內至少要有幾幀才允許發布(避免低 FPS 時 1 幀就發)


# class TUIState:
#     """執行緒安全的狀態容器,給 TUI 顯示用"""
#     def __init__(self):
#         self.lock = threading.Lock()
#         self.frame_count = 0
#         self.fps = 0.0
#         self.frame_size = (0, 0)

#         self.april_action = None
#         self.april_tag_ids = []

#         self.yolo_action = None
#         self.yolo_name = None
#         self.yolo_conf = 0.0
#         self.yolo_center = None
#         self.yolo_area = 0
#         self.yolo_num_candidates = 0

#         self.current_action = None
#         self.last_confirmed_action = None
#         self.vote_counts = {}            # 視窗內各動作票數
#         self.vote_leader = None          # 票數最多的動作
#         self.vote_leader_ratio = 0.0     # 領先者占比
#         self.window_votes = 0            # 視窗內總票數
#         self.published_count = 0
#         self.last_publish_time = 0.0
#         self.last_source = "—"
#         self.publish_log = deque(maxlen=5)  # 最近 5 次發布記錄

#         self.last_error = ""

#     def snapshot(self):
#         with self.lock:
#             snap = {k: v for k, v in self.__dict__.items() if k != "lock"}
#             snap['publish_log'] = list(self.publish_log)
#             return snap


# def tui_loop(stdscr, state: TUIState, stop_event: threading.Event):
#     curses.curs_set(0)
#     stdscr.nodelay(True)
#     curses.start_color()
#     curses.use_default_colors()
#     curses.init_pair(1, curses.COLOR_GREEN,  -1)
#     curses.init_pair(2, curses.COLOR_YELLOW, -1)
#     curses.init_pair(3, curses.COLOR_CYAN,   -1)
#     curses.init_pair(4, curses.COLOR_RED,    -1)
#     curses.init_pair(5, curses.COLOR_MAGENTA, -1)

#     GREEN, YELLOW, CYAN, RED, MAGENTA = (curses.color_pair(i) for i in range(1, 6))

#     while not stop_event.is_set():
#         s = state.snapshot()
#         stdscr.erase()
#         try:
#             row = 0
#             stdscr.addstr(row, 2, "═══ Navigation Node Monitor ═══", CYAN | curses.A_BOLD); row += 2

#             stdscr.addstr(row, 2, f"Frames : {s['frame_count']:>8}    "
#                                   f"FPS: {s['fps']:>5.1f}    "
#                                   f"Size: {s['frame_size'][0]}x{s['frame_size'][1]}")
#             row += 2

#             stdscr.addstr(row, 2, "── AprilTag ──", YELLOW | curses.A_BOLD); row += 1
#             tag_color = GREEN if s['april_action'] else curses.A_DIM
#             stdscr.addstr(row, 4, f"Action : {s['april_action'] or '—'}", tag_color); row += 1
#             stdscr.addstr(row, 4, f"Tag IDs: {s['april_tag_ids'] or '—'}"); row += 2

#             stdscr.addstr(row, 2, "── YOLO ──", YELLOW | curses.A_BOLD); row += 1
#             yolo_color = GREEN if s['yolo_action'] else curses.A_DIM
#             stdscr.addstr(row, 4, f"Action : {s['yolo_action'] or '—'}", yolo_color); row += 1
#             stdscr.addstr(row, 4, f"Class  : {s['yolo_name'] or '—'}  conf={s['yolo_conf']:.2f}"); row += 1
#             stdscr.addstr(row, 4, f"Center : {s['yolo_center'] or '—'}  area={s['yolo_area']}"); row += 1
#             stdscr.addstr(row, 4, f"Candi. : {s['yolo_num_candidates']}"); row += 2

#             stdscr.addstr(row, 2, "── Decision ──", YELLOW | curses.A_BOLD); row += 1
#             cur_color = GREEN if s['current_action'] else RED
#             stdscr.addstr(row, 4, f"Current  : {s['current_action'] or 'NONE'}", cur_color); row += 1

#             # 投票進度條
#             ratio = s['vote_leader_ratio']
#             bar_w = 20
#             filled = int(bar_w * ratio)
#             bar = "█" * filled + "░" * (bar_w - filled)
#             ratio_color = GREEN if ratio >= VOTE_THRESHOLD else YELLOW
#             stdscr.addstr(row, 4, f"Window   : {s['window_votes']} votes / {WINDOW_SECONDS:.1f}s")
#             row += 1
#             stdscr.addstr(row, 4,
#                 f"Leader   : {s['vote_leader'] or '—':<8} [{bar}] {ratio*100:5.1f}%",
#                 ratio_color); row += 1
#             # 顯示各動作票數
#             counts_str = "  ".join(f"{k}:{v}" for k, v in s['vote_counts'].items()) or "—"
#             stdscr.addstr(row, 4, f"Counts   : {counts_str}"); row += 1
#             stdscr.addstr(row, 4, f"Confirmed: {s['last_confirmed_action'] or '—'}",
#                           MAGENTA | curses.A_BOLD); row += 2

#             stdscr.addstr(row, 2, "── Publish ──", YELLOW | curses.A_BOLD); row += 1
#             stdscr.addstr(row, 4, f"Count    : {s['published_count']}"); row += 1
#             stdscr.addstr(row, 4, f"Source   : {s['last_source']}"); row += 1
#             if s['last_publish_time']:
#                 ago = time.time() - s['last_publish_time']
#                 stdscr.addstr(row, 4, f"Last pub : {ago:5.1f}s ago"); row += 1
#             row += 1

#             stdscr.addstr(row, 2, "── Recent Publishes (last 5) ──", YELLOW | curses.A_BOLD); row += 1
#             if not s['publish_log']:
#                 stdscr.addstr(row, 4, "(no publishes yet)", curses.A_DIM); row += 1
#             else:
#                 stdscr.addstr(row, 4,
#                     f"{'#':<3}{'time':<10}{'action':<10}{'src':<6}{'data'}",
#                     curses.A_UNDERLINE); row += 1
#                 # 最新的放最上面
#                 for entry in reversed(s['publish_log']):
#                     ago = time.time() - entry['t']
#                     color = GREEN if entry['src'] == 'TAG' else CYAN
#                     line = (f"{entry['idx']:<3}"
#                             f"{ago:>5.1f}s ago "
#                             f"{entry['action']:<10}"
#                             f"{entry['src']:<6}"
#                             f"{entry['data']}")
#                     stdscr.addstr(row, 4, line[:curses.COLS - 6], color); row += 1
#             row += 1

#             if s['last_error']:
#                 stdscr.addstr(row, 2, f"ERROR: {s['last_error']}", RED); row += 1

#             stdscr.addstr(row + 1, 2, "Press Ctrl+C to quit", curses.A_DIM)
#         except curses.error:
#             pass  # 視窗太小

#         stdscr.refresh()
#         time.sleep(0.1)


# class NavigationNode(Node):
#     def __init__(self, state: TUIState):
#         super().__init__('navigation_node')
#         self.state = state
#         self.bridge = CvBridge()
#         # 每筆 = (timestamp, action)。action 可能為 None
#         self.vote_window = deque()
#         self.last_confirmed_action = None

#         self._fps_t0 = time.time()
#         self._fps_count = 0

#         self.yolo_model = YOLO('strategy/strategy/mar/best1.onnx')
#         self.yolo_class_names = {0: "left", 1: "right", 2: "straight"}
#         self.yolo_to_action  = {"left": "LEFT", "right": "RIGHT", "straight": "FORWARD"}
#         self.conf_threshold  = 0.5

#         self.image_sub = self.create_subscription(
#             Image, '/camera1/image_raw', self.image_callback, 10)
#         self.class_id_pub = self.create_publisher(String, 'class_id_topic', 1)
#         self.coord_pub    = self.create_publisher(Point,  'sign_coordinates', 1)

#     def image_callback(self, msg: Image):
#         try:
#             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except Exception as e:
#             with self.state.lock:
#                 self.state.last_error = f'cv_bridge: {e}'
#             return

#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         h, w = frame.shape[:2]
#         cx_frame, cy_frame = w // 2, h // 2

#         # FPS
#         self._fps_count += 1
#         now = time.time()
#         if now - self._fps_t0 >= 1.0:
#             fps = self._fps_count / (now - self._fps_t0)
#             self._fps_count = 0
#             self._fps_t0 = now
#         else:
#             fps = None

#         # AprilTag
#         april_results = april_detector.detect(gray)
#         april_action = None
#         april_tag_ids = []
#         for r in april_results:
#             april_tag_ids.append(r.tag_id)
#             action = TAG_ACTION.get(r.tag_id)
#             if action:
#                 april_action = action

#         # YOLO
#         yolo_action = yolo_name = yolo_center = None
#         yolo_conf = 0.0
#         yolo_area = 0
#         yolo_bbox_bottom = None
#         num_candidates = 0
#         if not april_action:
#             results = self.yolo_model(frame, imgsz=640, verbose=False)
#             candidates = []
#             for box in results[0].boxes:
#                 conf = float(box.conf[0])
#                 cls  = int(box.cls[0])
#                 if cls not in self.yolo_class_names or conf < self.conf_threshold:
#                     continue
#                 x1, y1, x2, y2 = map(int, box.xyxy[0])
#                 area = (x2 - x1) * (y2 - y1)
#                 candidates.append({
#                     'name': self.yolo_class_names[cls],
#                     'conf': conf,
#                     'center': ((x1+x2)//2, (y1+y2)//2),
#                     'area': area,
#                     'bbox_bottom': y2,
#                 })
#             num_candidates = len(candidates)
#             if candidates:
#                 best = max(candidates, key=lambda x: x['area'])
#                 yolo_name        = best['name']
#                 yolo_conf        = best['conf']
#                 yolo_action      = self.yolo_to_action.get(best['name'])
#                 yolo_center      = best['center']
#                 yolo_area        = best['area']
#                 yolo_bbox_bottom = best['bbox_bottom']

#         current_action = april_action or yolo_action

#         # 將本幀加入投票視窗,並丟掉超出時間範圍的舊資料
#         now_t = time.time()
#         self.vote_window.append((now_t, current_action))
#         cutoff = now_t - WINDOW_SECONDS
#         while self.vote_window and self.vote_window[0][0] < cutoff:
#             self.vote_window.popleft()

#         # 統計視窗內各動作票數(忽略 None)
#         vote_counts = {}
#         for _, a in self.vote_window:
#             if a is None:
#                 continue
#             vote_counts[a] = vote_counts.get(a, 0) + 1

#         total_votes = len(self.vote_window)  # 含 None,當分母才公平
#         leader = None
#         leader_ratio = 0.0
#         if vote_counts:
#             leader = max(vote_counts, key=vote_counts.get)
#             leader_ratio = vote_counts[leader] / total_votes if total_votes else 0.0

#         published_now = False
#         source = None
#         if (total_votes >= MIN_VOTES and
#                 leader is not None and
#                 leader_ratio >= VOTE_THRESHOLD and
#                 leader != self.last_confirmed_action):
#             confirmed = leader
#             self.last_confirmed_action = confirmed
#             # 來源以「本幀是誰偵測到的」為準;若本幀沒偵測到 leader,標 VOTE
#             if april_action == confirmed:
#                 source = "TAG"
#             elif yolo_action == confirmed:
#                 source = "YOLO"
#             else:
#                 source = "VOTE"

#             if yolo_center and yolo_area:
#                 cx, cy = yolo_center
#                 data = f"{confirmed.lower()},{cx},{yolo_bbox_bottom},{yolo_area}"
#             else:
#                 data = f"{confirmed.lower()},{cx_frame},{cy_frame},0"

#             msg_str = String(); msg_str.data = data
#             self.class_id_pub.publish(msg_str)

#             msg_pt = Point()
#             msg_pt.x = float(yolo_center[0]) if yolo_center else float(cx_frame)
#             msg_pt.y = float(yolo_center[1]) if yolo_center else float(cy_frame)
#             msg_pt.z = 0.0
#             self.coord_pub.publish(msg_pt)
#             published_now = True

#         # 更新 TUI 狀態
#         with self.state.lock:
#             self.state.frame_count += 1
#             if fps is not None:
#                 self.state.fps = fps
#             self.state.frame_size = (w, h)
#             self.state.april_action = april_action
#             self.state.april_tag_ids = april_tag_ids
#             self.state.yolo_action = yolo_action
#             self.state.yolo_name = yolo_name
#             self.state.yolo_conf = yolo_conf
#             self.state.yolo_center = yolo_center
#             self.state.yolo_area = yolo_area
#             self.state.yolo_num_candidates = num_candidates
#             self.state.current_action = current_action
#             self.state.vote_counts = vote_counts
#             self.state.vote_leader = leader
#             self.state.vote_leader_ratio = leader_ratio
#             self.state.window_votes = total_votes
#             self.state.last_confirmed_action = self.last_confirmed_action
#             if published_now:
#                 self.state.published_count += 1
#                 self.state.last_publish_time = time.time()
#                 self.state.last_source = source
#                 self.state.publish_log.append({
#                     'idx': self.state.published_count,
#                     't': time.time(),
#                     'action': self.last_confirmed_action,
#                     'src': source,
#                     'data': data,
#                 })


# def main(args=None):
#     rclpy.init(args=args)
#     state = TUIState()
#     stop_event = threading.Event()
#     node = NavigationNode(state)

#     # ROS spin 放到背景執行緒
#     spin_thread = threading.Thread(
#         target=lambda: rclpy.spin(node), daemon=True)
#     spin_thread.start()

#     try:
#         curses.wrapper(lambda scr: tui_loop(scr, state, stop_event))
#     except KeyboardInterrupt:
#         pass
#     finally:
#         stop_event.set()
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == '__main__':
#     main()
# # import rclpy
# # from rclpy.node import Node
# # from sensor_msgs.msg import Image
# # from std_msgs.msg import String
# # from geometry_msgs.msg import Point
# # from cv_bridge import CvBridge
# # import cv2
# # import numpy as np
# # from collections import deque
# # from pupil_apriltags import Detector
# # from ultralytics import YOLO

# # april_detector = Detector(
# #     families="tag36h11",
# #     nthreads=2,
# #     quad_decimate=1.0,
# #     quad_sigma=0.5,
# #     refine_edges=True,
# #     decode_sharpening=0.25,
# # )
# # TAG_ACTION = {1: "FORWARD", 2: "RIGHT", 3: "LEFT"}

# # MIN_ARROW_AREA = 2000
# # MIN_DARK_AREA  = 1500
# # BLACK_THRESH   = 80
# # WHITE_THRESH   = 150
# # CONFIRM_FRAMES = 3


# # def find_arrow_boxes(frame, masked, black_thresh, min_dark_area):
# #     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
# #     h_f, w_f = frame.shape[:2]

# #     clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
# #     gray_eq = clahe.apply(gray)

# #     median_val = np.median(gray_eq)
# #     canny_lo = int(max(0, 0.5 * median_val))
# #     canny_hi = int(min(255, 1.5 * median_val))
# #     edges = cv2.Canny(gray_eq, canny_lo, canny_hi)

# #     kernel = np.ones((3, 3), np.uint8)
# #     edges = cv2.dilate(edges, kernel, iterations=1)

# #     contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# #     boxes = []

# #     for cnt in contours:
# #         if cv2.contourArea(cnt) < min_dark_area:
# #             continue
# #         x, y, w, h = cv2.boundingRect(cnt)
# #         if w > w_f * 0.9 or h > h_f * 0.9:
# #             continue
# #         if w * h > w_f * h_f * 0.4:
# #             continue
# #         if not (0.4 < w / h < 2.5):
# #             continue
# #         roi_gray = gray[y:y+h, x:x+w]
# #         if np.mean(roi_gray) > 180:
# #             continue
# #         dark_px = np.sum(roi_gray < np.percentile(roi_gray, 50))
# #         if dark_px / (w * h) < 0.25:
# #             continue
# #         roi_bgr_check = masked[y:y+h, x:x+w]
# #         gray_mean = cv2.mean(cv2.cvtColor(roi_bgr_check, cv2.COLOR_BGR2GRAY))[0]
# #         if 115 < gray_mean < 140:
# #             continue
# #         boxes.append((x, y, w, h))
# #     return boxes


# # def detect_arrow(roi_bgr, min_arrow_area):
# #     gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
# #     clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(4, 4))
# #     gray = clahe.apply(gray)

# #     top_bright = np.percentile(gray, 95)
# #     bottom_dark = np.percentile(gray, 5)
# #     diff = top_bright - bottom_dark
# #     if diff < 30:
# #         _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
# #     else:
# #         threshold = bottom_dark + diff * 0.6
# #         _, binary = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)

# #     if cv2.countNonZero(binary) < min_arrow_area:
# #         return None

# #     contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# #     if not contours:
# #         return None
# #     largest = max(contours, key=cv2.contourArea)
# #     if cv2.contourArea(largest) < min_arrow_area:
# #         return None

# #     M = cv2.moments(largest)
# #     if M["m00"] == 0:
# #         return None
# #     cx = M["m10"] / M["m00"]
# #     cy = M["m01"] / M["m00"]

# #     h, w = binary.shape
# #     cx_i, cy_i = int(cx), int(cy)

# #     w_top    = cv2.countNonZero(binary[:cy_i, :])
# #     w_bottom = cv2.countNonZero(binary[cy_i:, :])
# #     tb_diff  = w_top - w_bottom

# #     inv = cv2.bitwise_not(binary)
# #     b_bottom_left  = cv2.countNonZero(inv[cy_i:, :cx_i])
# #     b_bottom_right = cv2.countNonZero(inv[cy_i:, cx_i:])

# #     center_band  = binary[:, w//3 : 2*w//3]
# #     center_ratio = cv2.countNonZero(center_band) / center_band.size

# #     if center_ratio > 0.25 and tb_diff > 0:
# #         return "FORWARD"
# #     if b_bottom_right > b_bottom_left:
# #         return "RIGHT"
# #     return "LEFT"


# # class NavigationNode(Node):
# #     def __init__(self):
# #         super().__init__('navigation_node')

# #         self.bridge = CvBridge()
# #         self.action_history = deque(maxlen=CONFIRM_FRAMES)
# #         self.last_confirmed_action = None

# #         # 載入 YOLO（備援）
# #         self.get_logger().info('載入 YOLO 模型...')
# #         self.yolo_model = YOLO('strategy/strategy/mar/best1.onnx')
# #         self.yolo_class_names = {0: "left", 1: "right", 2: "straight"}
# #         self.yolo_to_action  = {"left": "LEFT", "right": "RIGHT", "straight": "FORWARD"}
# #         self.conf_threshold  = 0.5
# #         self.get_logger().info('✅ 模型載入完成')

# #         # 訂閱
# #         self.image_sub = self.create_subscription(
# #             Image, '/camera1/image_raw', self.image_callback, 10)

# #         # 發布（與原 YOLO 節點相同 topic）
# #         self.class_id_pub = self.create_publisher(String, 'class_id_topic', 1)
# #         self.coord_pub    = self.create_publisher(Point,  'sign_coordinates', 1)

# #     def image_callback(self, msg: Image):
# #         try:
# #             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
# #         except Exception as e:
# #             self.get_logger().error(f'cv_bridge: {e}')
# #             return

# #         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
# #         h, w = frame.shape[:2]
# #         cx_frame, cy_frame = w // 2, h // 2

# #         # ── 1. AprilTag（最高優先）────────────────────────
# #         april_results = april_detector.detect(gray)
# #         april_action = None
# #         april_corners_list = []
# #         for r in april_results:
# #             action = TAG_ACTION.get(r.tag_id)
# #             if action:
# #                 april_action = action
# #                 april_corners_list.append(r.corners.astype(int))

# #         # 遮蔽 AprilTag 區域
# #         masked = frame.copy()
# #         for corners in april_corners_list:
# #             cv2.fillPoly(masked, [corners], (127, 127, 127))

# #         # ── 2. YOLO（AprilTag 無時備援）──────────────────
# #         yolo_action = None
# #         yolo_center = None
# #         yolo_area   = None
# #         yolo_bbox_bottom = None
# #         if not april_action:
# #             results = self.yolo_model(frame, imgsz=640, verbose=False)
# #             candidates = []
# #             for box in results[0].boxes:
# #                 conf = float(box.conf[0])
# #                 cls  = int(box.cls[0])
# #                 if cls not in self.yolo_class_names or conf < self.conf_threshold:
# #                     continue
# #                 x1, y1, x2, y2 = map(int, box.xyxy[0])
# #                 area = (x2 - x1) * (y2 - y1)
# #                 candidates.append({
# #                     'name': self.yolo_class_names[cls],
# #                     'center': ((x1+x2)//2, (y1+y2)//2),
# #                     'area': area,
# #                     'bbox_bottom': y2,
# #                 })
# #             if candidates:
# #                 best = max(candidates, key=lambda x: x['area'])
# #                 yolo_action      = self.yolo_to_action.get(best['name'])
# #                 yolo_center      = best['center']
# #                 yolo_area        = best['area']
# #                 yolo_bbox_bottom = best['bbox_bottom']

# #         # ── 決策 ─────────────────────────────────────────
# #         current_action = april_action or yolo_action

# #         # 連續幀確認
# #         self.action_history.append(current_action)
# #         if (len(self.action_history) == CONFIRM_FRAMES and
# #                 len(set(self.action_history)) == 1):
# #             confirmed = self.action_history[0]
# #             if confirmed and confirmed != self.last_confirmed_action:
# #                 self.last_confirmed_action = confirmed
# #                 source = "TAG" if april_action else "YOLO"
# #                 self.get_logger().info(
# #                     f'ACTION: {confirmed}  [{source}]'
# #                 )

# #                 # 發布 class_id_topic
# #                 if yolo_center and yolo_area:
# #                     cx, cy = yolo_center
# #                     data = f"{confirmed.lower()},{cx},{yolo_bbox_bottom},{yolo_area}"
# #                 else:
# #                     data = f"{confirmed.lower()},{cx_frame},{cy_frame},0"

# #                 msg_str = String()
# #                 msg_str.data = data
# #                 self.class_id_pub.publish(msg_str)

# #                 # 發布 sign_coordinates
# #                 msg_pt = Point()
# #                 msg_pt.x = float(yolo_center[0]) if yolo_center else float(cx_frame)
# #                 msg_pt.y = float(yolo_center[1]) if yolo_center else float(cy_frame)
# #                 msg_pt.z = 0.0
# #                 self.coord_pub.publish(msg_pt)


# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = NavigationNode()
# #     try:
# #         rclpy.spin(node)
# #     except KeyboardInterrupt:
# #         pass
# #     finally:
# #         node.destroy_node()
# #         if rclpy.ok():
# #             rclpy.shutdown()


# # if __name__ == '__main__':
# #     main()