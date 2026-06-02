#!/usr/bin/env python3
# coding=utf-8
import rclpy
import time
import math
import numpy as np
import cv2
import curses
import threading
import os
from datetime import datetime
from strategy.API import API
from tku_msgs.msg import Dio
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tku_msgs.msg import SensorPackage

# ─────────────────────────────────────────────
#  全域參數
# ─────────────────────────────────────────────
WIGHT = 70
HEAD_MOTOR_START = 1500
HEAD_MOTOR_FINISH = 1350
FLAG1 = False
PRETURN = 1

if WIGHT == 86:
    THIRD_LINE = 200;  SPEED = 2000
    PICK_ONE = 861;    PICK_TWO = 862;  PICK_THREE = 863
    LIFT = 864;        FINAL = 865
elif WIGHT == 90:
    THIRD_LINE = 200;  SPEED = 2100
    PICK_ONE = 901;    PICK_TWO = 902;  PICK_THREE = 903
    LIFT = 904;        FINAL = 905
elif WIGHT == 80:
    THIRD_LINE = 224;  FOURTH_LINE = 224;  SPEED = 1300
    PICK_ONE = 801;    PICK_TWO = 802;  PICK_THREE = 803
    LIFT = 804;        FINAL = 805
elif WIGHT == 70:
    THIRD_LINE = 225;  FOURTH_LINE = 225;  SPEED = 1300
    PICK_ONE = 701;    PICK_TWO = 702;  PICK_THREE = 703
    LIFT = 704;        FINAL = 705
elif WIGHT == 60:
    SPEED = 1300;      THIRD_LINE = 200;   FOURTH_LINE = 210
    PICK_ONE = 601;    PICK_TWO = 602;  PICK_THREE = 603
    LIFT = 604;        FINAL = 605
elif WIGHT == 50:
    SPEED = 1800;      THIRD_LINE = 205
    PICK_ONE = 501;    PICK_TWO = 502;  PICK_THREE = 503
    LIFT = 504;        FINAL = 505
else:
    THIRD_LINE = 215;  SPEED = 1300
    PICK_ONE = 401;    PICK_TWO = 402;  PICK_THREE = 403
    LIFT = 404;        FINAL = 405

# ─────────────────────────────────────────────
#  TUI Logger
# ─────────────────────────────────────────────
class TUILogger:
    def __init__(self, log_dir="logs"):
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = os.path.join(log_dir, f"wl_{ts}.txt")
        self._file = open(self.log_path, "w", encoding="utf-8")
        self._file.write(f"=== WeightLift Log  {ts} ===\n")
        self._file.write(f"WIGHT={WIGHT}  PRETURN={PRETURN}  FLAG1={FLAG1}\n")
        self._file.write(f"THIRD_LINE={THIRD_LINE}  SPEED={SPEED}\n")
        self._file.write("=" * 50 + "\n\n")
        self._file.flush()
        self.messages = []
        self._lock = threading.Lock()
        self.MAX_MSGS = 200

    def log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"[{ts}] {msg}"
        with self._lock:
            self.messages.append(line)
            if len(self.messages) > self.MAX_MSGS:
                self.messages.pop(0)
        self._file.write(line + "\n")
        self._file.flush()

    def close(self):
        self._file.write(f"\n=== END  {datetime.now().strftime('%H:%M:%S')} ===\n")
        self._file.close()

    def recent(self, n=20):
        with self._lock:
            return list(self.messages[-n:])


# ─────────────────────────────────────────────
#  Coordinate / ObjectInfo
# ─────────────────────────────────────────────
class Coordinate:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class ObjectInfo:
    color_dict = {'Orange': 0, 'Yellow': 1, 'Blue': 2,
                  'Green': 3, 'Black': 4, 'Red': 5, 'White': 6}

    def __init__(self, master_node, color, object_type):
        self.node        = master_node
        self.color       = self.color_dict[color]
        self.color_name  = color
        self.object_type = object_type
        self.edge_max    = Coordinate(0, 0)
        self.edge_min    = Coordinate(0, 0)
        self.center      = Coordinate(0, 0)
        self.get_target  = False
        self.target_size = 0
        update_strategy = {'rise_line':   self.get_line_object,
                           'weight_bar':  self.get_bar_object}
        self.find_object = update_strategy[object_type]

    def get_line_object(self):
        sizes = self.node.object_sizes[self.color]
        if sizes:
            max_size = max(sizes)
            return sizes.index(max_size) if max_size > 1000 else None

    def get_bar_object(self):
        sizes = self.node.object_sizes[self.color]
        if sizes:
            max_size = max(sizes)
            return sizes.index(max_size) if max_size > 50 else None

    def update(self, ID):
        object_idx = self.find_object()
        if object_idx is not None:
            self.get_target  = True
            self.edge_max.x  = self.node.object_x_max[self.color][object_idx]
            self.edge_min.x  = self.node.object_x_min[self.color][object_idx]
            self.edge_max.y  = self.node.object_y_max[self.color][object_idx]
            self.edge_min.y  = self.node.object_y_min[self.color][object_idx]
            self.center.x    = (self.edge_min.x + self.edge_max.x) // 2
            self.center.y    = (self.edge_min.y + self.edge_max.y) // 2
            self.target_size = self.node.object_sizes[self.color][object_idx]
            self.node.drawImageFunction(
                int(ID), 1,
                self.edge_min.x, self.edge_max.x,
                self.edge_min.y, self.edge_max.y,
                0, 0, 255)
            self.node.tlog.log(
                f"[{self.color_name}] VISIBLE  size={self.target_size}  "
                f"cx={self.center.x}  cy={self.center.y}  "
                f"Ymin={self.edge_min.y}  Ymax={self.edge_max.y}")
        else:
            self.node.tlog.log(f"[{self.color_name}] NOT VISIBLE")
            self.edge_max.x = self.edge_min.x = self.edge_max.y = self.edge_min.y = 0
            self.center.x = self.center.y = self.target_size = 0
            self.get_target = False


# ─────────────────────────────────────────────
#  TUI 繪製
# ─────────────────────────────────────────────
STATUS_ORDER = [
    'head_shake', 'preturn', 'start_line', 'turn_straight',
    'pick_up', 'second_line', 'rise_up', 'fourth_line', 'final', 'end'
]
STATUS_LABEL = {
    'head_shake':    '初始化',
    'preturn':       '預轉對準',
    'start_line':    '走向槓鈴',
    'turn_straight': 'IMU 轉正',
    'pick_up':       '撿起槓鈴',
    'second_line':   '走向 Lift Line',
    'rise_up':       '舉起',
    'fourth_line':   '走向 Finish Line',
    'final':         '放下完成',
    'end':           '結束',
}

def _put(stdscr, pair, text, y, x, attr=0):
    h, w = stdscr.getmaxyx()
    if y < 0 or y >= h or x < 0 or x >= w:
        return
    max_len = w - x - 1
    if max_len <= 0:
        return
    try:
        stdscr.addstr(y, x, text[:max_len], curses.color_pair(pair) | attr)
    except curses.error:
        pass

def draw_tui(stdscr, node_ref):
    curses.curs_set(0)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_CYAN,    -1)   # 標題
    curses.init_pair(2, curses.COLOR_GREEN,   -1)   # 正常/可見
    curses.init_pair(3, curses.COLOR_YELLOW,  -1)   # 標籤/警告
    curses.init_pair(4, curses.COLOR_RED,     -1)   # 錯誤/看不到
    curses.init_pair(5, curses.COLOR_WHITE,   -1)   # 一般
    curses.init_pair(6, curses.COLOR_MAGENTA, -1)   # 當前狀態
    curses.init_pair(7, curses.COLOR_BLACK,   curses.COLOR_CYAN)   # 進度已過
    curses.init_pair(8, curses.COLOR_BLACK,   curses.COLOR_WHITE)  # 進度未到
    stdscr.nodelay(True)

    while True:
        node = node_ref[0]
        if node is None:
            time.sleep(0.1)
            continue

        stdscr.erase()
        h, w = stdscr.getmaxyx()
        now  = datetime.now().strftime("%H:%M:%S")

        # ── 標題 ──────────────────────────────
        _put(stdscr, 1, f" HuroCup WeightLift Monitor  {now} ".center(w-1), 0, 0, curses.A_BOLD)

        # ── 比賽狀態列 ────────────────────────
        is_start = getattr(node, 'is_start', False)
        _put(stdscr, 2 if is_start else 4,
             " ● RUNNING " if is_start else " ■ STOPPED ",
             2, 2, curses.A_BOLD)
        _put(stdscr, 3, f"  重量: {WIGHT} kg    PRETURN: {PRETURN}    FLAG1: {FLAG1}", 2, 14)

        tlog = getattr(node, 'tlog', None)
        if tlog:
            _put(stdscr, 5, f" Log → {tlog.log_path}", 3, 2)

        # ── 當前狀態 ──────────────────────────
        ctrl  = getattr(node, 'ctrl_status', '---')
        label = STATUS_LABEL.get(ctrl, ctrl)
        _put(stdscr, 3, " 狀態 ", 5, 2)
        _put(stdscr, 6, f" {ctrl} ", 5, 9, curses.A_BOLD)
        _put(stdscr, 5, f"  ({label})", 5, 9 + len(ctrl) + 2)

        # ── 進度條 ────────────────────────────
        try:
            cur_idx = STATUS_ORDER.index(ctrl)
        except ValueError:
            cur_idx = 0
        seg_w = max(1, (w - 4) // len(STATUS_ORDER))
        for i, s in enumerate(STATUS_ORDER):
            block = s[:seg_w-1].center(seg_w)
            _put(stdscr, 7 if i <= cur_idx else 8, block, 6, 2 + i * seg_w)

        # ── 分隔線 ────────────────────────────
        _put(stdscr, 1, "─" * (w - 2), 8, 1)
        _put(stdscr, 1, " 視覺偵測 ", 8, 2, curses.A_BOLD)

        # ── 紅色槓鈴 ──────────────────────────
        bar = getattr(node, 'bar', None)
        _put(stdscr, 3, " 紅色槓鈴 ", 9, 2)
        if bar and bar.get_target:
            _put(stdscr, 2,
                 f"可見 ✓  面積={bar.target_size:>6}  "
                 f"中心 X={bar.center.x:>4} Y={bar.center.y:>4}  "
                 f"Xmin={bar.edge_min.x:>4} Xmax={bar.edge_max.x:>4}  "
                 f"Ymin={bar.edge_min.y:>4} Ymax={bar.edge_max.y:>4}",
                 9, 13)
        else:
            _put(stdscr, 4, "看不到 ✗", 9, 13)

        # ── 白色線條 ──────────────────────────
        line = getattr(node, 'line', None)
        _put(stdscr, 3, " 白色線條 ", 10, 2)
        if line and line.get_target:
            _put(stdscr, 2,
                 f"可見 ✓  面積={line.target_size:>6}  "
                 f"中心 X={line.center.x:>4} Y={line.center.y:>4}  "
                 f"Ymin={line.edge_min.y:>4} Ymax={line.edge_max.y:>4}",
                 10, 13)
        else:
            _put(stdscr, 4, "看不到 ✗", 10, 13)

        # ── 線條旗標 ──────────────────────────
        third_flag  = getattr(node, 'third_line',  False)
        fourth_flag = getattr(node, 'fourth_line', False)
        _put(stdscr, 3, " 線條旗標 ", 11, 2)
        _put(stdscr, 5, f"THIRD_LINE={THIRD_LINE}  ", 11, 13)
        _put(stdscr, 2 if third_flag else 5,
             f"third={'TRUE' if third_flag else 'false'}   ", 11, 28)
        try:
            _put(stdscr, 5, f"FOURTH_LINE={FOURTH_LINE}  ", 11, 46)
            _put(stdscr, 2 if fourth_flag else 5,
                 f"fourth={'TRUE' if fourth_flag else 'false'}", 11, 62)
        except NameError:
            pass

        # ── 分隔線 ────────────────────────────
        _put(stdscr, 1, "─" * (w - 2), 13, 1)
        _put(stdscr, 1, " 感測器 & 行走 ", 13, 2, curses.A_BOLD)

        # ── IMU ───────────────────────────────
        imu      = getattr(node, 'imu_rpy',         [0.0, 0.0, 0.0])
        theta    = getattr(node, 'theta',            0)
        body_auto= getattr(node, 'body_auto',        False)
        crew     = getattr(node, 'crew',             False)
        real_bar = getattr(node, 'real_bar_center',  160)

        _put(stdscr, 3, " IMU RPY  ", 14, 2)
        _put(stdscr, 5,
             f"R={imu[0]:+.2f}  P={imu[1]:+.2f}  Y={imu[2]:+.2f}   theta補償={theta:+d}",
             14, 13)

        _put(stdscr, 3, " 行走     ", 15, 2)
        _put(stdscr, 2 if body_auto else 4,
             "行走中 ✓" if body_auto else "停止   ✗", 15, 13)
        _put(stdscr, 3, "  槓鈴持有 ", 15, 26)
        _put(stdscr, 2 if crew else 5,
             "持有 ✓" if crew else "未持有", 15, 38)
        _put(stdscr, 3, "  real_bar_center ", 15, 49)
        _put(stdscr, 1, f"{real_bar:>4}", 15, 68)

        # ── 訊息紀錄 ──────────────────────────
        _put(stdscr, 1, "─" * (w - 2), 17, 1)
        _put(stdscr, 1, " 訊息紀錄（最新在下方）", 17, 2, curses.A_BOLD)

        log_start = 18
        log_area_h = h - log_start - 2
        if tlog and log_area_h > 0:
            msgs = tlog.recent(log_area_h)
            for i, msg in enumerate(msgs):
                if log_start + i >= h - 1:
                    break
                if "NOT VISIBLE" in msg or "緊急" in msg:
                    pair = 4
                elif "VISIBLE" in msg or "rise_up" in msg or "pick_up" in msg:
                    pair = 2
                elif "white" in msg.lower() or "flag" in msg.lower():
                    pair = 3
                else:
                    pair = 5
                _put(stdscr, pair, msg, log_start + i, 2)

        _put(stdscr, 5, " Ctrl+C 結束  │  Log 自動儲存 ", h - 1, 2)
        stdscr.refresh()
        time.sleep(0.15)


# ─────────────────────────────────────────────
#  WeightLift 節點
# ─────────────────────────────────────────────
class WeightLift(API):
    def __init__(self, tlog: TUILogger):
        super().__init__('WeightLift_Node')
        self.tlog = tlog
        self.line = ObjectInfo(self, 'White', 'rise_line')
        self.bar  = ObjectInfo(self, 'Red',   'weight_bar')
        self.init_logic()
        self.create_timer(0.2, self.main_strategy)

    def _log(self, msg: str):
        self.tlog.log(msg)

    def init_logic(self):
        self.theta           = 0
        self.ctrl_status     = 'head_shake'
        self.body_auto       = False
        self.third_line      = False
        self.fourth_line     = False
        self.stop            = True
        self.real_bar_center = 160
        self.speed           = 0
        self.crew            = False

    def walk_switch(self):
        time.sleep(0.5)
        if self.body_auto:
            self.body_auto = False
            self.sendbodyAuto(0)
            self._log("walk_switch → 停止行走")
        else:
            self.body_auto = True
            self.sendbodyAuto(1)
            self._log("walk_switch → 開始行走")

    def imu_fix(self):
        theta = 0
        if self.imu_rpy[2] > 2.0:
            theta = -2
        elif self.imu_rpy[2] < -2.0:
            theta = 2
        return theta

    def walk_parameter(self, yaw, Y_COM):
        self.sendSensorReset(True)
        self.sendLCWalkParameter(
            com_y_swing  = float(Y_COM),
            width_size   = float(5),
            period_t     = int(300),
            t_dsp        = float(0.1),
            lift_height  = float(1.5),
            stand_height = float(23.5),
            com_height   = float(29.5)
        )

    def walking(self, yaw, Y_COM):
        if not self.body_auto:
            self.walk_parameter(yaw, Y_COM)
            self.walk_switch()
        self.theta = self.imu_fix()
        if self.ctrl_status == 'fourth_line':
            self.sendContinuousValue(1300, -200, self.theta)
        elif self.ctrl_status == 'second_line':
            self.sendContinuousValue(1500, -200, self.theta)
        else:
            self.sendContinuousValue(0, 0, self.theta)

    def main_strategy(self):
        if self.is_start:
            self._log(f"[tick] ctrl_status={self.ctrl_status}")

            if self.ctrl_status == 'head_shake':
                self._log("[head_shake] 初始化")
                self.stop = False
                time.sleep(0.5)
                self.ctrl_status = 'preturn'
                self._log("[head_shake] → preturn")

            if self.ctrl_status == 'preturn':
                self._log(f"[preturn] PRETURN={PRETURN}")
                if not self.body_auto:
                    self.walk_parameter(0, 0)
                    self.walk_switch()
                    if PRETURN == 1:
                        self.sendHeadMotor(2, HEAD_MOTOR_START, 100)
                        self.bar.update(1);  self.line.update(2)
                        while True:
                            rclpy.spin_once(self, timeout_sec=0)
                            if 120 <= self.bar.center.x or self.bar.center.x > 260:
                                break
                            self.bar.update(1);  self.line.update(2)
                            self.sendContinuousValue(500, 1000, -1)
                            self._log(f"[preturn R] bar.x={self.bar.center.x}")
                    elif PRETURN == 2:
                        self.bar.update(1);  self.line.update(2)
                        while True:
                            rclpy.spin_once(self, timeout_sec=0)
                            if 30 <= self.bar.center.x or self.bar.center.x > 180:
                                break
                            self.bar.update(1);  self.line.update(2)
                            self.sendContinuousValue(400, -1000, -1)
                            self._log(f"[preturn L] bar.x={self.bar.center.x}")
                            if self.bar.center.x <= 188:
                                self.sendContinuousValue(400, -1000, 0, -1)
                self.ctrl_status = 'start_line'
                time.sleep(0.5)
                self.sendHeadMotor(2, 1300, 100)
                self._log("[preturn] → start_line")

            elif self.ctrl_status == 'start_line':
                self.bar.update(1)
                imu_y = self.imu_rpy[2]
                bx, by = self.bar.center.x, self.bar.center.y
                if imu_y > 1.5 or imu_y < -1.5:
                    if bx > 150:
                        self.sendContinuousValue(800, -800, -1)
                        self._log(f"[start_line] 右轉  bar.x={bx}  imu_y={imu_y:.2f}")
                    elif 0 < bx < 145:
                        self.sendContinuousValue(800, 400, -1)
                        self._log(f"[start_line] 左轉  bar.x={bx}  imu_y={imu_y:.2f}")
                else:
                    if bx > 150:
                        self.sendContinuousValue(800, -800, 0)
                        self._log(f"[start_line] 右平移  bar.x={bx}")
                    elif 0 < bx < 145:
                        self.sendContinuousValue(800, 400, 0)
                        self._log(f"[start_line] 左平移  bar.x={bx}")
                    else:
                        self.walking(0, 0)
                        self._log(f"[start_line] 直走  bar.x={bx}  bar.y={by}")
                if by >= 210:
                    self._log(f"[start_line] bar.y={by}≥210 → turn_straight")
                    self.ctrl_status = 'turn_straight'

            elif self.ctrl_status == 'turn_straight':
                self.theta = self.imu_fix()
                self.sendContinuousValue(-500, -300, self.theta - 1)
                self._log(f"[turn_straight] theta={self.theta}  imu_y={self.imu_rpy[2]:.2f}")
                if abs(self.theta) <= 1:
                    time.sleep(0.5)
                    self.ctrl_status = 'pick_up'
                    self._log("[turn_straight] 轉正 → pick_up")

            elif self.ctrl_status == 'pick_up':
                self._log("[pick_up] 停止行走，穩定 2.5s")
                if self.body_auto:
                    self.walk_switch()
                time.sleep(2.5)
                self._log("[pick_up] sendBodySector(123)")
                self.sendBodySector(123)
                time.sleep(1)
                self.sendHeadMotor(2, 1320, 100)
                self._log(f"[pick_up] PICK_ONE={PICK_ONE}")
                self.sendBodySector(PICK_ONE)
                self.crew = True
                time.sleep(6.5)
                self._log(f"[pick_up] PICK_TWO={PICK_TWO}")
                self.sendBodySector(PICK_TWO)
                time.sleep(5)
                self._log(f"[pick_up] PICK_THREE={PICK_THREE}")
                self.sendBodySector(PICK_THREE)
                time.sleep(8)
                self.bar.update(1)
                self.sendHeadMotor(2, HEAD_MOTOR_START, 100)
                time.sleep(1)
                self.real_bar_center = self.bar.center.x
                self._log(f"[pick_up] real_bar_center={self.real_bar_center} → second_line")
                self.ctrl_status = 'second_line'

            elif self.ctrl_status == 'second_line':
                self.line.update(2)
                self.walking(-1, 0)
                ymin = self.line.edge_min.y
                ymax = self.line.edge_max.y
                if 75 < ymin < 150:
                    self.third_line = True
                self._log(f"[second_line] Ymin={ymin}  Ymax={ymax}  "
                          f"third_flag={self.third_line}  threshold={THIRD_LINE}")
                self.sendHeadMotor(2, 1450, 100)
                if ymax >= THIRD_LINE and self.third_line:
                    self._log(f"[second_line] ✓ Ymax={ymax}≥{THIRD_LINE} → rise_up，等 6.5s")
                    self.ctrl_status = 'rise_up'
                    time.sleep(6.5)

            elif self.ctrl_status == 'rise_up':
                self._log("[rise_up] 停止行走，穩定 2s")
                if self.body_auto:
                    self.walk_switch()
                time.sleep(2)
                self._log("[rise_up] sendBodySector(234) 縮左腳")
                self.sendBodySector(234)
                time.sleep(1)
                self._log(f"[rise_up] LIFT={LIFT}")
                self.sendBodySector(int(LIFT))
                wait_time = 23.5 if WIGHT == 90 else 21
                self._log(f"[rise_up] 等待 {wait_time}s")
                time.sleep(wait_time)
                if 165 < self.real_bar_center < 210:
                    cnt = min(int((self.real_bar_center - 165) // 7), 4)
                    self._log(f"[rise_up] 偏右 606×{cnt}  real_bar={self.real_bar_center}")
                    for _ in range(cnt):
                        self.sendBodySector(606)
                    time.sleep(3.5)
                elif 120 < self.real_bar_center < 155:
                    cnt = min(int((165 - self.real_bar_center) // 7), 4)
                    self._log(f"[rise_up] 偏左 607×{cnt}  real_bar={self.real_bar_center}")
                    for _ in range(cnt):
                        self.sendBodySector(607)
                    time.sleep(3.5)
                else:
                    self._log(f"[rise_up] 無需修正  real_bar={self.real_bar_center}")
                if FLAG1:
                    sector_fix = (3336 if WIGHT == 90 else 3335 if WIGHT == 80
                                  else 3334 if WIGHT == 70 else 3333 if WIGHT == 60 else None)
                    if sector_fix:
                        self._log(f"[rise_up] FLAG1 修正 {sector_fix}")
                        self.sendBodySector(int(sector_fix))
                self._log("[rise_up] → fourth_line")
                self.ctrl_status = 'fourth_line'

            elif self.ctrl_status == 'fourth_line':
                self.line.update(2)
                self.walking(0, -2)
                ymin = self.line.edge_min.y
                ymax = self.line.edge_max.y
                if 75 < ymin < 95:
                    self.fourth_line = True
                self._log(f"[fourth_line] Ymin={ymin}  Ymax={ymax}  "
                          f"fourth_flag={self.fourth_line}  threshold={FOURTH_LINE}")
                self.sendHeadMotor(2, 1400, 100)
                if ymax >= FOURTH_LINE and self.fourth_line:
                    self._log(f"[fourth_line] ✓ Ymax={ymax}≥{FOURTH_LINE} → final，等 5.3s")
                    self.ctrl_status = 'final'
                    time.sleep(5.3)

            elif self.ctrl_status == 'final':
                self._log("[final] 停止行走")
                if self.body_auto:
                    self.walk_switch()
                time.sleep(2)
                self._log(f"[final] FINAL={FINAL} 放下")
                self.sendBodySector(int(FINAL))
                time.sleep(9.5)
                self._log("[final] 完成 → end")
                self.ctrl_status = 'end'

        else:
            self.bar.update(1)
            self.line.update(2)
            if self.body_auto:
                self.walk_switch()
            if not self.stop and self.crew:
                self._log("[STOP] 緊急放下 6666")
                self.sendBodySector(6666)
                time.sleep(10)
                self.sendBodySector(29)
                self.sendHeadMotor(2, HEAD_MOTOR_FINISH, 100)
                self._log("[STOP] 歸位完成，重置狀態")
                self.init_logic()


# ─────────────────────────────────────────────
#  main
# ─────────────────────────────────────────────
def main(args=None):
    tlog     = TUILogger()
    node_ref = [None]

    tui_thread = threading.Thread(
        target=lambda: curses.wrapper(draw_tui, node_ref),
        daemon=True
    )
    tui_thread.start()

    rclpy.init(args=args)
    node        = WeightLift(tlog)
    node_ref[0] = node
    tlog.log(f"ROS2 節點啟動  WIGHT={WIGHT}  PRETURN={PRETURN}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        tlog.log("使用者中斷 (KeyboardInterrupt)")
    finally:
        tlog.log("程式結束")
        tlog.close()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        print(f"\nLog 已儲存至：{tlog.log_path}")


if __name__ == '__main__':
    main()
    