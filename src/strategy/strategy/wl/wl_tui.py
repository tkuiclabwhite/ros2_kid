#!/usr/bin/env python3
# coding=utf-8
import rclpy
import time
import threading
from strategy.API import API
from tku_msgs.msg import Dio
import math
import numpy as np
import cv2
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tku_msgs.msg import SensorPackage
from rich.live import Live
from rich.table import Table
from rich.panel import Panel
from rich import box

# --- 全域參數 ---
WIGHT = 60
HEAD_MOTOR_START = 1500
HEAD_MOTOR_FINISH = 1350
FLAG1 = False
PRETURN = 1

if WIGHT == 86:
    THIRD_LINE = 200
    SPEED = 2000
    PICK_ONE = 861; PICK_TWO = 862; PICK_THREE = 863; LIFT = 864
elif WIGHT == 90:
    THIRD_LINE = 200
    SPEED = 2100
    PICK_ONE = 901; PICK_TWO = 902; PICK_THREE = 903; LIFT = 904; FINAL = 905
elif WIGHT == 80:
    THIRD_LINE = 224; FOURTH_LINE = 224
    SPEED = 1300
    PICK_ONE = 801; PICK_TWO = 802; PICK_THREE = 803; LIFT = 804; FINAL = 805
elif WIGHT == 70:
    THIRD_LINE = 225; FOURTH_LINE = 225
    SPEED = 1300
    PICK_ONE = 701; PICK_TWO = 702; PICK_THREE = 703; LIFT = 704; FINAL = 705
elif WIGHT == 60:
    SPEED = 1300
    THIRD_LINE = 225; FOURTH_LINE = 225
    PICK_ONE = 601; PICK_TWO = 602; PICK_THREE = 603; LIFT = 604; FINAL = 605
elif WIGHT == 50:
    SPEED = 1800
    THIRD_LINE = 205
    PICK_ONE = 501; PICK_TWO = 502; PICK_THREE = 503; LIFT = 504; FINAL = 505
else:
    THIRD_LINE = 215
    SPEED = 1300
    PICK_ONE = 401; PICK_TWO = 402; PICK_THREE = 403; LIFT = 404; FINAL = 405


# ─── TUI 監控狀態板 ────────────────────────────────────────────────────────────
class StatusBoard:
    def __init__(self):
        self._lock = threading.Lock()
        self._d = {
            'weight':      WIGHT,
            'ctrl_status': 'init',
            'head_motor':  None,
            'speed':       None,
            'theta':       None,
            'white_y_min': None,
            'white_y_max': None,
            'bar_x':       None,
            'bar_y':       None,
            'sector':      None,
            'imu_yaw':     None,
            'is_start':    False,
        }

    def update(self, **kw):
        with self._lock:
            self._d.update(kw)

    def clear(self, *keys):
        with self._lock:
            for k in keys:
                self._d[k] = None

    def snap(self):
        with self._lock:
            return dict(self._d)


_sb = StatusBoard()


def _v(val, unit=''):
    return f"{val}{unit}" if val is not None else '[dim]—[/dim]'


def _render():
    d = _sb.snap()
    t = Table(box=box.SIMPLE, show_header=False, padding=(0, 2), expand=False)
    t.add_column('k', style='cyan', width=14, no_wrap=True)
    t.add_column('v', width=24, no_wrap=True)
    started = d['is_start']
    start_text = '[bold green]START[/bold green]' if started else '[bold red]OFF[/bold red]'
    t.add_row('重量級別',    f'[bold yellow]{_v(d["weight"], " kg")}[/bold yellow]')
    t.add_row('啟動狀態',    start_text)
    t.add_row('狀態機',      f'[bold green]{d["ctrl_status"]}[/bold green]')
    t.add_row('', '')
    yaw = d['imu_yaw']
    yaw_text = f'{yaw:.2f} °' if yaw is not None else '[dim]—[/dim]'
    t.add_row('IMU YAW',  yaw_text)
    t.add_row('', '')
    t.add_row('頭部馬達',    _v(d['head_motor']))
    t.add_row('', '')
    t.add_row('前進速度',    _v(d['speed']))
    t.add_row('Theta',       _v(d['theta']))
    t.add_row('', '')
    t.add_row('White Y Min', _v(d['white_y_min']))
    t.add_row('White Y Max', _v(d['white_y_max']))
    t.add_row('', '')
    t.add_row('槓鈴中心 X',  _v(d['bar_x']))
    t.add_row('槓鈴中心 Y',  _v(d['bar_y']))
    t.add_row('', '')
    sec = d['sector']
    t.add_row('執行動作',
              f'[bold magenta]{sec}[/bold magenta]' if sec is not None else '[dim]—[/dim]')
    return Panel(t, title='[bold blue] 舉重機器人監控 [/bold blue]', border_style='blue')


def _tui_loop():
    with Live(_render(), refresh_per_second=10, screen=True) as live:
        while True:
            live.update(_render())
            time.sleep(0.1)


# ─── 視覺物件 ──────────────────────────────────────────────────────────────────
class Coordinate:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class ObjectInfo:
    color_dict = {'Orange': 0, 'Yellow': 1, 'Blue': 2, 'Green': 3,
                  'Black': 4, 'Red': 5, 'White': 6}

    def __init__(self, master_node, color, object_type):
        self.node        = master_node
        self.color       = self.color_dict[color]
        self.edge_max    = Coordinate(0, 0)
        self.edge_min    = Coordinate(0, 0)
        self.center      = Coordinate(0, 0)
        self.get_target  = False
        self.target_size = 0
        self._is_line    = (object_type == 'rise_line')
        update_strategy  = {'rise_line': self.get_line_object,
                            'weight_bar': self.get_bar_object}
        self.find_object = update_strategy[object_type]

    def get_line_object(self):
        sizes = self.node.object_sizes[self.color]
        if sizes:
            m = max(sizes)
            return sizes.index(m) if m > 1000 else None

    def get_bar_object(self):
        sizes = self.node.object_sizes[self.color]
        if sizes:
            m = max(sizes)
            return sizes.index(m) if m > 50 else None

    def update(self, ID):
        idx = self.find_object()
        if idx is not None:
            self.get_target  = True
            c = self.color
            self.edge_max.x  = self.node.object_x_max[c][idx]
            self.edge_min.x  = self.node.object_x_min[c][idx]
            self.edge_max.y  = self.node.object_y_max[c][idx]
            self.edge_min.y  = self.node.object_y_min[c][idx]
            self.center.x    = (self.edge_min.x + self.edge_max.x) // 2
            self.center.y    = (self.edge_min.y + self.edge_max.y) // 2
            self.target_size = self.node.object_sizes[c][idx]
            self.node.drawImageFunction(int(ID), 1,
                                        self.edge_min.x, self.edge_max.x,
                                        self.edge_min.y, self.edge_max.y,
                                        0, 0, 255)
            if self._is_line:
                _sb.update(white_y_min=self.edge_min.y, white_y_max=self.edge_max.y)
            else:
                _sb.update(bar_x=self.center.x, bar_y=self.center.y)
        else:
            self.edge_max.x = self.edge_min.x = self.edge_max.y = self.edge_min.y = 0
            self.center.x   = self.center.y = self.target_size = 0
            self.get_target = False
            # 沒偵測到時立刻清除，不殘留舊值
            if self._is_line:
                _sb.clear('white_y_min', 'white_y_max')
            else:
                _sb.clear('bar_x', 'bar_y')


# ─── 主策略節點 ────────────────────────────────────────────────────────────────
class WeightLift(API):
    def __init__(self):
        super().__init__('WeightLift_Node')
        self.line = ObjectInfo(self, 'White', 'rise_line')
        self.bar  = ObjectInfo(self, 'Red',   'weight_bar')
        self.init_logic()
        self.create_timer(0.2, self.main_strategy)

    # ── 攔截 API 呼叫以同步更新 TUI ──────────────────────────────────────────
    def sendHeadMotor(self, *args, **kwargs):
        if len(args) >= 2:
            _sb.update(head_motor=args[1])
        super().sendHeadMotor(*args, **kwargs)

    def sendContinuousValue(self, *args, **kwargs):
        fwd = args[0] if args else 0
        lat = args[1] if len(args) >= 2 else 0
        if fwd == 0 and lat == 0:
            _sb.clear('speed', 'theta')
        else:
            _sb.update(speed=fwd, theta=args[2] if len(args) >= 3 else None)
        super().sendContinuousValue(*args, **kwargs)

    def _do_sector(self, sid):
        _sb.update(sector=int(sid))
        super().sendBodySector(int(sid))

    def _done_sector(self):
        _sb.clear('sector')

    def _go(self, state):
        self.ctrl_status = state
        _sb.update(ctrl_status=state)
        # 切換到不走路的狀態時清除速度資訊
        if state in ('pick_up', 'rise_up', 'head_shake', 'end'):
            _sb.clear('speed', 'theta')

    # ── 初始化 ───────────────────────────────────────────────────────────────
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
        _sb.update(ctrl_status='head_shake')
        _sb.clear('speed', 'theta', 'sector')

    def walk_switch(self):
        time.sleep(0.5)
        if self.body_auto:
            self.body_auto = False
            self.sendbodyAuto(0)
            _sb.clear('speed', 'theta')
        else:
            self.body_auto = True
            self.sendbodyAuto(1)

    def imu_fix(self):
        if   self.imu_rpy[2] >  2.0: return -2
        elif self.imu_rpy[2] < -2.0: return  2
        return 0

    def walk_parameter(self, yaw, Y_COM):
        self.sendSensorReset(True)
        self.sendLCWalkParameter(
            com_y_swing=float(Y_COM), width_size=float(5),
            period_t=int(300), t_dsp=float(0.1),
            lift_height=float(1.5), stand_height=float(23.5),
            com_height=float(29.5))

    def walking(self, yaw, Y_COM):
        if not self.body_auto:
            self.walk_parameter(yaw, Y_COM)
            self.walk_switch()
        self.theta = self.imu_fix()
        if   self.ctrl_status == 'fourth_line':
            self.sendContinuousValue(1300, -300, self.theta+0.5)
        elif self.ctrl_status == 'second_line':
            self.sendContinuousValue(1500, -300, -1)
        else:
            self.sendContinuousValue(0, 0, self.theta)

    # ── 主策略 ───────────────────────────────────────────────────────────────
    def main_strategy(self):
        _sb.update(is_start=self.is_start, imu_yaw=self.imu_rpy[2])
        if self.is_start:
            if self.ctrl_status == 'head_shake':
                self.stop = False
                time.sleep(0.5)
                self._go('preturn')

            if self.ctrl_status == 'preturn':
                if not self.body_auto:
                    self.walk_parameter(0, 0)
                    self.walk_switch()
                    if PRETURN == 1:
                        self.sendHeadMotor(2, HEAD_MOTOR_START, 100)
                        self.bar.update(1); self.line.update(2)
                        while True:
                            rclpy.spin_once(self, timeout_sec=0)
                            if 120 <= self.bar.center.x or self.bar.center.x > 260:
                                break
                            self.bar.update(1); self.line.update(2)
                            self.sendContinuousValue(500, 1000, -1)
                    elif PRETURN == 2:
                        self.bar.update(1); self.line.update(2)
                        while True:
                            rclpy.spin_once(self, timeout_sec=0)
                            if 30 <= self.bar.center.x or self.bar.center.x > 180:
                                break
                            self.bar.update(1); self.line.update(2)
                            self.sendContinuousValue(400, -1000, -1)
                            if self.bar.center.x <= 188:
                                self.sendContinuousValue(400, -1000, 0, -1)
                self._go('start_line')
                time.sleep(0.5)
                self.sendHeadMotor(2, 1300, 100)

            elif self.ctrl_status == 'start_line':
                self.bar.update(1)
                if self.imu_rpy[2] > 1.5 or self.imu_rpy[2] < -1.5:
                    if self.bar.center.x > 150:
                        self.sendContinuousValue(800, -800, -1)
                    elif 0 < self.bar.center.x < 145:
                        self.sendContinuousValue(800,  400, -1)
                else:
                    if self.bar.center.x > 150:
                        self.sendContinuousValue(800, -800, -1)
                    elif 0 < self.bar.center.x < 145:
                        self.sendContinuousValue(800,  400, -1)
                    else:
                        self.walking(0, 0)
                if self.bar.center.y >= 230:
                    self._go('turn_straight')

            elif self.ctrl_status == 'turn_straight':
                self.theta = self.imu_fix()
                self.sendContinuousValue(-500, -300, self.theta - 2)
                if abs(self.theta) <= 1:
                    time.sleep(0.5)
                    self._go('pick_up')

            elif self.ctrl_status == 'pick_up':
                if self.body_auto:
                    self.walk_switch()
                time.sleep(2.5)
                time.sleep(1)
                self.sendHeadMotor(2, 1320, 100)
                self._do_sector(PICK_ONE); self.crew = True
                time.sleep(6.5)
                self._do_sector(PICK_TWO)
                time.sleep(5.5)
                self._do_sector(PICK_THREE)
                time.sleep(8)
                self._done_sector()
                self.bar.update(1)
                self.sendHeadMotor(2, HEAD_MOTOR_START, 100)
                time.sleep(1)
                self._do_sector(123)
                self.real_bar_center = self.bar.center.x
                self._go('second_line')

            elif self.ctrl_status == 'second_line':
                self.line.update(2)
                self.walking(-1, 0)
                if 75 < self.line.edge_min.y < 150:
                    self.third_line = True
                self.sendHeadMotor(2, 1450, 100)
                if self.line.edge_max.y >= THIRD_LINE and self.third_line:
                    self._go('rise_up')
                    time.sleep(6.5)

            elif self.ctrl_status == 'rise_up':
                if self.body_auto:
                    self.walk_switch()
                time.sleep(2)
                self._do_sector(LIFT)
                time.sleep(23.5 if WIGHT == 90 else 21)
                if 165 < self.real_bar_center < 210:
                    for _ in range(min(int((self.real_bar_center - 165) // 7), 4)):
                        self._do_sector(606)
                    time.sleep(3.5)
                elif 120 < self.real_bar_center < 155:
                    for _ in range(min(int((165 - self.real_bar_center) // 7), 4)):
                        self._do_sector(607)
                    time.sleep(3.5)
                self._do_sector(234)
                time.sleep(1)
                self._done_sector()
                if FLAG1:
                    sx = (3336 if WIGHT == 90 else 3335 if WIGHT == 80
                          else 3334 if WIGHT == 70 else 3333 if WIGHT == 60 else None)
                    if sx:
                        self._do_sector(sx)
                        self._done_sector()
                self._go('fourth_line')

            elif self.ctrl_status == 'fourth_line':
                self.line.update(2)
                self.walking(0, -2)
                if 75 < self.line.edge_min.y < 95:
                    self.fourth_line = True
                self.sendHeadMotor(2, 1400, 100)
                if self.line.edge_max.y >= FOURTH_LINE and self.fourth_line:
                    self._go('final')
                    time.sleep(5.3)

            elif self.ctrl_status == 'final':
                if self.body_auto:
                    self.walk_switch()
                time.sleep(2)
                self._do_sector(FINAL)
                time.sleep(9.5)
                self._done_sector()
                self._go('end')

        else:
            self.bar.update(1)
            self.line.update(2)
            if self.body_auto:
                self.walk_switch()
            if not self.stop and self.crew:
                self._do_sector(6666)
                time.sleep(10)
                self._do_sector(29)
                self._done_sector()
                self.sendHeadMotor(2, HEAD_MOTOR_FINISH, 100)
                self.init_logic()


def main(args=None):
    rclpy.init(args=args)
    node = WeightLift()

    threading.Thread(target=_tui_loop, daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
