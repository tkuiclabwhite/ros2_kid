#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import serial
import threading
import time
import re
from typing import Optional, Tuple

# 匯入訊息型別
from tku_msgs.msg import SensorPackage, SensorSet, Dio

# latest_imudata() 回傳的欄位順序（共 14 個），發布時再填進 SensorPackage 的具名欄位：
#   [0:4]   QUAT  w, x, y, z        (相對於歸零姿態)
#   [4:7]   ACC   x, y, z           (g)
#   [7:10]  GYR   x, y, z           (deg/s)
#   [10:13] MAG   x, y, z           (uT)
#   [13]    TMP   t                 (度C)
# 尚未收到的欄位以 0 填入
IMUDATA_LAYOUT = [('QUAT', 4), ('ACC', 3), ('GYR', 3), ('MAG', 3), ('TMP', 1)]

# ==========================================
# 1. 保留 imu.py 的 IMUService 類別
# ==========================================
class IMUService:
    """
    串口讀取 IMU 的 YPR 並加入外部回呼處理開關訊號
    """
    def __init__(self, port="/dev/ttyACM0", baud=115200, rel_mode=True, debug_raw=False, open_wait_sec=2.0):
        self.port = port
        self.baud = baud
        self.rel_mode = rel_mode
        self.debug_raw = debug_raw
        self.open_wait_sec = open_wait_sec

        self._ser = None
        self._th: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

        self._latest_abs: Optional[Tuple[float, float, float]] = None
        self._has_data = True
        self.zero = [0.0, 0.0, 0.0]
        self._extra = {}  # 'QUAT'/'ACC'/'GYR'/'MAG'/'TMP' -> tuple of float

        # 新增：用於接收開關字串的回呼
        self.on_switch_callback = None

    def start(self):
        if self._th and self._th.is_alive(): return
        self._stop.clear()
        self._th = threading.Thread(target=self.run, daemon=True)
        self._th.start()

    def stop(self):
        self._stop.set()
        if self._th: self._th.join(timeout=1.5)
        self._close_serial()

    def latest(self) -> Optional[Tuple[float, float, float]]:
        with self._lock:
            if not self._has_data or self._latest_abs is None: return None
            y, p, r = self._latest_abs
            if self.rel_mode:
                return (y - self.zero[0], p - self.zero[1], r - self.zero[2])
            return (y, p, r)

    def latest_imudata(self) -> list:
        """ 依 IMUDATA_LAYOUT 攤平成一維陣列 """
        with self._lock:
            out = []
            for tag, n in IMUDATA_LAYOUT:
                vals = self._extra.get(tag)
                out.extend(vals if vals is not None and len(vals) == n else [0.0] * n)
            return out

    def zero_here(self):
        with self._lock:
            if self._ser and self._ser.is_open:
                try:
                    self._ser.write(b' ')
                    self._ser.flush()
                    print("[IMU] 已發送空白鍵執行 Arduino 硬體歸零")
                except Exception as e:
                    print(f"[IMU] 發送歸零指令失敗: {e}")
            else:
                print("[IMU] 串口未開啟，無法傳送歸零指令")

    def _open_serial(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
            if self.open_wait_sec > 0: time.sleep(self.open_wait_sec)
            self._ser.reset_input_buffer()
        except Exception as e:
            self._ser = None

    def _close_serial(self):
        try:
            if self._ser and self._ser.is_open: self._ser.close()
        except Exception: pass
        self._ser = None

    def run(self):
        self._open_serial()
        buf = ""
        ypr_regex = re.compile(
            r'^\s*#?\s*ypr\s*[:=]\s*([-+]?\d+(?:\.\d+)?)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*$',
            re.IGNORECASE
        )
        # 其他數值行：#QUAT=... / #ACC=... / #GYR=... / #MAG=... / #TMP=...
        extra_regex = re.compile(r'^\s*#?\s*(QUAT|ACC|GYR|MAG|TMP)\s*[:=]\s*(.+)$', re.IGNORECASE)

        while not self._stop.is_set():
            try:
                if self._ser is None or not self._ser.is_open:
                    self._open_serial()
                    time.sleep(0.5)
                    continue

                n = self._ser.in_waiting
                if n:
                    raw = self._ser.read(n)
                    buf += raw.decode(errors="ignore").replace('\r', '\n')

                    while '\n' in buf:
                        line, buf = buf.split('\n', 1)
                        s = line.strip()
                        if not s: continue

                        # 判斷是開關訊號還是 IMU 數據
                        if "START" in s or "STOP" in s:
                            if self.on_switch_callback:
                                self.on_switch_callback(s)
                            continue

                        m = ypr_regex.match(s)
                        if m:
                            y, p, r = float(m.group(1)), float(m.group(2)), float(m.group(3))
                            with self._lock:
                                self._latest_abs = (y, p, r)
                                self._has_data = True
                            continue

                        m = extra_regex.match(s)
                        if m:
                            try:
                                vals = tuple(float(v) for v in m.group(2).split(','))
                            except ValueError:
                                continue
                            with self._lock:
                                self._extra[m.group(1).upper()] = vals
                time.sleep(0.001)
            except Exception:
                time.sleep(0.1)

# ==========================================
# 2. 整合 imu_node.py 與 switch.py 的 ROS 邏輯
# ==========================================
class UnifiedSensorNode(Node):
    def __init__(self):
        super().__init__('unified_sensor_node')

        # 讀取原本 imu_node.py 的參數
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('pub_hz', 20.0)
        self.declare_parameter('open_wait_sec', 2.0)
        self.declare_parameter('print_imu', False)       # true 時每秒在終端機印出帶標籤的 IMU 數值
        self.declare_parameter('print_hz', 1.0)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        pub_hz = self.get_parameter('pub_hz').value
        open_wait_sec = self.get_parameter('open_wait_sec').value
        self.print_imu = self.get_parameter('print_imu').value
        self.print_period = 1.0 / max(float(self.get_parameter('print_hz').value), 0.1)
        self._last_print = 0.0

        # 初始化 IMU 服務並綁定開關回呼
        self.imu = IMUService(port=port, baud=baud, open_wait_sec=open_wait_sec)
        self.imu.on_switch_callback = self.publish_dio
        self.imu.start()

        # QoS 配置
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # 發布者與訂閱者
        self.pub_pkg = self.create_publisher(SensorPackage, '/package/sensorpackage', qos)
        self.pub_dio = self.create_publisher(Dio, '/package/dioarray', 10)
        self.sub_reset = self.create_subscription(SensorSet, '/sensorset', self.on_sensor_set, qos)

        self.timer = self.create_timer(1.0 / pub_hz, self.on_timer)
        self.get_logger().info(f"✅ Unified Node Started. Monitoring {port}")

    def publish_dio(self, line):
        """ 原 switch.py 的邏輯：發布開關狀態 """
        msg = Dio()
        msg.strategy = True if "START" in line else False
        msg.data = 0
        self.pub_dio.publish(msg)
        self.get_logger().info(f"Switch Event: {line}")

    def on_sensor_set(self, msg: SensorSet):
        """ 原 imu_node.py 的邏輯：歸零校準 """
        flag = getattr(msg, 'reset', False) or getattr(msg, 'Reset', False)
        if flag:
            self.imu.zero_here()
            self.get_logger().info('IMU Zeroed')

    def on_timer(self):
        """ 原 imu_node.py 的邏輯：定時發布 IMU """
        ypr = self.imu.latest()
        if ypr is None: return

        y, p, r = ypr
        msg = SensorPackage()
        
        # 保留原有的多命名相容性
        for field, val in zip(['yaw', 'pitch', 'roll'], [y, p, r]):
            for f in [field, field.capitalize()]:
                if hasattr(msg, f):
                    setattr(msg, f, float(round(val, 2)))

        d = self.imu.latest_imudata()
        (msg.quat_w, msg.quat_x, msg.quat_y, msg.quat_z,
         msg.acc_x, msg.acc_y, msg.acc_z,
         msg.gyr_x, msg.gyr_y, msg.gyr_z,
         msg.mag_x, msg.mag_y, msg.mag_z,
         msg.temperature) = [float(v) for v in d]

        now = time.time()
        if self.print_imu and now - self._last_print >= self.print_period:
            self._last_print = now
            self.get_logger().info(
                f"\n  YPR  (deg)  : yaw={y:8.2f}  pitch={p:8.2f}  roll={r:8.2f}"
                f"\n  QUAT        : w={d[0]:7.4f}  x={d[1]:7.4f}  y={d[2]:7.4f}  z={d[3]:7.4f}"
                f"\n  ACC  (g)    : x={d[4]:8.3f}  y={d[5]:8.3f}  z={d[6]:8.3f}"
                f"\n  GYR  (deg/s): x={d[7]:8.2f}  y={d[8]:8.2f}  z={d[9]:8.2f}"
                f"\n  MAG  (uT)   : x={d[10]:8.2f}  y={d[11]:8.2f}  z={d[12]:8.2f}"
                f"\n  TMP  (C)    : {d[13]:.2f}"
            )

        self.pub_pkg.publish(msg)

    def destroy_node(self):
        self.imu.stop()
        super().destroy_node()

def main():
    rclpy.init()
    node = UnifiedSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()