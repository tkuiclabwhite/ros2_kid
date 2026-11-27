#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray, String
from tku_msgs.msg import HeadPackage
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, DurabilityPolicy

import time
import json
import threading
from collections import defaultdict
from dynamixel_sdk import *

# ==== 設定參數 ====
DEFAULT_BAUDRATE = 1000000

# 預設目標 ID 範圍：1~23 (涵蓋所有手、腳、頭 ID)
ALL_TARGET_IDS = list(range(1, 24))

# ==== Control Table 位址 (X-Series / MX 2.0) ====
ADDR_TORQUE_ENABLE          = 64   # 扭力開關（1=開, 0=關）
ADDR_PROFILE_ACCELERATION   = 108  # 加速度設定（寫入用）
ADDR_PROFILE_VELOCITY       = 112  # 最大速度設定（寫入用）
ADDR_GOAL_POSITION          = 116  # 目標位置（寫入用）
ADDR_PRESENT_CURRENT        = 126  # 目前電流（讀取用，unit ≈ 2.69 mA）
ADDR_PRESENT_VELOCITY       = 128  # 目前速度（讀取用）
ADDR_PRESENT_POSITION       = 132  # 目前位置（讀取用）

# ==== 資料長度（bytes）====
LEN_SYNC_WRITE_ALL          = 12   # 同步寫入：加速度(4) + 速度(4) + 目標位置(4)

# ==== 讀取視窗（依開關組合）====
# read_current=False, read_velocity=False → addr=132, len=4  (位置)
# read_current=False, read_velocity=True  → addr=128, len=8  (速度+位置)
# read_current=True                       → addr=126, len=10 (電流+速度+位置)

PROTOCOL_VERSION = 2.0

# Protocol 2.0 狀態封包的 error byte：bit7 = Alert（馬達 Hardware Error Status 有警示），
# bit0~6 才是這個指令本身的錯誤。Alert 時指令照樣執行、讀值有效，所以判斷成功只看 bit0~6。
ERR_MASK_NO_ALERT = 0x7F


class _Stat:
    """累積一段時間內的 ms 數值，取 avg / max 後清空（給網頁延遲監測用）。"""
    def __init__(self):
        self.n = 0; self.sum = 0.0; self.max = 0.0

    def add(self, ms):
        self.n += 1; self.sum += ms
        if ms > self.max: self.max = ms

    def take(self):
        """回傳 {'avg','max'}，沒有樣本回傳 None；取完清空。"""
        if self.n == 0:
            return None
        d = {'avg': self.sum / self.n, 'max': self.max}
        self.n = 0; self.sum = 0.0; self.max = 0.0
        return d


class DynamixelDriver(Node):
    def __init__(self):
        super().__init__('dynamixel_driver_node')

        # 1. 宣告參數
        self.declare_parameter('baudrate', DEFAULT_BAUDRATE)
        self.declare_parameter('ports', ['/dev/U2D2_P1', '/dev/U2D2_P2', '/dev/U2D2_P3'])
        self.declare_parameter('read_velocity', False)  # 開啟速度讀取
        self.declare_parameter('read_current',  False)  # 開啟電流讀取（自動包含速度）

        self.baudrate      = self.get_parameter('baudrate').value
        self.port_list     = self.get_parameter('ports').value
        self.target_ids    = sorted(ALL_TARGET_IDS)
        self.read_velocity = self.get_parameter('read_velocity').value
        self.read_current  = self.get_parameter('read_current').value

        # 2. 記憶體狀態
        self.joint_data = defaultdict(lambda: {
            'present': None, 'present_vel': 0, 'present_cur': 0,
            'goal': None, 'velocity': 0, 'accel': 0
        })

        # 頭部 ID 映射（網頁虛擬 ID → 實際馬達 ID）
        self.head_map = {1: 22, 2: 23}

        # 3. 以 Port 為單位的硬體容器
        # self.ports[port_name] = {'ph', 'gw', 'gr', 'ids', 'alive'}
        self.ports        = {}
        self.id_port_map  = {}  # {motor_id: port_name}
        self._reconnecting = set()

        # 扭力實際狀態：開機/重連掃描時從馬達讀回 Torque Enable，之後每次寫入成功就更新。
        # 開機不關扭力：只重開 launch、馬達沒斷電時扭力維持原狀，機器人不會軟掉。
        # 這是硬體讀回的值，不是指令快取；沒連上的馬達記為 0。
        self.torque_enable = {mid: 0 for mid in self.target_ids}
        self._torque_lock = threading.Lock()
        # latched：motionpackage 晚啟動或單獨重啟，一訂閱就會拿到最後一次的完整狀態
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.torque_state_pub = self.create_publisher(Int16MultiArray, '/torque_state', latched)

        self.get_logger().info(f"正在初始化 U2D2 Ports... Baudrate: {self.baudrate}")
        self._init_hardware_blind()

        # 4. ROS 2 通訊介面
        self.joint_pub = self.create_publisher(JointState, '/joint_states',     10)
        self.vel_pub   = self.create_publisher(JointState, '/joint_velocities', 10)
        self.cur_pub   = self.create_publisher(JointState, '/joint_currents',   10)

        self.joint_sub    = self.create_subscription(JointState,      '/joint_commands',   self._command_cb,   10)
        self.torque_sub   = self.create_subscription(Int16MultiArray, '/set_torque',       self._torque_cb,    10)
        self.head_sub     = self.create_subscription(HeadPackage,     'Head_Topic',        self._head_cb,      10)
        self.readmode_sub = self.create_subscription(Int16MultiArray, '/set_motor_read',   self._read_mode_cb, 10)

        # 延遲監測（網頁 WalkingInterface 的延遲表）
        #   /latency_ping → 原封回 /latency_pong 並附上 t_robot（time.time()，秒）
        #   /motor_timing → 每秒發一次 JSON：指令延遲、各 Port USB 寫/讀時間、控制迴圈週期
        self.latency_pong_pub = self.create_publisher(String, '/latency_pong', 10)
        self.motor_timing_pub = self.create_publisher(String, '/motor_timing', 10)
        self.create_subscription(String, '/latency_ping', self._latency_ping_cb, 10)
        self._st_cmd_age     = _Stat()   # 指令 header.stamp → driver 收到
        self._st_cmd_wait    = _Stat()   # driver 收到 → 控制迴圈寫出
        self._st_loop_period = _Stat()   # 控制迴圈實際週期
        self._st_loop_busy   = _Stat()   # 每圈 USB 讀寫 + 發布耗時
        self._st_ports       = {}        # port_name → {'write': _Stat, 'read': _Stat, 'fail': int}
        self._cmd_pending_t  = None      # 最早一筆還沒寫出的指令收到時間 (perf_counter)
        self._last_loop_t    = None
        self.motor_timing_timer = self.create_timer(1.0, self._publish_motor_timing)

        # 5. 動態參數回呼（執行中切換讀取開關）
        self.add_on_set_parameters_callback(self._param_cb)

        # 6. 控制迴圈 20Hz
        self.timer = self.create_timer(0.02, self._control_loop)

        # 7. 每秒重發扭力狀態：rosbridge 不一定會把 latched 訊息補給晚連上的網頁，
        #    定期重發讓網頁／motionpackage 不管何時連上，最多 1 秒就校正。
        self.torque_state_timer = self.create_timer(1.0, self._publish_torque_state)

        addr, length = self._get_read_params()
        self.get_logger().info(
            f"Driver Ready! (Per-Port Isolated)  "
            f"讀取: addr={addr} len={length}B  "
            f"velocity={'ON' if self.read_velocity or self.read_current else 'OFF'}  "
            f"current={'ON' if self.read_current else 'OFF'}"
        )

    # ──────────────────────────────────────────────────────────────────────
    # 讀取模式輔助
    # ──────────────────────────────────────────────────────────────────────

    def _get_read_params(self):
        """依目前開關決定 GroupSyncRead 起始位址和長度。"""
        if self.read_current:
            return ADDR_PRESENT_CURRENT, 10   # 電流(2) + 速度(4) + 位置(4)
        elif self.read_velocity:
            return ADDR_PRESENT_VELOCITY, 8   # 速度(4) + 位置(4)
        else:
            return ADDR_PRESENT_POSITION, 4   # 位置(4)

    def _reinit_readers(self):
        """重建所有存活 Port 的 GroupSyncRead（切換讀取模式時呼叫）。"""
        addr, length = self._get_read_params()
        pk = PacketHandler(PROTOCOL_VERSION)
        for port_name, port in self.ports.items():
            if not port.get('alive'):
                continue
            try:
                gr = GroupSyncRead(port['ph'], pk, addr, length)
                for mid in port['ids']:
                    gr.addParam(mid)
                port['gr'] = gr
            except Exception as e:
                self.get_logger().error(f"重建 {port_name} reader 失敗: {e}")
        self.get_logger().info(
            f"讀取模式已切換: addr={addr} len={length}B  "
            f"velocity={'ON' if self.read_velocity or self.read_current else 'OFF'}  "
            f"current={'ON' if self.read_current else 'OFF'}"
        )

    def _param_cb(self, params):
        """動態參數回呼：切換 read_velocity / read_current 開關。"""
        changed = False
        for p in params:
            if p.name == 'read_velocity' and isinstance(p.value, bool):
                self.read_velocity = p.value
                changed = True
            elif p.name == 'read_current' and isinstance(p.value, bool):
                self.read_current = p.value
                changed = True
        if changed:
            self._reinit_readers()
        return SetParametersResult(successful=True)

    # ──────────────────────────────────────────────────────────────────────
    # 硬體初始化
    # ──────────────────────────────────────────────────────────────────────

    def _init_hardware_blind(self, target_ports=None):
        """
        掃描並綁定馬達。
        target_ports=None → 初始化全部；指定列表 → 只重建指定 Port。
        """
        if target_ports is None:
            target_ports = self.port_list

        pk = PacketHandler(PROTOCOL_VERSION)
        addr, length = self._get_read_params()

        for port_name in target_ports:
            try:
                ph = PortHandler(port_name)
                if not (ph.openPort() and ph.setBaudRate(self.baudrate)):
                    self.get_logger().warn(f"無法開啟 Port: {port_name}")
                    continue

                self.get_logger().info(f"成功開啟 Port: {port_name}")
                # SDK 每次送封包前 clearPort() = ser.flush()（tcdrain），寫完緊接著讀會被卡約 7 ms
                # （實測 bench_u2d2_write.py：先寫再讀 10 ms → 不 flush 2~5 ms，失敗/位置異常皆 0/200）
                # 同一條序列埠本來就依序送出，不需要等輸出清空
                ph.clearPort = lambda: None
                gw = GroupSyncWrite(ph, pk, ADDR_PROFILE_ACCELERATION, LEN_SYNC_WRITE_ALL)
                gr = GroupSyncRead(ph, pk, addr, length)

                # 重連時只掃描此 Port 上原本的 ID；首次掃描全體
                scan_ids = (self.ports.get(port_name) or {}).get('ids') or self.target_ids

                found_ids = set()
                for mid in scan_ids:
                    _, result, _ = pk.ping(ph, mid)
                    if result == COMM_SUCCESS:
                        gr.addParam(mid)
                        found_ids.add(mid)
                        self.joint_data[mid]['goal'] = None
                        te = self._read_torque_enable(pk, ph, mid)
                        # 扭力開著時，初始目標用馬達「原本的 Goal Position」而不是現在位置，
                        # 否則負重下的下垂量會被當成新目標，每重開一次就沉一點。
                        # （goal 為 None 時 control loop 會在第一次讀取時改用現在位置）
                        if te:
                            gval, gres, gerr = pk.read4ByteTxRx(ph, mid, ADDR_GOAL_POSITION)
                            if gres == COMM_SUCCESS and (gerr & ERR_MASK_NO_ALERT) == 0:
                                if gval > 0x7FFFFFFF: gval -= 4294967296
                                self.joint_data[mid]['goal'] = gval
                        self.get_logger().info(
                            f" -> ID {mid} 發現於 {port_name}，扭力={'ON' if te else 'OFF'}")
                    else:
                        # 只有「上次掃描時在這個 Port 上」的馬達掃不到才記為 OFF（重連時馬達不見了）。
                        # 首次掃描會在每個 Port 掃全部 ID，別的 Port 上的馬達在這裡必然失敗，不能蓋掉。
                        # （重連時 id_port_map 已先被清掉，所以看 self.ports 裡舊的 ids，迴圈結束前還沒被覆蓋）
                        if mid in ((self.ports.get(port_name) or {}).get('ids') or ()):
                            with self._torque_lock:
                                self.torque_enable[mid] = 0
                        self.get_logger().warn(f" -> ID {mid} 掃描失敗")

                self.ports[port_name] = {
                    'ph': ph, 'gw': gw, 'gr': gr,
                    'ids': found_ids, 'alive': True
                }
                for mid in found_ids:
                    self.id_port_map[mid] = port_name

            except Exception as e:
                self.get_logger().error(f"初始化 {port_name} 失敗: {e}")

        total = sum(len(p['ids']) for p in self.ports.values() if p.get('alive'))
        self.get_logger().info(f"硬體初始化完成，共連接 {total} 顆馬達。")
        self._publish_torque_state()

    def _read_torque_enable(self, pk, ph, mid):
        """從馬達讀回 Torque Enable（addr 64），失敗重試 3 次，仍失敗視為 OFF。"""
        value = 0
        for _ in range(3):
            try:
                v, result, error = pk.read1ByteTxRx(ph, mid, ADDR_TORQUE_ENABLE)
                if result == COMM_SUCCESS and (error & ERR_MASK_NO_ALERT) == 0:
                    value = 1 if v else 0
                    break
            except Exception:
                pass
        with self._torque_lock:
            self.torque_enable[mid] = value
        return value

    def _publish_torque_state(self):
        """發布完整扭力狀態：index 0 對應 ID 1，1 = ON、0 = OFF。"""
        with self._torque_lock:
            data = [self.torque_enable.get(mid, 0) for mid in self.target_ids]
        msg = Int16MultiArray()
        msg.data = data
        self.torque_state_pub.publish(msg)

    # ──────────────────────────────────────────────────────────────────────
    # 單一 Port 背景重連（不阻塞主迴圈）
    # ──────────────────────────────────────────────────────────────────────

    def _trigger_reconnect(self, port_name: str):
        """標記 Port 死亡並觸發背景重連（同一 Port 只允許一個重連執行緒）。"""
        if port_name in self._reconnecting:
            return
        self._reconnecting.add(port_name)

        port = self.ports.get(port_name)
        if port:
            port['alive'] = False
            for mid in list(port['ids']):
                self.id_port_map.pop(mid, None)

        self.get_logger().error(f"❌ {port_name} 已隔離，1 秒後嘗試重連...")
        threading.Thread(target=self._reconnect_worker, args=(port_name,), daemon=True).start()

    def _reconnect_worker(self, port_name: str):
        """背景執行緒：關埠 → 等 1 秒 → 重新掃描 → 復活。"""
        old_port = self.ports.get(port_name)
        try:
            old_port['ph'].closePort()
        except Exception:
            pass

        time.sleep(1.0)
        self.get_logger().info(f"🔄 正在重新初始化 {port_name}...")

        try:
            self._init_hardware_blind(target_ports=[port_name])
            recovered = len(self.ports.get(port_name, {}).get('ids', []))
            self.get_logger().info(f"✅ {port_name} 復活，恢復 {recovered} 顆馬達")
        except Exception as e:
            self.get_logger().error(f"重連 {port_name} 失敗: {e}")

        self._reconnecting.discard(port_name)

    # ──────────────────────────────────────────────────────────────────────
    # ROS 2 訂閱回呼
    # ──────────────────────────────────────────────────────────────────────

    def _read_mode_cb(self, msg: Int16MultiArray):
        """
        切換讀取開關。訊息格式：[field, state]
          field 0 = read_velocity，field 1 = read_current
          state 0 = 關閉，state 1 = 開啟
        """
        if len(msg.data) < 2:
            return
        field = msg.data[0]
        state = bool(msg.data[1])
        changed = False
        if field == 0 and self.read_velocity != state:
            self.read_velocity = state
            changed = True
        elif field == 1 and self.read_current != state:
            self.read_current = state
            changed = True
        if changed:
            self._reinit_readers()

    def _command_cb(self, msg: JointState):
        """接收一般行走指令：位置 + 速度"""
        t_recv = time.perf_counter()
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns > 0:   # 沒填 header.stamp 的發布者（例如部分 motion 指令）不算
            age_ms = (self.get_clock().now().nanoseconds - stamp_ns) / 1e6
            if -1000.0 < age_ms < 10000.0:
                self._st_cmd_age.add(age_ms)
        if self._cmd_pending_t is None:
            self._cmd_pending_t = t_recv
        for i, name in enumerate(msg.name):
            try:
                mid = int(name)
                target_pos = int(msg.position[i])
                target_vel = int(msg.velocity[i]) if len(msg.velocity) > i else 0
                if mid in self.id_port_map:
                    self.joint_data[mid]['goal']     = target_pos
                    self.joint_data[mid]['velocity'] = target_vel
            except ValueError:
                pass

    def _latency_ping_cb(self, msg: String):
        """網頁延遲監測：原封回傳並附上 driver 收到的牆上時間。"""
        t_robot = time.time()
        try:
            d = json.loads(msg.data)
            if not isinstance(d, dict):
                return
        except Exception:
            return
        d['t_robot'] = t_robot
        out = String()
        out.data = json.dumps(d)
        self.latency_pong_pub.publish(out)

    def _publish_motor_timing(self):
        ports = {}
        for name, st in self._st_ports.items():
            ports[name] = {'write': st['write'].take(), 'read': st['read'].take(), 'fail': st['fail']}
            st['fail'] = 0
        # 只回報目前存活的 Port（死掉的 Port 不會有新樣本）
        ports = {n: v for n, v in ports.items() if (self.ports.get(n) or {}).get('alive')}
        d = {
            'cmd_age':     self._st_cmd_age.take(),
            'cmd_wait':    self._st_cmd_wait.take(),
            'loop_period': self._st_loop_period.take(),
            'loop_busy':   self._st_loop_busy.take(),
            'ports':       ports,
            'sim':         False,
        }
        out = String()
        out.data = json.dumps(d)
        self.motor_timing_pub.publish(out)

    def _head_cb(self, msg: HeadPackage):
        """接收網頁頭部指令"""
        real_id = self.head_map.get(msg.id)
        if real_id and real_id in self.id_port_map:
            try:
                # self.get_logger().info(f"收到指令! ID:{real_id} 目標:{msg.position} 速度:{msg.speed}")
                self.joint_data[real_id]['goal']     = int(msg.position)
                self.joint_data[real_id]['velocity'] = int(msg.speed)
                if hasattr(msg, 'acceleration'):
                    self.joint_data[real_id]['accel'] = int(msg.acceleration)
            except ValueError:
                self.get_logger().warn("頭部指令數值轉換失敗")

    def _torque_cb(self, msg: Int16MultiArray):
        """扭力控制（盲送版）"""
        if len(msg.data) < 2:
            return
        target_id = msg.data[0]
        state     = msg.data[1]

        ids_to_process = self.target_ids if target_id == 0 else [target_id]
        self.get_logger().info(
            f"盲送{'全體' if target_id == 0 else ''}扭力: ID={target_id}, State={state}"
        )

        if state == 1:
            for mid in ids_to_process:
                self.joint_data[mid]['goal'] = self.joint_data[mid]['present']

        pk = PacketHandler(PROTOCOL_VERSION)
        for mid in ids_to_process:
            port_name = self.id_port_map.get(mid)
            if not port_name:
                continue
            port = self.ports.get(port_name)
            if not port or not port['alive']:
                continue
            ok = False
            last = None
            for _ in range(3):
                try:
                    result, error = pk.write1ByteTxRx(port['ph'], mid, ADDR_TORQUE_ENABLE, state)
                    last = (result, error)
                    ok = ok or (result == COMM_SUCCESS and (error & ERR_MASK_NO_ALERT) == 0)
                    time.sleep(0.001)
                except Exception:
                    pass
            if ok:
                with self._torque_lock:
                    self.torque_enable[mid] = 1 if state else 0
            else:
                self.get_logger().warn(
                    f"ID {mid} 扭力寫入未確認成功，狀態維持原值 (result={last and last[0]}, error={last and last[1]})")

        self._publish_torque_state()

    # ──────────────────────────────────────────────────────────────────────
    # 主控制迴圈（Per-Port 隔離）
    # ──────────────────────────────────────────────────────────────────────

    def _control_loop(self):
        """
        每個 Port 獨立 try/except：某 Port 壞掉只影響該 Port。
        背景執行緒重連，主迴圈不 sleep、不加鎖。
        """
        t_loop0 = time.perf_counter()
        if self._last_loop_t is not None:
            self._st_loop_period.add((t_loop0 - self._last_loop_t) * 1000.0)
        self._last_loop_t = t_loop0
        if self._cmd_pending_t is not None:
            self._st_cmd_wait.add((t_loop0 - self._cmd_pending_t) * 1000.0)
            self._cmd_pending_t = None

        read_vel = self.read_velocity or self.read_current  # 速度包含在電流模式裡
        read_cur = self.read_current
        addr, length = self._get_read_params()

        for port_name, port in list(self.ports.items()):
            if not port['alive']:
                continue

            gw  = port['gw']
            gr  = port['gr']
            ids = [mid for mid in port['ids'] if mid in self.id_port_map]
            st = self._st_ports.setdefault(port_name, {'write': _Stat(), 'read': _Stat(), 'fail': 0})

            try:
                # --- Step 1: 寫入指令 ---
                gw.clearParam()
                dirty = False
                for mid in ids:
                    goal = self.joint_data[mid]['goal']
                    if goal is None:
                        continue

                    goal = max(0, min(4096, goal))
                    vel  = max(0, min(1023, self.joint_data[mid]['velocity']))
                    acc  = max(0, min(32767, self.joint_data[mid]['accel']))

                    param_all = (int(acc).to_bytes(4, 'little', signed=True) +
                                 int(vel).to_bytes(4, 'little', signed=True) +
                                 int(goal).to_bytes(4, 'little', signed=True))
                    gw.addParam(mid, param_all)
                    dirty = True

                if dirty:
                    t0 = time.perf_counter()
                    gw.txPacket()
                    st['write'].add((time.perf_counter() - t0) * 1000.0)

                # --- Step 2: 讀取狀態 ---
                t0 = time.perf_counter()
                rx_result = gr.txRxPacket()
                st['read'].add((time.perf_counter() - t0) * 1000.0)
                if rx_result != COMM_SUCCESS:
                    st['fail'] += 1
                for mid in ids:
                    if not gr.isAvailable(mid, addr, length):
                        continue

                    # 位置（永遠讀取）
                    pos = gr.getData(mid, ADDR_PRESENT_POSITION, 4)
                    if pos > 0x7FFFFFFF:
                        pos -= 4294967296
                    self.joint_data[mid]['present'] = pos
                    if self.joint_data[mid]['goal'] is None:
                        self.joint_data[mid]['goal'] = pos

                    # 速度（read_velocity 或 read_current 時讀取）
                    if read_vel:
                        vel = gr.getData(mid, ADDR_PRESENT_VELOCITY, 4)
                        if vel > 0x7FFFFFFF:
                            vel -= 4294967296
                        self.joint_data[mid]['present_vel'] = vel

                    # 電流（read_current 時讀取）
                    if read_cur:
                        cur = gr.getData(mid, ADDR_PRESENT_CURRENT, 2)
                        if cur > 0x7FFF:
                            cur -= 65536  # signed 16-bit，unit = 1 raw ≈ 2.69 mA
                        self.joint_data[mid]['present_cur'] = cur

            except Exception as e:
                self.get_logger().error(f"❌ {port_name} I/O 異常: {e}")
                self._trigger_reconnect(port_name)

        # --- Step 3: Publish（無論哪個 Port 壞掉都照常發布）---
        now   = self.get_clock().now().to_msg()
        ids   = sorted(self.joint_data.keys())
        names = [str(i) for i in ids]

        pos_msg = JointState()
        pos_msg.header.stamp = now
        pos_msg.name     = names
        pos_msg.position = [
            float(self.joint_data[i]['present']) if self.joint_data[i]['present'] is not None else 0.0
            for i in ids
        ]
        self.joint_pub.publish(pos_msg)

        vel_msg = JointState()
        vel_msg.header.stamp = now
        vel_msg.name     = names
        vel_msg.velocity = [float(self.joint_data[i]['present_vel']) for i in ids]
        self.vel_pub.publish(vel_msg)

        cur_msg = JointState()
        cur_msg.header.stamp = now
        cur_msg.name   = names
        cur_msg.effort = [float(self.joint_data[i]['present_cur']) for i in ids]
        self.cur_pub.publish(cur_msg)
        self._st_loop_busy.add((time.perf_counter() - t_loop0) * 1000.0)


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Int16MultiArray
# # [新增 1] 匯入 HeadPackage 訊息格式
# from tku_msgs.msg import HeadPackage

# import time
# import threading
# from collections import defaultdict
# from dynamixel_sdk import * # 使用官方 SDK

# # ==== 設定參數 ====
# DEFAULT_BAUDRATE = 1000000

# # 預設目標 ID 範圍：1~23 (涵蓋你所有的手、腳、頭 ID)
# ALL_TARGET_IDS = list(range(1, 24))

# # Control Table Address (X series / MX 2.0)
# ADDR_TORQUE_ENABLE          = 64
# ADDR_PROFILE_ACCELERATION   = 108
# ADDR_PROFILE_VELOCITY       = 112
# ADDR_GOAL_POSITION          = 116
# ADDR_PRESENT_POSITION       = 132

# LEN_GOAL_POSITION           = 4
# LEN_PRESENT_POSITION        = 4
# LEN_PROFILE_VELOCITY        = 4
# LEN_PROFILE_ACCELERATION    = 4
# LEN_SYNC_WRITE_ALL          = 12   # Acc(4) + Vel(4) + Pos(4)

# PROTOCOL_VERSION            = 2.0


# class DynamixelDriver(Node):
#     def __init__(self):
#         super().__init__('dynamixel_driver_node')

#         # 1. 宣告參數
#         self.declare_parameter('baudrate', DEFAULT_BAUDRATE)
#         self.declare_parameter('ports', ['/dev/U2D2_P1', '/dev/U2D2_P2', '/dev/U2D2_P3'])

#         self.baudrate   = self.get_parameter('baudrate').value
#         self.port_list  = self.get_parameter('ports').value
#         self.target_ids = sorted(ALL_TARGET_IDS)

#         # 2. 記憶體狀態
#         self.joint_data = defaultdict(lambda: {'present': None, 'goal': None, 'velocity': 0, 'accel': 0})

#         # [新增 2] 頭部 ID 映射表
#         # JS 送來的 ID 1 (水平) -> 對應真實馬達 ID 22
#         # JS 送來的 ID 2 (垂直) -> 對應真實馬達 ID 23
#         self.head_map = {1: 22, 2: 23}

#         # 3. 以 Port 為單位的硬體容器
#         # self.ports[port_name] = {
#         #   'ph': PortHandler, 'gw': GroupSyncWrite, 'gr': GroupSyncRead,
#         #   'ids': set(),  'alive': bool
#         # }
#         self.ports      = {}
#         self.id_port_map = {}  # {motor_id: port_name}

#         # 正在重連中的 Port 集合（避免重複觸發）
#         self._reconnecting = set()

#         self.get_logger().info(f"正在初始化 U2D2 Ports (盲送模式)... Baudrate: {self.baudrate}")
#         self._init_hardware_blind()

#         # 4. ROS 2 通訊介面
#         self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

#         self.joint_sub = self.create_subscription(
#             JointState,
#             '/joint_commands',
#             self._command_cb,
#             10
#         )

#         self.torque_sub = self.create_subscription(
#             Int16MultiArray,
#             '/set_torque',
#             self._torque_cb,
#             10
#         )

#         # [新增 3] 訂閱來自網頁的頭部控制指令
#         self.head_sub = self.create_subscription(
#             HeadPackage,
#             'Head_Topic',
#             self._head_cb,
#             10
#         )

#         # 5. Timer (控制迴圈 20Hz)
#         self.timer = self.create_timer(0.05, self._control_loop)

#         self.get_logger().info("Dynamixel Driver Ready! (Per-Port Isolated)")

#     # ──────────────────────────────────────────────────────────────────────
#     # 硬體初始化
#     # ──────────────────────────────────────────────────────────────────────

#     def _init_hardware_blind(self, target_ports=None):
#         """
#         掃描並綁定馬達。
#         target_ports=None → 初始化全部；指定列表 → 只重建指定 Port。
#         """
#         if target_ports is None:
#             target_ports = self.port_list

#         pk = PacketHandler(PROTOCOL_VERSION)

#         for port_name in target_ports:
#             try:
#                 ph = PortHandler(port_name)
#                 if not (ph.openPort() and ph.setBaudRate(self.baudrate)):
#                     self.get_logger().warn(f"無法開啟 Port: {port_name}")
#                     continue

#                 self.get_logger().info(f"成功開啟 Port: {port_name}")
#                 gw = GroupSyncWrite(ph, pk, ADDR_PROFILE_ACCELERATION, LEN_SYNC_WRITE_ALL)
#                 gr = GroupSyncRead(ph, pk, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

#                 # 重連時只掃描此 Port 上原本的 ID；首次掃描全體
#                 scan_ids = (self.ports.get(port_name) or {}).get('ids') or self.target_ids

#                 found_ids = set()
#                 for mid in scan_ids:
#                     _, result, _ = pk.ping(ph, mid)
#                     if result == COMM_SUCCESS:
#                         gr.addParam(mid)
#                         found_ids.add(mid)
#                         self.joint_data[mid]['goal'] = None
#                         self.get_logger().info(f" -> ID {mid} 發現於 {port_name}")
#                     else:
#                         self.get_logger().warn(f" -> ID {mid} 掃描失敗")

#                 self.ports[port_name] = {
#                     'ph': ph, 'gw': gw, 'gr': gr,
#                     'ids': found_ids, 'alive': True
#                 }
#                 for mid in found_ids:
#                     self.id_port_map[mid] = port_name

#             except Exception as e:
#                 self.get_logger().error(f"初始化 {port_name} 失敗: {e}")

#         total = sum(len(p['ids']) for p in self.ports.values() if p.get('alive'))
#         self.get_logger().info(f"硬體初始化完成，共連接 {total} 顆馬達。")

#     # ──────────────────────────────────────────────────────────────────────
#     # 單一 Port 背景重連（不阻塞主迴圈）
#     # ──────────────────────────────────────────────────────────────────────

#     def _trigger_reconnect(self, port_name: str):
#         """標記 Port 死亡並觸發背景重連（同一 Port 只允許一個重連執行緒）。"""
#         if port_name in self._reconnecting:
#             return
#         self._reconnecting.add(port_name)

#         port = self.ports.get(port_name)
#         if port:
#             port['alive'] = False
#             for mid in list(port['ids']):
#                 self.id_port_map.pop(mid, None)

#         self.get_logger().error(f"❌ {port_name} 已隔離，1 秒後嘗試重連...")
#         threading.Thread(target=self._reconnect_worker, args=(port_name,), daemon=True).start()

#     def _reconnect_worker(self, port_name: str):
#         """背景執行緒：關埠 → 等 1 秒 → 重新掃描 → 復活。"""
#         old_port = self.ports.get(port_name)
#         try:
#             old_port['ph'].closePort()
#         except Exception:
#             pass

#         time.sleep(1.0)
#         self.get_logger().info(f"🔄 正在重新初始化 {port_name}...")

#         try:
#             self._init_hardware_blind(target_ports=[port_name])
#             recovered = len(self.ports.get(port_name, {}).get('ids', []))
#             self.get_logger().info(f"✅ {port_name} 復活，恢復 {recovered} 顆馬達")
#         except Exception as e:
#             self.get_logger().error(f"重連 {port_name} 失敗: {e}")

#         self._reconnecting.discard(port_name)

#     # ──────────────────────────────────────────────────────────────────────
#     # ROS 2 訂閱回呼（與原版相同）
#     # ──────────────────────────────────────────────────────────────────────

#     def _command_cb(self, msg: JointState):
#         """接收一般行走指令：位置 + 速度"""
#         for i, name in enumerate(msg.name):
#             try:
#                 mid = int(name)
#                 target_pos = int(msg.position[i])

#                 target_vel = 0
#                 if len(msg.velocity) > i:
#                     target_vel = int(msg.velocity[i])

#                 if mid in self.id_port_map:
#                     self.joint_data[mid]['goal']     = target_pos
#                     self.joint_data[mid]['velocity'] = target_vel
#             except ValueError:
#                 pass

#     def _head_cb(self, msg: HeadPackage):
#         """[新增 4] 接收網頁頭部指令"""
#         real_id = self.head_map.get(msg.id)
#         if real_id and real_id in self.id_port_map:
#             try:
#                 self.get_logger().info(f"收到指令! ID:{real_id} 目標:{msg.position} 速度:{msg.speed}")
#                 self.joint_data[real_id]['goal']     = int(msg.position)
#                 self.joint_data[real_id]['velocity'] = int(msg.speed)
#                 if hasattr(msg, 'acceleration'):
#                     self.joint_data[real_id]['accel'] = int(msg.acceleration)
#             except ValueError:
#                 self.get_logger().warn("頭部指令數值轉換失敗")

#     def _torque_cb(self, msg: Int16MultiArray):
#         """扭力控制 (盲送版)"""
#         if len(msg.data) < 2: return
#         target_id = msg.data[0]
#         state     = msg.data[1]

#         ids_to_process = self.target_ids if target_id == 0 else [target_id]
#         self.get_logger().info(
#             f"盲送{'全體' if target_id == 0 else ''}扭力: ID={target_id}, State={state}"
#         )

#         if state == 1:
#             for mid in ids_to_process:
#                 self.joint_data[mid]['goal'] = self.joint_data[mid]['present']

#         pk = PacketHandler(PROTOCOL_VERSION)
#         for mid in ids_to_process:
#             port_name = self.id_port_map.get(mid)
#             if not port_name:
#                 continue
#             port = self.ports.get(port_name)
#             if not port or not port['alive']:
#                 continue
#             ph = port['ph']
#             for _ in range(3):
#                 try:
#                     pk.write1ByteTxRx(ph, mid, ADDR_TORQUE_ENABLE, state)
#                     time.sleep(0.001)
#                 except Exception:
#                     pass

#     # ──────────────────────────────────────────────────────────────────────
#     # 主控制迴圈（Per-Port 隔離）
#     # ──────────────────────────────────────────────────────────────────────

#     def _control_loop(self):
#         """
#         [精準同步版 + Per-Port 斷線隔離]
#         - 每個 Port 獨立 try/except：某 Port 壞掉只影響該 Port 的馬達
#         - 其他 Port 繼續正常 20Hz 運作
#         - 背景執行緒重連，主迴圈不 sleep、不加鎖
#         """
#         for port_name, port in list(self.ports.items()):
#             if not port['alive']:
#                 continue

#             gw = port['gw']
#             gr = port['gr']
#             ids = [mid for mid in port['ids'] if mid in self.id_port_map]

#             try:
#                 # --- Step 1: 寫入指令 ---
#                 gw.clearParam()
#                 dirty = False
#                 for mid in ids:
#                     goal = self.joint_data[mid]['goal']
#                     if goal is None: continue

#                     goal = max(0, min(4096, goal))
#                     vel  = max(0, min(1023, self.joint_data[mid]['velocity']))
#                     acc  = max(0, min(32767, self.joint_data[mid]['accel']))

#                     param_all = (int(acc).to_bytes(4, 'little', signed=True) +
#                                  int(vel).to_bytes(4, 'little', signed=True) +
#                                  int(goal).to_bytes(4, 'little', signed=True))
#                     gw.addParam(mid, param_all)
#                     dirty = True

#                 if dirty:
#                     gw.txPacket()

#                 # --- Step 2: 讀取狀態 ---
#                 gr.txRxPacket()
#                 for mid in ids:
#                     if gr.isAvailable(mid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
#                         val = gr.getData(mid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
#                         if val > 0x7FFFFFFF: val -= 4294967296
#                         self.joint_data[mid]['present'] = val
#                         if self.joint_data[mid]['goal'] is None:
#                             self.joint_data[mid]['goal'] = val

#             except Exception as e:
#                 self.get_logger().error(f"❌ {port_name} I/O 異常: {e}")
#                 self._trigger_reconnect(port_name)

#         # --- Step 3: Publish（無論哪個 Port 壞掉都照常發布）---
#         pub_msg = JointState()
#         pub_msg.header.stamp = self.get_clock().now().to_msg()
#         ids = sorted(self.joint_data.keys())
#         pub_msg.name     = [str(i) for i in ids]
#         pub_msg.position = [
#             float(self.joint_data[i]['present']) if self.joint_data[i]['present'] is not None else 0.0
#             for i in ids
#         ]
#         self.joint_pub.publish(pub_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = DynamixelDriver()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == '__main__':
#     main()