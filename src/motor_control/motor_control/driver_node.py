#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray
from tku_msgs.msg import HeadPackage
from rcl_interfaces.msg import SetParametersResult

import time
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

        # 5. 動態參數回呼（執行中切換讀取開關）
        self.add_on_set_parameters_callback(self._param_cb)

        # 6. 控制迴圈 20Hz
        self.timer = self.create_timer(0.05, self._control_loop)

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
                        self.get_logger().info(f" -> ID {mid} 發現於 {port_name}")
                    else:
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

    def _head_cb(self, msg: HeadPackage):
        """接收網頁頭部指令"""
        real_id = self.head_map.get(msg.id)
        if real_id and real_id in self.id_port_map:
            try:
                self.get_logger().info(f"收到指令! ID:{real_id} 目標:{msg.position} 速度:{msg.speed}")
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
            for _ in range(3):
                try:
                    pk.write1ByteTxRx(port['ph'], mid, ADDR_TORQUE_ENABLE, state)
                    time.sleep(0.001)
                except Exception:
                    pass

    # ──────────────────────────────────────────────────────────────────────
    # 主控制迴圈（Per-Port 隔離）
    # ──────────────────────────────────────────────────────────────────────

    def _control_loop(self):
        """
        每個 Port 獨立 try/except：某 Port 壞掉只影響該 Port。
        背景執行緒重連，主迴圈不 sleep、不加鎖。
        """
        read_vel = self.read_velocity or self.read_current  # 速度包含在電流模式裡
        read_cur = self.read_current
        addr, length = self._get_read_params()

        for port_name, port in list(self.ports.items()):
            if not port['alive']:
                continue

            gw  = port['gw']
            gr  = port['gr']
            ids = [mid for mid in port['ids'] if mid in self.id_port_map]

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
                    gw.txPacket()

                # --- Step 2: 讀取狀態 ---
                gr.txRxPacket()
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