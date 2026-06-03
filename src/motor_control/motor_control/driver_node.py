#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray
from tku_msgs.msg import HeadPackage

import time
import threading
try:
    import serial  # type: ignore
except ImportError:
    serial = None  # type: ignore[assignment]
import termios
from collections import defaultdict
from dynamixel_sdk import (
    PortHandler, PacketHandler,
    GroupSyncWrite, GroupSyncRead,
    COMM_SUCCESS,
)

# ==== 設定參數 ====
DEFAULT_BAUDRATE = 1000000

ALL_TARGET_IDS = list(range(1, 24))

ADDR_TORQUE_ENABLE          = 64
ADDR_PROFILE_ACCELERATION   = 108
ADDR_PROFILE_VELOCITY       = 112
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_CURRENT        = 126
ADDR_PRESENT_VELOCITY       = 128
ADDR_PRESENT_POSITION       = 132
ADDR_PRESENT_INPUT_VOLTAGE  = 144
ADDR_PRESENT_TEMPERATURE    = 146

LEN_GOAL_POSITION           = 4
LEN_PRESENT_POSITION        = 4
LEN_PROFILE_VELOCITY        = 4
LEN_PROFILE_ACCELERATION    = 4
LEN_SYNC_WRITE_ALL          = 12   # Acc(4) + Vel(4) + Pos(4)
LEN_SYNC_READ               = 21   # addr 126 → 146 inclusive

PROTOCOL_VERSION            = 2.0

# 三層防禦攔截的異常類型：pySerial、Linux I/O、termios C 庫
_HW_EXCEPTIONS = (serial.SerialException, OSError, termios.error)


class DynamixelDriver(Node):
    def __init__(self):
        super().__init__('dynamixel_driver_node')

        self.declare_parameter('baudrate', DEFAULT_BAUDRATE)
        self.declare_parameter('ports', ['/dev/U2D2_P1', '/dev/U2D2_P2', '/dev/U2D2_P3'])

        self.baudrate     = self.get_parameter('baudrate').value
        self.port_list    = self.get_parameter('ports').value
        self.target_ids   = sorted(ALL_TARGET_IDS)

        self.joint_data = defaultdict(lambda: {
            'present': None, 'goal': None, 'velocity': 0, 'accel': 0,
            'present_velocity': 0, 'present_current': 0,
            'present_voltage': 0, 'present_temperature': 0,
        })

        self.head_map = {1: 22, 2: 23}

        # 以 Port 為中心的硬體資料結構：
        # self.ports = {
        #   port_name: {
        #     'ph': PortHandler, 'gw': GroupSyncWrite, 'gr': GroupSyncRead,
        #     'alive': bool, 'motor_ids': set()
        #   }
        # }
        self.ports      = {}
        self.id_port_map = {}   # {motor_id: port_name}

        # 重連狀態追蹤（執行緒安全）
        self._recovering_ports = set()
        self._hw_lock = threading.RLock()

        self.get_logger().info(f"正在初始化 U2D2 Ports (盲送模式)... Baudrate: {self.baudrate}")
        self._init_hardware_blind()

        self.joint_pub = self.create_publisher(JointState, '/joint_states',       10)
        self.vel_pub   = self.create_publisher(JointState, '/joint_velocities',   10)
        self.cur_pub   = self.create_publisher(JointState, '/joint_currents',     10)
        self.volt_pub  = self.create_publisher(JointState, '/joint_voltages',     10)
        self.temp_pub  = self.create_publisher(JointState, '/joint_temperatures', 10)

        self.joint_sub  = self.create_subscription(JointState,       '/joint_commands', self._command_cb, 10)
        self.torque_sub = self.create_subscription(Int16MultiArray,  '/set_torque',     self._torque_cb,  10)
        self.head_sub   = self.create_subscription(HeadPackage,      'Head_Topic',      self._head_cb,    10)

        self.timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info("Dynamixel Driver Ready! (3-Layer Fault-Isolated)")

    # ──────────────────────────────────────────────────────────────────────
    # 硬體初始化
    # ──────────────────────────────────────────────────────────────────────

    def _init_hardware_blind(self, target_port_names=None):
        """
        開啟 Port 並 Ping 掃描馬達分佈。
        target_port_names=None → 初始化全部；指定列表 → 只重建指定 Port。
        """
        if target_port_names is None:
            target_port_names = self.port_list

        pk = PacketHandler(PROTOCOL_VERSION)

        for port_name in target_port_names:
            try:
                ph = PortHandler(port_name)
                if not (ph.openPort() and ph.setBaudRate(self.baudrate)):
                    self.get_logger().warn(f"無法開啟 Port: {port_name}")
                    continue

                self.get_logger().info(f"成功開啟 Port: {port_name}")
                gw = GroupSyncWrite(ph, pk, ADDR_PROFILE_ACCELERATION, LEN_SYNC_WRITE_ALL)
                gr = GroupSyncRead(ph, pk, ADDR_PRESENT_CURRENT, LEN_SYNC_READ)

                # 重連時只掃描此 Port 已知的 ID；首次掃描全體 target_ids
                existing_ids = self.ports.get(port_name, {}).get('motor_ids') or self.target_ids

                port_entry = {'ph': ph, 'gw': gw, 'gr': gr, 'alive': True, 'motor_ids': set()}

                for mid in existing_ids:
                    _, result, _ = pk.ping(ph, mid)
                    if result == COMM_SUCCESS:
                        gr.addParam(mid)
                        port_entry['motor_ids'].add(mid)
                        self.joint_data[mid]['goal'] = None
                        self.get_logger().info(f" -> ID {mid} 發現於 {port_name}")

                with self._hw_lock:
                    self.ports[port_name] = port_entry
                    for mid in port_entry['motor_ids']:
                        self.id_port_map[mid] = port_name

            except Exception as e:
                self.get_logger().error(f"初始化 {port_name} 失敗: {e}")

        total = sum(len(p['motor_ids']) for p in self.ports.values() if p.get('alive'))
        self.get_logger().info(f"硬體初始化完成，共連接 {total} 顆馬達。")

    # ──────────────────────────────────────────────────────────────────────
    # 第二道防線：分 Port 隔離硬體重啟
    # ──────────────────────────────────────────────────────────────────────

    def _schedule_hard_reset(self, port_name: str):
        """標記單一 Port 死亡並觸發背景重啟，其他 Port 繼續 100Hz 正常運作。"""
        with self._hw_lock:
            if port_name in self._recovering_ports:
                return
            self._recovering_ports.add(port_name)
            port = self.ports.get(port_name)
            if port:
                port['alive'] = False
                # 從 id_port_map 移除此 Port 上的所有馬達
                dead_ids = [mid for mid, pn in list(self.id_port_map.items()) if pn == port_name]
                for mid in dead_ids:
                    del self.id_port_map[mid]

        self.get_logger().error(
            f"[硬重啟] {port_name} 確定失效，已隔離。其他 Port 繼續運作，1 秒後重啟..."
        )
        threading.Thread(target=self._hard_reset_port, args=(port_name,), daemon=True).start()

    def _hard_reset_port(self, port_name: str):
        """背景執行緒：靜置 1.5 秒 → 關埠重開 → 盲送 Ping 掃描 → 動態綁定馬達。"""
        time.sleep(1)
        self.get_logger().info(f"[硬重啟] 正在重新開啟 {port_name}...")

        # 安全關閉舊 Port
        old_port = self.ports.get(port_name)
        if old_port:
            try:
                old_port['ph'].closePort()
            except Exception:
                pass

        try:
            pk = PacketHandler(PROTOCOL_VERSION)
            ph = PortHandler(port_name)
            if not (ph.openPort() and ph.setBaudRate(self.baudrate)):
                raise RuntimeError(f"無法重新開啟 {port_name}")

            gw = GroupSyncWrite(ph, pk, ADDR_PROFILE_ACCELERATION, LEN_SYNC_WRITE_ALL)
            gr = GroupSyncRead(ph, pk, ADDR_PRESENT_CURRENT, LEN_SYNC_READ)

            known_ids = (old_port['motor_ids'] if old_port else set()) or set(self.target_ids)
            recovered = []
            for mid in known_ids:
                _, result, _ = pk.ping(ph, mid)
                if result == COMM_SUCCESS:
                    gr.addParam(mid)
                    recovered.append(mid)

            with self._hw_lock:
                self.ports[port_name] = {
                    'ph': ph, 'gw': gw, 'gr': gr,
                    'alive': True, 'motor_ids': set(recovered)
                }
                for mid in recovered:
                    self.id_port_map[mid] = port_name
                self._recovering_ports.discard(port_name)

            self.get_logger().info(
                f"[硬重啟] {port_name} 復活成功，恢復 {len(recovered)} 顆馬達: {sorted(recovered)}"
            )

        except Exception as e:
            self.get_logger().error(f"[硬重啟] {port_name} 重啟失敗: {e}，等待下次觸發重試。")
            with self._hw_lock:
                self._recovering_ports.discard(port_name)

    # ──────────────────────────────────────────────────────────────────────
    # ROS 2 訂閱回呼
    # ──────────────────────────────────────────────────────────────────────

    def _command_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            try:
                mid = int(name)
                if mid not in self.id_port_map:
                    continue
                self.joint_data[mid]['goal'] = int(msg.position[i])
                if len(msg.velocity) > i:
                    self.joint_data[mid]['velocity'] = int(msg.velocity[i])
            except ValueError:
                pass

    def _head_cb(self, msg: HeadPackage):
        real_id = self.head_map.get(msg.id)
        if real_id and real_id in self.id_port_map:
            try:
                self.joint_data[real_id]['goal']     = int(msg.position)
                self.joint_data[real_id]['velocity'] = int(msg.speed)
                if hasattr(msg, 'acceleration'):
                    self.joint_data[real_id]['accel'] = int(msg.acceleration)
            except ValueError:
                self.get_logger().warn("頭部指令數值轉換失敗")

    def _torque_cb(self, msg: Int16MultiArray):
        if len(msg.data) < 2:
            return
        target_id, state = msg.data[0], msg.data[1]
        ids_to_process = self.target_ids if target_id == 0 else [target_id]
        self.get_logger().info(f"扭力指令: ID={target_id if target_id else '全體'}, State={state}")

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
    # 三層防禦核心控制迴圈
    # ──────────────────────────────────────────────────────────────────────

    def _control_loop(self):
        """
        第 0 層：正常 100Hz 讀寫。
        第 1 層（軟重試）：偵測到閃斷 → flush 緩衝區 → 同週期立即重試 → 0ms 無縫回歸。
        第 2 層（硬重啟）：軟重試失敗 → 只隔離死亡 Port → 其他 Port 繼續運作 → 1.5s 後原地復活。
        """
        # 取得活躍 Port 快照（不阻塞整個迴圈）
        with self._hw_lock:
            active_ports = {pn: p for pn, p in self.ports.items() if p.get('alive')}

        for port_name, port in active_ports.items():
            ph  = port['ph']
            gw  = port['gw']
            gr  = port['gr']
            ids = [mid for mid in port['motor_ids'] if mid in self.id_port_map]

            # ── Step 1: Write ──────────────────────────────────────────────
            gw.clearParam()
            dirty = False
            for mid in ids:
                goal = self.joint_data[mid]['goal']
                if goal is None:
                    continue
                goal = max(0, min(4096, goal))
                vel  = max(0, min(1023, self.joint_data[mid]['velocity']))
                acc  = max(0, min(32767, self.joint_data[mid]['accel']))
                param = (int(acc).to_bytes(4, 'little', signed=True) +
                         int(vel).to_bytes(4, 'little', signed=True) +
                         int(goal).to_bytes(4, 'little', signed=True))
                gw.addParam(mid, param)
                dirty = True

            if dirty:
                try:
                    gw.txPacket()
                except _HW_EXCEPTIONS as e:
                    # 第 1 層：軟重試
                    self.get_logger().warn(f"[軟重試] {port_name} 寫入異常 ({e})，flush 後重試...")
                    try:
                        ph.clearPort()
                        gw.txPacket()
                        self.get_logger().info(f"[軟重試] {port_name} 寫入恢復")
                    except _HW_EXCEPTIONS as e2:
                        # 第 2 層：硬重啟此 Port
                        self.get_logger().error(f"[硬重啟] {port_name} 軟重試失敗: {e2}")
                        self._schedule_hard_reset(port_name)
                        continue
                except Exception as e:
                    self.get_logger().error(f"[硬重啟] {port_name} 未知異常: {e}")
                    self._schedule_hard_reset(port_name)
                    continue

            # ── Step 2: Read ───────────────────────────────────────────────
            try:
                gr.txRxPacket()
            except _HW_EXCEPTIONS as e:
                # 第 1 層：軟重試
                self.get_logger().warn(f"[軟重試] {port_name} 讀取異常 ({e})，flush 後重試...")
                try:
                    ph.clearPort()
                    gr.txRxPacket()
                    self.get_logger().info(f"[軟重試] {port_name} 讀取恢復")
                except _HW_EXCEPTIONS as e2:
                    # 第 2 層：硬重啟此 Port
                    self.get_logger().error(f"[硬重啟] {port_name} 讀取軟重試失敗: {e2}")
                    self._schedule_hard_reset(port_name)
                    continue
            except Exception as e:
                self.get_logger().error(f"[硬重啟] {port_name} 讀取未知異常: {e}")
                self._schedule_hard_reset(port_name)
                continue

            for mid in ids:
                if not gr.isAvailable(mid, ADDR_PRESENT_CURRENT, LEN_SYNC_READ):
                    continue

                raw = gr.getData(mid, ADDR_PRESENT_POSITION, 4)
                pos = raw - 4294967296 if raw > 0x7FFFFFFF else raw
                self.joint_data[mid]['present'] = pos
                if self.joint_data[mid]['goal'] is None:
                    self.joint_data[mid]['goal'] = pos

                raw = gr.getData(mid, ADDR_PRESENT_VELOCITY, 4)
                self.joint_data[mid]['present_velocity'] = raw - 4294967296 if raw > 0x7FFFFFFF else raw

                raw = gr.getData(mid, ADDR_PRESENT_CURRENT, 2)
                self.joint_data[mid]['present_current'] = raw - 65536 if raw > 0x7FFF else raw

                self.joint_data[mid]['present_voltage']     = gr.getData(mid, ADDR_PRESENT_INPUT_VOLTAGE, 2)
                self.joint_data[mid]['present_temperature'] = gr.getData(mid, ADDR_PRESENT_TEMPERATURE, 1)

        # ── Step 3: Publish ────────────────────────────────────────────────
        now   = self.get_clock().now().to_msg()
        ids   = sorted(self.joint_data.keys())
        names = [str(i) for i in ids]

        def _f(key, default=0.0):
            return [float(self.joint_data[i][key]) if self.joint_data[i][key] is not None else default
                    for i in ids]

        pos_msg = JointState()
        pos_msg.header.stamp = now; pos_msg.name = names
        pos_msg.position = _f('present')
        self.joint_pub.publish(pos_msg)

        vel_msg = JointState()
        vel_msg.header.stamp = now; vel_msg.name = names
        vel_msg.velocity = _f('present_velocity')
        self.vel_pub.publish(vel_msg)

        cur_msg = JointState()
        cur_msg.header.stamp = now; cur_msg.name = names
        cur_msg.effort = _f('present_current')
        self.cur_pub.publish(cur_msg)

        volt_msg = JointState()
        volt_msg.header.stamp = now; volt_msg.name = names
        volt_msg.position = _f('present_voltage')
        self.volt_pub.publish(volt_msg)

        temp_msg = JointState()
        temp_msg.header.stamp = now; temp_msg.name = names
        temp_msg.position = _f('present_temperature')
        self.temp_pub.publish(temp_msg)


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
