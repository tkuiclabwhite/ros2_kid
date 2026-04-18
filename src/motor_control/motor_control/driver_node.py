#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray
# [新增 1] 匯入 HeadPackage 訊息格式
from tku_msgs.msg import HeadPackage 

import time
from collections import defaultdict
from dynamixel_sdk import * # 使用官方 SDK

# ==== 設定參數 ====
DEFAULT_BAUDRATE = 1000000

# 預設目標 ID 範圍：1~23 (涵蓋你所有的手、腳、頭 ID)
# 在盲送模式下，這些 ID 都會過濾並綁定
ALL_TARGET_IDS = list(range(1, 24)) 

# Control Table Address (X series / MX 2.0)
ADDR_TORQUE_ENABLE          = 64
ADDR_PROFILE_ACCELERATION   = 108  
ADDR_PROFILE_VELOCITY       = 112
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132

LEN_GOAL_POSITION           = 4
LEN_PRESENT_POSITION        = 4
LEN_PROFILE_VELOCITY        = 4
LEN_PROFILE_ACCELERATION    = 4    
LEN_SYNC_WRITE_ALL          = 12   # [優化] Acc(4) + Vel(4) + Pos(4)

PROTOCOL_VERSION            = 2.0

class DynamixelDriver(Node):
    def __init__(self):
        super().__init__('dynamixel_driver_node')

        # 1. 宣告參數
        self.declare_parameter('baudrate', DEFAULT_BAUDRATE)
        self.declare_parameter('ports', ['/dev/U2D2_P1', '/dev/U2D2_P2', '/dev/U2D2_P3']) 
        
        self.baudrate = self.get_parameter('baudrate').value
        self.port_list = self.get_parameter('ports').value

        # 2. 記憶體狀態
        # Key: ID (int) -> Value: {'present': int, 'goal': int, 'velocity': int, 'accel': int}
        self.joint_data = defaultdict(lambda: {'present': None, 'goal': None, 'velocity': 0, 'accel': 0})
        
        # [新增 2] 頭部 ID 映射表
        # JS 送來的 ID 1 (水平) -> 對應真實馬達 ID 22
        # JS 送來的 ID 2 (垂直) -> 對應真實馬達 ID 23
        # 如果方向相反，請交換這裡的 Value (22, 23)
        self.head_map = {
            1: 22, 
            2: 23
        }

        # 3. SDK 物件容器
        self.port_handlers = []
        self.group_readers = []
        self.group_writers_all = [] # [優化] 統一為一個 Writer
        
        # 記錄 ID 屬於哪一組 Writer/Reader
        self.id_port_map = {} 

        self.target_ids = sorted(ALL_TARGET_IDS)
        
        self.get_logger().info(f"正在初始化 U2D2 Ports (盲送模式)... Baudrate: {self.baudrate}")
        self._init_hardware_blind()

        # 4. ROS 2 通訊介面
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        self.joint_sub = self.create_subscription(
            JointState, 
            '/joint_commands', 
            self._command_cb, 
            10
        )

        self.torque_sub = self.create_subscription(
            Int16MultiArray, 
            '/set_torque', 
            self._torque_cb, 
            10
        )

        # [新增 3] 訂閱來自網頁的頭部控制指令
        self.head_sub = self.create_subscription(
            HeadPackage, 
            'Head_Topic', 
            self._head_cb, 
            10
        )

        # 5. Timer (控制迴圈 100Hz)
        self.timer = self.create_timer(0.05, self._control_loop)
        
        self.get_logger().info("Dynamixel Driver Ready! (12-byte Optimized)")

    def _init_hardware_blind(self):
        """
        [修正版] 初始化並自動掃描：找出馬達究竟在哪個 Port
        """
        # 1. 開啟所有 Port 並建立對應的 Reader/Writer
        valid_ports = [] # 存 (ph, gw_all, gr)
        
        for port_name in self.port_list:
            try:
                ph = PortHandler(port_name)
                if ph.openPort() and ph.setBaudRate(self.baudrate):
                    self.get_logger().info(f"成功開啟 Port: {port_name}")
                    
                    pk = PacketHandler(PROTOCOL_VERSION)
                    # [優化] 使用 12-byte SyncWrite 位址從 108 開始
                    gw_all = GroupSyncWrite(ph, pk, ADDR_PROFILE_ACCELERATION, LEN_SYNC_WRITE_ALL)
                    gr = GroupSyncRead(ph, pk, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                    
                    self.port_handlers.append(ph)
                    self.group_writers_all.append(gw_all)
                    self.group_readers.append(gr)
                    
                    valid_ports.append((ph, gw_all, gr))
                else:
                    self.get_logger().warn(f"無法開啟 Port: {port_name}")
            except Exception as e:
                self.get_logger().error(f"Error init {port_name}: {e}")

        if not valid_ports:
            self.get_logger().error("沒有任何 Port 開啟成功！")
            return

        # 2. 開始掃描：對每個 ID，去每個 Port Ping 看看
        self.get_logger().info(f"開始掃描 ID {self.target_ids} 分佈在哪些 Port...")
        
        pk = PacketHandler(PROTOCOL_VERSION)
        
        for mid in self.target_ids:
            found = False
            for (ph, gw_all, gr) in valid_ports:
                # 嘗試 Ping
                model_number, result, error = pk.ping(ph, mid)
                if result == COMM_SUCCESS:
                    # 找到了！綁定這個 ID 到這個 Port
                    self.id_port_map[mid] = (gw_all, gr, ph)
                    gr.addParam(mid) # 加入讀取清單
                    
                    # 初始化 Goal 為 None (稍後讀到值會自動更新)
                    self.joint_data[mid]['goal'] = None
                    
                    self.get_logger().info(f" -> ID {mid} 發現於 {ph.getPortName()}")
                    found = True
                    break # 找到就不用試下一個 Port 了
            
            if not found:
                self.get_logger().warn(f" -> ID {mid} 掃描失敗 (不在任何 Port 上)")

        # 3. 確保 group reader 準備好
        self.get_logger().info(f"硬體初始化完成，共連接 {len(self.id_port_map)} 顆馬達。")

    def _command_cb(self, msg: JointState):
        """
        接收一般行走指令：位置 + 速度
        """
        for i, name in enumerate(msg.name):
            try:
                mid = int(name)
                target_pos = int(msg.position[i])
                
                target_vel = 0
                if len(msg.velocity) > i:
                    target_vel = int(msg.velocity[i])

                # 不管有沒有連線，只要在我們預設的名單內就收
                if mid in self.id_port_map:
                    self.joint_data[mid]['goal'] = target_pos
                    self.joint_data[mid]['velocity'] = target_vel
            except ValueError:
                pass

    def _head_cb(self, msg: HeadPackage):
        """
        [新增 4] 接收網頁頭部指令
        """
        # 1. 查找對應的真實 ID
        real_id = self.head_map.get(msg.id)
        
        # 2. 驗證該 ID 是否真的連上線
        if real_id and real_id in self.id_port_map:
            
            try:
                self.get_logger().info(f"收到指令! ID:{real_id} 目標:{msg.position} 速度:{msg.speed}")
                # 3. 更新記憶體資料 (下一次 Timer loop 就會寫入馬達)
                self.joint_data[real_id]['goal'] = int(msg.position)
                self.joint_data[real_id]['velocity'] = int(msg.speed)
                
                # [新增] 處理加速度
                if hasattr(msg, 'acceleration'):
                    self.joint_data[real_id]['accel'] = int(msg.acceleration)

            except ValueError:
                self.get_logger().warn("頭部指令數值轉換失敗")
        else:
            # 這是正常的，如果沒插馬達或者 ID 對應錯了就不會動
            pass

    def _torque_cb(self, msg: Int16MultiArray):
        """
        扭力控制 (盲送版)
        """
        if len(msg.data) < 2: return
        target_id = msg.data[0]
        state = msg.data[1]
        
        ids_to_process = []
        if target_id == 0:
            ids_to_process = self.target_ids # 直接用全體名單
            self.get_logger().info(f"盲送全體扭力: State={state}")
        else:
            ids_to_process = [target_id]
            self.get_logger().info(f"盲送扭力: ID={target_id}, State={state}")

        # [Soft Start] 嘗試同步目標 (如果有讀到值的話)
        if state == 1:
            for mid in ids_to_process:
                # 如果之前有成功讀到 present，就用 present；否則維持預設
                current = self.joint_data[mid]['present']
                self.joint_data[mid]['goal'] = current

        # 強制寫入
        for mid in ids_to_process:
            if mid in self.id_port_map:
                ph = self.id_port_map[mid][2] # 索引調整
                pk = PacketHandler(PROTOCOL_VERSION)
                
                # 不檢查 Ping，直接送 3 次
                for _ in range(3):
                    try:
                        pk.write1ByteTxRx(ph, mid, ADDR_TORQUE_ENABLE, state)
                        # 稍微 sleep 避免塞爆 bus
                        time.sleep(0.001) 
                    except: pass

    def _control_loop(self):
        """
        [精準同步版]
        1. 保持 Velocity-based 邏輯，讓你的速度指令有效。
        2. 確保寫入順序與時序一致。
        """
        # --- Step 1: 寫入指令 (最優先) ---
        for gw in self.group_writers_all: 
            gw.clearParam()
        
        dirty = False
        for mid, (gw_all, gr, ph) in self.id_port_map.items():
            goal = self.joint_data[mid]['goal']
            vel = self.joint_data[mid]['velocity']
            acc = self.joint_data[mid]['accel']
            
            if goal is None: continue
            
            # 限制範圍
            goal = max(0, min(4096, goal))
            vel = max(0, min(1023, vel))
            # 建議：若要同步，acc 可嘗試給 0 或較大數值(如 500 以上)
            acc = max(0, min(32767, acc))
            
            param_all = int(acc).to_bytes(4, 'little', signed=True) + \
                        int(vel).to_bytes(4, 'little', signed=True) + \
                        int(goal).to_bytes(4, 'little', signed=True)
            
            gw_all.addParam(mid, param_all)
            dirty = True

        if dirty:
            # 這裡是一次性發送給該 Port 上的所有馬達
            for gw in self.group_writers_all: 
                gw.txPacket()

        # --- Step 2: 讀取狀態 (放在發送之後，避免影響運動起始時機) ---
        for gr in self.group_readers:
            # txRxPacket 是最耗時的動作
            gr.txRxPacket()
            for mid, vals in self.id_port_map.items():
                if vals[1] == gr: 
                    if gr.isAvailable(mid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                        val = gr.getData(mid, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                        if val > 0x7FFFFFFF: val -= 4294967296
                        self.joint_data[mid]['present'] = val
                        
                        if self.joint_data[mid]['goal'] is None:
                            self.joint_data[mid]['goal'] = val

        # --- Step 3: Publish ---
        pub_msg = JointState()
        pub_msg.header.stamp = self.get_clock().now().to_msg()
        ids = sorted(self.joint_data.keys())
        pub_msg.name = [str(i) for i in ids]
        pub_msg.position = [float(self.joint_data[i]['present']) if self.joint_data[i]['present'] is not None else 0.0 for i in ids]
        self.joint_pub.publish(pub_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # 只有在 ROS 還活著的時候才關機
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()