#!/usr/bin/env python3
# coding=utf-8
# [修改版] 適用於箭頭貼在地板上的情境
# 主要改動：
#   1. 頭部固定朝下俯角，不再左右擺頭搜尋
#   2. 沒看到箭頭時改成「慢速前進搜尋」，而非原地擺頭
#   3. 直走策略移除向右平移，改為直接向前走
#   4. APPROACH 邏輯改為「對準箭頭中心 + 走到正上方」

import rclpy
from collections import deque
import time
from strategy.API import API
from tku_msgs.msg import Dio
from std_msgs.msg import String  
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tku_msgs.msg import SensorPackage

# === 全域設定 ===
HEAD_CENTER_X = 2048
HEAD_CENTER_Y = 2048
ORIGIN_THETA = 0

# === [修改] 頭部俯角設定（朝下看地板）===
# 數值需依實際機器人調整，數字越大越往下
HEAD_DOWN_Y = 1500   # 俯角位置（實測值）
HEAD_DOWN_X = 2048   # 水平置中

# === 參數設定區 ===
AREA_THRESHOLD = 1000       # 地板箭頭面積閥值（實測值）
TURN_TARGET_ANGLE = 85      # 轉彎目標角度
GO_STRAIGHT_TIME = 3.0      # 直走模式：直走穿越時間
LEAVE_WALK_TIME = 2.0       # 離開路口時間
LOST_TARGET_TIMEOUT = 3.0   # 沒看到標誌多久後才開始前進搜尋 (秒)
SEARCH_FORWARD_SPEED = 1000 # 搜尋時的前進速度（慢速）

class Coordinate:
    def __init__(self, x, y):
        self.x, self.y = x, y

class Mar(API):
    def __init__(self):

        # 1. 先執行父類別初始化
        super().__init__('mar_strategy')
        
        # === [關鍵修正] 覆寫 IMU 訂閱為 Best Effort ===
        if self.imu_sub:
            self.destroy_subscription(self.imu_sub)
            
        qos_policy = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.imu_sub = self.create_subscription(
            SensorPackage, 
            '/package/sensorpackage', 
            self.imu, 
            qos_policy,
            callback_group=self.imu_cbg
        )
        self.get_logger().info("已在 mar1_floor.py 中強制修正 IMU 訂閱為 Best Effort")
        # ============================================
        
        self.is_start = True
        self.status = 'First'
        self.sub_state = 'SEARCH'
        
        self.arrow_center = Coordinate(0, 0)
        self.current_area = 0
        self.target_label = 'None'
        
        self.reg = 1
        self.action_start_time = 0
        self.last_yolo_time = 0
        self.latest_yolo_data = None 

        self.yolo_sub = self.create_subscription(String, 'class_id_topic', self.yolo_callback, 10)
        self.create_timer(0.05, self.main_loop) 
        
        self.get_logger().info("Mar Strategy Initialized (Floor Arrow Mode)")

    def _sync_start_from_param(self): pass

    def _dio_callback(self, msg: Dio):
        if hasattr(msg, 'data'): self.DIOValue = msg.data
        elif hasattr(msg, 'value'): self.DIOValue = msg.value

    def yolo_callback(self, msg):
        try:
            parts = msg.data.split(',')
            if len(parts) == 4:
                self.latest_yolo_data = {
                    'label': parts[0],
                    'cx': int(parts[1]),
                    'cy': int(parts[2]),
                    'area': float(parts[3])
                }
                self.last_yolo_time = time.time() 
        except Exception as e:
            self.get_logger().error(f"YOLO Parse Error: {e}")

    def get_arrow_info(self):
        if time.time() - self.last_yolo_time > 0.5 or self.latest_yolo_data is None:
            return 'None', 0, 0, 0
        d = self.latest_yolo_data
        return d['label'], d['cx'], d['cy'], d['area']

    def initial(self):
        self.sendSensorReset(True) 
        # [修改] 重置時頭部朝下（看地板），不再回正前方
        self._look_down()
        self.get_logger().info("SET initial (Floor Mode)")
        self.sub_state = 'SEARCH'

    def _look_down(self):
        """將頭部轉到俯角（看地板）"""
        self.sendHeadMotor(1, HEAD_DOWN_X, 50)
        self.sendHeadMotor(2, HEAD_DOWN_Y, 50)

    # === [大改] 地板搜尋模式：沒看到就慢速前進 ===
    def view_search(self):
        label, cx, cy, area = self.get_arrow_info()
        
        # 頭部持續朝下
        self._look_down()
        
        if label != 'None':
            self.target_label = label
            self.arrow_center.x = cx
            self.arrow_center.y = cy
            self.current_area = area
            self.get_logger().info(f"[Search] 找到地板箭頭: {label} -> 開始接近")
            self.sub_state = 'APPROACH'
            return

        # 剛跟丟 -> 原地等待一下
        time_since_lost = time.time() - self.last_yolo_time
        if time_since_lost < LOST_TARGET_TIMEOUT:
            self.sendContinuousValue(0, 0, 0)
            self.get_logger().info(
                f"等待箭頭出現... ({time_since_lost:.1f}s / {LOST_TARGET_TIMEOUT}s)",
                throttle_duration_sec=1.0
            )
            return

        # 超過等待時間 -> 慢速前進繼續找（地板箭頭不需要擺頭，向前走就會看到）
        self.get_logger().info("未發現箭頭 -> 慢速前進搜尋", throttle_duration_sec=1.0)
        self.sendContinuousValue(SEARCH_FORWARD_SPEED, 0, 0)

    # === [修改] 地板接近邏輯：用畫面 X 軸對準，走到箭頭正上方 ===
    def body_align_and_approach(self):
        # 頭保持朝下
        self._look_down()
        
        # 畫面中心：640x320 解析度，X 中心為 320
        center_x = 320
        deadband = 40
        theta = 0

        # 根據箭頭在畫面中的 X 位置修正偏航
        if self.arrow_center.x < (center_x - deadband):
            theta = 3   # 箭頭偏左 -> 機器人向左修正
        elif self.arrow_center.x > (center_x + deadband):
            theta = -3  # 箭頭偏右 -> 機器人向右修正
        else:
            theta = 0
            # 對齊後用 IMU 微調直行
            if self.yaw > 5: theta = -2
            elif self.yaw < -5: theta = 2

        self.sendContinuousValue(2000, 0, theta + ORIGIN_THETA)

    # === [修改] 執行動作邏輯（直走移除平移，修正 IMU 持續轉圈問題）===
    def execute_action_logic(self):
        label = self.target_label
        
        if label in ['left', 'right']:
            # [修正] 使用相對角度：記錄進入 ACTION 時的 yaw，轉到差 85 度就停
            # sendSensorReset(True) 會把 imu_rpy[2] 歸零，所以直接用絕對值判斷即可
            target_yaw = TURN_TARGET_ANGLE if label == 'left' else -TURN_TARGET_ANGLE
            current_yaw = self.imu_rpy[2]
            
            is_finish = False
            if label == 'left' and current_yaw >= target_yaw: is_finish = True
            if label == 'right' and current_yaw <= target_yaw: is_finish = True
            
            if is_finish:
                self.get_logger().info(f"[Action] {label} 轉彎完成 Yaw:{current_yaw:.1f}")
                self.sendContinuousValue(0, 0, 0)  # [修正] 先停止，再 reset IMU
                time.sleep(0.1)
                self.sendSensorReset(True)          # reset 後 yaw 歸零，不會繼續轉
                self.action_start_time = time.time()
                self.sub_state = 'LEAVE'
            else:
                turn_speed = 8 if label == 'left' else -8
                self.sendContinuousValue(0, 0, turn_speed)
                self.get_logger().info(f"轉彎中 Yaw:{current_yaw:.1f} / 目標:{target_yaw}", throttle_duration_sec=0.5)

        elif label in ['straight', 'stright']:
            # [修改] 地板直走：不需要向右平移，直接向前穿越
            elapsed = time.time() - self.action_start_time
            if elapsed < GO_STRAIGHT_TIME:
                self.sendContinuousValue(2500, 0, 0)
                self.get_logger().info("直走穿越中", throttle_duration_sec=0.5)
            else:
                self.get_logger().info("直走策略完成")
                self.sendSensorReset(True)
                self._look_down()  # 頭保持看地板
                self.sub_state = 'SEARCH'

    # === 主邏輯 ===
    def main_loop(self):
        if not self.is_start:
            return

        if self.status == 'First':
            self.initial()
            self.sendbodyAuto(1)
            self.status = 'Arrow_Part'
            self.sub_state = 'SEARCH'
            self.last_yolo_time = time.time()
            self.get_logger().info("=== 系統啟動 (地板箭頭模式) ===")

        elif self.status == 'Arrow_Part':
            
            label, cx, cy, area = self.get_arrow_info()
            if label != 'None':
                self.arrow_center.x = cx
                self.arrow_center.y = cy
                self.current_area = area
                if self.sub_state == 'SEARCH' or label == self.target_label:
                    self.target_label = label

            if self.sub_state == 'SEARCH':
                self.view_search()  # [修改] 不再傳入左右角度參數
                
            elif self.sub_state == 'APPROACH':
                if self.current_area > AREA_THRESHOLD:
                    self.get_logger().info(f"到達箭頭正上方 Area:{self.current_area:.0f}")
                    self.sendContinuousValue(0, 0, 0)
                    self.sendSensorReset(True) 
                    self.action_start_time = time.time()
                    self.sub_state = 'ACTION'
                else:
                    self.body_align_and_approach()
                    self.get_logger().info(f"接近中 | Area:{self.current_area:.0f}", throttle_duration_sec=0.5)

            elif self.sub_state == 'ACTION':
                self.execute_action_logic()

            elif self.sub_state == 'LEAVE':
                elapsed = time.time() - self.action_start_time
                if elapsed < LEAVE_WALK_TIME:
                    self.sendContinuousValue(2500, 0, 0)
                    self.get_logger().info("離開路口中", throttle_duration_sec=0.5)
                else:
                    self.get_logger().info("離開完成 -> 重新搜尋")
                    self.initial()

def main(args=None):
    rclpy.init(args=args)
    node = Mar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()