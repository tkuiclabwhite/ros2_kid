# #!/usr/bin/env python3
# # coding=utf-8
# # [新版策略] HuroCup Marathon - Pro League
# # 架構：底層永遠前進 + 上層事件驅動決策
# # 視覺：AprilTag（主）+ YOLO 箭頭（備援），由 navigation_node 處理後發布
# # 作者：依據 2026 規則重寫

# import rclpy
# import time
# from strategy.API import API
# from tku_msgs.msg import Dio
# from std_msgs.msg import String
# from rclpy.qos import QoSProfile, ReliabilityPolicy
# from tku_msgs.msg import SensorPackage

# # ═══════════════════════════════════════════════
# #  參數設定區（實測後調整）
# # ═══════════════════════════════════════════════

# # 頭部俯角（看地板標記）
# HEAD_DOWN_X = 2048   # 水平置中
# HEAD_DOWN_Y = 1700   # 俯角位置（看地板）

# # 行走速度
# WALK_SPEED_NORMAL  = 2000   # 正常前進速度
# WALK_SPEED_SLOW    = 1000   # 接近標記 / 搜尋時的慢速
# WALK_SPEED_LEAVE   = 2500   # 離開路口速度

# # 畫面參數
# FRAME_W       = 640
# FRAME_H       = 320
# FRAME_CX      = FRAME_W // 2   # 320
# ALIGN_DEADBAND = 40            # X 軸對準死區（像素）

# # 標記 Y 座標閾值（畫面高度 320，值越大越靠近畫面底部）
# SIGN_Y_THRESHOLD = 250         # 標記 Y 座標超過此值視為「進入下方」

# # 接近直走時間（標記進入畫面下方後，繼續直走多久再執行動作）
# APPROACH_WALK_TIME = 1.0       # 秒（實測後調整）

# # 連續確認幀數（防誤判）
# CONFIRM_THRESHOLD = 8

# # 動作參數
# TURN_TARGET_ANGLE = 85      # 轉彎目標角度（度）
# TURN_SPEED        = 8       # 轉彎角速度
# GO_STRAIGHT_TIME  = 3.0     # 直走穿越時間（秒）
# LEAVE_TIME        = 2.0     # 離開路口時間（秒）

# # 搜尋參數
# LOST_WAIT_TIME    = 2.0     # 跟丟後等待時間（秒）才開始慢速前進

# # IMU 補正參數
# IMU_CORRECT_DEADBAND = 3    # yaw 在此範圍內不補正（度）
# IMU_CORRECT_SPEED    = 3    # IMU 補正角速度

# # ═══════════════════════════════════════════════
# #  子狀態定義
# # ═══════════════════════════════════════════════
# # WALK    : 底層持續前進，同時掃描標記
# # ALIGN   : 看到標記，調整方向對準標記中心
# # CONFIRM : 面積夠大，停下來連續確認動作類型
# # ACTION  : 執行轉彎或直走
# # LEAVE   : 動作完成，走出路口區域


# class Mar(API):
#     def __init__(self):
#         super().__init__('mar_strategy')

#         # === 覆寫 IMU 訂閱為 Best Effort ===
#         if self.imu_sub:
#             self.destroy_subscription(self.imu_sub)
#         qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
#         self.imu_sub = self.create_subscription(
#             SensorPackage,
#             '/package/sensorpackage',
#             self.imu,
#             qos_be,
#             callback_group=self.imu_cbg
#         )

#         # === 狀態 ===
#         self.is_start       = True
#         self.initialized    = False
#         self.sub_state      = 'WALK'

#         # === 視覺資料 ===
#         self.sign_label     = 'none'
#         self.sign_cx        = FRAME_CX
#         self.sign_cy        = FRAME_H // 2
#         self.sign_area      = 0.0
#         self.last_sign_time = 0.0

#         # === 確認機制 ===
#         self.confirm_count  = 0
#         self.confirm_label  = 'none'

#         # === 動作計時 ===
#         self.action_start_time = 0.0
#         self.target_label      = 'none'
#         self.lost_start_time   = 0.0    # 第一次跟丟的時間點

#         # === 訂閱 navigation_node 發布的結果 ===
#         self.yolo_sub = self.create_subscription(
#             String, 'class_id_topic', self._sign_callback, 10
#         )

#         # === 主迴圈 20Hz ===
#         self.create_timer(0.05, self._main_loop)

#         self.get_logger().info("=== Mar Strategy Initialized (Event-Driven Mode) ===")

#     # ───────────────────────────────────────────
#     #  不使用 API 的參數同步（覆寫避免衝突）
#     # ───────────────────────────────────────────
#     def _sync_start_from_param(self): pass

#     def _dio_callback(self, msg: Dio):
#         """同步硬體指撥開關狀態到 is_start"""
#         # 同步 DIOValue（保留原本行為）
#         if hasattr(msg, 'data'):    self.DIOValue = msg.data
#         elif hasattr(msg, 'value'): self.DIOValue = msg.value

#         # 同步指撥開關 -> is_start
#         if hasattr(msg, 'strategy') and msg.strategy != self.is_start:
#             self.is_start = msg.strategy
#             if self.is_start:
#                 self.get_logger().info("\033[92m[DIO] 指撥開關開啟 -> 策略啟動\033[0m")
#             else:
#                 self.get_logger().info("\033[91m[DIO] 指撥開關關閉 -> 策略停止\033[0m")
#                 # 立刻停止機器人
#                 self.sendContinuousValue(0, 0, 0)

#     # ───────────────────────────────────────────
#     #  視覺 callback
#     # ───────────────────────────────────────────
#     def _sign_callback(self, msg: String):
#         """接收 navigation_node 發布的標記資訊"""
#         try:
#             parts = msg.data.split(',')
#             if len(parts) == 4:
#                 self.sign_label = parts[0].lower()
#                 self.sign_cx    = int(parts[1])
#                 self.sign_cy    = int(parts[2])
#                 self.sign_area  = float(parts[3])
#                 self.last_sign_time = time.time()
#         except Exception as e:
#             self.get_logger().error(f"Sign parse error: {e}")

#     def _sign_visible(self):
#         """標記是否在有效時間內（0.5 秒內有新資料）"""
#         return (time.time() - self.last_sign_time) < 0.5

#     # ───────────────────────────────────────────
#     #  頭部控制
#     # ───────────────────────────────────────────
#     def _look_down(self):
#         """頭部俯角看地板"""
#         self.sendHeadMotor(1, HEAD_DOWN_X, 50)
#         self.sendHeadMotor(2, HEAD_DOWN_Y, 50)

#     # ───────────────────────────────────────────
#     #  IMU 補正計算
#     # ───────────────────────────────────────────
#     def _imu_correction(self):
#         """根據 yaw 計算補正角速度（假設已 reset，基準為 0）"""
#         yaw = self.imu_rpy[2]
#         if yaw > IMU_CORRECT_DEADBAND:
#             return -IMU_CORRECT_SPEED
#         elif yaw < -IMU_CORRECT_DEADBAND:
#             return IMU_CORRECT_SPEED
#         return 0

#     # ───────────────────────────────────────────
#     #  初始化
#     # ───────────────────────────────────────────
#     def _initialize(self):
#         self.sendSensorReset(True)
#         self._look_down()
#         self.sub_state      = 'WALK'
#         self.confirm_count  = 0
#         self.confirm_label  = 'none'
#         self.target_label   = 'none'
#         self.last_sign_time = time.time()
#         self.lost_start_time = 0.0
#         self.get_logger().info("[Init] 完成，進入 WALK 狀態")

#     # ───────────────────────────────────────────
#     #  狀態：WALK（底層持續前進 + 掃描標記）
#     # ───────────────────────────────────────────
#     def _state_walk(self):
#         """
#         核心設計：機器人永遠在走，不等待。
#         - 看到標記 → 進入 ALIGN
#         - 跟丟超過 LOST_WAIT_TIME → 慢速前進繼續找
#         - 跟丟未超時 → 正常速度前進（標記可能就在前方）
#         """
#         self._look_down()

#         if self._sign_visible():
#             # 看到標記 -> 進入對準
#             self.lost_start_time = 0.0
#             self.get_logger().info(
#                 f"[WALK] 發現標記 {self.sign_label} area={self.sign_area:.0f} -> ALIGN",
#                 throttle_duration_sec=1.0
#             )
#             self.sub_state = 'ALIGN'
#             return

#         # 沒看到標記
#         now = time.time()
#         if self.lost_start_time == 0.0:
#             self.lost_start_time = now

#         lost_duration = now - self.lost_start_time

#         if lost_duration < LOST_WAIT_TIME:
#             # 剛跟丟，正常速度前進（標記可能快到了）
#             correction = self._imu_correction()
#             self.sendContinuousValue(WALK_SPEED_NORMAL, 0, correction)
#             self.get_logger().info(
#                 f"[WALK] 搜尋中... ({lost_duration:.1f}s) yaw={self.imu_rpy[2]:.1f}",
#                 throttle_duration_sec=1.0
#             )
#         else:
#             # 超過等待時間，改慢速確保不跨過標記
#             correction = self._imu_correction()
#             self.sendContinuousValue(WALK_SPEED_SLOW, 0, correction)
#             self.get_logger().info(
#                 "[WALK] 慢速搜尋中...",
#                 throttle_duration_sec=1.0
#             )

#     # ───────────────────────────────────────────
#     #  狀態：ALIGN（對準標記中心 + 邊走邊靠近）
#     # ───────────────────────────────────────────
#     def _state_align(self):
#         """
#         看到標記後，用畫面 X 軸位置修正航向，
#         同時慢速前進接近標記。
#         面積夠大後進入 CONFIRM。
#         """
#         self._look_down()

#         if not self._sign_visible():
#             # 標記消失，退回 WALK
#             self.get_logger().info("[ALIGN] 標記消失 -> WALK")
#             self.sub_state = 'WALK'
#             self.lost_start_time = time.time()
#             return

#         # 根據標記 X 位置計算修正量
#         x_error = self.sign_cx - FRAME_CX
#         if x_error < -ALIGN_DEADBAND:
#             theta = IMU_CORRECT_SPEED       # 標記偏左，向左轉
#         elif x_error > ALIGN_DEADBAND:
#             theta = -IMU_CORRECT_SPEED      # 標記偏右，向右轉
#         else:
#             theta = self._imu_correction()  # 對齊後用 IMU 維持直走

#         self.sendContinuousValue(WALK_SPEED_SLOW, 0, theta)

#         self.get_logger().info(
#             f"[ALIGN] label={self.sign_label} cx={self.sign_cx} cy={self.sign_cy} theta={theta}",
#             throttle_duration_sec=0.5
#         )

#         # 標記 Y 座標進入畫面下方 -> 進入確認
#         if self.sign_cy >= SIGN_Y_THRESHOLD:
#             self.sub_state = 'CONFIRM'
#             self.get_logger().info(f"[ALIGN] 標記進入畫面下方 cy={self.sign_cy} -> CONFIRM")

#     # ───────────────────────────────────────────
#     #  狀態：CONFIRM（確認 label + 繼續直走接近）
#     # ───────────────────────────────────────────
#     def _state_confirm(self):
#         """
#         標記進入畫面下方後：
#         1. 先連續確認 label（防誤判）
#         2. 確認完成後繼續直走 APPROACH_WALK_TIME 秒
#         3. 時間到才進入 ACTION 執行轉向
#         """
#         self._look_down()

#         if not self._sign_visible():
#             self.get_logger().warn("[CONFIRM] 標記消失，重置確認計數")
#             self.confirm_count = 0
#             self.confirm_label = 'none'
#             self.sub_state = 'WALK'
#             return

#         current = self.sign_label

#         # --- 階段一：連續確認 label ---
#         if self.confirm_count < CONFIRM_THRESHOLD:
#             if current == self.confirm_label:
#                 self.confirm_count += 1
#             else:
#                 self.confirm_count = 1
#                 self.confirm_label = current

#             correction = self._imu_correction()
#             self.sendContinuousValue(WALK_SPEED_SLOW, 0, correction)
#             self.get_logger().info(
#                 f"[CONFIRM] 確認中 {current} {self.confirm_count}/{CONFIRM_THRESHOLD} cy={self.sign_cy}",
#                 throttle_duration_sec=0.3
#             )

#         # --- 階段二：確認完成，繼續直走 APPROACH_WALK_TIME 秒 ---
#         else:
#             if self.target_label == 'none':
#                 # 剛確認完成，記錄 label 並開始計時
#                 self.target_label = self.confirm_label
#                 self.action_start_time = time.time()
#                 self.get_logger().info(
#                     f"[CONFIRM] ✅ label={self.target_label} -> 直走接近 {APPROACH_WALK_TIME}s"
#                 )

#             elapsed = time.time() - self.action_start_time
#             if elapsed < APPROACH_WALK_TIME:
#                 correction = self._imu_correction()
#                 self.sendContinuousValue(WALK_SPEED_SLOW, 0, correction)
#                 self.get_logger().info(
#                     f"[CONFIRM] 直走接近中 {elapsed:.1f}s/{APPROACH_WALK_TIME}s",
#                     throttle_duration_sec=0.3
#                 )
#             else:
#                 # 直走完成 -> 進入 ACTION
#                 self.get_logger().info(f"[CONFIRM] 直走完成 -> ACTION label={self.target_label}")
#                 self.sendContinuousValue(0, 0, 0)
#                 self.confirm_count = 0
#                 self.confirm_label = 'none'
#                 self.sendSensorReset(True)      # IMU 歸零，作為轉向基準
#                 self.action_start_time = time.time()
#                 self.sub_state = 'ACTION'

#     # ───────────────────────────────────────────
#     #  狀態：ACTION（執行轉彎或直走）
#     # ───────────────────────────────────────────
#     def _state_action(self):
#         """
#         根據 target_label 執行對應動作：
#         - left / right：IMU 控制轉到目標角度
#         - forward      ：IMU 輔助直走穿越
#         """
#         label = self.target_label

#         if label in ('left', 'right'):
#             target_yaw = TURN_TARGET_ANGLE if label == 'left' else -TURN_TARGET_ANGLE
#             current_yaw = self.imu_rpy[2]

#             done = (label == 'left'  and current_yaw >= target_yaw) or \
#                    (label == 'right' and current_yaw <= target_yaw)

#             if done:
#                 self.get_logger().info(
#                     f"[ACTION] {label} 轉彎完成 yaw={current_yaw:.1f}"
#                 )
#                 self.sendContinuousValue(0, 0, 0)
#                 time.sleep(0.1)
#                 self.sendSensorReset(True)
#                 self.action_start_time = time.time()
#                 self.sub_state = 'LEAVE'
#             else:
#                 turn_speed = TURN_SPEED if label == 'left' else -TURN_SPEED
#                 self.sendContinuousValue(0, 0, turn_speed)
#                 self.get_logger().info(
#                     f"[ACTION] 轉彎中 yaw={current_yaw:.1f} / 目標={target_yaw}",
#                     throttle_duration_sec=0.5
#                 )

#         elif label in ('forward', 'straight'):
#             elapsed = time.time() - self.action_start_time
#             if elapsed < GO_STRAIGHT_TIME:
#                 correction = self._imu_correction()
#                 self.sendContinuousValue(WALK_SPEED_NORMAL, 0, correction)
#                 self.get_logger().info(
#                     f"[ACTION] 直走穿越中 {elapsed:.1f}s/{GO_STRAIGHT_TIME}s yaw={self.imu_rpy[2]:.1f}",
#                     throttle_duration_sec=0.5
#                 )
#             else:
#                 self.get_logger().info("[ACTION] 直走完成")
#                 self.sendSensorReset(True)
#                 self.action_start_time = time.time()
#                 self.sub_state = 'LEAVE'

#         else:
#             # 未知 label，退回 WALK
#             self.get_logger().warn(f"[ACTION] 未知 label={label}，退回 WALK")
#             self.sub_state = 'WALK'

#     # ───────────────────────────────────────────
#     #  狀態：LEAVE（走出路口，回到 WALK）
#     # ───────────────────────────────────────────
#     def _state_leave(self):
#         """
#         動作完成後直走 LEAVE_TIME 秒，
#         確保離開標記偵測範圍再重新搜尋，
#         避免剛轉完又馬上看到同一個標記。
#         """
#         elapsed = time.time() - self.action_start_time

#         if elapsed < LEAVE_TIME:
#             correction = self._imu_correction()
#             self.sendContinuousValue(WALK_SPEED_LEAVE, 0, correction)
#             self.get_logger().info(
#                 f"[LEAVE] 離開路口中 {elapsed:.1f}s/{LEAVE_TIME}s",
#                 throttle_duration_sec=0.5
#             )
#         else:
#             self.get_logger().info("[LEAVE] 完成 -> WALK")
#             self._initialize()  # reset IMU + 頭部 + 回到 WALK

#     # ───────────────────────────────────────────
#     #  主迴圈
#     # ───────────────────────────────────────────
#     def _main_loop(self):
#         if not self.is_start:
#             return

#         # 第一次執行初始化
#         if not self.initialized:
#             self._initialize()
#             self.sendbodyAuto(1)
#             self.initialized = True
#             self.get_logger().info("=== 比賽開始 ===")
#             return

#         # 狀態分派
#         if   self.sub_state == 'WALK':    self._state_walk()
#         elif self.sub_state == 'ALIGN':   self._state_align()
#         elif self.sub_state == 'CONFIRM': self._state_confirm()
#         elif self.sub_state == 'ACTION':  self._state_action()
#         elif self.sub_state == 'LEAVE':   self._state_leave()
#         else:
#             self.get_logger().warn(f"未知狀態 {self.sub_state}，退回 WALK")
#             self.sub_state = 'WALK'


# def main(args=None):
#     rclpy.init(args=args)
#     node = Mar()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
#!/usr/bin/env python3
# coding=utf-8
# [新版策略] HuroCup Marathon - Pro League
# 架構：底層永遠前進 + 上層事件驅動決策
# 視覺：AprilTag（主）+ YOLO 箭頭（備援），由 navigation_node 處理後發布
# 作者：依據 2026 規則重寫

import rclpy
import time
from strategy.API import API
from tku_msgs.msg import Dio
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tku_msgs.msg import SensorPackage

# ═══════════════════════════════════════════════
#  參數設定區（實測後調整）
# ═══════════════════════════════════════════════

# 頭部俯角（看地板標記）
HEAD_DOWN_X = 2048   # 水平置中
HEAD_DOWN_Y = 1600   # 俯角位置（看地板）

# 行走速度
WALK_SPEED_NORMAL  = 1000   # 正常前進速度
WALK_SPEED_SLOW    = 500   # 接近標記 / 搜尋時的慢速
WALK_SPEED_LEAVE   = 1500   # 離開路口速度

# 畫面參數
FRAME_W       = 320
FRAME_H       = 240
FRAME_CX      = FRAME_W // 2   # 320
ALIGN_DEADBAND = 40            # X 軸對準死區（像素）

# 標記 Y 座標閾值（畫面高度 320，值越大越靠近畫面底部）
SIGN_Y_THRESHOLD = 190         # 標記 Y 座標超過此值視為「進入下方」

# 接近直走時間（標記進入畫面下方後，繼續直走多久再執行動作）
APPROACH_WALK_TIME = 2.0       # 秒（實測後調整）

# 動作參數
TURN_TARGET_ANGLE = 85      # 轉彎目標角度（度）
TURN_SPEED        = 8       # 轉彎角速度
GO_STRAIGHT_TIME  = 3.0     # 直走穿越時間（秒）
LEAVE_TIME        = 2.0     # 離開路口時間（秒）

# 搜尋參數
LOST_WAIT_TIME    = 2.0     # 跟丟後等待時間（秒）才開始慢速前進

# IMU 補正參數
IMU_CORRECT_DEADBAND = 3    # yaw 在此範圍內不補正（度）
IMU_CORRECT_SPEED    = 3    # IMU 補正角速度

# ═══════════════════════════════════════════════
#  子狀態定義
# ═══════════════════════════════════════════════
# WALK   : 底層持續前進，同時掃描標記
# ALIGN  : 看到標記，對準中心 + 慢速靠近
#          標記 Y 座標進入畫面下方後計時直走
#          時間到直接進入 ACTION
# ACTION : 收到 left/right 直接轉彎；straight 直走穿越
# LEAVE  : 動作完成，走出路口區域


class Mar(API):
    def __init__(self):
        super().__init__('mar_strategy')

        # === 覆寫 IMU 訂閱為 Best Effort ===
        if self.imu_sub:
            self.destroy_subscription(self.imu_sub)
        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.imu_sub = self.create_subscription(
            SensorPackage,
            '/package/sensorpackage',
            self.imu,
            qos_be,
            callback_group=self.imu_cbg
        )

        # === 狀態 ===
        self.is_start       = True
        self.initialized    = False
        self.sub_state      = 'WALK'

        # === 視覺資料 ===
        self.sign_label     = 'none'
        self.sign_cx        = FRAME_CX
        self.sign_cy        = FRAME_H // 2
        self.sign_area      = 0.0
        self.last_sign_time = 0.0

        # === 動作計時 ===
        self.action_start_time = 0.0
        self.target_label      = 'none'
        self.lost_start_time   = 0.0    # 第一次跟丟的時間點

        # === 訂閱 navigation_node 發布的結果 ===
        self.yolo_sub = self.create_subscription(
            String, 'class_id_topic', self._sign_callback, 10
        )

        # === 主迴圈 20Hz ===
        self.create_timer(0.05, self._main_loop)

        self.get_logger().info("=== Mar Strategy Initialized (Event-Driven Mode) ===")

    # ───────────────────────────────────────────
    #  不使用 API 的參數同步（覆寫避免衝突）
    # ───────────────────────────────────────────
    def _sync_start_from_param(self): pass

    def _dio_callback(self, msg: Dio):
        """同步硬體指撥開關狀態到 is_start"""
        # 同步 DIOValue（保留原本行為）
        if hasattr(msg, 'data'):    self.DIOValue = msg.data
        elif hasattr(msg, 'value'): self.DIOValue = msg.value

        # 同步指撥開關 -> is_start
        if hasattr(msg, 'strategy') and msg.strategy != self.is_start:
            self.is_start = msg.strategy
            if self.is_start:
                self.get_logger().info("\033[92m[DIO] 指撥開關開啟 -> 策略啟動\033[0m")
            else:
                self.get_logger().info("\033[91m[DIO] 指撥開關關閉 -> 策略停止\033[0m")
                # 立刻停止機器人
                self.sendContinuousValue(0, 0, 0)

    # ───────────────────────────────────────────
    #  視覺 callback
    # ───────────────────────────────────────────
    def _sign_callback(self, msg: String):
        """接收 navigation_node 發布的標記資訊"""
        try:
            parts = msg.data.split(',')
            if len(parts) == 4:
                self.sign_label = parts[0].lower()
                self.sign_cx    = int(parts[1])
                self.sign_cy    = int(parts[2])
                self.sign_area  = float(parts[3])
                self.last_sign_time = time.time()
        except Exception as e:
            self.get_logger().error(f"Sign parse error: {e}")

    def _sign_visible(self):
        """標記是否在有效時間內（0.5 秒內有新資料）"""
        return (time.time() - self.last_sign_time) < 1.0 ###!!!

    # ───────────────────────────────────────────
    #  頭部控制
    # ───────────────────────────────────────────
    def _look_down(self):
        """頭部俯角看地板"""
        self.sendHeadMotor(1, HEAD_DOWN_X, 50)
        self.sendHeadMotor(2, HEAD_DOWN_Y, 50)

    # ───────────────────────────────────────────
    #  IMU 補正計算
    # ───────────────────────────────────────────
    def _imu_correction(self):
        """根據 yaw 計算補正角速度（假設已 reset，基準為 0）"""
        yaw = self.imu_rpy[2]
        if yaw > IMU_CORRECT_DEADBAND:
            return -IMU_CORRECT_SPEED
        elif yaw < -IMU_CORRECT_DEADBAND:
            return IMU_CORRECT_SPEED
        return 0

    # ───────────────────────────────────────────
    #  初始化
    # ───────────────────────────────────────────
    def _initialize(self):
        self.sendSensorReset(True)
        self._look_down()
        self.sub_state      = 'WALK'
        self.target_label   = 'none'
        self.last_sign_time = time.time()
        self.lost_start_time = 0.0
        self.get_logger().info("[Init] 完成，進入 WALK 狀態")

    # ───────────────────────────────────────────
    #  狀態：WALK（底層持續前進 + 掃描標記）
    # ───────────────────────────────────────────
    def _state_walk(self):
        """
        核心設計：機器人永遠在走，不等待。
        - 看到標記 → 進入 ALIGN
        - 跟丟超過 LOST_WAIT_TIME → 慢速前進繼續找
        - 跟丟未超時 → 正常速度前進（標記可能就在前方）
        """
        self._look_down()

        if self._sign_visible():
            # 看到標記 -> 進入對準
            self.lost_start_time = 0.0
            self.get_logger().info(
                f"[WALK] 發現標記 {self.sign_label} area={self.sign_area:.0f} -> ALIGN",
                throttle_duration_sec=1.0
            )
            self.sub_state = 'ALIGN'
            return

        # 沒看到標記
        now = time.time()
        if self.lost_start_time == 0.0:
            self.lost_start_time = now

        lost_duration = now - self.lost_start_time

        if lost_duration < LOST_WAIT_TIME:
            # 剛跟丟，正常速度前進（標記可能快到了）
            correction = self._imu_correction()
            self.sendContinuousValue(WALK_SPEED_NORMAL, 0, correction)
            self.get_logger().info(
                f"[WALK] 搜尋中... ({lost_duration:.1f}s) yaw={self.imu_rpy[2]:.1f}",
                throttle_duration_sec=1.0
            )
        else:
            # 超過等待時間，改慢速確保不跨過標記
            correction = self._imu_correction()
            self.sendContinuousValue(WALK_SPEED_SLOW, 0, correction)
            self.get_logger().info(
                "[WALK] 慢速搜尋中...",
                throttle_duration_sec=1.0
            )

    # ───────────────────────────────────────────
    #  狀態：ALIGN（對準標記中心 + 邊走邊靠近）
    # ───────────────────────────────────────────
    def _state_align(self):
        """
        看到標記後，用畫面 X 軸位置修正航向，同時慢速前進。
        標記 Y 座標進入畫面下方後開始計時直走，
        APPROACH_WALK_TIME 秒後直接進入 ACTION。
        """
        self._look_down()

        if not self._sign_visible():
            self.get_logger().info("[ALIGN] 標記消失 -> WALK")
            self.action_start_time = 0.0    # 重置計時
            self.target_label = 'none'
            self.sub_state = 'WALK'
            self.lost_start_time = time.time()
            return

        # 根據標記 X 位置計算修正量
        x_error = self.sign_cx - FRAME_CX
        if x_error < -ALIGN_DEADBAND:
            theta = IMU_CORRECT_SPEED       # 標記偏左，向左修正
        elif x_error > ALIGN_DEADBAND:
            theta = -IMU_CORRECT_SPEED      # 標記偏右，向右修正
        else:
            theta = self._imu_correction()  # 對齊後用 IMU 維持直走

        # 標記尚未進入畫面下方：持續慢速前進對準
        if self.sign_cy < SIGN_Y_THRESHOLD:
            self.action_start_time = 0.0    # 尚未觸發，保持重置
            self.target_label = 'none'
            self.sendContinuousValue(WALK_SPEED_SLOW, 0, theta)
            self.get_logger().info(
                f"[ALIGN] 靠近中 label={self.sign_label} cx={self.sign_cx} cy={self.sign_cy} theta={theta}",
                throttle_duration_sec=0.5
            )
            return

        # 標記進入畫面下方：記錄 label 並開始計時直走
        if self.action_start_time == 0.0:
            self.target_label = self.sign_label
            self.action_start_time = time.time()
            self.get_logger().info(
                f"[ALIGN] 標記到位 cy={self.sign_cy} label={self.target_label} -> 直走 {APPROACH_WALK_TIME}s"
            )

        elapsed = time.time() - self.action_start_time
        if elapsed < APPROACH_WALK_TIME:
            self.sendContinuousValue(WALK_SPEED_SLOW, 0, theta)
            self.get_logger().info(
                f"[ALIGN] 直走接近中 {elapsed:.1f}s/{APPROACH_WALK_TIME}s",
                throttle_duration_sec=0.3
            )
        else:
            # 直走完成 -> 直接進 ACTION
            self.sendContinuousValue(0, 0, 0)
            self.sendSensorReset(True)
            self.action_start_time = time.time()
            self.get_logger().info(
                f"[ALIGN] 直走完成 -> ACTION label={self.target_label}"
            )
            self.sub_state = 'ACTION'

    # ───────────────────────────────────────────
    #  狀態：ACTION（執行轉彎或直走）
    # ───────────────────────────────────────────
    def _state_action(self):
        """
        根據 target_label 執行對應動作：
        - left / right：IMU 控制轉到目標角度
        - forward      ：IMU 輔助直走穿越
        """
        label = self.target_label

        if label in ('left', 'right'):
            target_yaw = TURN_TARGET_ANGLE if label == 'left' else -TURN_TARGET_ANGLE
            current_yaw = self.imu_rpy[2]

            done = (label == 'left'  and current_yaw >= target_yaw) or \
                   (label == 'right' and current_yaw <= target_yaw)

            if done:
                self.get_logger().info(
                    f"[ACTION] {label} 轉彎完成 yaw={current_yaw:.1f}"
                )
                self.sendContinuousValue(0, 0, 0)
                time.sleep(0.1)
                self.sendSensorReset(True)
                self.action_start_time = time.time()
                self.sub_state = 'LEAVE'
            else:
                turn_speed = TURN_SPEED if label == 'left' else -TURN_SPEED
                self.sendContinuousValue(0, 0, turn_speed)
                self.get_logger().info(
                    f"[ACTION] 轉彎中 yaw={current_yaw:.1f} / 目標={target_yaw}",
                    throttle_duration_sec=0.5
                )

        elif label in ('forward', 'straight'):
            elapsed = time.time() - self.action_start_time
            if elapsed < GO_STRAIGHT_TIME:
                correction = self._imu_correction()
                self.sendContinuousValue(WALK_SPEED_NORMAL, 0, correction)
                self.get_logger().info(
                    f"[ACTION] 直走穿越中 {elapsed:.1f}s/{GO_STRAIGHT_TIME}s yaw={self.imu_rpy[2]:.1f}",
                    throttle_duration_sec=0.5
                )
            else:
                self.get_logger().info("[ACTION] 直走完成")
                self.sendSensorReset(True)
                self.action_start_time = time.time()
                self.sub_state = 'LEAVE'

        else:
            # 未知 label，退回 WALK
            self.get_logger().warn(f"[ACTION] 未知 label={label}，退回 WALK")
            self.sub_state = 'WALK'

    # ───────────────────────────────────────────
    #  狀態：LEAVE（走出路口，回到 WALK）
    # ───────────────────────────────────────────
    def _state_leave(self):
        """
        動作完成後直走 LEAVE_TIME 秒，
        確保離開標記偵測範圍再重新搜尋，
        避免剛轉完又馬上看到同一個標記。
        """
        elapsed = time.time() - self.action_start_time

        if elapsed < LEAVE_TIME:
            correction = self._imu_correction()
            self.sendContinuousValue(WALK_SPEED_LEAVE, 0, correction)
            self.get_logger().info(
                f"[LEAVE] 離開路口中 {elapsed:.1f}s/{LEAVE_TIME}s",
                throttle_duration_sec=0.5
            )
        else:
            self.get_logger().info("[LEAVE] 完成 -> WALK")
            self._initialize()  # reset IMU + 頭部 + 回到 WALK

    # ───────────────────────────────────────────
    #  主迴圈
    # ───────────────────────────────────────────
    def _main_loop(self):
        if not self.is_start:
            return

        # 第一次執行初始化
        if not self.initialized:
            self._initialize()
            self.sendbodyAuto(1)
            self.initialized = True
            self.get_logger().info("=== 比賽開始 ===")
            return

        # 狀態分派
        if   self.sub_state == 'WALK':   self._state_walk()
        elif self.sub_state == 'ALIGN':  self._state_align()
        elif self.sub_state == 'ACTION': self._state_action()
        elif self.sub_state == 'LEAVE':  self._state_leave()
        else:
            self.get_logger().warn(f"未知狀態 {self.sub_state}，退回 WALK")
            self.sub_state = 'WALK'


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