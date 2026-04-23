#!/usr/bin/env python3
# coding=utf-8
# [新版策略] HuroCup Marathon - Pro League
# 架構：底層永遠前進 + 上層事件驅動決策
# 視覺：AprilTag（主）+ YOLO 箭頭（備援），由 navigation_node 處理後發布
# 作者：依據 2026 規則重寫

import rclpy
import time
from strategy.API import API
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tku_msgs.msg import SensorPackage

# ═══════════════════════════════════════════════
#  參數設定區（實測後調整）
# ═══════════════════════════════════════════════

# 頭部俯角（看地板標記）
HEAD_DOWN_X = 2048   # 水平置中
HEAD_DOWN_Y = 1500   # 俯角位置（看地板）

# 行走速度
WALK_SPEED_NORMAL  = 2000   # 正常前進速度
WALK_SPEED_SLOW    = 1500   # 接近標記 / 搜尋時的慢速

# 轉彎參數（左右分開，邊走邊轉）
LEFT_FORWARD_SPEED  = 1600   # 左轉時的前進速度
LEFT_TURN_SPEED     = 8     # 左轉時的旋轉速度（正值）
RIGHT_FORWARD_SPEED = 1500   # 右轉時的前進速度
RIGHT_TURN_SPEED    = -8    # 右轉時的旋轉速度（負值）

# 畫面參數
FRAME_W       = 320
FRAME_H       = 240
FRAME_CX      = FRAME_W // 2   # 160
ALIGN_DEADBAND = 40            # X 軸對準死區（像素）

# 標記 Y 座標閾值（畫面高度 240，Y 越大越靠近底部）
SIGN_Y_THRESHOLD = 215         # 標記 Y 座標超過此值視為「夠靠近」

# 靠近累積幀數（Y 座標連續超過閾值幾幀才觸發）
APPROACH_COUNT = 20            # 累積幀數（實測後調整）

# 動作參數
TURN_TARGET_ANGLE = 85         # 轉彎目標角度（度）

# 搜尋參數
LOST_WAIT_TIME    = 2.0        # 跟丟後等待時間（秒）才開始慢速前進
SIGN_LOST_TIMEOUT = 3.0        # ALIGN 狀態下連續幾秒沒資料才真的退回 WALK
INIT_COOLDOWN     = 2.0        # 初始化後冷卻時間（秒），避免殘留發布誤觸發

# IMU 補正參數
IMU_CORRECT_DEADBAND = 3       # yaw 在此範圍內不補正（度）
IMU_CORRECT_SPEED    = 8       # IMU 補正角速度

# ═══════════════════════════════════════════════
#  子狀態定義
# ═══════════════════════════════════════════════
# WALK   : 底層持續前進，同時掃描標記
# ALIGN  : 看到標記，對準中心 + 慢速靠近
#          Y 座標連續超過閾值 APPROACH_COUNT 幀後進入 ACTION
# ACTION : left/right 邊走邊轉；straight 視同直走直接回 WALK


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
        # is_start 由 API 的 parameter 'start' 自動控制
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
        self.lost_start_time   = 0.0
        self._cooldown_until   = 0.0
        self.approach_count    = 0

        # === 訂閱 navigation_node 發布的結果 ===
        self.yolo_sub = self.create_subscription(
            String, 'class_id_topic', self._sign_callback, 10
        )

        # === 主迴圈 20Hz ===
        self.create_timer(0.05, self._main_loop)

        self.get_logger().info("=== Mar Strategy Initialized (Event-Driven Mode) ===")

    # ───────────────────────────────────────────
    #  is_start 由 API 的 ROS2 parameter 'start' 控制
    # ───────────────────────────────────────────

    # ───────────────────────────────────────────
    #  視覺 callback
    # ───────────────────────────────────────────
    def _sign_callback(self, msg: String):
        """接收 navigation_node 發布的標記資訊"""
        if time.time() < self._cooldown_until:
            return
        try:
            parts = msg.data.split(',')
            if len(parts) >= 4:
                self.sign_label = parts[0].lower()
                self.sign_cx    = int(parts[1])
                self.sign_cy    = int(parts[2])
                self.sign_area  = float(parts[3])
                self.last_sign_time = time.time()
        except Exception as e:
            self.get_logger().error(f"Sign parse error: {e}")

    def _sign_visible(self):
        """標記是否在有效時間內（1.0 秒內有新資料）"""
        return (time.time() - self.last_sign_time) < 1.0

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
        self.sub_state       = 'WALK'
        self.target_label    = 'none'
        self.last_sign_time  = time.time() - 999
        self.lost_start_time = 0.0
        self.approach_count  = 0
        self._cooldown_until = time.time() + INIT_COOLDOWN
        self.get_logger().info(f"[Init] 完成，進入 WALK 狀態（冷卻 {INIT_COOLDOWN}s）")

    # ───────────────────────────────────────────
    #  狀態：WALK
    # ───────────────────────────────────────────
    def _state_walk(self):
        self._look_down()

        if self._sign_visible():
            self.lost_start_time = 0.0
            self.get_logger().info(
                f"[WALK] 發現標記 {self.sign_label} area={self.sign_area:.0f} -> ALIGN",
                throttle_duration_sec=1.0
            )
            self.sub_state = 'ALIGN'
            return

        now = time.time()
        if self.lost_start_time == 0.0:
            self.lost_start_time = now

        lost_duration = now - self.lost_start_time
        correction = self._imu_correction()

        if lost_duration < LOST_WAIT_TIME:
            self.sendContinuousValue(WALK_SPEED_NORMAL, 0, correction)
            self.get_logger().info(
                f"[WALK] 搜尋中... ({lost_duration:.1f}s) yaw={self.imu_rpy[2]:.1f}",
                throttle_duration_sec=1.0
            )
        else:
            self.sendContinuousValue(WALK_SPEED_SLOW, 0, correction)
            self.get_logger().info(
                "[WALK] 慢速搜尋中...",
                throttle_duration_sec=1.0
            )

    # ───────────────────────────────────────────
    #  狀態：ALIGN
    # ───────────────────────────────────────────
    def _state_align(self):
        self._look_down()

        if (time.time() - self.last_sign_time) > SIGN_LOST_TIMEOUT:
            self.get_logger().info(
                f"[ALIGN] 標記消失 {SIGN_LOST_TIMEOUT}s -> WALK"
            )
            self.approach_count  = 0
            self.target_label    = 'none'
            self.sub_state       = 'WALK'
            self.lost_start_time = time.time()
            return

        # X 軸對準
        x_error = self.sign_cx - FRAME_CX
        if x_error < -ALIGN_DEADBAND:
            theta = IMU_CORRECT_SPEED
        elif x_error > ALIGN_DEADBAND:
            theta = -IMU_CORRECT_SPEED
        else:
            theta = self._imu_correction()

        # Y 座標未到閾值：繼續靠近，重置計數
        if self.sign_cy < SIGN_Y_THRESHOLD:
            self.approach_count = 0
            self.target_label   = 'none'
            self.sendContinuousValue(WALK_SPEED_SLOW, 0, theta)
            self.get_logger().info(
                f"[ALIGN] 靠近中 label={self.sign_label} cx={self.sign_cx} cy={self.sign_cy} theta={theta}",
                throttle_duration_sec=0.5
            )
            return

        # Y 座標超過閾值：累積計數
        self.approach_count += 1
        if self.target_label == 'none':
            self.target_label = self.sign_label

        self.sendContinuousValue(WALK_SPEED_SLOW, 0, theta)
        self.get_logger().info(
            f"[ALIGN] 靠近計數 {self.approach_count}/{APPROACH_COUNT} "
            f"cy={self.sign_cy} label={self.target_label}"
        )

        if self.approach_count >= APPROACH_COUNT:
            self.sendSensorReset(True)
            self.approach_count    = 0
            self.action_start_time = time.time()
            self.get_logger().info(
                f"[ALIGN] ✅ 靠近確認完成 -> ACTION label={self.target_label}"
            )
            self.sub_state = 'ACTION'

    # ───────────────────────────────────────────
    #  狀態：ACTION
    # ───────────────────────────────────────────
    def _state_action(self):
        label = self.target_label

        if label == 'left':
            target_yaw  = TURN_TARGET_ANGLE
            current_yaw = self.imu_rpy[2]
            if current_yaw >= target_yaw:
                self.get_logger().info(f"[ACTION] left 轉彎完成 yaw={current_yaw:.1f}")
                self._initialize()
            else:
                self.sendContinuousValue(LEFT_FORWARD_SPEED, 0, LEFT_TURN_SPEED)
                self.get_logger().info(
                    f"[ACTION] 左轉中 yaw={current_yaw:.1f} / 目標={target_yaw}",
                    throttle_duration_sec=0.5
                )

        elif label == 'right':
            target_yaw  = -TURN_TARGET_ANGLE
            current_yaw = self.imu_rpy[2]
            if current_yaw <= target_yaw:
                self.get_logger().info(f"[ACTION] right 轉彎完成 yaw={current_yaw:.1f}")
                self._initialize()
            else:
                self.sendContinuousValue(RIGHT_FORWARD_SPEED, 0, RIGHT_TURN_SPEED)
                self.get_logger().info(
                    f"[ACTION] 右轉中 yaw={current_yaw:.1f} / 目標={target_yaw}",
                    throttle_duration_sec=0.5
                )

        elif label in ('forward', 'straight'):
            self.get_logger().info("[ACTION] straight -> 直接回 WALK")
            self._initialize()

        else:
            self.get_logger().warn(f"[ACTION] 未知 label={label}，退回 WALK")
            self.sub_state = 'WALK'

    # ───────────────────────────────────────────
    #  主迴圈
    # ───────────────────────────────────────────
    def _main_loop(self):
        if not self.is_start:
            if self.initialized:
                self.sendbodyAuto(0)
                self.initialized = False
                self.get_logger().info("=== 指撥關閉 -> 機器人停止 ===")
            return

        if not self.initialized:
            self._initialize()
            self.sendbodyAuto(1)
            self.initialized = True
            self.get_logger().info("=== 比賽開始 ===")
            return

        if   self.sub_state == 'WALK':   self._state_walk()
        elif self.sub_state == 'ALIGN':  self._state_align()
        elif self.sub_state == 'ACTION': self._state_action()
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