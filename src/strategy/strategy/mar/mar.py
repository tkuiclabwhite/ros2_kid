import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor   # ← 加這行
from std_msgs.msg import String
from geometry_msgs.msg import Point
import threading
import time

# 導入機器人控制 API
try:
    from strategy.API import API
    from tku_msgs.msg import Dio
    API_AVAILABLE = True
except ImportError:
    API_AVAILABLE = False
    print("[WARNING] strategy.API or tku_msgs not found. Robot control will be disabled.")


class YOLOSignNode(Node):
    def __init__(self):
        super().__init__('yolo_sign_node')

        # ============================================================
        #  可調參數（原本在 GUI slider 上的，現在直接改這裡）
        # ============================================================
        self.TRIGGER_Y = 180          # bbox 底部 Y 超過此值就觸發動作
        self.LEFT_TURN_ANGLE = 90     # 左轉角度 (°)
        self.RIGHT_TURN_ANGLE = 90    # 右轉角度 (°)
        self.TURN_SPEED = 5           # 原地轉的 theta 值
        self.HEADING_KP = 0.5         # 直走航向修正 P 增益
        self.ALIGN_KP = 0.03          # 視覺對齊修正 P 增益 (X偏移→theta)
        self.LEFT_TURN_DURATION = 2.0   # IMU 不可用時的左轉時間 (s)
        self.RIGHT_TURN_DURATION = 2.0  # IMU 不可用時的右轉時間 (s)
        self.AVOID_DURATION = 1.5       # 直走踩過號誌的時間 (s)
        self.PRE_TURN_DURATION = 2.0    # 轉彎前先直走的時間 (s)
        self.ROBOT_CONTROL_ENABLED = True  # False = 只印 log 不送指令

        # === 基礎運動參數 ===
        self.ORIGIN_THETA = 0
        self.BASE_SPEED = 1500

        # ============================================================
        #  初始化機器人控制 API
        # ============================================================
        self.api = None
        if API_AVAILABLE:
            try:
                self.api = API('yolo_strategy_api')
                self.api_mode = True
                self.get_logger().info('[API] Robot API loaded successfully')
            except Exception as e:
                self.api_mode = False
                self.get_logger().error(f'[API] Failed to initialize API: {e}')
        else:
            self.api_mode = False
            self.get_logger().warn('[API] Robot control API not available (Test mode)')

        # === 運動控制目標值 ===
        self.target_speed_x = 0
        self.target_speed_y = 0
        self.target_theta = 0
        self.target_head_horizontal = 2048
        self.target_head_vertical = 1470
        self.target_head_speed = 50

        # === 機器人行走狀態 ===
        self.robot_walking = False

        # === IMU ===
        self.current_yaw = 0.0
        self.target_yaw = None
        self.yaw_tolerance = 1.0
        self.imu_available = False
        self.straight_yaw = None  # 直走基準航向

        # === 偵測資訊 ===
        self.sign_name = None
        self.sign_area = 0
        self.center_x = 0
        self.center_y = 0
        self.bbox_bottom_y = 0

        # === 指令鎖 ===
        self.locked_action = None
        self.command_received = False

        # === 影像參數 (320x240) ===
        self.img_center_x = 160

        # === 狀態機 ===
        self.current_state = '[SEARCH] Waiting...'

        # === 動作執行 ===
        self.action_executing = False
        self.action_start_time = None
        self.action_duration = 0.0
        self.action_timeout = 25.0
        self.current_action_name = ""
        self.use_imu_control = True

        # === 轉彎前直走階段 ===
        self.pre_turn_phase = False
        self.pre_turn_start_time = None
        self.pending_turn_action = None  # 存放等下要執行的轉彎動作

        # === 狀態顯示節流 ===
        self.last_printed_state = ""
        self.print_counter = 0

        # === 訂閱 ===
        self.create_subscription(String, 'class_id_topic', self.class_id_callback, 10)
        self.create_subscription(Point, 'sign_coordinates', self.coord_callback, 10)

        # === 控制迴圈 ===
        self.control_timer = self.create_timer(0.01, self.control_loop)    # 100Hz
        self.strategy_timer = self.create_timer(0.1, self.strategy_update)  # 10Hz
        self.status_timer = self.create_timer(1.0, self.print_status)       # 1Hz 狀態印出

        self.get_logger().info('=' * 55)
        self.get_logger().info('  YOLO Sign Node (No GUI / In-Place Turn / IMU Heading)')
        self.get_logger().info(f'  Control: {"ENABLED" if self.ROBOT_CONTROL_ENABLED else "DISABLED"}')
        self.get_logger().info(f'  Trigger Y: {self.TRIGGER_Y}  |  Heading Kp: {self.HEADING_KP}  |  Align Kp: {self.ALIGN_KP}')
        self.get_logger().info(f'  Turn Speed: {self.TURN_SPEED}  |  L/R Angle: {self.LEFT_TURN_ANGLE}°/{self.RIGHT_TURN_ANGLE}°')
        self.get_logger().info('=' * 55)

    # ================================================================
    #  IMU 工具
    # ================================================================

    def normalize_angle(self, angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def angle_difference(self, target, current):
        return self.normalize_angle(target - current)

    def compute_heading_correction(self):
        """直走時的 IMU 航向 P 修正"""
        if not self.imu_available or self.straight_yaw is None:
            return self.ORIGIN_THETA
        yaw_error = self.angle_difference(self.straight_yaw, self.current_yaw)
        correction = self.HEADING_KP * yaw_error
        correction = max(-5.0, min(5.0, correction))
        return int(self.ORIGIN_THETA + correction)

    def ensure_straight_yaw(self):
        if self.imu_available and self.straight_yaw is None:
            self.straight_yaw = self.current_yaw
            self.get_logger().info(f'[IMU] Heading baseline set: {self.straight_yaw:.1f}°')

    def update_imu(self, yaw):
        self.current_yaw = yaw
        self.imu_available = True

    # ================================================================
    #  機器人行走控制
    # ================================================================

    def start_robot_walking(self):
        if not self.api_mode:
            return
        if not self.robot_walking:
            try:
                self.send_body_auto(1)
                self.robot_walking = True
                self.get_logger().info('[ROBOT] Walking started ✓')
            except Exception as e:
                self.get_logger().error(f'[ROBOT] Failed to start: {e}')

    def stop_robot_walking(self):
        if not self.api_mode:
            return
        if self.robot_walking:
            try:
                self.send_body_auto(0)
                self.robot_walking = False
                self.get_logger().info('[ROBOT] Walking stopped ✓')
            except Exception as e:
                self.get_logger().error(f'[ROBOT] Failed to stop: {e}')

    # ================================================================
    #  ROS2 回呼
    # ================================================================

    def class_id_callback(self, msg):
        try:
            parts = msg.data.split(',')
            if len(parts) != 4:
                return

            new_sign = parts[0]
            self.center_x = int(parts[1])
            self.bbox_bottom_y = int(parts[2])
            self.sign_area = int(parts[3])

            if self.action_executing:
                return

            if self.locked_action is not None:
                self.sign_name = new_sign
                return

            if new_sign and new_sign != 'None':
                self.sign_name = new_sign
                self.locked_action = new_sign
                self.command_received = True
                self.get_logger().info(
                    f'[CMD] Locked: {new_sign.upper()} '
                    f'(x={self.center_x}, botY={self.bbox_bottom_y}, area={self.sign_area})')

        except Exception as e:
            self.get_logger().error(f'Parse failed: {e}')

    def coord_callback(self, msg):
        self.center_x = int(msg.x)
        self.center_y = int(msg.y)

    # ================================================================
    #  主控制迴圈 (100Hz)
    # ================================================================

    def control_loop(self):
        if self.api_mode and self.api and hasattr(self.api, 'imu_rpy'):
            self.update_imu(self.api.imu_rpy[2])

        if not self.ROBOT_CONTROL_ENABLED:
            if self.robot_walking:
                self.stop_robot_walking()
            return

        if not self.robot_walking:
            self.start_robot_walking()

        if self.api_mode and self.api:
            self.send_head_motor(1, self.target_head_horizontal, self.target_head_speed)
            self.send_head_motor(2, self.target_head_vertical, self.target_head_speed)
            self.send_continuous_value(self.target_speed_x, self.target_speed_y, self.target_theta)

    # ================================================================
    #  策略更新迴圈 (10Hz)
    # ================================================================

    def strategy_update(self):
        current_time = self.get_clock().now().nanoseconds / 1e9

        # --- 動作執行中 ---
        if self.action_executing and self.action_start_time is not None:
            elapsed = current_time - self.action_start_time

            # if self.use_imu_control and self.target_yaw is not None:
            #     signed_error = self.angle_difference(self.target_yaw, self.current_yaw)
            #     angle_error = abs(signed_error)

            #     if angle_error < self.yaw_tolerance:
            #         self.get_logger().info(f'[IMU] Turn complete! Error: {angle_error:.2f}°')
            #         self.finish_action()
            #     elif elapsed > self.action_timeout:
            #         self.get_logger().warn(f'[IMU] Timeout! Error: {angle_error:.2f}°')
            #         self.finish_action()
            #     else:
            #         # P 控制轉速：離目標遠就快，接近就慢
            #         scale = min(1.0, angle_error / 45.0)
            #         turn_theta = max(1, int(self.TURN_SPEED * scale))
            #         if signed_error < 0:
            #             turn_theta = -turn_theta
            #         self.target_theta = turn_theta

            #         remaining = self.action_timeout - elapsed
            #         self.current_state = (f'[TURNING] {self.current_action_name} | '
            #                               f'Err: {angle_error:.1f}° | θ: {turn_theta} | T: {remaining:.1f}s')
            if self.use_imu_control and self.target_yaw is not None:
                angle_error = abs(self.angle_difference(self.target_yaw, self.current_yaw))

                if angle_error < self.yaw_tolerance:
                    self.get_logger().info(f'[IMU] Turn complete! Error: {angle_error:.2f}°')
                    self.finish_action()
                elif elapsed > self.action_timeout:
                    self.get_logger().warn(f'[IMU] Timeout! Error: {angle_error:.2f}°')
                    self.finish_action()
                else:
                    # 定速轉：theta 維持 execute_action() 裡設定的值
                    remaining = self.action_timeout - elapsed
                    self.current_state = (f'[TURNING] {self.current_action_name} | '
                                          f'Err: {angle_error:.1f}° | θ: {self.target_theta} | '
                                          f'T: {remaining:.1f}s')
            else:
                if elapsed < self.action_duration:
                    remaining = self.action_duration - elapsed
                    self.current_state = f'[EXECUTING] {self.current_action_name} ... {remaining:.1f}s'
                else:
                    self.finish_action()
            return

        # --- 轉彎前直走階段 ---
        if self.pre_turn_phase and self.pre_turn_start_time is not None:
            elapsed = current_time - self.pre_turn_start_time
            if elapsed < self.PRE_TURN_DURATION:
                remaining = self.PRE_TURN_DURATION - elapsed
                self.current_state = f'[PRE-TURN] Walking forward ... {remaining:.1f}s'
                self.target_speed_x = self.BASE_SPEED
                self.target_speed_y = 0
                self.target_theta = self.compute_heading_correction()
            else:
                # 直走完成，開始轉彎
                self.pre_turn_phase = False
                self.get_logger().info('[PRE-TURN] Forward done, now turning')
                self.execute_action(self.pending_turn_action)
            return

        # --- 沒有鎖定指令 → 直走 ---
        if self.locked_action is None:
            self.ensure_straight_yaw()
            theta = self.compute_heading_correction()
            self.target_speed_x = self.BASE_SPEED
            self.target_speed_y = 0
            self.target_theta = theta
            self.current_state = '[SEARCH] Waiting for sign...'
            return

        # --- 有鎖定指令：直走接近（視覺對齊）→ 執行 ---
        if self.bbox_bottom_y < self.TRIGGER_Y:
            self.current_state = (f'[APPROACH] {self.locked_action.upper()} | '
                                  f'Y: {self.bbox_bottom_y}/{self.TRIGGER_Y} | '
                                  f'X off: {self.center_x - self.img_center_x}px')
            self.perform_approach()
        else:
            # 觸發！左右轉先直走一段再轉，straight 直接執行
            if self.locked_action in ('left', 'right'):
                self.pre_turn_phase = True
                self.pre_turn_start_time = current_time
                self.pending_turn_action = self.locked_action
                self.target_speed_x = self.BASE_SPEED
                self.target_speed_y = 0
                self.target_theta = self.compute_heading_correction()
                self.get_logger().info(
                    f'[PRE-TURN] Walking forward {self.PRE_TURN_DURATION}s before {self.locked_action.upper()} turn')
                self.current_state = f'[PRE-TURN] Walking forward before {self.locked_action.upper()}'
            else:
                self.current_state = f'[ACTION] Execute {self.locked_action.upper()}'
                self.execute_action(self.locked_action)

    # ================================================================
    #  動作方法
    # ================================================================

    def perform_approach(self):
        """直走接近 + 視覺 X 偏移修正 theta（邊走邊對齊）"""
        x_diff = self.center_x - self.img_center_x  # 正=號誌在右邊
        theta_correction = -self.ALIGN_KP * x_diff  # 號誌在右→右轉(負theta)
        theta_correction = max(-5.0, min(5.0, theta_correction))

        self.target_speed_x = self.BASE_SPEED
        self.target_speed_y = 0
        self.target_theta = int(theta_correction)

    def execute_action(self, action):
        """原地轉 or 直走踩過"""
        self.action_executing = True
        self.action_start_time = self.get_clock().now().nanoseconds / 1e9

        if action == 'left':
            self.current_action_name = 'LEFT TURN'
            if self.imu_available:
                self.use_imu_control = True
                self.target_yaw = self.normalize_angle(self.current_yaw + self.LEFT_TURN_ANGLE)
                self.target_speed_x = 0
                self.target_speed_y = 0
                self.target_theta = self.TURN_SPEED
                self.get_logger().info(
                    f'[IMU] Left in-place: {self.current_yaw:.1f}° → {self.target_yaw:.1f}°')
            else:
                self.use_imu_control = False
                self.action_duration = self.LEFT_TURN_DURATION
                self.target_speed_x = 0
                self.target_speed_y = 0
                self.target_theta = 8
                self.get_logger().warn('[WARN] No IMU, timed left turn')

        elif action == 'right':
            self.current_action_name = 'RIGHT TURN'
            if self.imu_available:
                self.use_imu_control = True
                self.target_yaw = self.normalize_angle(self.current_yaw - self.RIGHT_TURN_ANGLE)
                self.target_speed_x = 0
                self.target_speed_y = 0
                self.target_theta = -self.TURN_SPEED
                self.get_logger().info(
                    f'[IMU] Right in-place: {self.current_yaw:.1f}° → {self.target_yaw:.1f}°')
            else:
                self.use_imu_control = False
                self.action_duration = self.RIGHT_TURN_DURATION
                self.target_speed_x = 0
                self.target_speed_y = 0
                self.target_theta = -8
                self.get_logger().warn('[WARN] No IMU, timed right turn')

        elif action == 'straight':
            self.use_imu_control = False
            self.action_duration = self.AVOID_DURATION
            self.current_action_name = 'STRAIGHT THROUGH'
            self.target_speed_x = self.BASE_SPEED
            self.target_speed_y = 0
            self.target_theta = self.compute_heading_correction()

        self.get_logger().warn(f'===== Execute {action.upper()} =====')

    def finish_action(self):
        """動作完成,更新基準航向,重置狀態"""
        self.action_executing = False
        self.target_yaw = None
        self.locked_action = None
        self.sign_name = None
        self.sign_area = 0
        self.bbox_bottom_y = 0
        self.command_received = False
        self.pre_turn_phase = False
        self.pending_turn_action = None

        if self.imu_available:
            self.straight_yaw = self.current_yaw
            self.get_logger().info(f'[IMU] New heading baseline: {self.straight_yaw:.1f}°')

        self.target_speed_x = self.BASE_SPEED
        self.target_speed_y = 0
        self.target_theta = self.ORIGIN_THETA
        self.current_state = '[SEARCH] Waiting for sign...'
        self.get_logger().info('[RESET] Ready for next command')

    # ================================================================
    #  定期狀態印出 (1Hz)
    # ================================================================

    def print_status(self):
        """每秒印出一次當前狀態，避免 log 洗頻"""
        yaw_str = f'{self.current_yaw:.1f}°' if self.imu_available else 'N/A'
        straight_str = f'{self.straight_yaw:.1f}°' if self.straight_yaw is not None else '--'
        target_str = f'{self.target_yaw:.1f}°' if self.target_yaw is not None else '--'

        self.get_logger().info(
            f'{self.current_state} | '
            f'Yaw: {yaw_str} (base: {straight_str}, tgt: {target_str}) | '
            f'Spd: ({self.target_speed_x}, {self.target_speed_y}, {self.target_theta:.2f})')

    # ================================================================
    #  底層 API 封裝
    # ================================================================

    def send_head_motor(self, motor_id, position, speed):
        if not self.api_mode or not self.api:
            return
        self.api.sendHeadMotor(motor_id, position, speed)

    def send_continuous_value(self, speed_x, speed_y, theta):
        if not self.api_mode or not self.api:
            return
        self.api.sendContinuousValue(int(speed_x), int(speed_y), int(theta))

    def send_body_auto(self, mode):
        if not self.api_mode or not self.api:
            return
        self.api.sendbodyAuto(mode)


def main(args=None):
    rclpy.init(args=args)
    node = YOLOSignNode()

    # 用 MultiThreadedExecutor 同時 spin 策略節點 + API 節點
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    if node.api_mode and node.api is not None:
        executor.add_node(node.api)
        node.get_logger().info('[API] API node added to executor ✓')

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.api_mode and node.api:
                node.send_body_auto(0)
        except Exception:
            pass
        executor.shutdown()
        try:
            if node.api is not None:
                node.api.destroy_node()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()