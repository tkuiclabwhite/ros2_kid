#!/usr/bin/env python3
# coding=utf-8
import sys
import time
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from strategy.API import API

TEST                       = False
STAND_CORRECT              = False
X_RATIO                    = 120
Y_RATIO                    = 100
CORRECT                    = [0,    0,      0]
#                            [大前進, 前進, 小前進, 原地, 小前進,   後退, 大後退]
FORWARD                    = [2500, 1000,   500,    0,  -1000, -1500, -2000]
#                            [大左移,左移,小左移, 原地, 小右移,  右移,大右移]
TRANSLATION                = [2000, 1000,   500,    0,  -500, -1000, -2000]
#                            [大左旋,左旋, 原地,   右旋,大右旋]
THETA                      = [   7,    4,    0,    -4,    -7]
#                            [kick_center.x,kick_center.y,shot_center.x]
LEFT_POINT                 = [          130,          170,          195]
RIGHT_POINT                = [          170,          150,          80]
STRAIGHT_POINT_L           = [          150,          145,          140]
STRAIGHT_POINT_R           = [          170,          145,          180]
#----------#                       右腳           左腳
#                              左 ,  中,  右 |  左,  中,   右
FOOT                       = [105 , 124, 143, 160, 176, 196]
FIRST_SHOT_VERTICAL        = 1250
SHOT_VERTICAL              = 1220
SHOT_HORIZONTAL_RIGHT      = 1815
SHOT_HORIZONTAL_LEFT       = 2365
DODGE_OBS_ANGLE            = 1000
HEAD_ERROR_RANGE_X         = 15
HEAD_ERROR_RANGE_Y         = 35
ROTATE_ERROR               = 40
DRAW_FUNCTION_FLAG         = True
HEAD_HORIZONTAL            = 2048
HEAD_VERTICAL              = 1450
MAX_HEAD_HORIZONTAL        = 3072
MIN_HEAD_HORIZONTAL        = 1024
MAX_HEAD_VERTICAL          = 2048
MIN_HEAD_VERTICAL          = 1024
SCALE2DEGREE               = 360 / 4096


class Coordinate:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class ObjectInfo:
    # ROS2 API color indices: Orange=0, Yellow=1, Blue=2, Green=3, Black=4, Red=5, White=6
    color_dict = {
        'Orange': 0,
        'Yellow': 1,
        'Blue':   2,
        'Green':  3,
        'Black':  4,
        'Red':    5,
        'White':  6,
    }

    def __init__(self, color, object_type, api):
        self.api         = api
        self.color       = self.color_dict[color]
        self.edge_max    = Coordinate(0, 0)
        self.edge_min    = Coordinate(0, 0)
        self.center      = Coordinate(0, 0)
        self.get_target  = False
        self.target_size = 0

        update_strategy = {
            'OBS_left':  self.get_left_obs_object,
            'OBS_right': self.get_right_obs_object,
            'OBS':       self.get_obs_object,
            'Door':      self.get_door_object,
            'Ball':      self.get_ball_object,
        }
        self.find_object = update_strategy[object_type]

    def get_left_obs_object(self):
        object_idx = None
        for i in range(self.api.color_counts[self.color]):
            if 2000 < self.api.object_sizes[self.color][i] and self.api.object_x_max[self.color][i] < 170:
                object_idx = i
        return object_idx

    def get_right_obs_object(self):
        object_idx = None
        for i in range(self.api.color_counts[self.color]):
            if 2000 < self.api.object_sizes[self.color][i] and self.api.object_x_min[self.color][i] > 150:
                object_idx = i
        return object_idx

    def get_obs_object(self):
        if self.api.object_sizes[self.color]:
            max_size = max(self.api.object_sizes[self.color])
            max_idx  = self.api.object_sizes[self.color].index(max_size)
            return max_idx if max_size > 2000 else None

    def get_door_object(self):
        if self.api.object_sizes[self.color]:
            max_size = max(self.api.object_sizes[self.color])
            max_idx  = self.api.object_sizes[self.color].index(max_size)
            return max_idx if max_size > 2000 else None

    def get_ball_object(self):
        object_idx = None
        for i in range(self.api.color_counts[self.color]):
            length_width_diff = abs(
                abs(self.api.object_x_max[self.color][i] - self.api.object_x_min[self.color][i]) -
                abs(self.api.object_y_max[self.color][i] - self.api.object_y_min[self.color][i])
            )
            if 100 < self.api.object_sizes[self.color][i] < 2000 and length_width_diff < 12:
                object_idx = i
        return object_idx

    def update(self):
        object_idx = self.find_object()

        if object_idx is not None:
            c = self.color
            self.get_target  = True
            self.edge_max.x  = self.api.object_x_max[c][object_idx]
            self.edge_min.x  = self.api.object_x_min[c][object_idx]
            self.edge_max.y  = self.api.object_y_max[c][object_idx]
            self.edge_min.y  = self.api.object_y_min[c][object_idx]
            self.center.x    = (self.api.object_x_min[c][object_idx] + self.api.object_x_max[c][object_idx]) // 2
            self.center.y    = (self.api.object_y_min[c][object_idx] + self.api.object_y_max[c][object_idx]) // 2
            self.target_size = self.api.object_sizes[c][object_idx]
        else:
            self.edge_max.x  = 0
            self.edge_min.x  = 0
            self.edge_max.y  = 0
            self.edge_min.y  = 0
            self.center.x    = 0
            self.center.y    = 0
            self.target_size = 0
            self.get_target  = False


class StatusPrinterThread(threading.Thread):
    def __init__(self, us_node):
        super().__init__()
        self.us      = us_node
        self.daemon  = True
        self.running = True

    def run(self):
        while self.running and rclpy.ok():
            self._print()
            time.sleep(0.1)

    def _print(self):
        if not hasattr(self.us, 'state'):
            return
        try:
            us = self.us
            sys.stdout.write("\033[H\033[J")
            sys.stdout.write(
f"#==============系統狀態==============#\n"
f"is_start         : {us.is_start}\n"
f"state            : {us.state}\n"
f"walk_flag        : {us.walk_flag}\n"
f"object_center    : {us.object_center}\n"
f"#===============速度狀態===============#\n"
f"Now  (x, y, th)  : {us.now_forward}, {us.now_translation}, {us.now_theta}\n"
f"Goal (x, y, th)  : {us.forward}, {us.translation}, {us.theta}\n"
f"#===============頭部狀態===============#\n"
f"head (H, V)      : {us.head_horizon}, {us.head_vertical}\n"
f"search           : {us.search}  count={us.search_count}\n"
f"#===============感測資訊===============#\n"
f"imu_yaw          : {us.yaw:.2f}\n"
f"obs_angle_err    : {us.obs_angle_err:.2f}  door_angle_err: {us.door_angle_err:.2f}\n"
f"#===============視覺目標===============#\n"
f"ball  [{us.ball.get_target}]  x={us.ball.center.x}  y={us.ball.center.y}  size={us.ball.target_size}\n"
f"obs   [{us.obs.get_target}]  x={us.obs.center.x}  y={us.obs.center.y}  size={us.obs.target_size}\n"
f"door  [{us.door.get_target}]  x={us.door.center.x}  y={us.door.center.y}  size={us.door.target_size}\n"
f"#===============當前動作===============#\n"
f"action_status    : {getattr(us, 'action_status', '')}\n"
f"current_function : {getattr(us, 'current_func', '')}\n"
f"function_detail  : {getattr(us, 'func_detail', '')}\n"
f"shoot_count      : {us.count}\n"
f"#====================================#\n"
            )
            sys.stdout.flush()
        except Exception:
            pass


class UnitedSoccer(API):
    def __init__(self):
        super().__init__('us')
        self.ball      = ObjectInfo('Yellow', 'Ball',      self)
        self.obs_right = ObjectInfo('Blue',   'OBS_right', self)
        self.obs_left  = ObjectInfo('Blue',   'OBS_left',  self)
        self.obs       = ObjectInfo('Blue',   'OBS',       self)
        self.door      = ObjectInfo('Red',    'Door',      self)
        self.t         = 0.0
        self.t2        = 0.0
        self.action_status = "初始化中..."
        self.current_func  = ""
        self.func_detail   = ""

        self._printer = StatusPrinterThread(self)
        self._printer.start()

        self._reset()
        self.create_timer(0.1, self.main)

    def _reset(self):
        self.action_status   = "初始化"
        self.current_func    = "_reset"
        self.func_detail     = ""
        self.object_center   = False
        self.walk_flag       = False
        self.object_at_right = False
        self.object_at_left  = False
        self.state           = 'find_ball'
        self.door_angle_err  = 0
        self.obs_angle_err   = 0
        self.forward         = 0
        self.translation     = 0
        self.theta           = 0
        self.now_forward     = 0
        self.now_translation = 0
        self.now_theta       = 0
        self.imu_error       = 0
        self.count           = 0
        self.sendSensorReset(True)
        self.reset_head()
        if TEST:
            self._draw('start')
        else:
            self._draw('init')
        self.current_func  = "main"
        self.action_status = "等待開始"

    def reset_head(self):
        self.search        = 'right'
        self.head_horizon  = HEAD_HORIZONTAL
        self.head_vertical = HEAD_VERTICAL
        if not TEST:
            if self.state in ('shoot', 'shoot_ball_obs', 'shoot_ball_door'):
                self.sendHeadMotor(1, self.head_horizon, 100)
                self.sendHeadMotor(2, SHOT_VERTICAL, 100)
            else:
                self.sendHeadMotor(1, self.head_horizon, 100)
                self.sendHeadMotor(2, self.head_vertical, 100)
        self.search_count = 0

    def _draw(self, state):
        if not DRAW_FUNCTION_FLAG:
            return
        if state == 'init':
            self.drawImageFunction(1, 0, 0, 0, 0, 0, 0, 255, 0)
            self.drawImageFunction(2, 0, 0, 0, 0, 0, 0, 255, 0)
            self.drawImageFunction(3, 1, 0, 0, 0, 0, 0, 127, 255)
            self.drawImageFunction(4, 1, 0, 0, 0, 0, 0, 127, 255)
            self.drawImageFunction(5, 1, 0, 0, 0, 0, 0, 127, 255)
            self.drawImageFunction(6, 1, 0, 0, 0, 0, 0, 127, 255)
            self.drawImageFunction(7, 1, 0, 0, 0, 0, 255, 128, 0)
            self.drawImageFunction(8, 1, 0, 0, 0, 0, 255, 13, 166)
            self.drawImageFunction(9, 1, 0, 0, 0, 0, 0, 0, 0)
        elif state == 'start':
            # 中心十字線
            self.drawImageFunction(1, 1, 160, 160, 110, 130, 255, 255, 255)
            self.drawImageFunction(2, 1, 150, 170, 120, 120, 255, 255, 255)
            # 球框
            self.drawImageFunction(3, 2, self.ball.edge_min.x, self.ball.edge_max.x,
                                   self.ball.edge_min.y, self.ball.edge_max.y, 0, 127, 255)
            # OBS right 框
            self.drawImageFunction(7, 2, self.obs_right.edge_min.x, self.obs_right.edge_max.x,
                                   self.obs_right.edge_min.y, self.obs_right.edge_max.y, 255, 128, 0)
            # OBS left 框
            self.drawImageFunction(8, 2, self.obs_left.edge_min.x, self.obs_left.edge_max.x,
                                   self.obs_left.edge_min.y, self.obs_left.edge_max.y, 255, 13, 166)
            # OBS 框
            self.drawImageFunction(9, 2, self.obs.edge_min.x, self.obs.edge_max.x,
                                   self.obs.edge_min.y, self.obs.edge_max.y, 0, 0, 0)

    def object_update(self):
        self.ball.update()
        self.obs_right.update()
        self.obs_left.update()
        self.obs.update()
        self.door.update()

    def control_head(self, ID, position, speed):
        if ID == 1:
            if position > MAX_HEAD_HORIZONTAL:
                self.head_horizon = MAX_HEAD_HORIZONTAL
            elif position < MIN_HEAD_HORIZONTAL:
                self.head_horizon = MIN_HEAD_HORIZONTAL
            else:
                self.head_horizon = position
            self.sendHeadMotor(ID, self.head_horizon, speed)
        elif ID == 2:
            if position > MAX_HEAD_VERTICAL:
                self.head_vertical = MAX_HEAD_VERTICAL
            elif position < MIN_HEAD_VERTICAL:
                self.head_vertical = MIN_HEAD_VERTICAL
            else:
                self.head_vertical = position
            self.sendHeadMotor(ID, self.head_vertical, speed)

    def control_walkinggait(self, forward, translation, theta,
                            add_forward, add_translation, add_theta,
                            reduce_forward, reduce_translation, reduce_theta):
        if self.now_forward < forward:
            self.now_forward += add_forward
        elif self.now_forward > forward:
            self.now_forward -= reduce_forward
        else:
            self.now_forward = forward

        if self.now_translation < translation:
            self.now_translation += add_translation
            if self.head_horizon > 1948:
                self.now_translation += add_translation * 2
        elif self.now_translation > translation:
            self.now_translation -= reduce_translation
            if self.head_horizon < 2148:
                self.now_translation -= reduce_translation * 2
        else:
            self.now_translation = translation

        if self.now_theta < theta:
            self.now_theta += add_theta
        elif self.now_theta > theta:
            self.now_theta -= reduce_theta
        else:
            self.now_theta = theta

        self.sendContinuousValue(int(self.now_forward), int(self.now_translation), int(round(self.now_theta)))

    def search_object(self, right_max=2048-600, left_max=2048+600,
                      up_max=2048, down_max=2048-300, scale=30, *, count_flag=False):
        if self.search == 'right':
            self.control_head(1, self.head_horizon, scale)
            self.head_horizon -= scale
            if self.head_horizon < right_max:
                self.head_horizon = right_max
                self.search = 'up'
        elif self.search == 'down':
            self.control_head(2, self.head_vertical, scale)
            self.head_vertical -= scale
            if self.head_vertical < down_max:
                self.head_vertical = down_max
                self.search = 'right'
                if count_flag:
                    self.search_count += 1
        elif self.search == 'left':
            self.control_head(1, self.head_horizon, scale)
            self.head_horizon += scale
            if self.head_horizon > left_max:
                self.head_horizon = left_max
                self.search = 'down'
        elif self.search == 'up':
            self.control_head(2, self.head_vertical, scale)
            self.head_vertical += scale
            if self.head_vertical > up_max:
                self.head_vertical = up_max
                self.search = 'left'

    def trace_object(self, x_target, y_target, *, scale=100):
        if x_target != 0 and y_target != 0:
            x_differ = x_target - 160
            y_differ = y_target - 120
            x_degree = x_differ * (X_RATIO / 320)
            y_degree = y_differ * (Y_RATIO / 240)
            self.head_horizon  -= round(x_degree * 4096 / 360 * 0.15)
            self.head_vertical -= round(y_degree * 4096 / 360 * 0.15)
            if self.head_vertical < 1150:
                self.head_vertical = 1150
            self.control_head(1, self.head_horizon, scale)
            self.control_head(2, self.head_vertical, scale)
            if abs(160 - x_target) < 5 and abs(120 - y_target) < 5:
                self.object_center = True
        else:
            self.func_detail = "看不到目標,重新搜尋"

    def body_trace_rotate(self, error):
        rotate_error = self.head_horizon - HEAD_HORIZONTAL
        if rotate_error > error:
            self.forward     = FORWARD[1] + CORRECT[0]
            self.translation = TRANSLATION[3] + CORRECT[1]
            self.theta       = THETA[0] + CORRECT[2] if self.head_horizon > 2648 else THETA[1] + CORRECT[2]
            self.func_detail = f"左轉修正 rotate_err={rotate_error}"
            return False
        elif rotate_error < -error:
            self.forward     = FORWARD[1] + CORRECT[0]
            self.translation = TRANSLATION[3] + CORRECT[1]
            self.theta       = THETA[4] + CORRECT[2] if self.head_horizon > 1448 else THETA[3] + CORRECT[2]
            self.func_detail = f"右轉修正 rotate_err={rotate_error}"
            return False
        else:
            self.forward     = FORWARD[1] + CORRECT[0]
            self.translation = TRANSLATION[3] + CORRECT[1]
            self.theta       = THETA[3] + CORRECT[2]
            self.func_detail = "朝向正確,直走"
            return True

    def body_trace_straight(self, goal_degree, error):
        distance_error = self.head_vertical - goal_degree
        if distance_error > error:
            return FORWARD[2] + CORRECT[0]
        elif distance_error < -error:
            return FORWARD[4] + CORRECT[0]
        else:
            return FORWARD[3] + CORRECT[0]

    def body_trace_translation(self, goal_degree, error):
        translation_error = self.head_horizon - goal_degree
        if translation_error > 250:
            return TRANSLATION[1] + CORRECT[1]
        elif translation_error < -250:
            return TRANSLATION[5] + CORRECT[1]
        elif translation_error > error:
            return TRANSLATION[2] + CORRECT[1]
        elif translation_error < -error:
            return TRANSLATION[4] + CORRECT[1]
        else:
            return TRANSLATION[3] + CORRECT[1]

    def imu_correct(self, fix, error_range):
        yaw_error = self.yaw - fix
        if abs(yaw_error) >= error_range + 10:
            if yaw_error > 0:
                self.func_detail = f"大右轉 yaw_err={yaw_error:.1f}"
                return THETA[4] + CORRECT[2]
            else:
                self.func_detail = f"大左轉 yaw_err={yaw_error:.1f}"
                return THETA[0] + CORRECT[2]
        elif abs(yaw_error) >= error_range:
            if yaw_error > 0:
                self.func_detail = f"右轉 yaw_err={yaw_error:.1f}"
                return THETA[3] + CORRECT[2]
            else:
                self.func_detail = f"左轉 yaw_err={yaw_error:.1f}"
                return THETA[1] + CORRECT[2]
        self.func_detail = f"角度已對齊 yaw_err={yaw_error:.1f}"
        return THETA[2] + CORRECT[2]

    def walk_change(self, walk_flag):
        if walk_flag:
            self.action_status = "停止步態"
            self.walk_flag = False
            self.sendBodyAutoCmd(0, 0, 0)
            time.sleep(1.5)
            self.sendBodySector(29)
            time.sleep(1)
            if STAND_CORRECT:
                self.sendBodySector(29)
            self.forward     = FORWARD[3]     + CORRECT[0]
            self.translation = TRANSLATION[3] + CORRECT[1]
            self.theta       = THETA[2]       + CORRECT[2]
        else:
            self.action_status = "啟動步態"
            self.walk_flag = True
            self.sendBodyAutoCmd(FORWARD[2] + CORRECT[0], TRANSLATION[3] + CORRECT[1], THETA[3] + CORRECT[2])

    # def get_up(self, imu):
    #     if imu > 15:
    #         self.action_status = "向前倒，起身中"
    #         self.sendHeadMotor(1, 2048, 0)
    #         self.sendHeadMotor(2, 2420, 0)
    #         self.sendBodyAutoCmd(0, 0, 0)
    #         time.sleep(1.5)
    #         self.sendBodySector(29)
    #         time.sleep(1)
    #         self.sendBodySector(1212)
    #         time.sleep(19)
    #         self.sendBodySector(29)
    #         time.sleep(0.01)
    #         self.sendBodySector(1)
    #         time.sleep(1)
    #         self._reset()
    #     elif imu < -15:
    #         self.action_status = "向後倒，起身中"
    #         self.sendHeadMotor(1, 2048, 0)
    #         self.sendHeadMotor(2, 2420, 0)
    #         self.sendBodyAutoCmd(0, 0, 0)
    #         time.sleep(1.5)
    #         self.sendBodySector(29)
    #         time.sleep(1)
    #         self.sendBodySector(1211)
    #         time.sleep(10)
    #         self.sendBodySector(29)
    #         time.sleep(0.01)
    #         self.sendBodySector(1)
    #         time.sleep(1)
    #         self._reset()

    def motion(self, state):
        self.current_func = f"motion({state})"

        if state == 'find_ball':
            if not self.ball.get_target:
                self.action_status = f"尋找球中 (搜尋方向:{self.search} 次數:{self.search_count})"
                self.search_object(up_max=1848, down_max=2048-800, scale=40, count_flag=True)
                self.forward     = FORWARD[2]     + CORRECT[0]
                self.translation = TRANSLATION[3] + CORRECT[1]
                if self.search_count == 1:
                    self.theta    = THETA[0] + CORRECT[2]
                    self.t2       = time.time()
                    self.func_detail = f"右轉搜尋 已過:{self.t2 - self.t:.1f}s"
                    if self.t2 - self.t > 2:
                        self.search_count = 0
                else:
                    self.theta    = THETA[2] + CORRECT[2]
                    self.t        = time.time()
                    self.func_detail = ""
            else:
                self.search_count = 0
                self.action_status = "追蹤球，前往球"
                self.trace_object(self.ball.center.x, self.ball.center.y)
                if self.head_vertical < FIRST_SHOT_VERTICAL - 20 or self.head_horizon > 2648 or self.head_horizon < 1448:
                    self.forward   = FORWARD[5] + CORRECT[0]
                    self.func_detail = f"頭偏移過大,後退調整 V={self.head_vertical} H={self.head_horizon}"
                elif self.body_trace_rotate(220):
                    if self.head_vertical < FIRST_SHOT_VERTICAL:
                        self.action_status = "球已就位 → 切換到 find_obs"
                        self.state = 'find_obs'
                        self.reset_head()
                        self.object_center = False
                        self.walk_change(self.walk_flag)
                    else:
                        if self.head_vertical > 1300:
                            self.forward = FORWARD[1] + CORRECT[0]
                        else:
                            self.forward = FORWARD[2] + CORRECT[0]
                        self.translation = TRANSLATION[3] + CORRECT[1]
                        self.theta       = THETA[2]       + CORRECT[2]
                        self.func_detail = f"直走靠近球 V={self.head_vertical}"

        elif state == 'find_obs':
            if self.obs.get_target:
                self.action_status = "鎖定障礙物"
                self.trace_object(self.obs.center.x, self.obs.center.y)
                if self.object_center:
                    self.obs_angle_err = self.yaw + (self.head_horizon * SCALE2DEGREE - 180) * 0.8
                    if abs(self.obs_angle_err) < 10:
                        self.action_status = "障礙物角度已對齊 → 直接射門"
                        self.state = 'shoot'
                    else:
                        self.action_status = f"障礙物偏移 → 邊走邊瞄 obs_err={self.obs_angle_err:.1f}"
                        self.state = 'shoot_ball_obs'
                        self.now_forward = FORWARD[4] + CORRECT[0]
                    self.object_center = False
                    self.reset_head()
                    time.sleep(2)
                else:
                    self.func_detail = f"對準障礙物中 x={self.obs.center.x} y={self.obs.center.y}"

            elif self.door.get_target:
                self.action_status = "鎖定門"
                self.trace_object(self.door.center.x, self.door.center.y)
                if self.object_center:
                    self.door_angle_err = self.yaw + (self.head_horizon * SCALE2DEGREE - 180) * 0.8
                    if abs(self.door_angle_err) < 10:
                        self.action_status = "門角度已對齊 → 直接射門"
                        self.state = 'shoot'
                    else:
                        self.action_status = f"門偏移 → 邊走邊瞄 door_err={self.door_angle_err:.1f}"
                        self.state = 'shoot_ball_door'
                    self.object_center = False
                    self.reset_head()
                    time.sleep(2)
                else:
                    self.func_detail = f"對準門中 x={self.door.center.x} y={self.door.center.y}"

            else:
                self.action_status = f"尋找障礙物/門 (搜尋方向:{self.search} 次數:{self.search_count})"
                self.func_detail   = ""
                self.search_object(
                    right_max=2048 - 600 - self.search_count * 50,
                    left_max=2048 + 600 + self.search_count * 50,
                    up_max=2048,
                    down_max=2048 - 800,
                    scale=50,
                    count_flag=True,
                )

        elif state in ('shoot_ball_obs', 'shoot_ball_door'):
            self.trace_object(self.ball.center.x, self.ball.center.y)

            if self.state == 'shoot_ball_door':
                self.action_status = "邊走邊瞄 (對門)"
                if abs(self.door_angle_err + self.yaw) < 3:
                    self.theta = THETA[2] + CORRECT[2]
                    self.func_detail = f"門角度已對齊 door_err={self.door_angle_err:.1f}"
                else:
                    self.theta = self.imu_correct(self.door_angle_err, 15)
            elif self.state == 'shoot_ball_obs':
                self.action_status = "邊走邊瞄 (對障礙物)"
                if abs(self.yaw - self.obs_angle_err) < 3:
                    self.theta = THETA[2] + CORRECT[2]
                    self.func_detail = f"障礙物角度已對齊 obs_err={self.obs_angle_err:.1f}"
                else:
                    self.theta = self.imu_correct(self.obs_angle_err, 15)

            if not self.ball.get_target or self.head_horizon > 2448 or self.head_horizon < 1648:
                self.func_detail = f"[A] 調整位置 看不到球或頭偏移 H={self.head_horizon}"
                self.forward = FORWARD[5] + CORRECT[0]
                if self.head_horizon > 2648:
                    self.translation = TRANSLATION[1] + CORRECT[1]
                    self.count = max(0, self.count - 1)
                elif self.head_horizon > 2448:
                    self.translation = TRANSLATION[2] + CORRECT[1]
                elif self.head_horizon < 1448:
                    self.translation = TRANSLATION[5] + CORRECT[1]
                    self.count = max(0, self.count - 1)
                elif self.head_horizon < 1648:
                    self.translation = TRANSLATION[4] + CORRECT[1]

            elif self.head_vertical < SHOT_VERTICAL + 50:
                self.func_detail = f"[B] 靠近射門位置 V={self.head_vertical}"
                if abs(self.theta) > THETA[1] or self.head_vertical < SHOT_VERTICAL:
                    self.forward = FORWARD[4] + CORRECT[0]
                else:
                    self.forward = FORWARD[2] + CORRECT[0]
                if self.ball.center.x > 160:
                    self.translation = self.body_trace_translation(SHOT_HORIZONTAL_RIGHT, 5)
                else:
                    self.translation = self.body_trace_translation(SHOT_HORIZONTAL_LEFT, 5)

            else:
                self.func_detail = f"[C] 前後調整到射門距離 V={self.head_vertical}"
                if self.head_horizon > 2448 or self.head_horizon < 1648:
                    self.forward = FORWARD[4] + CORRECT[0]
                else:
                    self.forward = self.body_trace_straight(SHOT_VERTICAL, 8)
                self.translation = self.body_trace_translation(2048, 5)

            if abs(self.yaw - self.obs_angle_err) < 20 and \
               (not self.head_horizon > 2448 or not self.head_horizon < 1648) and \
               abs(self.head_vertical - SHOT_VERTICAL) < 10:
                self.count += 1
                self.func_detail = f"射門條件確認中 count={self.count}/3"
                if self.count > 2:
                    self.action_status = "射門條件達成，準備射門"
                    self.reset_head()
                    time.sleep(0.1)
                    self.walk_change(self.walk_flag)
                    self.object_update()
                    time.sleep(5)
                    sector = 100 if self.ball.center.x < 160 else 200
                    self.func_detail = f"選擇射門腳 ball_x={self.ball.center.x} sector={sector}"
                    self.sendBodySector(sector)
                    time.sleep(15)
                    self._reset()
                    self.sendBodySector(29)
                    time.sleep(1)

        elif state == 'shoot':
            sector = 100 if self.ball.center.x < 155 else 200
            self.action_status = f"直接射門 ball_x={self.ball.center.x} sector={sector}"
            self.walk_change(self.walk_flag)
            self.sendBodySector(sector)
            time.sleep(15)
            self._reset()
            self.sendBodySector(29)
            time.sleep(1)

    def main(self):
        if self.is_start:
            self.object_update()
            self._draw('start')
            if not self.walk_flag and self.state != 'find_obs':
                self.walk_change(self.walk_flag)
            self.motion(self.state)
            self.control_walkinggait(
                self.forward, self.translation, self.theta,
                50, 50, 0.5, 100, 50, 0.5,
            )
        else:
            if TEST:
                self.object_update()
            if self.walk_flag:
                self.walk_change(self.walk_flag)
            self._reset()


def main(args=None):
    rclpy.init(args=args)
    node = UnitedSoccer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
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
