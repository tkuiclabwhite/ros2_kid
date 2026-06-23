#!/usr/bin/env python3
# coding=utf-8

import sys
import time
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from strategy.API import API

COLOR_BALL = 'yellow'
COLOR_OBSTACLE = 'blue'

# 影像解析度與畫面中心
IMG_W  = 320
IMG_H  = 240
IMG_CX = IMG_W // 2
IMG_CY = IMG_H // 2

# 步態
STOP_MOVE = [0, 0, 0]
LEFT_CORRECT = [0, 0, 4]
RIGHT_CORRECT = [50, 0, -4]
FORWARD_MOVE = [1500, -100, -1]

CONFIRM_FORWARD_MOVE  = [500, 0, 0]    # 確認球時慢慢前進
CONFIRM_BACKWARD_MOVE = [-500, 0, 0]   # 確認球時慢慢後退

# 藍色障礙物身體微調：
# 目標是讓機器人繞著球調整角度，讓身體朝向藍色障礙物
# 右邊看到藍色 -> ORBIT_RIGHT_MOVE
# 左邊看到藍色 -> ORBIT_LEFT_MOVE
ORBIT_LEFT_MOVE  = [0, -250, 4]
ORBIT_RIGHT_MOVE = [0, 250, -4]
ORBIT_ALIGN_FRAMES = 5
ORBIT_OBSTACLE_LOST_FRAMES = 5  # orbit 時藍色短暫消失幾次才回 find_obstacle

OBSTACLE_BODY_TOL_X = 10  # 藍色障礙物中心 x 距離畫面中心的容許誤差
OBSTACLE_MIN_AREA = 300

# 射門
RIGHT_KICK_SECTOR = 100 #右腳射門
LEFT_KICK_SECTOR  = 200 #左腳射門
KICK_FOOT = 'right' #預設右腳

# 頭部
HEAD_H_CENTER = 2048
HEAD_V_CENTER = 1600
HEAD_H_MAX = 3072
HEAD_H_MIN = 1024
HEAD_V_MAX = 2048
HEAD_V_MIN = 1200
HEAD_SPEED = 30

HEAD_KP_H = 1.0  # 太大容易抖動，太小會追太慢
HEAD_KP_V = 1.0
HEAD_MAX_STEP_H = 80  # 頭部水平單次最大修正量
HEAD_MAX_STEP_V = 60  # 頭部垂直單次最大修正量
HEAD_TOL_X = 12  # 水平方向死區
HEAD_TOL_Y = 20  # 垂直方向死區

HEAD_SEARCH_STEP = 50  # 數值越大掃描越快，但可能略過目標，數值越小掃描越慢，但比較穩
SEARCH_LEVELS = [1450, 1650, 1850] # 搜尋球時使用的三層垂直角度

# 球
BALL_MIN_AREA = 80
BALL_ASPECT_MIN = 0.5 # 球長寬比下限
BALL_ASPECT_MAX = 2.0 #上限
BALL_MIN_Y_CENTROID = 60 # 球中心 y 座標下限
BALL_FOUND_FRAMES = 5 #連續看球幾次 才算對準
BALL_LOST_FRAMES = 8  #連續錯過幾次 才算真的lost ball

# 走向球時的身體修正條件
WALK_HEAD_H_TOL = 120 # 頭部水平偏差超過此值時，代表身體還沒對準球需用左右旋轉修正身體方向
WALK_FORWARD_TOL = 80 # 頭部水平偏差小於此值時，代表球大致在身體前方，允許機器人往前走

BALL_REACH_HEAD_V_MIN = 1230 # 看球垂直刻度範圍
BALL_REACH_HEAD_V_MAX = 1350
BALL_REACH_FRAMES = 5

# 障礙物與射門前確認
HEAD_OBSTACLE_SEARCH_V = 1900
OBSTACLE_CENTER_FRAMES = 4

HEAD_CONFIRM_BALL_V = 1400
CONFIRM_BALL_FRAMES = 4


class BallInfo:
    def __init__(self, api: API):
        self.api = api
        self.visible = False
        self.cx = 0
        self.cy = 0
        self.area = 0
        self.aspect = 0.0

    # 從影像辨識節點取得所有黃色物件
    def update(self):
        objs = self.api.get_objects(COLOR_BALL)

        if not objs:
            self.visible = False
            return

        candidates = [
            o for o in objs
            if o['area'] > BALL_MIN_AREA
            and BALL_ASPECT_MIN < o['aspect_ratio'] < BALL_ASPECT_MAX
            and o['centroid'][1] > BALL_MIN_Y_CENTROID
        ]

        if not candidates:
            self.visible = False
            return

        best = max(candidates, key=lambda o: o['area'])

        self.visible = True
        self.cx = best['centroid'][0]
        self.cy = best['centroid'][1]
        self.area = best['area']
        self.aspect = best['aspect_ratio']


class ObstacleInfo:
    def __init__(self, api: API):
        self.api = api
        self.visible = False
        self.cx = 0
        self.cy = 0
        self.area = 0
        self.aspect = 0.0

    def update(self):
        objs = self.api.get_objects(COLOR_OBSTACLE)

        if not objs:
            self.visible = False
            return

        candidates = [
            o for o in objs
            if o['area'] > OBSTACLE_MIN_AREA
        ]

        if not candidates:
            self.visible = False
            return

        best = max(candidates, key=lambda o: o['area'])

        self.visible = True
        self.cx = best['centroid'][0]
        self.cy = best['centroid'][1]
        self.area = best['area']
        self.aspect = best['aspect_ratio']


class StatusPrinter(threading.Thread):
    def __init__(self, node):
        super().__init__(daemon=True)
        self.node = node

    def run(self):
        while rclpy.ok():
            try:
                n = self.node
                sys.stdout.write("\033[H\033[J")
                sys.stdout.write(
                    f"#========== United Soccer — full strategy ==========#\n"
                    f" is_start        : {n.is_start}\n"
                    f" state           : {n.state}\n"
                    f" action_detail   : {n.action_detail}\n"

                    f"#================== 黃色球 =======================#\n"
                    f" ball.visible    : {n.ball.visible}\n"
                    f" ball.lost       : {n.ball_lost_count}/{BALL_LOST_FRAMES}\n"
                    f" ball.found      : {n.found_ball_count}/{BALL_FOUND_FRAMES}\n"
                    f" ball.reach      : {n.reach_ball_count}/{BALL_REACH_FRAMES}\n"
                    f" ball.confirm    : {n.confirm_ball_count}/{CONFIRM_BALL_FRAMES}\n"
                    f" ball.cx / cy    : {n.ball.cx} / {n.ball.cy}\n"
                    f" ball.area       : {n.ball.area}\n"
                    f" ball.aspect     : {n.ball.aspect:.2f}\n"

                    f"#================== 藍色障礙物 =======================#\n"
                    f" obstacle.visible: {n.obstacle.visible}\n"
                    f" obstacle.center : {n.obstacle_center_count}/{OBSTACLE_CENTER_FRAMES}\n"
                    f" orbit.align     : {n.orbit_align_count}/{ORBIT_ALIGN_FRAMES}\n"
                    f" orbit.lost      : {n.orbit_obstacle_lost_count}/{ORBIT_OBSTACLE_LOST_FRAMES}\n"
                    f" orbit.head_lock : {n.orbit_head_locked}\n"
                    f" obstacle.cx/cy  : {n.obstacle.cx} / {n.obstacle.cy}\n"
                    f" obstacle.area   : {n.obstacle.area}\n"
                    f" obstacle.aspect : {n.obstacle.aspect:.2f}\n"

                    f"#================== 頭部狀態 =======================#\n"
                    f" head_h          : {n.head_h}  center={HEAD_H_CENTER}\n"
                    f" head_v          : {n.head_v}\n"
                    f" kick_foot       : {KICK_FOOT}\n"
                    f" search_dir      : {n.search_dir}\n"
                    f" search_count    : {n.search_count}\n"
                    f" search_level    : {n.search_level} / {SEARCH_LEVELS[n.search_level]}\n"
                    f"#====================================================#\n"
                )
                sys.stdout.flush()
            except Exception:
                pass

            time.sleep(0.1)


class UnitedSoccer(API):
    def __init__(self):
        super().__init__('us_full_strategy_orbit')

        self.ball = BallInfo(self)
        self.obstacle = ObstacleInfo(self)

        self.head_h = HEAD_H_CENTER
        self.head_v = HEAD_V_CENTER

        self.search_dir = 'right'
        self.search_count = 0
        self.search_level = 0

        self.ball_lost_count = 0
        self.found_ball_count = 0
        self.reach_ball_count = 0
        self.confirm_ball_count = 0
        self.confirm_ball_lost_count = 0
        self.obstacle_center_count = 0

        # 繞球對準計數器
        self.orbit_align_count = 0
        self.orbit_obstacle_lost_count = 0

        # orbit 階段頭部鎖定旗標：避免頭又回去追球
        self.orbit_head_locked = False

        self.state = 'find_ball'
        self.action_detail = '初始化'

        self._printer = StatusPrinter(self)
        self._printer.start()

        self._reset_head()

        self.create_timer(0.1, self.main)

    # 頭部控制函式：防止頭部超出可動範圍
    def _clamp_head(self):
        self.head_h = max(HEAD_H_MIN, min(HEAD_H_MAX, self.head_h))
        self.head_v = max(HEAD_V_MIN, min(HEAD_V_MAX, self.head_v))

    # 傳送頭部馬達命令
    def _send_head(self):
        self._clamp_head()
        self.sendHeadMotor(1, self.head_h, HEAD_SPEED)
        self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

    # 頭部回正
    def _reset_head(self):
        self.head_h = HEAD_H_CENTER
        self.head_v = HEAD_V_CENTER
        self._send_head()

        self.search_dir = 'right'
        self.search_count = 0
        self.search_level = 0

    # 停止步態
    def _stop_walk(self):
        self.sendContinuousValue(*STOP_MOVE)
        self.sendbodyAuto(0)

    # 頭部追蹤控制器
    def _track_object(self, cx, cy):
        err_x = cx - IMG_CX
        err_y = cy - IMG_CY

        centered_x = abs(err_x) < HEAD_TOL_X
        centered_y = abs(err_y) < HEAD_TOL_Y

        if not centered_x:
            step_h = int(err_x * HEAD_KP_H)
            step_h = max(-HEAD_MAX_STEP_H, min(HEAD_MAX_STEP_H, step_h))

            self.head_h -= step_h
            self.head_h = max(HEAD_H_MIN, min(HEAD_H_MAX, self.head_h))
            self.sendHeadMotor(1, self.head_h, HEAD_SPEED)

        if not centered_y:
            step_v = int(err_y * HEAD_KP_V)
            step_v = max(-HEAD_MAX_STEP_V, min(HEAD_MAX_STEP_V, step_v))

            self.head_v -= step_v
            self.head_v = max(HEAD_V_MIN, min(HEAD_V_MAX, self.head_v))
            self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

        return centered_x and centered_y

    # 搜尋黃色球
    def _search_head_for_ball(self):
        self.head_v = SEARCH_LEVELS[self.search_level]
        self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

        if self.search_dir == 'right':
            self.head_h -= HEAD_SEARCH_STEP

            if self.head_h <= HEAD_H_MIN:
                self.head_h = HEAD_H_MIN
                self.search_dir = 'left'
                self.search_count += 1
                self.search_level = (self.search_level + 1) % len(SEARCH_LEVELS)

        else:
            self.head_h += HEAD_SEARCH_STEP

            if self.head_h >= HEAD_H_MAX:
                self.head_h = HEAD_H_MAX
                self.search_dir = 'right'
                self.search_count += 1
                self.search_level = (self.search_level + 1) % len(SEARCH_LEVELS)

        self.sendHeadMotor(1, self.head_h, HEAD_SPEED)

    # 搜尋藍色障礙物
    def _search_head_for_obstacle(self):
        self.head_v = HEAD_OBSTACLE_SEARCH_V
        self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

        if self.search_dir == 'right':
            self.head_h -= HEAD_SEARCH_STEP

            if self.head_h <= HEAD_H_MIN:
                self.head_h = HEAD_H_MIN
                self.search_dir = 'left'

        else:
            self.head_h += HEAD_SEARCH_STEP

            if self.head_h >= HEAD_H_MAX:
                self.head_h = HEAD_H_MAX
                self.search_dir = 'right'

        self.sendHeadMotor(1, self.head_h, HEAD_SPEED)

    # 狀態：find_ball
    def _state_find_ball(self):
        if self.ball.visible:
            self.ball_lost_count = 0
            self.found_ball_count += 1

            self.confirm_ball_lost_count = 0
            centered = self._track_object(self.ball.cx, self.ball.cy)

            if self.found_ball_count >= BALL_FOUND_FRAMES:
                self.state = 'walk_to_ball'
                self.reach_ball_count = 0
                self.confirm_ball_count = 0
                self.obstacle_center_count = 0
                self.orbit_align_count = 0
                self.orbit_obstacle_lost_count = 0
                self.orbit_head_locked = False
                self.action_detail = '連續找到球 5 次，切換到 walk_to_ball'
                return

            self.action_detail = (
                f'追蹤球中 centered={centered} '
                f'found={self.found_ball_count}/{BALL_FOUND_FRAMES}'
            )

        else:
            self.ball_lost_count += 1
            self.found_ball_count = 0
            self.reach_ball_count = 0

            if self.ball_lost_count <= BALL_LOST_FRAMES:
                self.action_detail = (
                    f'球暫時消失 lost={self.ball_lost_count}/{BALL_LOST_FRAMES}'
                )
            else:
                self._search_head_for_ball()
                self.action_detail = (
                    f'搜尋球中 dir={self.search_dir} level={self.search_level}'
                )

    # 狀態：walk_to_ball
    def _state_walk_to_ball(self):
        if self.ball.visible:
            self.ball_lost_count = 0
            self._track_object(self.ball.cx, self.ball.cy)

            if BALL_REACH_HEAD_V_MIN <= self.head_v <= BALL_REACH_HEAD_V_MAX:
                self.reach_ball_count += 1

                if self.reach_ball_count >= BALL_REACH_FRAMES:
                    self._stop_walk()

                    print("\n########################################")
                    print("######### 到達射門位置 #########")
                    print("########################################")
                    print(f"head_v = {self.head_v}")
                    print(f"area   = {self.ball.area}\n")

                    self.head_h = HEAD_H_CENTER
                    self.head_v = HEAD_OBSTACLE_SEARCH_V
                    self.sendHeadMotor(1, self.head_h, HEAD_SPEED)
                    self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

                    self.obstacle_center_count = 0
                    self.orbit_align_count = 0
                    self.orbit_obstacle_lost_count = 0
                    self.orbit_head_locked = False

                    self.state = 'find_obstacle'
                    self.action_detail = '到達射門位置，抬頭找藍色障礙物'
                    return
            else:
                self.reach_ball_count = 0

            head_error = self.head_h - HEAD_H_CENTER

            self.sendbodyAuto(1)

            if head_error > WALK_HEAD_H_TOL:
                self.sendContinuousValue(*LEFT_CORRECT)
                self.action_detail = f'走向球：左轉 head_error={head_error}'

            elif head_error < -WALK_HEAD_H_TOL:
                self.sendContinuousValue(*RIGHT_CORRECT)
                self.action_detail = f'走向球：右轉 head_error={head_error}'

            elif abs(head_error) < WALK_FORWARD_TOL:
                self.sendContinuousValue(*FORWARD_MOVE)
                self.action_detail = (
                    f'走向球：前進 head_error={head_error} '
                    f'head_v={self.head_v}'
                )

            else:
                self.sendContinuousValue(*STOP_MOVE)
                self.action_detail = f'走向球：等待對正 head_error={head_error}'

        else:
            self.ball_lost_count += 1
            self.reach_ball_count = 0

            if self.ball_lost_count <= BALL_LOST_FRAMES:
                self.sendbodyAuto(1)
                self.sendContinuousValue(*STOP_MOVE)
                self.action_detail = (
                    f'走向球中暫時失去球 lost={self.ball_lost_count}/{BALL_LOST_FRAMES}'
                )
            else:
                self._stop_walk()
                self.found_ball_count = 0
                self.state = 'find_ball'
                self.action_detail = '走向球時失去球，回到 find_ball'

    # 狀態：尋找藍色障礙物 find_obstacle
    def _state_find_obstacle(self):
        self._stop_walk()
        self.obstacle.update()

        if self.obstacle.visible:
            print("\n########################################")
            print("######### 藍色障礙物已看到 #########")
            print("########################################")
            print(f"obstacle_x = {self.obstacle.cx}")
            print(f"obstacle_y = {self.obstacle.cy}")
            print(f"area       = {self.obstacle.area}\n")

            # 進 orbit 前先把頭鎖到障礙物高度
            # 後面 orbit 不再追球，只用身體修正藍色障礙物位置
            self.head_h = HEAD_H_CENTER
            self.head_v = HEAD_OBSTACLE_SEARCH_V
            self.sendHeadMotor(1, self.head_h, HEAD_SPEED)
            self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

            self.obstacle_center_count = 0
            self.orbit_align_count = 0
            self.orbit_obstacle_lost_count = 0
            self.orbit_head_locked = True

            self.state = 'orbit_ball_align'
            self.action_detail = '看到藍色障礙物，頭部鎖定，開始繞球對準'
            return

        self.obstacle_center_count = 0
        self._search_head_for_obstacle()
        self.action_detail = (
            f'抬頭搜尋藍色障礙物 head_h={self.head_h} head_v={self.head_v}'
        )

    # 狀態：繞球對準障礙物 orbit_ball_align
    def _state_orbit_ball_align(self):
        """
        繞著球去對準藍色障礙物。

        重點：
        1. orbit 階段不追球。
        2. 頭部鎖在 HEAD_H_CENTER / HEAD_OBSTACLE_SEARCH_V。
        3. 只用藍色障礙物的位置決定身體側移與旋轉。
        4. 繞完後才低頭進 confirm_ball 重新確認球。
        """

        # orbit 階段只更新藍色障礙物，不更新球、不追球
        self.obstacle.update()

        # 頭只在剛進 orbit 或被重設時送一次
        if not self.orbit_head_locked:
            self.head_h = HEAD_H_CENTER
            self.head_v = HEAD_OBSTACLE_SEARCH_V
            self.sendHeadMotor(1, self.head_h, HEAD_SPEED)
            self.sendHeadMotor(2, self.head_v, HEAD_SPEED)
            self.orbit_head_locked = True

        if not self.obstacle.visible:
            self.orbit_obstacle_lost_count += 1
            self.orbit_align_count = 0
            self.sendContinuousValue(*STOP_MOVE)
            self.sendbodyAuto(0)

            if self.orbit_obstacle_lost_count > ORBIT_OBSTACLE_LOST_FRAMES:
                self.orbit_head_locked = False
                self.state = 'find_obstacle'
                self.action_detail = '繞球時連續看不到藍色障礙物，回到 find_obstacle'
                return

            self.action_detail = (
                f'繞球時藍色短暫消失 '
                f'{self.orbit_obstacle_lost_count}/{ORBIT_OBSTACLE_LOST_FRAMES}'
            )
            return

        self.orbit_obstacle_lost_count = 0

        obstacle_error = self.obstacle.cx - IMG_CX

        if obstacle_error > OBSTACLE_BODY_TOL_X:
            self.sendbodyAuto(1)
            self.sendContinuousValue(*ORBIT_RIGHT_MOVE)
            self.orbit_align_count = 0
            self.action_detail = (
                f'繞球右修正 obstacle_error={obstacle_error}'
            )

        elif obstacle_error < -OBSTACLE_BODY_TOL_X:
            self.sendbodyAuto(1)
            self.sendContinuousValue(*ORBIT_LEFT_MOVE)
            self.orbit_align_count = 0
            self.action_detail = (
                f'繞球左修正 obstacle_error={obstacle_error}'
            )

        else:
            self.sendContinuousValue(*STOP_MOVE)
            self.sendbodyAuto(0)

            self.orbit_align_count += 1
            self.action_detail = (
                f'繞球對準完成中 '
                f'{self.orbit_align_count}/{ORBIT_ALIGN_FRAMES} '
                f'obstacle_error={obstacle_error}'
            )

            if self.orbit_align_count >= ORBIT_ALIGN_FRAMES:
                print("\n########################################")
                print("######### 繞球對準障礙物完成 #########")
                print("########################################")
                print(f"obstacle_x    = {self.obstacle.cx}")
                print(f"obstacle_error= {obstacle_error}")
                print(f"area          = {self.obstacle.area}\n")

                self._stop_walk()

                self.head_h = HEAD_H_CENTER
                self.head_v = HEAD_CONFIRM_BALL_V
                self.sendHeadMotor(1, self.head_h, HEAD_SPEED)
                self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

                self.confirm_ball_count = 0
                self.confirm_ball_lost_count = 0
                self.orbit_head_locked = False
                self.state = 'confirm_ball'
                self.action_detail = '繞球對準完成，低頭再次確認球'

    # 狀態：確認球 confirm_ball
    def _state_confirm_ball(self):
        if self.ball.visible:
            self.confirm_ball_lost_count = 0
            self._track_object(self.ball.cx, self.ball.cy)

            head_error = self.head_h - HEAD_H_CENTER

            if BALL_REACH_HEAD_V_MIN <= self.head_v <= BALL_REACH_HEAD_V_MAX:
                self.sendContinuousValue(*STOP_MOVE)
                self.sendbodyAuto(0)

                self.confirm_ball_count += 1

                if self.confirm_ball_count >= CONFIRM_BALL_FRAMES:
                    print("\n########################################")
                    print("######### 射門前球位置確認完成 #########")
                    print("########################################")
                    print(f"head_v = {self.head_v}")
                    print(f"area   = {self.ball.area}\n")

                    self.state = 'shoot'
                    self.action_detail = '球位置再次確認完成，準備射門'
                    return

                self.action_detail = (
                    f'確認球位置中 '
                    f'{self.confirm_ball_count}/{CONFIRM_BALL_FRAMES}'
                )

            else:
                self.confirm_ball_count = 0
                self.sendbodyAuto(1)

                if head_error > WALK_HEAD_H_TOL:
                    self.sendContinuousValue(*LEFT_CORRECT)
                    self.action_detail = (
                        f'確認球：球偏左，左轉修正 head_error={head_error}'
                    )

                elif head_error < -WALK_HEAD_H_TOL:
                    self.sendContinuousValue(*RIGHT_CORRECT)
                    self.action_detail = (
                        f'確認球：球偏右，右轉修正 head_error={head_error}'
                    )

                elif self.head_v > BALL_REACH_HEAD_V_MAX:
                    self.sendContinuousValue(*CONFIRM_FORWARD_MOVE)
                    self.action_detail = (
                        f'確認球：球太遠，慢慢前進 head_v={self.head_v}'
                    )

                elif self.head_v < BALL_REACH_HEAD_V_MIN:
                    self.sendContinuousValue(*CONFIRM_BACKWARD_MOVE)
                    self.action_detail = (
                        f'確認球：球太近，慢慢後退 head_v={self.head_v}'
                    )

                else:
                    self.sendContinuousValue(*STOP_MOVE)
                    self.action_detail = (
                        f'確認球：等待修正 head_v={self.head_v}'
                    )

        else:
            self.confirm_ball_count = 0
            self.confirm_ball_lost_count += 1

            self.sendContinuousValue(*STOP_MOVE)
            self.sendbodyAuto(0)

            self.head_h = HEAD_H_CENTER
            self.head_v = HEAD_CONFIRM_BALL_V
            self.sendHeadMotor(1, self.head_h, HEAD_SPEED)
            self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

            if self.confirm_ball_lost_count > BALL_LOST_FRAMES:
                self.found_ball_count = 0
                self.ball_lost_count = 0
                self.state = 'find_ball'
                self.action_detail = '確認球失敗，回到 find_ball'
                return

            self.action_detail = (
                f'確認球時看不到球 '
                f'{self.confirm_ball_lost_count}/{BALL_LOST_FRAMES}'
            )

    # 狀態：射門 shoot
    def _state_shoot(self):
        self._stop_walk()

        print("\n########################################")
        print("############### SHOOT ##################")
        print("########################################")

        if KICK_FOOT == 'right':
            self.sendBodySector(RIGHT_KICK_SECTOR)
            self.action_detail = '執行右腳射門 sector=100'
        else:
            self.sendBodySector(LEFT_KICK_SECTOR)
            self.action_detail = '執行左腳射門 sector=200'

        self.state = 'finish'

    def _state_finish(self):
        self._stop_walk()
        self.action_detail = '射門完成，策略結束'

    def main(self):
        if not self.is_start:
            self._stop_walk()
            return

        # ball.update 只讀影像資料，不控制頭部
        # orbit 階段不會呼叫 _track_object(ball)，因此頭不會被球拉走
        self.ball.update()

        if self.state == 'find_ball':
            self._state_find_ball()

        elif self.state == 'walk_to_ball':
            self._state_walk_to_ball()

        elif self.state == 'find_obstacle':
            self._state_find_obstacle()

        elif self.state == 'orbit_ball_align':
            self._state_orbit_ball_align()

        elif self.state == 'confirm_ball':
            self._state_confirm_ball()

        elif self.state == 'shoot':
            self._state_shoot()

        elif self.state == 'finish':
            self._state_finish()


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
