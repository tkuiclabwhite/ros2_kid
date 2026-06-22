#!/usr/bin/env python3
# coding=utf-8

import sys
import time
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from strategy.API import API

# ===========================================================================
# 調參區
# ===========================================================================

COLOR_BALL = 'yellow'
COLOR_OBSTACLE = 'blue'

IMG_W  = 320
IMG_H  = 240
IMG_CX = IMG_W // 2
IMG_CY = IMG_H // 2

# 步態
STOP_MOVE = [0, 0, 0]
LEFT_CORRECT = [0, 0, 4]
RIGHT_CORRECT = [50, 0, -4]
FORWARD_MOVE = [1500, -100, -1]

# 新增：藍色障礙物身體微調
OBSTACLE_LEFT_CORRECT  = [0, 0, 2]
OBSTACLE_RIGHT_CORRECT = [0, 0, -2]
OBSTACLE_BODY_TOL_X = 5
OBSTACLE_ALIGN_FRAMES = 5

# --- 射門動作 ---
RIGHT_KICK_SECTOR = 100
LEFT_KICK_SECTOR  = 200
KICK_FOOT = 'right'

# --- 頭部馬達範圍 ---
HEAD_H_CENTER = 2048
HEAD_V_CENTER = 1600
HEAD_H_MAX = 3072
HEAD_H_MIN = 1024
HEAD_V_MAX = 2048
HEAD_V_MIN = 1200
HEAD_SPEED = 30

HEAD_KP_H = 1.0
HEAD_KP_V = 1.0
HEAD_MAX_STEP_H = 80
HEAD_MAX_STEP_V = 60
HEAD_TOL_X = 12
HEAD_TOL_Y = 20

HEAD_SEARCH_STEP = 60
SEARCH_LEVELS = [1450, 1650, 1850]

# --- 球 ---
BALL_MIN_AREA = 80
BALL_ASPECT_MIN = 0.5
BALL_ASPECT_MAX = 2.0
BALL_MIN_Y_CENTROID = 60
BALL_FOUND_FRAMES = 5
BALL_LOST_FRAMES = 8

WALK_HEAD_H_TOL = 120
WALK_FORWARD_TOL = 80

BALL_REACH_HEAD_V_MIN = 1230
BALL_REACH_HEAD_V_MAX = 1350
BALL_REACH_FRAMES = 5

# --- 障礙物與射門前確認 ---
HEAD_OBSTACLE_SEARCH_V = 1700
OBSTACLE_CENTER_FRAMES = 5

HEAD_CONFIRM_BALL_V = 1300
CONFIRM_BALL_FRAMES = 5


class BallInfo:
    def __init__(self, api: API):
        self.api = api
        self.visible = False
        self.cx = 0
        self.cy = 0
        self.area = 0
        self.aspect = 0.0

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

        best = max(objs, key=lambda o: o['area'])

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
                    f" obstacle.align  : {n.obstacle_align_count}/{OBSTACLE_ALIGN_FRAMES}\n"
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
        super().__init__('us_full_strategy_align')

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

        self.obstacle_center_count = 0
        self.obstacle_align_count = 0

        self.state = 'find_ball'
        self.action_detail = '初始化'

        self._printer = StatusPrinter(self)
        self._printer.start()

        self._reset_head()

        self.create_timer(0.1, self.main)

    def _clamp_head(self):
        self.head_h = max(HEAD_H_MIN, min(HEAD_H_MAX, self.head_h))
        self.head_v = max(HEAD_V_MIN, min(HEAD_V_MAX, self.head_v))

    def _send_head(self):
        self._clamp_head()
        self.sendHeadMotor(1, self.head_h, HEAD_SPEED)
        self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

    def _reset_head(self):
        self.head_h = HEAD_H_CENTER
        self.head_v = HEAD_V_CENTER
        self._send_head()

        self.search_dir = 'right'
        self.search_count = 0
        self.search_level = 0

    def _stop_walk(self):
        self.sendContinuousValue(*STOP_MOVE)
        self.sendbodyAuto(0)

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

    def _state_find_ball(self):
        if self.ball.visible:
            self.ball_lost_count = 0
            self.found_ball_count += 1

            centered = self._track_object(self.ball.cx, self.ball.cy)

            if self.found_ball_count >= BALL_FOUND_FRAMES:
                self.state = 'walk_to_ball'
                self.reach_ball_count = 0
                self.confirm_ball_count = 0
                self.obstacle_center_count = 0
                self.obstacle_align_count = 0
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

                    self.head_v = HEAD_OBSTACLE_SEARCH_V
                    self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

                    self.obstacle_center_count = 0
                    self.obstacle_align_count = 0
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

    def _state_find_obstacle(self):
        self._stop_walk()
        self.obstacle.update()

        if self.obstacle.visible:
            centered = self._track_object(self.obstacle.cx, self.obstacle.cy)

            if centered:
                self.obstacle_center_count += 1

                if self.obstacle_center_count >= OBSTACLE_CENTER_FRAMES:
                    print("\n########################################")
                    print("######### 藍色障礙物已看到 #########")
                    print("########################################")
                    print(f"obstacle_x = {self.obstacle.cx}")
                    print(f"obstacle_y = {self.obstacle.cy}")
                    print(f"area       = {self.obstacle.area}\n")

                    self.obstacle_align_count = 0
                    self.state = 'align_obstacle'
                    self.action_detail = '藍色障礙物已看到，開始用步態微調身體'
                    return

                self.action_detail = (
                    f'藍色障礙物頭部置中 '
                    f'{self.obstacle_center_count}/{OBSTACLE_CENTER_FRAMES}'
                )

            else:
                self.obstacle_center_count = 0
                self.action_detail = (
                    f'追蹤藍色障礙物 cx={self.obstacle.cx} '
                    f'cy={self.obstacle.cy}'
                )

        else:
            self.obstacle_center_count = 0
            self._search_head_for_obstacle()
            self.action_detail = (
                f'抬頭搜尋藍色障礙物 head_h={self.head_h} head_v={self.head_v}'
            )

    def _state_align_obstacle(self):
        self.obstacle.update()

        if not self.obstacle.visible:
            self._stop_walk()
            self.obstacle_align_count = 0
            self.state = 'find_obstacle'
            self.action_detail = '微調時看不到藍色障礙物，回到 find_obstacle'
            return

        error_x = self.obstacle.cx - IMG_CX

        self._track_object(self.obstacle.cx, self.obstacle.cy)

        if error_x > OBSTACLE_BODY_TOL_X:
            self.sendbodyAuto(1)
            self.sendContinuousValue(*OBSTACLE_RIGHT_CORRECT)
            self.obstacle_align_count = 0
            self.action_detail = f'障礙物在右，身體右轉微調 error_x={error_x}'

        elif error_x < -OBSTACLE_BODY_TOL_X:
            self.sendbodyAuto(1)
            self.sendContinuousValue(*OBSTACLE_LEFT_CORRECT)
            self.obstacle_align_count = 0
            self.action_detail = f'障礙物在左，身體左轉微調 error_x={error_x}'

        else:
            self.sendContinuousValue(*STOP_MOVE)
            self.sendbodyAuto(0)

            self.obstacle_align_count += 1
            self.action_detail = (
                f'身體已對準藍色障礙物 '
                f'{self.obstacle_align_count}/{OBSTACLE_ALIGN_FRAMES} '
                f'error_x={error_x}'
            )

            if self.obstacle_align_count >= OBSTACLE_ALIGN_FRAMES:
                print("\n########################################")
                print("######### 身體已對準藍色障礙物 #########")
                print("########################################")
                print(f"obstacle_x = {self.obstacle.cx}")
                print(f"error_x    = {error_x}")
                print(f"area       = {self.obstacle.area}\n")

                self._stop_walk()

                self.head_v = HEAD_CONFIRM_BALL_V
                self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

                self.confirm_ball_count = 0
                self.state = 'confirm_ball'
                self.action_detail = '身體對準障礙物完成，低頭再次確認球'

    def _state_confirm_ball(self):
        self._stop_walk()

        if self.ball.visible:
            self._track_object(self.ball.cx, self.ball.cy)

            if BALL_REACH_HEAD_V_MIN <= self.head_v <= BALL_REACH_HEAD_V_MAX:
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
                self.action_detail = f'球還沒回到射門位置 head_v={self.head_v}'

        else:
            self.confirm_ball_count = 0
            self.head_v = HEAD_CONFIRM_BALL_V
            self.sendHeadMotor(2, self.head_v, HEAD_SPEED)
            self.action_detail = '確認球時看不到球，低頭等待'

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

        self.ball.update()

        if self.state == 'find_ball':
            self._state_find_ball()

        elif self.state == 'walk_to_ball':
            self._state_walk_to_ball()

        elif self.state == 'find_obstacle':
            self._state_find_obstacle()

        elif self.state == 'align_obstacle':
            self._state_align_obstacle()

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