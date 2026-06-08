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

IMG_W  = 320
IMG_H  = 240
IMG_CX = IMG_W // 2
IMG_CY = IMG_H // 2

# --- 移動參數 ---
STOP_MOVE     = [0, 0, 0]
LEFT_CORRECT  = [0, 0, 4]
RIGHT_CORRECT = [50, 0, -4]
FORWARD_MOVE  = [1500, -100, -1]

# --- 頭部馬達範圍 ---
HEAD_H_CENTER = 2048
HEAD_V_CENTER = 1600
HEAD_H_MAX    = 3072
HEAD_H_MIN    = 1024
HEAD_V_MAX    = 2048
HEAD_V_MIN    = 1200
HEAD_V_FOOT   = 1250
HEAD_SPEED    = 30

# --- 頭部追蹤 P 控制器 ---
HEAD_KP_H = 1.0
HEAD_KP_V = 1.0

HEAD_MAX_STEP_H = 80
HEAD_MAX_STEP_V = 60

HEAD_TOL_X = 12
HEAD_TOL_Y = 20

# --- 搜尋參數 ---
HEAD_SEARCH_STEP = 60
SEARCH_LEVELS = [1450, 1650, 1850]

# --- 球判定 ---
BALL_LOST_FRAMES  = 8
BALL_FOUND_FRAMES = 5

BALL_MIN_AREA       = 80
BALL_ASPECT_MIN     = 0.5
BALL_ASPECT_MAX     = 2.0
BALL_MIN_Y_CENTROID = 60

# --- 走向球參數 ---
WALK_HEAD_H_TOL  = 120
WALK_FORWARD_TOL = 80

# --- 射門位置判斷 ---
BALL_REACH_HEAD_V_MIN = 1230
BALL_REACH_HEAD_V_MAX = 1350
BALL_REACH_FRAMES     = 5


# ===========================================================================
# 視覺：球
# ===========================================================================

class BallInfo:
    def __init__(self, api: API):
        self.api     = api
        self.visible = False
        self.cx      = 0
        self.cy      = 0
        self.area    = 0
        self.aspect  = 0.0

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
        self.cx      = best['centroid'][0]
        self.cy      = best['centroid'][1]
        self.area    = best['area']
        self.aspect  = best['aspect_ratio']


# ===========================================================================
# Debug 印表
# ===========================================================================

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
                    f"#========== United Soccer — find / walk ball ==========#\n"
                    f" is_start        : {n.is_start}\n"
                    f" state           : {n.state}\n"
                    f" action_detail   : {n.action_detail}\n"
                    f"#================== 視覺狀態 =======================#\n"
                    f" ball.visible    : {n.ball.visible}\n"
                    f" ball.lost       : {n.ball_lost_count}/{BALL_LOST_FRAMES}\n"
                    f" ball.found      : {n.found_ball_count}/{BALL_FOUND_FRAMES}\n"
                    f" ball.reach      : {n.reach_ball_count}/{BALL_REACH_FRAMES}\n"
                    f" ball.cx / cy    : {n.ball.cx} / {n.ball.cy}\n"
                    f" ball.area       : {n.ball.area}\n"
                    f" ball.aspect     : {n.ball.aspect:.2f}\n"
                    f"#================== 頭部狀態 =======================#\n"
                    f" head_h          : {n.head_h}  center={HEAD_H_CENTER}\n"
                    f" head_v          : {n.head_v}  shoot={BALL_REACH_HEAD_V_MIN}~{BALL_REACH_HEAD_V_MAX}\n"
                    f" search_dir      : {n.search_dir}\n"
                    f" search_count    : {n.search_count}\n"
                    f" search_level    : {n.search_level} / {SEARCH_LEVELS[n.search_level]}\n"
                    f"#====================================================#\n"
                )
                sys.stdout.flush()
            except Exception:
                pass

            time.sleep(0.1)


# ===========================================================================
# 主策略節點
# ===========================================================================

class UnitedSoccer(API):
    def __init__(self):
        super().__init__('us_v2_walk_ball')

        self.ball = BallInfo(self)

        self.head_h = HEAD_H_CENTER
        self.head_v = HEAD_V_CENTER

        self.search_dir = 'right'
        self.search_count = 0
        self.search_level = 0

        self.ball_lost_count = 0
        self.found_ball_count = 0
        self.reach_ball_count = 0

        self.state = 'find_ball'
        self.action_detail = '初始化'

        self._printer = StatusPrinter(self)
        self._printer.start()

        self._reset_head()

        self.create_timer(0.1, self.main)

    # -----------------------------------------------------------------------
    # 頭部控制
    # -----------------------------------------------------------------------

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

            # 球在畫面下方 err_y > 0，頭要往下看，所以 head_v 減少
            self.head_v -= step_v

            self.head_v = max(HEAD_V_MIN, min(HEAD_V_MAX, self.head_v))
            self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

        return centered_x and centered_y

    def _search_head(self):
        self.head_v = SEARCH_LEVELS[self.search_level]
        self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

        if self.search_dir == 'right':
            self.head_h -= HEAD_SEARCH_STEP

            if self.head_h <= HEAD_H_MIN:
                self.head_h = HEAD_H_MIN
                self.search_dir = 'left'
                self.search_count += 1

                self.search_level += 1
                if self.search_level >= len(SEARCH_LEVELS):
                    self.search_level = 0

        else:
            self.head_h += HEAD_SEARCH_STEP

            if self.head_h >= HEAD_H_MAX:
                self.head_h = HEAD_H_MAX
                self.search_dir = 'right'
                self.search_count += 1

                self.search_level += 1
                if self.search_level >= len(SEARCH_LEVELS):
                    self.search_level = 0

        self.sendHeadMotor(1, self.head_h, HEAD_SPEED)

    # -----------------------------------------------------------------------
    # find_ball
    # -----------------------------------------------------------------------

    def _state_find_ball(self):
        if self.ball.visible:
            self.ball_lost_count = 0
            self.found_ball_count += 1

            centered = self._track_object(self.ball.cx, self.ball.cy)

            if self.found_ball_count >= BALL_FOUND_FRAMES:
                self.state = 'walk_to_ball'
                self.reach_ball_count = 0
                self.action_detail = '連續找到球 5 次，切換到 walk_to_ball'
                return

            if centered:
                self.action_detail = (
                    f'球對準中心 found={self.found_ball_count}/{BALL_FOUND_FRAMES} '
                    f'area={self.ball.area}'
                )
            else:
                self.action_detail = (
                    f'追蹤球中 cx={self.ball.cx} cy={self.ball.cy} '
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
                self._search_head()
                self.action_detail = (
                    f'掃描中 dir={self.search_dir} '
                    f'count={self.search_count} '
                    f'head_h={self.head_h}'
                )

    # -----------------------------------------------------------------------
    # walk_to_ball
    # -----------------------------------------------------------------------

    def _state_walk_to_ball(self):
        if self.ball.visible:
            self.ball_lost_count = 0

            # 持續鎖球
            self._track_object(self.ball.cx, self.ball.cy)

            # -------------------------------------------------
            # 射門位置判斷：head_v 連續 5 次在範圍內才停止
            # -------------------------------------------------
            if BALL_REACH_HEAD_V_MIN <= self.head_v <= BALL_REACH_HEAD_V_MAX:
                self.reach_ball_count += 1

                if self.reach_ball_count >= BALL_REACH_FRAMES:
                    self.sendContinuousValue(*STOP_MOVE)
                    self.sendbodyAuto(0)

                    print("\n")
                    print("########################################")
                    print("######### 到達射門位置 #########")
                    print("########################################")
                    print(f"head_v = {self.head_v}")
                    print(f"area   = {self.ball.area}")
                    print("\n")

                    self.action_detail = (
                        f'到達射門位置 head_v={self.head_v} '
                        f'count={self.reach_ball_count}/{BALL_REACH_FRAMES}'
                    )
                    return

            else:
                self.reach_ball_count = 0

            head_error = self.head_h - HEAD_H_CENTER

            self.sendbodyAuto(1)

            if head_error > WALK_HEAD_H_TOL:
                self.sendContinuousValue(*LEFT_CORRECT)
                self.action_detail = (
                    f'走向球：左轉修正 head_error={head_error} '
                    f'reach={self.reach_ball_count}/{BALL_REACH_FRAMES}'
                )

            elif head_error < -WALK_HEAD_H_TOL:
                self.sendContinuousValue(*RIGHT_CORRECT)
                self.action_detail = (
                    f'走向球：右轉修正 head_error={head_error} '
                    f'reach={self.reach_ball_count}/{BALL_REACH_FRAMES}'
                )

            elif abs(head_error) < WALK_FORWARD_TOL:
                self.sendContinuousValue(*FORWARD_MOVE)
                self.action_detail = (
                    f'走向球：前進 head_error={head_error} '
                    f'head_v={self.head_v} '
                    f'reach={self.reach_ball_count}/{BALL_REACH_FRAMES}'
                )

            else:
                self.sendContinuousValue(*STOP_MOVE)
                self.action_detail = (
                    f'走向球：等待對正 head_error={head_error} '
                    f'reach={self.reach_ball_count}/{BALL_REACH_FRAMES}'
                )

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

    # -----------------------------------------------------------------------
    # 主迴圈
    # -----------------------------------------------------------------------

    def main(self):
        if not self.is_start:
            self._stop_walk()
            return

        self.ball.update()

        if self.state == 'find_ball':
            self._state_find_ball()

        elif self.state == 'walk_to_ball':
            self._state_walk_to_ball()


# ===========================================================================
# 進入點
# ===========================================================================

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