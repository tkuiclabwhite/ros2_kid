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


#小白
# # 步態
# STOP_MOVE = [-600, -400, 0]
# LEFT_CORRECT = [-900, -970, 3]
# RIGHT_CORRECT = [-900, 970, -7]
# FORWARD_MOVE = [1500, -100, -1]
# BACKWARD_MOVE = [-1000, 0, 0] 

# CONFIRM_FORWARD_MOVE  = [500, 0, 0]    # 確認球時慢慢前進
# CONFIRM_BACKWARD_MOVE = [-700, 0, 0]   # 確認球時慢慢後退
# CONFIRM_LEFT_MOVE  = [-900, -670, 3]    
# CONFIRM_RIGHT_MOVE = [-900, 670, -5]


# # 藍色障礙物身體微調：
# # 目標是讓機器人繞著球調整角度，讓身體朝向藍色障礙物
# # 右邊看到藍色 -> ORBIT_RIGHT_MOVE
# # 左邊看到藍色 -> ORBIT_LEFT_MOVE
# ORBIT_LEFT_MOVE  = [-900, -770, 2]
# ORBIT_RIGHT_MOVE = [-900, 770, -4]


#小黑
# 步態
STOP_MOVE = [-800, 200, 0]
LEFT_CORRECT = [-1100, -1000, 3]
RIGHT_CORRECT = [-1100, 1200, -2]
FORWARD_MOVE = [1500, 200, 0]
BACKWARD_MOVE = [-1000, 0, 0] 

CONFIRM_FORWARD_MOVE  = [500, 0, 0]    # 確認球時慢慢前進
CONFIRM_BACKWARD_MOVE = [-700, 0, 0]   # 確認球時慢慢後退
CONFIRM_RIGHT_MOVE  = [-1100, -500, 3]    
CONFIRM_LEFT_MOVE = [-900, 600, -2]


# 藍色障礙物身體微調：
# 目標是讓機器人繞著球調整角度，讓身體朝向藍色障礙物
# 右邊看到藍色 -> ORBIT_RIGHT_MOVE
# 左邊看到藍色 -> ORBIT_LEFT_MOVE
ORBIT_LEFT_MOVE  = [-1100, -800, 2]
ORBIT_RIGHT_MOVE = [-1100, 900, -2]



ORBIT_ALIGN_FRAMES = 5
ORBIT_OBSTACLE_LOST_FRAMES = 25 # orbit 時藍色短暫消失幾次才回 find_obstacle

OBSTACLE_BODY_TOL_X = 45  # orbit 時藍色在此範圍內就交給身體修正，不再一直追頭
OBSTACLE_HEAD_H_TOL = 80  # 繞球時，頭部水平刻度接近中心才代表身體朝向障礙物
OBSTACLE_HEAD_MAX_STEP_H = 30  # orbit 時頭部水平追藍色的最大步幅，避免跟身體修正互相拉扯
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
HEAD_TOL_X = 18  # 水平方向死區
HEAD_TOL_Y = 30  # 垂直方向死區

HEAD_SEARCH_STEP = 50  # 數值越大掃描越快，但可能略過目標，數值越小掃描越慢，但比較穩
SEARCH_LEVELS = [1450, 1650, 1850] # 搜尋球時使用的三層垂直角度

# 尋找藍色障礙物專用分層搜尋參數，不共用 find_ball 的參數
# 改成比較大範圍、比較慢，並且每一層掃完整個左右範圍才換下一層
OBSTACLE_SEARCH_STEP = 30
OBSTACLE_SEARCH_LEVELS = [1450, 1600, 1750, 1900, 2020]
OBSTACLE_EDGES_PER_LEVEL = 2       # 2 = 左右兩側都掃完，才換下一層

# confirm_ball 找不到球時專用分層搜尋參數，不共用 find_ball 的參數
# confirm 主要找近距離球，所以比 obstacle 更偏低頭，但也給足範圍
CONFIRM_SEARCH_STEP = 30
CONFIRM_SEARCH_LEVELS = [1250, 1400, 1550, 1700, 1850]
CONFIRM_EDGES_PER_LEVEL = 2        # 2 = 左右兩側都掃完，才換下一層
CONFIRM_BACKUP_FRAMES = 8          # confirm_ball 看不到球時，先後退幾次再開始搜尋
CONFIRM_BALL_LOST_FRAMES = 60      # confirm_ball 連續看不到球幾次才回 find_ball，拉長避免太快跳走

# 球
BALL_MIN_AREA = 80
BALL_ASPECT_MIN = 0.5 # 球長寬比下限
BALL_ASPECT_MAX = 2.0 #上限
BALL_MIN_Y_CENTROID = 60 # 球中心 y 座標下限
BALL_FOUND_FRAMES = 5 #連續看球幾次 才算對準
BALL_LOST_FRAMES = 20  #連續錯過幾次 才算真的lost ball

# 用球的面積 size 判斷距離，不再用 head_v 刻度判斷距離
BALL_SIZE_OK_MIN = 930    # 球太小，代表太遠，要前進
BALL_SIZE_OK_MAX = 1150   # 球太大，代表太近，要後退
BALL_SIZE_REACH_FRAMES = 5

# 走向球時的身體修正條件
WALK_HEAD_H_TOL = 60 # 頭部水平偏差超過此值時，代表身體還沒對準球需用左右旋轉修正身體方向
WALK_FORWARD_TOL = 60 # 頭部水平偏差小於此值時，代表球大致在身體前方，允許機器人往前走

BALL_REACH_HEAD_V_MIN = 1230 # 看球垂直刻度範圍
BALL_REACH_HEAD_V_MAX = 1600     #1350
BALL_REACH_FRAMES = 5

# 障礙物與射門前確認
HEAD_OBSTACLE_SEARCH_V = 1900
OBSTACLE_CENTER_FRAMES = 4

HEAD_CONFIRM_BALL_V = 1600
CONFIRM_BALL_FRAMES = 4
CONFIRM_BALL_BEARING_TOL = 80  # [新增] 球相對於障礙物對正方向的允許水平誤差  


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
                    f" ball.size.ok    : {BALL_SIZE_OK_MIN} ~ {BALL_SIZE_OK_MAX}\n"
                    f" ball.aspect     : {n.ball.aspect:.2f}\n"

                    f"#================== 藍色障礙物 =======================#\n"
                    f" obstacle.visible: {n.obstacle.visible}\n"
                    f" obstacle.center : {n.obstacle_center_count}/{OBSTACLE_CENTER_FRAMES}\n"
                    f" orbit.align     : {n.orbit_align_count}/{ORBIT_ALIGN_FRAMES}\n"
                    f" orbit.lost      : {n.orbit_obstacle_lost_count}/{ORBIT_OBSTACLE_LOST_FRAMES}\n"
                    f" orbit.head_lock : {n.orbit_head_locked}\n"
                    f" obstacle.cx/cy  : {n.obstacle.cx} / {n.obstacle.cy}\n"
                    f" obstacle.headErr: {n.head_h - HEAD_H_CENTER} / {OBSTACLE_HEAD_H_TOL}\n"
                    f" obstacle.area   : {n.obstacle.area}\n"
                    f" obstacle.aspect : {n.obstacle.aspect:.2f}\n"

                    f"#================== 頭部狀態 =======================#\n"
                    f" head_h          : {n.head_h}  center={HEAD_H_CENTER}\n"
                    f" head_v          : {n.head_v}\n"
                    f" kick_foot       : {KICK_FOOT}\n"
                    f" search_dir      : {n.search_dir}\n"
                    f" search_count    : {n.search_count}\n"
                    f" search_level    : {n.search_level} / {SEARCH_LEVELS[n.search_level]}\n"
                    f" obstacle.search : {n.obstacle_search_dir} level={n.obstacle_search_level} / {OBSTACLE_SEARCH_LEVELS[n.obstacle_search_level]}\n"
                    f" confirm.search  : {n.confirm_search_dir} level={n.confirm_search_level} / {CONFIRM_SEARCH_LEVELS[n.confirm_search_level]}\n"
                    f" confirm.base_h  : {n.confirm_base_head_h}\n"
                    f" confirm.seen_h  : {n.confirm_ball_seen_head_h}\n"
                    f" confirm.px_err  : {n.confirm_ball_pixel_error}\n"
                    f" confirm.bearing : {n.confirm_ball_bearing_error} / {CONFIRM_BALL_BEARING_TOL}\n"
                    f"#====================================================#\n"
                )
                sys.stdout.flush()
            except Exception:
                pass

            time.sleep(0.1)


class UnitedSoccer(API):
    def __init__(self):
        super().__init__('us_full_strategy_layered_search')

        self.ball = BallInfo(self)
        self.obstacle = ObstacleInfo(self)

        self.head_h = HEAD_H_CENTER
        self.head_v = HEAD_V_CENTER

        self.search_dir = 'right'
        self.search_count = 0
        self.search_level = 0

        # 尋找藍色障礙物專用搜尋狀態
        self.obstacle_search_dir = 'right'
        self.obstacle_search_count = 0
        self.obstacle_search_level = 0

        # confirm_ball 找不到球時專用搜尋狀態
        self.confirm_search_dir = 'right'
        self.confirm_search_count = 0
        self.confirm_search_level = 0

        self.ball_lost_count = 0
        self.found_ball_count = 0
        self.reach_ball_count = 0
        self.confirm_ball_count = 0
        self.confirm_ball_lost_count = 0
        self.confirm_base_head_h = HEAD_H_CENTER  # [新增] 障礙物對正完成當下的頭部水平基準
        self.confirm_base_head_v = HEAD_CONFIRM_BALL_V  # [新增] 進入重新找球時的垂直基準
        self.confirm_ball_bearing_error = 0  # [新增] 球相對於障礙物對正方向的水平誤差
        self.confirm_ball_seen_head_h = HEAD_H_CENTER  # [新增] 找到球當下的頭部水平位置
        self.confirm_ball_pixel_error = 0  # [新增] 找到球時，球在畫面中的水平偏差
        self.obstacle_center_count = 0

        # 繞球對準計數器
        self.orbit_align_count = 0
        self.orbit_obstacle_lost_count = 0

        # orbit 階段視覺鎖定旗標：避免頭又回去追球，鎖定後改追蹤藍色
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

        self.obstacle_search_dir = 'right'
        self.obstacle_search_count = 0
        self.obstacle_search_level = 0

        self.confirm_search_dir = 'right'
        self.confirm_search_count = 0
        self.confirm_search_level = 0

    # 停止步態
    def _stop_walk(self):
        self.sendContinuousValue(*STOP_MOVE)
        self.sendbodyAuto(0)

    # 頭部追蹤控制器
    def _track_object(
        self,
        cx,
        cy,
        tol_x=HEAD_TOL_X,
        tol_y=HEAD_TOL_Y,
        max_step_h=HEAD_MAX_STEP_H,
        max_step_v=HEAD_MAX_STEP_V,
    ):
        err_x = cx - IMG_CX
        err_y = cy - IMG_CY

        centered_x = abs(err_x) < tol_x
        centered_y = abs(err_y) < tol_y

        if not centered_x:
            step_h = int(err_x * HEAD_KP_H)
            step_h = max(-max_step_h, min(max_step_h, step_h))

            self.head_h -= step_h
            self.head_h = max(HEAD_H_MIN, min(HEAD_H_MAX, self.head_h))
            self.sendHeadMotor(1, self.head_h, HEAD_SPEED)

        if not centered_y:
            step_v = int(err_y * HEAD_KP_V)
            step_v = max(-max_step_v, min(max_step_v, step_v))

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

    # 搜尋藍色障礙物：使用障礙物專用分層參數
    def _search_head_for_obstacle(self):
        # 每一層先固定 head_v，頭只做水平掃描
        self.head_v = OBSTACLE_SEARCH_LEVELS[self.obstacle_search_level]
        self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

        if self.obstacle_search_dir == 'right':
            self.head_h -= OBSTACLE_SEARCH_STEP

            if self.head_h <= HEAD_H_MIN:
                self.head_h = HEAD_H_MIN
                self.obstacle_search_dir = 'left'
                self.obstacle_search_count += 1

                # 重點：不要碰到邊界就立刻換層
                # 要左右兩側都掃過，才換下一個垂直層
                if self.obstacle_search_count % OBSTACLE_EDGES_PER_LEVEL == 0:
                    self.obstacle_search_level = (
                        self.obstacle_search_level + 1
                    ) % len(OBSTACLE_SEARCH_LEVELS)

        else:
            self.head_h += OBSTACLE_SEARCH_STEP

            if self.head_h >= HEAD_H_MAX:
                self.head_h = HEAD_H_MAX
                self.obstacle_search_dir = 'right'
                self.obstacle_search_count += 1

                if self.obstacle_search_count % OBSTACLE_EDGES_PER_LEVEL == 0:
                    self.obstacle_search_level = (
                        self.obstacle_search_level + 1
                    ) % len(OBSTACLE_SEARCH_LEVELS)

        self.sendHeadMotor(1, self.head_h, HEAD_SPEED)

    # confirm_ball 找不到球時：使用確認球專用分層搜尋參數
    def _search_head_for_confirm_ball(self):
        # 每一層固定 head_v，先完整掃左右，再換下一層
        self.head_v = CONFIRM_SEARCH_LEVELS[self.confirm_search_level]
        self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

        if self.confirm_search_dir == 'right':
            self.head_h -= CONFIRM_SEARCH_STEP

            if self.head_h <= HEAD_H_MIN:
                self.head_h = HEAD_H_MIN
                self.confirm_search_dir = 'left'
                self.confirm_search_count += 1

                if self.confirm_search_count % CONFIRM_EDGES_PER_LEVEL == 0:
                    self.confirm_search_level = (
                        self.confirm_search_level + 1
                    ) % len(CONFIRM_SEARCH_LEVELS)

        else:
            self.head_h += CONFIRM_SEARCH_STEP

            if self.head_h >= HEAD_H_MAX:
                self.head_h = HEAD_H_MAX
                self.confirm_search_dir = 'right'
                self.confirm_search_count += 1

                if self.confirm_search_count % CONFIRM_EDGES_PER_LEVEL == 0:
                    self.confirm_search_level = (
                        self.confirm_search_level + 1
                    ) % len(CONFIRM_SEARCH_LEVELS)

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

            head_error = self.head_h - HEAD_H_CENTER

            # ===== 用球的 size / area + 身體角度判斷射門位置 =====
            ball_size_ok = BALL_SIZE_OK_MIN <= self.ball.area <= BALL_SIZE_OK_MAX
            body_aligned = abs(head_error) <= WALK_HEAD_H_TOL

            if ball_size_ok and body_aligned:
                self.reach_ball_count += 1

                if self.reach_ball_count >= BALL_SIZE_REACH_FRAMES:
                    self._stop_walk()

                    print("\n########################################")
                    print("######### 到達射門位置(size判斷) #########")
                    print("########################################")
                    print(f"ball.area = {self.ball.area}")
                    print(f"head_v    = {self.head_v}\n")

                    self.head_h = HEAD_H_CENTER
                    self.head_v = HEAD_OBSTACLE_SEARCH_V
                    self.sendHeadMotor(1, self.head_h, HEAD_SPEED)
                    self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

                    self.obstacle_center_count = 0
                    self.orbit_align_count = 0
                    self.orbit_obstacle_lost_count = 0
                    self.orbit_head_locked = False

                    # 重置尋找障礙物專用分層搜尋
                    self.obstacle_search_dir = 'right'
                    self.obstacle_search_count = 0
                    self.obstacle_search_level = 0

                    self.state = 'find_obstacle'
                    self.action_detail = '球size到達射門位置，抬頭找藍色障礙物'
                    return
            else:
                self.reach_ball_count = 0

            self.sendbodyAuto(1)

            # 先修左右，避免還沒正對球就前後亂動
            if head_error > WALK_HEAD_H_TOL:
                self.sendContinuousValue(*LEFT_CORRECT)
                self.action_detail = f'走向球：左轉 head_error={head_error} area={self.ball.area}'

            elif head_error < -WALK_HEAD_H_TOL:
                self.sendContinuousValue(*RIGHT_CORRECT)
                self.action_detail = f'走向球：右轉 head_error={head_error} area={self.ball.area}'

            elif self.ball.area > BALL_SIZE_OK_MAX:
                self.sendContinuousValue(*BACKWARD_MOVE)
                self.action_detail = (
                    f'走向球：球太大，後退 area={self.ball.area} '
                    f'OK={BALL_SIZE_OK_MIN}~{BALL_SIZE_OK_MAX}'
                )

            elif self.ball.area < BALL_SIZE_OK_MIN and abs(head_error) < WALK_FORWARD_TOL:
                self.sendContinuousValue(*FORWARD_MOVE)
                self.action_detail = (
                    f'走向球：球太小，前進 area={self.ball.area} '
                    f'OK={BALL_SIZE_OK_MIN}~{BALL_SIZE_OK_MAX}'
                )

            else:
                self.sendContinuousValue(*STOP_MOVE)
                self.action_detail = (
                    f'走向球：等待對正 head_error={head_error} '
                    f'body_aligned={body_aligned} area={self.ball.area}'
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

    # 狀態：尋找藍色障礙物 find_obstacle
    def _state_find_obstacle(self):
        """
        找藍色障礙物 + 繞球對準合併在同一個 state。

        orbit_head_locked = False：還在用障礙物專用分層搜尋找藍色。
        orbit_head_locked = True ：已看到藍色，頭持續追蹤藍色，身體依頭部偏差繞球修正。
        """
        self.obstacle.update()

        # =====================================================
        # 1. 還沒鎖頭：用障礙物專用分層搜尋找藍色障礙物
        # =====================================================
        if not self.orbit_head_locked:
            self._stop_walk()

            if self.obstacle.visible:
                print("\n########################################")
                print("######### 藍色障礙物已看到 #########")
                print("########################################")
                print(f"obstacle_x = {self.obstacle.cx}")
                print(f"obstacle_y = {self.obstacle.cy}")
                print(f"area       = {self.obstacle.area}\n")

                # 看到藍色後進入視覺鎖定。後續仍持續追蹤藍色，
                # 讓身體繞球直到 head_h 回到中心附近。

                self.sendHeadMotor(1, self.head_h, HEAD_SPEED)
                self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

                self.obstacle_center_count = 0
                self.orbit_align_count = 0
                self.orbit_obstacle_lost_count = 0
                self.orbit_head_locked = True

                self.action_detail = '看到藍色障礙物，進入追蹤並用頭部偏差繞球對準'
                return

            self.obstacle_center_count = 0
            self.orbit_align_count = 0
            self.orbit_obstacle_lost_count = 0
            self._search_head_for_obstacle()
            self.action_detail = (
                f'障礙物分層搜尋中 '
                f'dir={self.obstacle_search_dir} '
                f'level={self.obstacle_search_level} '
                f'edge={self.obstacle_search_count}/{OBSTACLE_EDGES_PER_LEVEL} '
                f'head_h={self.head_h} head_v={self.head_v}'
            )
            return

        # =====================================================
        # 2. 已鎖頭：在 find_obstacle 內直接用身體繞球修正
        # =====================================================
        if not self.obstacle.visible:
            self.orbit_obstacle_lost_count += 1
            self.orbit_align_count = 0

            # 藍色短暫消失時，不切 state，不掃頭
            self.sendContinuousValue(*STOP_MOVE)
            self.sendbodyAuto(0)


            if self.orbit_obstacle_lost_count > ORBIT_OBSTACLE_LOST_FRAMES:
                # 真的太久找不到藍色，才解除鎖頭重新搜尋
                self.orbit_head_locked = False
                self.orbit_align_count = 0
                self.orbit_obstacle_lost_count = 0
                self.obstacle_search_dir = 'right'
                self.obstacle_search_count = 0
                self.obstacle_search_level = 0
                self.action_detail = '藍色連續消失太久，解除鎖頭重新分層搜尋藍色'
                return

            self.action_detail = (
                f'find_obstacle內繞球：藍色短暫消失，頭部保持鎖定 '
                f'{self.orbit_obstacle_lost_count}/{ORBIT_OBSTACLE_LOST_FRAMES}'
            )
            return

        self.orbit_obstacle_lost_count = 0
        obstacle_pixel_error = self.obstacle.cx - IMG_CX
        obstacle_centered = abs(obstacle_pixel_error) <= OBSTACLE_BODY_TOL_X

        # 鎖定後仍追蹤藍色。若只看 obstacle.cx，頭轉偏時會誤判身體已對準；
        # 所以身體繞球方向改用 head_h 相對中心的偏差判斷。
        self._track_object(
            self.obstacle.cx,
            self.obstacle.cy,
            tol_x=OBSTACLE_BODY_TOL_X,
            max_step_h=OBSTACLE_HEAD_MAX_STEP_H,
        )
        head_error = self.head_h - HEAD_H_CENTER

        if head_error < -OBSTACLE_HEAD_H_TOL:
            self.sendbodyAuto(1)
            self.sendContinuousValue(*ORBIT_RIGHT_MOVE)
            self.orbit_align_count = 0
            self.action_detail = (
                f'find_obstacle內繞球右修正 head_error={head_error} '
                f'obstacle_pixel_error={obstacle_pixel_error}'
            )

        elif head_error > OBSTACLE_HEAD_H_TOL:
            self.sendbodyAuto(1)
            self.sendContinuousValue(*ORBIT_LEFT_MOVE)
            self.orbit_align_count = 0
            self.action_detail = (
                f'find_obstacle內繞球左修正 head_error={head_error} '
                f'obstacle_pixel_error={obstacle_pixel_error}'
            )

        elif not obstacle_centered:
            self.sendContinuousValue(*STOP_MOVE)
            self.sendbodyAuto(0)
            self.orbit_align_count = 0
            self.action_detail = (
                f'find_obstacle內頭部追蹤藍色中 '
                f'head_error={head_error} '
                f'obstacle_pixel_error={obstacle_pixel_error}'
            )

        else:
            self.sendContinuousValue(*STOP_MOVE)
            self.sendbodyAuto(0)
            self.orbit_align_count += 1
            self.action_detail = (
                f'find_obstacle內繞球對準完成中 '
                f'{self.orbit_align_count}/{ORBIT_ALIGN_FRAMES} '
                f'head_error={head_error} '
                f'obstacle_pixel_error={obstacle_pixel_error}'
            )

            if self.orbit_align_count >= ORBIT_ALIGN_FRAMES:
                print("\n########################################")
                print("######### 繞球對準障礙物完成 #########")
                print("########################################")
                print(f"obstacle_x    = {self.obstacle.cx}")
                print(f"head_error    = {head_error}")
                print(f"pixel_error   = {obstacle_pixel_error}")
                print(f"area          = {self.obstacle.area}\n")

                self._stop_walk()

                # [新增] 記錄障礙物對正完成當下的頭部位置，當作射門方向基準。
                # [新增] 之後頭可以掃球，但只用來量測球相對於這個基準偏多少。
                self.confirm_base_head_h = self.head_h
                self.confirm_base_head_v = self.head_v
                self.confirm_ball_bearing_error = 0
                self.confirm_ball_seen_head_h = self.head_h
                self.confirm_ball_pixel_error = 0

                # [新增] 低頭找球時，水平位置先保留障礙物對正完成時的頭位。
                self.head_h = self.confirm_base_head_h
                self.head_v = HEAD_CONFIRM_BALL_V
                self.sendHeadMotor(1, self.head_h, HEAD_SPEED)
                self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

                self.confirm_ball_count = 0
                self.confirm_ball_lost_count = 0

                # 重置 confirm_ball 專用分層搜尋
                self.confirm_search_dir = 'right'
                self.confirm_search_count = 0
                self.confirm_search_level = 0

                self.orbit_head_locked = False
                self.orbit_align_count = 0
                self.orbit_obstacle_lost_count = 0

                self.state = 'confirm_ball'
                self.action_detail = '障礙物已對正，記錄頭部基準後開始重新找球'
                return

    # 狀態：確認球 confirm_ball
    def _state_confirm_ball(self):
        if self.ball.visible:
            self.confirm_ball_lost_count = 0

            # [新增] 這裡不再呼叫 _track_object(ball)，避免頭追球中心後把身體方向帶歪。
            # [新增] 頭部現在只當成量測器：用「掃到球時的頭位」+「球在畫面內偏差」
            # [新增] 算出球相對於障礙物對正基準的偏差。
            self.confirm_ball_seen_head_h = self.head_h
            self.confirm_ball_pixel_error = self.ball.cx - IMG_CX
            ball_bearing_h = (self.confirm_ball_seen_head_h- self.confirm_ball_pixel_error)
            self.confirm_ball_bearing_error = (ball_bearing_h - self.confirm_base_head_h )

            # [新增] bearing_ok 代表球已經落在剛剛障礙物對正方向附近。
            ball_size_ok = BALL_SIZE_OK_MIN <= self.ball.area <= BALL_SIZE_OK_MAX
            bearing_ok = (
                abs(self.confirm_ball_bearing_error)
                <= CONFIRM_BALL_BEARING_TOL
            )

            if ball_size_ok and bearing_ok:
                self.sendContinuousValue(*STOP_MOVE)
                self.sendbodyAuto(0)

                self.confirm_ball_count += 1

                if self.confirm_ball_count >= CONFIRM_BALL_FRAMES:
                    print("\n########################################")
                    print("######### 射門前球位置確認完成(方位+size) #########")
                    print("########################################")
                    print(f"base_head_h  = {self.confirm_base_head_h}")
                    print(f"seen_head_h  = {self.confirm_ball_seen_head_h}")
                    print(f"ball_px_err  = {self.confirm_ball_pixel_error}")
                    print(f"bearing_err  = {self.confirm_ball_bearing_error}")
                    print(f"ball.area    = {self.ball.area}")
                    print(f"head_v       = {self.head_v}\n")

                    self.state = 'shoot'
                    self.action_detail = '球相對障礙物方向確認完成，準備射門'
                    return

                self.action_detail = (
                    f'確認球方位中 {self.confirm_ball_count}/{CONFIRM_BALL_FRAMES} '
                    f'bearing_error={self.confirm_ball_bearing_error} '
                    f'area={self.ball.area}'
                )

            else:
                self.confirm_ball_count = 0
                self.sendbodyAuto(1)

                # [新增] 先依照「球相對於障礙物對正方向」修左右，再修前後距離。
                if self.confirm_ball_bearing_error > CONFIRM_BALL_BEARING_TOL:
                    self.sendContinuousValue(*CONFIRM_LEFT_MOVE)
                    self.action_detail = (
                        f'確認球：球相對射門方向偏左，修正位置 '
                        f'bearing_error={self.confirm_ball_bearing_error} '
                        f'base={self.confirm_base_head_h} '
                        f'seen={self.confirm_ball_seen_head_h} '
                        f'pixel={self.confirm_ball_pixel_error} '
                        f'area={self.ball.area}'
                    )

                elif self.confirm_ball_bearing_error < -CONFIRM_BALL_BEARING_TOL:
                    self.sendContinuousValue(*CONFIRM_RIGHT_MOVE)
                    self.action_detail = (
                        f'確認球：球相對射門方向偏右，修正位置 '
                        f'bearing_error={self.confirm_ball_bearing_error} '
                        f'base={self.confirm_base_head_h} '
                        f'seen={self.confirm_ball_seen_head_h} '
                        f'pixel={self.confirm_ball_pixel_error} '
                        f'area={self.ball.area}'
                    )

                elif self.ball.area < BALL_SIZE_OK_MIN:
                    self.sendContinuousValue(*CONFIRM_FORWARD_MOVE)
                    self.action_detail = (
                        f'確認球：方向OK但球太小，慢慢前進 area={self.ball.area} '
                        f'OK={BALL_SIZE_OK_MIN}~{BALL_SIZE_OK_MAX}'
                    )

                elif self.ball.area > BALL_SIZE_OK_MAX:
                    self.sendContinuousValue(*CONFIRM_BACKWARD_MOVE)
                    self.action_detail = (
                        f'確認球：方向OK但球太大，慢慢後退 area={self.ball.area} '
                        f'OK={BALL_SIZE_OK_MIN}~{BALL_SIZE_OK_MAX}'
                    )

                else:
                    self.sendContinuousValue(*STOP_MOVE)
                    self.action_detail = (
                        f'確認球：等待穩定 '
                        f'bearing_error={self.confirm_ball_bearing_error} '
                        f'bearing_ok={bearing_ok} area={self.ball.area}'
                    )

        else:
            self.confirm_ball_count = 0
            self.confirm_ball_lost_count += 1

            # 第一段：先往後退，讓球比較容易回到畫面內
            if self.confirm_ball_lost_count <= CONFIRM_BACKUP_FRAMES:
                self.sendbodyAuto(1)
                self.sendContinuousValue(*CONFIRM_BACKWARD_MOVE)

                # 第一次看不到球時，先把頭放到確認球搜尋的第一層

                self.action_detail = (
                    f'確認球時看不到球，先後退 '
                    f'{self.confirm_ball_lost_count}/{CONFIRM_BACKUP_FRAMES}'
                )
                return

            # 第二段：後退完成後，停止身體，使用 confirm_ball 專用分層搜尋找球
            if self.confirm_ball_lost_count <= CONFIRM_BALL_LOST_FRAMES:
                self.sendContinuousValue(*STOP_MOVE)
                self.sendbodyAuto(0)

                # [新增] 找不到球時允許頭分層掃描；掃到球後會拿掃到時的 head_h
                # [新增] 和前面記錄的 confirm_base_head_h 做相對誤差計算。
                self._search_head_for_confirm_ball()
                self.action_detail = (
                    f'確認球分層搜尋中 '
                    f'lost={self.confirm_ball_lost_count}/{CONFIRM_BALL_LOST_FRAMES} '
                    f'dir={self.confirm_search_dir} '
                    f'level={self.confirm_search_level} '
                    f'edge={self.confirm_search_count}/{CONFIRM_EDGES_PER_LEVEL} '
                    f'base_head_h={self.confirm_base_head_h} '
                    f'head_h={self.head_h} head_v={self.head_v}'
                )
                return

            # 第三段：真的太久找不到，才回 find_ball
            self.found_ball_count = 0
            self.ball_lost_count = 0
            self.confirm_ball_lost_count = 0
            self.confirm_search_dir = 'right'
            self.confirm_search_count = 0
            self.confirm_search_level = 0

            self.sendHeadMotor(1, self.head_h, HEAD_SPEED)
            self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

            self.state = 'find_ball'
            self.action_detail = '確認球分層搜尋失敗，回到 find_ball'
            return

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

        # orbit_ball_align 已合併進 find_obstacle，因此不再單獨切換 state。

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