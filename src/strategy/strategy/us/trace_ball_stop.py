#!/usr/bin/env python3
# coding=utf-8
"""
United Soccer 策略 v2 — find_ball + approach_ball + (find_pole/find_goal) + adjust_position + kick
STRATEGY_MODE 決定 approach_ball 完成後走哪條分支，兩條分支共用 adjust_position/kick：
  'KICK_OBSTACLE' : find_pole → adjust_position → kick
  'SHOOT'         : find_goal → adjust_position → kick（共用，之後可拆出專屬射門動作）

狀態機:
  find_ball    : 頭部掃描搜尋球，對準中心連續 10 幀後進入下一狀態
  approach_ball:
    turn_to_ball : 身體旋轉對正球（頭 H 偏角歸零）
    walk_to_ball : 直走靠近，小幅 theta 修正，head_v <= APPROACH_STOP_V 停止
  find_pole:
    scan         : 分層水平掃描搜尋藍色球門柱，記錄找到時的頭部刻度
    return_head  : 找到後頭部回到球的位置，等待 RETURN_HEAD_WAIT_FRAMES 幀
  find_goal（SHOOT 策略）:
    scan         : 分層水平掃描搜尋紅色球門，V 層偏高方便抬頭看橫桿
    center_goal  : P-controller 對準球門，連續穩定幀數後記錄 goal_found_h/v
    return_head  : 找到後頭部回到球的位置，等待 RETURN_HEAD_WAIT_FRAMES 幀
  adjust_position:
    頭部固定在 ball_head_h/v，依 pole_found_h 或 goal_found_h 誤差做弧形繞球修正，
    ball.cy 控制 x 維持距離，達到目標幀數後進入下一狀態
"""
import sys
import time
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from strategy.API import API

# ===========================================================================
# 調參區
# ===========================================================================

# --- 策略開關 ---
# 'KICK_OBSTACLE' : 踢向障礙物（find_pole → adjust_position → kick）
# 'SHOOT'         : 射門（find_goal → ...，尚未實作）
STRATEGY_MODE = 'SHOOT'

COLOR_BALL = 'yellow'
COLOR_POLE = 'blue'
COLOR_GOAL = 'red'

IMG_W  = 320
IMG_H  = 240
IMG_CX = IMG_W // 2
IMG_CY = IMG_H // 2

# --- 頭部馬達 ---
HEAD_H_CENTER = 2048
HEAD_V_CENTER = 1600
HEAD_H_MAX    = 3026   # 往左 90 度
HEAD_H_MIN    = 1024   # 往右 90 度
HEAD_V_MAX    = 2200   # find_goal 抬頭看橫桿需要更高，已實測機械限位可達
HEAD_V_MIN    = 1150
HEAD_V_FOOT   = 1230   # 球到腳邊的 V 刻度
HEAD_SPEED    = 30

# 1 度對應的刻度數：(3026-1024) / 180 ≈ 11.1
HEAD_DEG_PER_TICK = (HEAD_H_MAX - HEAD_H_MIN) / 180.0   # ≈ 11.1

# --- 追蹤 P 控制器 ---
HEAD_KP_H       = 1.0
HEAD_KP_V       = 1.0
HEAD_MAX_STEP_H = 40
HEAD_MAX_STEP_V = 30
HEAD_TOL_X      = 8
HEAD_TOL_Y      = 12

# --- 搜尋 ---
HEAD_SEARCH_STEP_H  = 60
HEAD_SEARCH_V_LEVELS = [1300, 1450, 1600, 1750]

# --- find_pole 掃描 ---
POLE_SEARCH_V_LEVELS   = [1450, 1600, 1750, 1900, 2020]
POLE_SEARCH_STEP_H     = 60

# --- 球門柱過濾條件 ---
POLE_MIN_AREA    = 80
POLE_ASPECT_MIN  = 0.1   # 柱子是細長形，不限制比例
POLE_ASPECT_MAX  = 5.0

# --- find_pole: 頭部回球位 ---
POLE_CONFIRM_FRAMES     = 5    # 連續看到藍柱幾幀才算找到（可調）
RETURN_HEAD_WAIT_FRAMES = 15   # 等待馬達移動到位的幀數（0.1s × 15 ≈ 1.5s）

# --- adjust_position: 繞球軌道修正 ---
ORBIT_Y_LEFT      = 900   # 往左繞（orbit_dir= 1）側向步長（可調）
ORBIT_Y_RIGHT     = -800   # 往右繞（orbit_dir=-1）側向步長（可調）
ORBIT_THETA_LEFT  = -4     # 往左繞旋轉步長（可調）
ORBIT_THETA_RIGHT = 4     # 往右繞旋轉步長（可調）
ORBIT_V_GAIN      = 60.0   # head_v 偏差 → x 步長係數（可調）
ORBIT_X_MAX       = 900   # x 步長上限
ORBIT_TICK_GAIN   = 0.18  # pole_h 誤差刻度 → 目標幀數係數（可調，影響繞球總量）

# --- kick: 踢球動作 ---
KICK_WAIT_FRAMES  = 30   # 等待踢球動作完成的幀數（0.1s × 30 = 3s，可調）

# --- find_goal 掃描（SHOOT 策略） ---
GOAL_SEARCH_STEP_H   = 60
GOAL_SEARCH_V_LEVELS = [1700, 1850, 2000, 2200]   # 偏高，因為要抬頭看橫桿

# --- 球門過濾條件（形狀變化大，先只卡面積，不卡 aspect_ratio） ---
GOAL_MIN_AREA = 80

# --- find_goal: 對準確認 ---
GOAL_CONFIRM_FRAMES = 5   # 連續對準幾幀才算找到（可調）

# --- 球消失容忍 ---
BALL_LOST_FRAMES = 8

# --- 球過濾條件 ---
BALL_MIN_AREA       = 80
BALL_ASPECT_MIN     = 0.5
BALL_ASPECT_MAX     = 2.0
BALL_MIN_Y_CENTROID = 60

# --- find_ball → approach_ball 進入條件 ---
BALL_CENTERED_FRAMES = 10   # 對準中心連續幾幀才切換

# --- approach_ball: turn_to_ball ---
# 身體旋轉時，頭部 H 偏角小於此值視為「已對正」
TURN_DONE_DEG  = 5.0    # 度，可調整
# 旋轉速度：偏角大時快轉，偏角小時慢轉
TURN_THETA_FAST = 5     # 偏角 > 20 度時
TURN_THETA_MID  = 3     # 偏角 10~20 度時
TURN_THETA_SLOW = 1     # 偏角 < 10 度時

# --- approach_ball: walk_to_ball ---
APPROACH_STOP_V  = 1200   # head_v 到達此值視為夠靠近（等於 HEAD_V_FOOT）
WALK_X_NORMAL    = 1200    # 正常前進步長
WALK_X_SLOW      = 200    # 靠近時慢速步長
# 直走時小幅 theta 修正：頭 H 偏角換算後乘以此係數
WALK_THETA_GAIN  = 1.15   # 可調整，值越大修正越積極
WALK_THETA_MAX   = 5      # theta 修正上限，避免走太斜
# 直走時 theta 死區：head_h 偏離中心小於此刻度數時 theta=0，讓機器人安心前進
# 建議從 150 開始（約 13 度），走太斜再調小
WALK_THETA_DEAD_TICKS = 100
# 球夠近後停止 theta 修正，直接直走到底
# head_v 低於此值（球已很近）就強制 theta=0
WALK_THETA_STOP_V = 1600

# ===========================================================================
# 視覺：球
# ===========================================================================

class BallInfo:
    def __init__(self, api: API):
        self.api     = api
        self.visible = False
        self.cx = self.cy = self.area = 0
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
        self.cx     = best['centroid'][0]
        self.cy     = best['centroid'][1]
        self.area   = best['area']
        self.aspect = best['aspect_ratio']

# ===========================================================================
# 視覺：球門柱
# ===========================================================================

class PoleInfo:
    def __init__(self, api: API):
        self.api     = api
        self.visible = False
        self.cx = self.cy = self.area = 0
        self.aspect = 0.0
        self.candidate_count = 0

    def update(self):
        objs = self.api.get_objects(COLOR_POLE)
        if not objs:
            self.visible = False
            self.candidate_count = 0
            return
        candidates = [
            o for o in objs
            if o['area'] > POLE_MIN_AREA
            and POLE_ASPECT_MIN < o['aspect_ratio'] < POLE_ASPECT_MAX
        ]
        if not candidates:
            self.visible = False
            self.candidate_count = 0
            return

        # 場上同時看到多個藍色障礙物時，優先鎖定畫面中面積最大的目標。
        best = max(candidates, key=lambda o: o['area'])
        self.visible = True
        self.candidate_count = len(candidates)
        self.cx     = best['centroid'][0]
        self.cy     = best['centroid'][1]
        self.area   = best['area']
        self.aspect = best['aspect_ratio']

# ===========================================================================
# 視覺：球門（SHOOT 策略）
# ===========================================================================

class GoalInfo:
    def __init__(self, api: API):
        self.api     = api
        self.visible = False
        self.cx = self.cy = self.area = 0
        self.aspect = 0.0
        self.x_min = self.x_max = 0   # 紅色區域左右邊界（供之後精確定位中心用）

    def update(self):
        objs = self.api.get_objects(COLOR_GOAL)
        if not objs:
            self.visible = False
            return
        candidates = [o for o in objs if o['area'] > GOAL_MIN_AREA]
        if not candidates:
            self.visible = False
            return
        best = max(candidates, key=lambda o: o['area'])
        self.visible = True
        self.cx     = best['centroid'][0]
        self.cy     = best['centroid'][1]
        self.area   = best['area']
        self.aspect = best['aspect_ratio']
        bx, by, bw, bh = best['bbox']
        self.x_min = bx
        self.x_max = bx + bw

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
                # 計算頭部偏角供顯示
                h_deg = (HEAD_H_CENTER - n.head_h) / HEAD_DEG_PER_TICK
                sys.stdout.write("\033[H\033[J")
                sys.stdout.write(
                    f"#========= United Soccer v2 ==========#\n"
                    f" strategy_mode : {STRATEGY_MODE}\n"
                    f" state         : {n.state}\n"
                    f" sub_state     : {n.sub_state}\n"
                    f" action_detail : {n.action_detail}\n"
                    f"#============== 視覺狀態 ===============#\n"
                    f" ball.visible  : {n.ball.visible}"
                    f"  lost={n.ball_lost_count}/{BALL_LOST_FRAMES}\n"
                    f" ball.cx / cy  : {n.ball.cx} / {n.ball.cy}\n"
                    f" ball.area     : {n.ball.area}\n"
                    f" centered_cnt  : {n.ball_centered_count}/{BALL_CENTERED_FRAMES}\n"
                    f" pole.visible  : {n.pole.visible}\n"
                    f" pole.cx / cy  : {n.pole.cx} / {n.pole.cy}\n"
                    f" pole.area/cnt : {n.pole.area} / {n.pole.candidate_count}\n"
                    f" goal.visible  : {n.goal.visible}\n"
                    f" goal.cx / cy  : {n.goal.cx} / {n.goal.cy}\n"
                    f" goal.x_min/max: {n.goal.x_min} / {n.goal.x_max}\n"
                    f"#============== 頭部狀態 ===============#\n"
                    f" head_h        : {n.head_h}  偏角={h_deg:+.1f}°\n"
                    f" head_v        : {n.head_v}"
                    f"  (停止門檻={APPROACH_STOP_V})\n"
                    f" search_v_idx  : {n.search_v_idx}/{len(HEAD_SEARCH_V_LEVELS)-1}"
                    f"  V={HEAD_SEARCH_V_LEVELS[n.search_v_idx]}\n"
                    f"#=========== find_pole 狀態 ============#\n"
                    f" pole_found_h  : {n.pole_found_h}\n"
                    f" pole_found_v  : {n.pole_found_v}\n"
                    f" pole_scan_idx : {n.pole_search_v_idx}/{len(POLE_SEARCH_V_LEVELS)-1}"
                    f"  V={POLE_SEARCH_V_LEVELS[n.pole_search_v_idx]}\n"
                    f" return_frames : {n._return_head_frames}/{RETURN_HEAD_WAIT_FRAMES}\n"
                    f"#======= adjust_position 狀態 ==========#\n"
                    f" orbit_dir     : {n._orbit_dir}"
                    f"  ({'往左繞' if n._orbit_dir == 1 else '往右繞'})\n"
                    f" orbit_frames  : {n._orbit_frames}/{n._orbit_target_frames}\n"
                    f"#=========== find_goal 狀態 ============#\n"
                    f" goal_found_h  : {n.goal_found_h}\n"
                    f" goal_found_v  : {n.goal_found_v}\n"
                    f" goal_scan_idx : {n.goal_search_v_idx}/{len(GOAL_SEARCH_V_LEVELS)-1}"
                    f"  V={GOAL_SEARCH_V_LEVELS[n.goal_search_v_idx]}\n"
                    f" goal_confirm  : {n._goal_confirm_frames}/{GOAL_CONFIRM_FRAMES}\n"
                    f"#=======================================#\n"
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
        super().__init__('us_v2')

        self.ball = BallInfo(self)
        self.pole = PoleInfo(self)
        self.goal = GoalInfo(self)

        self.head_h = HEAD_H_CENTER
        self.head_v = HEAD_V_CENTER

        self.search_dir    = 'right'
        self.search_v_idx  = 0

        self.ball_lost_count     = 0
        self.ball_centered_count = 0   # 對準中心的連續幀計數

        # approach_ball 結束時的頭部位置（供 find_pole 用）
        self.ball_head_h = HEAD_H_CENTER
        self.ball_head_v = HEAD_V_FOOT

        # find_pole 掃描方向與層索引
        self.pole_search_dir   = 'right'
        self.pole_search_v_idx = 0

        # find_pole 找到球門柱時記錄的頭部刻度
        self.pole_found_h = HEAD_H_CENTER
        self.pole_found_v = HEAD_V_CENTER

        # find_pole 確認與回正計數器
        self._pole_confirm_frames = 0
        self._return_head_frames  = 0

        # adjust_position 軌道修正
        self._orbit_dir           = 1    # 1=往左繞, -1=往右繞
        self._orbit_frames        = 0    # 已執行幀數
        self._orbit_target_frames = 0    # 目標幀數

        # kick 踢球
        self._kick_wait_frames    = 0

        # find_goal 掃描方向與層索引（SHOOT 策略）
        self.goal_search_dir   = 'right'
        self.goal_search_v_idx = 0

        # find_goal 找到球門時記錄的頭部刻度
        self.goal_found_h = HEAD_H_CENTER
        self.goal_found_v = HEAD_V_CENTER

        # find_goal 對準確認計數器
        self._goal_confirm_frames = 0

        self.initialized   = False
        self.state         = 'find_ball'
        self.sub_state     = ''
        self.action_detail = '等待開始'

        self._printer = StatusPrinter(self)
        self._printer.start()

        self._reset_head()
        self.create_timer(0.1, self.main)

    # -----------------------------------------------------------------------
    # 頭部控制
    # -----------------------------------------------------------------------

    def _reset_head(self):
        self.head_h = HEAD_H_CENTER
        self.head_v = HEAD_V_CENTER
        self.sendHeadMotor(1, self.head_h, HEAD_SPEED)
        self.sendHeadMotor(2, self.head_v, HEAD_SPEED)
        self.search_dir   = 'right'
        self.search_v_idx = 0

    def _track_object(self, cx, cy):
        """P 控制器追蹤，回傳 True 表示已對準中央"""
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

    def _search_head(self):
        """分層水平掃描"""
        target_v = HEAD_SEARCH_V_LEVELS[self.search_v_idx]
        if self.head_v != target_v:
            self.head_v = target_v
            self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

        if self.search_dir == 'right':
            self.head_h -= HEAD_SEARCH_STEP_H
            if self.head_h <= HEAD_H_MIN:
                self.head_h = HEAD_H_MIN
                self.search_dir = 'left'
        else:
            self.head_h += HEAD_SEARCH_STEP_H
            if self.head_h >= HEAD_H_MAX:
                self.head_h = HEAD_H_MAX
                self.search_dir = 'right'
                self.search_v_idx += 1
                if self.search_v_idx >= len(HEAD_SEARCH_V_LEVELS):
                    self.search_v_idx = 0

        self.sendHeadMotor(1, self.head_h, HEAD_SPEED)

    def _head_offset_deg(self):
        """回傳頭部 H 偏離中心的角度，正值=偏左，負值=偏右"""
        return (HEAD_H_CENTER - self.head_h) / HEAD_DEG_PER_TICK

    def _search_pole_head(self):
        """分層水平掃描球門柱（V 層較高，對應柱子高度）"""
        target_v = POLE_SEARCH_V_LEVELS[self.pole_search_v_idx]
        if self.head_v != target_v:
            self.head_v = target_v
            self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

        if self.pole_search_dir == 'right':
            self.head_h -= POLE_SEARCH_STEP_H
            if self.head_h <= HEAD_H_MIN:
                self.head_h = HEAD_H_MIN
                self.pole_search_dir = 'left'
        else:
            self.head_h += POLE_SEARCH_STEP_H
            if self.head_h >= HEAD_H_MAX:
                self.head_h = HEAD_H_MAX
                self.pole_search_dir = 'right'
                self.pole_search_v_idx += 1
                if self.pole_search_v_idx >= len(POLE_SEARCH_V_LEVELS):
                    self.pole_search_v_idx = 0   # 掃完一輪，重頭繼續

        self.sendHeadMotor(1, self.head_h, HEAD_SPEED)

    def _search_goal_head(self):
        """分層水平掃描紅色球門（V 層偏高，因為要抬頭看橫桿）"""
        target_v = GOAL_SEARCH_V_LEVELS[self.goal_search_v_idx]
        if self.head_v != target_v:
            self.head_v = target_v
            self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

        if self.goal_search_dir == 'right':
            self.head_h -= GOAL_SEARCH_STEP_H
            if self.head_h <= HEAD_H_MIN:
                self.head_h = HEAD_H_MIN
                self.goal_search_dir = 'left'
        else:
            self.head_h += GOAL_SEARCH_STEP_H
            if self.head_h >= HEAD_H_MAX:
                self.head_h = HEAD_H_MAX
                self.goal_search_dir = 'right'
                self.goal_search_v_idx += 1
                if self.goal_search_v_idx >= len(GOAL_SEARCH_V_LEVELS):
                    self.goal_search_v_idx = 0   # 掃完一輪，重頭繼續

        self.sendHeadMotor(1, self.head_h, HEAD_SPEED)

    # -----------------------------------------------------------------------
    # find_ball
    # -----------------------------------------------------------------------

    def _state_find_ball(self):
        if self.ball.visible:
            self.ball_lost_count = 0
            centered = self._track_object(self.ball.cx, self.ball.cy)

            if centered:
                self.ball_centered_count += 1
                self.action_detail = (
                    f'球對準中心 ✅  '
                    f'確認中 {self.ball_centered_count}/{BALL_CENTERED_FRAMES}'
                )
                # 連續對準 N 幀 → 切換到 approach_ball
                if self.ball_centered_count >= BALL_CENTERED_FRAMES:
                    self.ball_centered_count = 0
                    self.sub_state = 'turn_to_ball'
                    self.state = 'approach_ball'
                    self.sendbodyAuto(1)   # 先啟動步態原地踏步，之後用 sendContinuousValue 更新步長
                    self.action_detail = '進入 approach_ball → turn_to_ball'
            else:
                # 偏離了，重置計數
                self.ball_centered_count = 0
                self.action_detail = (
                    f'追蹤球中 cx={self.ball.cx} cy={self.ball.cy}'
                )
        else:
            self.ball_centered_count = 0
            self.ball_lost_count += 1
            if self.ball_lost_count <= BALL_LOST_FRAMES:
                self.action_detail = (
                    f'球暫時消失 lost={self.ball_lost_count}/{BALL_LOST_FRAMES}'
                )
            else:
                self._search_head()
                self.action_detail = (
                    f'掃描中 dir={self.search_dir}  '
                    f'層={self.search_v_idx}  '
                    f'V={HEAD_SEARCH_V_LEVELS[self.search_v_idx]}  '
                    f'head_h={self.head_h}'
                )

    # -----------------------------------------------------------------------
    # approach_ball
    # -----------------------------------------------------------------------

    def _state_approach_ball(self):
        """
        sub_state = 'turn_to_ball' : 身體旋轉對正球
        sub_state = 'walk_to_ball' : 直走靠近，小幅 theta 修正
        球消失超過容忍幀數 → 退回 find_ball
        """
        if not self.ball.visible:
            self.ball_lost_count += 1
            if self.ball_lost_count <= BALL_LOST_FRAMES:
                # 短暫消失：繼續維持上一個動作，等球回來
                self.action_detail = (
                    f'[{self.sub_state}] 球暫時消失 '
                    f'lost={self.ball_lost_count}/{BALL_LOST_FRAMES}'
                )
                return
            else:
                # 真的找不到了，停步退回 find_ball
                self.sendContinuousValue(x=0, y=0, theta=0)
                self.sendbodyAuto(0)
                self._reset_head()
                self.state = 'find_ball'
                self.sub_state = ''
                self.ball_centered_count = 0
                self.action_detail = '球消失太久，退回 find_ball'
                return

        # 球可見，重置消失計數，頭部持續追球
        self.ball_lost_count = 0
        self._track_object(self.ball.cx, self.ball.cy)

        if self.sub_state == 'turn_to_ball':
            self._turn_to_ball()
        elif self.sub_state == 'walk_to_ball':
            self._walk_to_ball()

    def _turn_to_ball(self):
        """
        身體原地旋轉，直到頭部 H 偏角 < TURN_DONE_DEG。
        偏角大 → 快轉，偏角小 → 慢轉，進入死區 → 停止切換步態。
        """
        offset_deg = self._head_offset_deg()   # 正=球在左, 負=球在右
        abs_deg    = abs(offset_deg)

        # 判斷對正：head_h 真正回到中心附近才算完成
        # 用 head_h 而非偏角，避免頭追球造成誤判
        head_h_err = abs(self.head_h - HEAD_H_CENTER)
        TURN_DONE_TICKS = TURN_DONE_DEG * HEAD_DEG_PER_TICK  # 5度換算成刻度
        if head_h_err < TURN_DONE_TICKS:
            # 已對正，步長歸零原地踏步，切換到直走
            self.sendContinuousValue(x=0, y=0, theta=0)
            self.sub_state = 'walk_to_ball'
            self.action_detail = (
                f'對正完成 head_h={self.head_h}（誤差={head_h_err}刻度）→ walk_to_ball'
            )
            return

        # 根據偏角大小決定旋轉速度
        if abs_deg > 20:
            theta = TURN_THETA_FAST
        elif abs_deg > 10:
            theta = TURN_THETA_MID
        else:
            theta = TURN_THETA_SLOW

        # head_h 偏離中心的換算：
        #   HEAD_H_CENTER - head_h > 0 → head_h 偏小 → 頭偏右 → 球在右邊 → 身體右轉（theta 負值）
        #   HEAD_H_CENTER - head_h < 0 → head_h 偏大 → 頭偏左 → 球在左邊 → 身體左轉（theta 正值）
        if offset_deg > 0:
            theta = -theta   # 球在右邊，右轉

        self.sendContinuousValue(x=0, y=0, theta=theta)
        self.action_detail = (
            f'旋轉對正中 偏角={offset_deg:+.1f}°  theta={theta}'
        )

    def _walk_to_ball(self):
        """
        直走靠近球，途中用頭 H 偏角做小幅 theta 修正維持方向。
        head_v <= APPROACH_STOP_V 時停步，完成 approach_ball。
        """
        # 到達條件
        if self.head_v + 50 <= APPROACH_STOP_V:
            self.sendContinuousValue(x=0, y=0, theta=0)
            self.sendbodyAuto(0)
            # 記錄抵達時的頭部位置，供下一狀態機回正用
            self.ball_head_h = self.head_h
            self.ball_head_v = self.head_v

            if STRATEGY_MODE == 'SHOOT':
                # 重置 find_goal 掃描狀態
                self.goal_search_dir      = 'right'
                self.goal_search_v_idx    = 0
                self._goal_confirm_frames = 0
                self.state     = 'find_goal'
                self.sub_state = 'scan'
                self.action_detail = (
                    f'抵達球旁 head_v={self.head_v} ✅ → find_goal'
                )
                return

            # 預設 / 'KICK_OBSTACLE'：重置 find_pole 掃描狀態
            self.pole_search_dir    = 'right'
            self.pole_search_v_idx  = 0
            self._pole_confirm_frames = 0
            self._return_head_frames  = 0
            self.state     = 'find_pole'
            self.sub_state = 'scan'
            self.action_detail = (
                f'抵達球旁 head_v={self.head_v} ✅ → find_pole'
            )
            return

        # theta 修正邏輯：
        # 1. 球夠近（head_v 低於 WALK_THETA_STOP_V）→ 強制 theta=0，直走到底
        # 2. head_h 偏離中心在死區內 → theta=0，避免微震盪
        # 3. 超出死區 → 計算修正量
        offset_deg = self._head_offset_deg()   # 供 action_detail 顯示用
        head_h_err = abs(self.head_h - HEAD_H_CENTER)
        if self.head_v <= WALK_THETA_STOP_V:
            theta = 0
        elif head_h_err < WALK_THETA_DEAD_TICKS:
            theta = 0
        else:
            theta = -(offset_deg * WALK_THETA_GAIN)
            theta = max(-WALK_THETA_MAX, min(WALK_THETA_MAX, theta))
            theta = int(round(theta))

        # 靠近時換慢速（head_v 再高 150 刻度以內）
        if self.head_v <= APPROACH_STOP_V + 150:
            x = WALK_X_SLOW
        else:
            x = WALK_X_NORMAL

        self.sendContinuousValue(x=x, y=0, theta=theta)
        self.action_detail = (
            f'直走靠近 head_v={self.head_v}  '
            f'偏角={offset_deg:+.1f}°  theta={theta}  x={x}'
        )

    # -----------------------------------------------------------------------
    # find_goal（SHOOT 策略）
    # -----------------------------------------------------------------------

    def _state_find_goal(self):
        """
        sub_state = 'scan'        : 分層水平掃描搜尋紅色球門
        sub_state = 'center_goal' : P-controller 將頭對準球門，連續穩定幀數後記錄 goal_found_h/v
        sub_state = 'return_head' : 找到後頭部回到球的位置，等待 RETURN_HEAD_WAIT_FRAMES 幀
        """
        if self.sub_state == 'scan':
            self.goal.update()
            if self.goal.visible:
                self._goal_confirm_frames = 0
                self.sub_state = 'center_goal'
                self.action_detail = f'發現球門，切換對準模式 cx={self.goal.cx}'
            else:
                self._search_goal_head()
                self.action_detail = (
                    f'掃描球門中 dir={self.goal_search_dir}  '
                    f'層={self.goal_search_v_idx}  '
                    f'V={GOAL_SEARCH_V_LEVELS[self.goal_search_v_idx]}  '
                    f'head_h={self.head_h}'
                )

        elif self.sub_state == 'center_goal':
            self.goal.update()
            if not self.goal.visible:
                self._goal_confirm_frames = 0
                self.sub_state = 'scan'
                self.action_detail = '球門追丟，回掃描'
            else:
                centered = self._track_object(self.goal.cx, self.goal.cy)
                if centered:
                    self._goal_confirm_frames += 1
                    self.action_detail = (
                        f'球門對準中，確認幀 ({self._goal_confirm_frames}/{GOAL_CONFIRM_FRAMES})  '
                        f'cx={self.goal.cx}  head_h={self.head_h}'
                    )
                    if self._goal_confirm_frames >= GOAL_CONFIRM_FRAMES:
                        # 穩定對準：記錄頭部刻度，送出回正指令
                        self.goal_found_h = self.head_h
                        self.goal_found_v = self.head_v
                        self.head_h = self.ball_head_h
                        self.head_v = self.ball_head_v
                        self.sendHeadMotor(1, self.head_h, HEAD_SPEED)
                        self.sendHeadMotor(2, self.head_v, HEAD_SPEED)
                        self._goal_confirm_frames = 0
                        self._return_head_frames  = 0
                        self.sub_state = 'return_head'
                        self.action_detail = (
                            f'找到球門 ✅ goal_h={self.goal_found_h} goal_v={self.goal_found_v} '
                            f'cx={self.goal.cx} cy={self.goal.cy} → 頭部回正中'
                        )
                else:
                    self._goal_confirm_frames = 0
                    self.action_detail = (
                        f'對準球門中 cx={self.goal.cx} head_h={self.head_h}'
                    )

        elif self.sub_state == 'return_head':
            self._return_head_frames += 1
            self.action_detail = (
                f'頭部回正中 ({self._return_head_frames}/{RETURN_HEAD_WAIT_FRAMES})  '
                f'目標 h={self.ball_head_h} v={self.ball_head_v}'
            )
            if self._return_head_frames >= RETURN_HEAD_WAIT_FRAMES:
                # 計算繞球方向與目標幀數（與 find_pole 同公式，改用 goal_found_h）
                # goal_found_h > HEAD_H_CENTER → 球門在左 → 繞到球右側 → 往右繞 → orbit_dir = -1
                # goal_found_h < HEAD_H_CENTER → 球門在右 → 繞到球左側 → 往左繞 → orbit_dir =  1
                goal_error = self.goal_found_h - HEAD_H_CENTER
                self._orbit_dir           = 1 if goal_error < 0 else -1
                self._orbit_target_frames = int(abs(goal_error) * ORBIT_TICK_GAIN)
                self._orbit_frames        = 0
                self.sendbodyAuto(1)
                self.state     = 'adjust_position'
                self.sub_state = ''
                self.action_detail = (
                    f'頭部回正完成 ✅ → adjust_position  '
                    f'orbit_dir={self._orbit_dir}  '
                    f'target={self._orbit_target_frames} 幀'
                )

    # -----------------------------------------------------------------------
    # find_pole
    # -----------------------------------------------------------------------

    def _state_find_pole(self):
        """
        sub_state = 'scan'        : 分層水平掃描搜尋藍色球門柱
        sub_state = 'center_pole' : P-controller 將頭對準藍柱中心，穩定後計幀數
        sub_state = 'return_head' : 找到後頭部回到球的位置，等待 RETURN_HEAD_WAIT_FRAMES 幀
        """
        if self.sub_state == 'scan':
            self.pole.update()
            if self.pole.visible:
                # 看到藍柱：停止掃描，切換到對準模式
                self._pole_confirm_frames = 0
                self.sub_state = 'center_pole'
                self.action_detail = (
                    f'發現藍柱，鎖定最大面積目標 '
                    f'cnt={self.pole.candidate_count} area={self.pole.area} cx={self.pole.cx}'
                )
            else:
                self._search_pole_head()
                self.action_detail = (
                    f'掃描藍柱中 dir={self.pole_search_dir}  '
                    f'層={self.pole_search_v_idx}  '
                    f'V={POLE_SEARCH_V_LEVELS[self.pole_search_v_idx]}  '
                    f'head_h={self.head_h}'
                )

        elif self.sub_state == 'center_pole':
            self.pole.update()
            if not self.pole.visible:
                # 追丟了：回去繼續掃
                self._pole_confirm_frames = 0
                self.sub_state = 'scan'
                self.action_detail = '藍柱追丟，回掃描'
            else:
                centered = self._track_object(self.pole.cx, self.pole.cy)
                if centered:
                    self._pole_confirm_frames += 1
                    self.action_detail = (
                        f'藍柱對準中，確認幀 ({self._pole_confirm_frames}/{POLE_CONFIRM_FRAMES})  '
                        f'cnt={self.pole.candidate_count} area={self.pole.area} '
                        f'cx={self.pole.cx} head_h={self.head_h}'
                    )
                    if self._pole_confirm_frames >= POLE_CONFIRM_FRAMES:
                        # 穩定對準：記錄頭部刻度，送出回正指令
                        self.pole_found_h = self.head_h
                        self.pole_found_v = self.head_v
                        self.head_h = self.ball_head_h
                        self.head_v = self.ball_head_v
                        self.sendHeadMotor(1, self.head_h, HEAD_SPEED)
                        self.sendHeadMotor(2, self.head_v, HEAD_SPEED)
                        self._pole_confirm_frames = 0
                        self._return_head_frames  = 0
                        self.sub_state = 'return_head'
                        self.action_detail = (
                            f'找到藍柱 ✅ pole_h={self.pole_found_h} pole_v={self.pole_found_v} '
                            f'cnt={self.pole.candidate_count} area={self.pole.area} '
                            f'cx={self.pole.cx} cy={self.pole.cy} → 頭部回正中'
                        )
                else:
                    self._pole_confirm_frames = 0   # 還沒對準，計數歸零
                    self.action_detail = (
                        f'對準藍柱中 cnt={self.pole.candidate_count} '
                        f'area={self.pole.area} cx={self.pole.cx} head_h={self.head_h}'
                    )

        elif self.sub_state == 'return_head':
            self._return_head_frames += 1
            self.action_detail = (
                f'頭部回正中 ({self._return_head_frames}/{RETURN_HEAD_WAIT_FRAMES})  '
                f'目標 h={self.ball_head_h} v={self.ball_head_v}'
            )
            if self._return_head_frames >= RETURN_HEAD_WAIT_FRAMES:
                # 計算繞球方向與目標幀數
                # pole_found_h > HEAD_H_CENTER → 柱在左 → 繞到球右側 → 往右繞 → orbit_dir = -1
                # pole_found_h < HEAD_H_CENTER → 柱在右 → 繞到球左側 → 往左繞 → orbit_dir =  1
                pole_error = self.pole_found_h - HEAD_H_CENTER
                self._orbit_dir           = 1 if pole_error < 0 else -1
                self._orbit_target_frames = int(abs(pole_error) * ORBIT_TICK_GAIN)
                self._orbit_frames        = 0
                self.sendbodyAuto(1)
                self.state     = 'adjust_position'
                self.sub_state = ''
                self.action_detail = (
                    f'頭部回正完成 ✅ → adjust_position  '
                    f'orbit_dir={self._orbit_dir}  '
                    f'target={self._orbit_target_frames} 幀'
                )

    # -----------------------------------------------------------------------
    # adjust_position
    # -----------------------------------------------------------------------

    def _state_adjust_position(self):
        """
        弧形繞球修正踢球角度（KICK_OBSTACLE / SHOOT 共用，_orbit_dir 與
        _orbit_target_frames 已由 find_pole 或 find_goal 的 return_head 算好）：
        - 頭部 H+V 雙軸追蹤球，保持球在畫面中央
        - head_v 偏離 ball_head_v → x 補償前後距離
          head_v > ball_head_v：頭抬起 → 球跑遠 → 前進
          head_v < ball_head_v：頭更下沉 → 球太近 → 後退
        - orbit_dir 決定 y + theta 方向，同時送出形成弧形
        - 達到 _orbit_target_frames 後停步進入下一狀態
        """
        # 到達目標幀數 → 完成修正
        if self._orbit_frames >= self._orbit_target_frames:
            self.sendContinuousValue(x=0, y=0, theta=0)
            self.sendbodyAuto(0)
            self._kick_wait_frames = 0
            self.state = 'kick'
            self.action_detail = '軌道修正完成 ✅ → kick'
            return

        # 頭部追蹤球（H+V），同時用 head_v 判斷距離
        if self.ball.visible:
            self._track_object(self.ball.cx, self.ball.cy)
            v_err = self.head_v - self.ball_head_v
            x = int(v_err * ORBIT_V_GAIN)
            x = max(-ORBIT_X_MAX, min(ORBIT_X_MAX, x))
        else:
            x = 0
            v_err = 0

        if self._orbit_dir == 1:   # 往左繞
            y     =  ORBIT_Y_LEFT
            theta =  ORBIT_THETA_LEFT
        else:                      # 往右繞
            y     = ORBIT_Y_RIGHT
            theta = ORBIT_THETA_RIGHT

        self.sendContinuousValue(x=x, y=y, theta=theta)
        self._orbit_frames += 1
        self.action_detail = (
            f'繞球中 [{self._orbit_frames}/{self._orbit_target_frames}]  '
            f'dir={self._orbit_dir}  x={x}  y={y}  theta={theta}  '
            f'head_v={self.head_v}  v_err={v_err}'
        )

    def _state_kick(self):
        """
        踢球：依繞球方向選擇左/右腳，等待動作完成後回到 find_ball。
        orbit_dir == -1（往右繞）→ 右腳踢 sendBodySector(100)
        orbit_dir ==  1（往左繞）→ 左腳踢 sendBodySector(200)
        """
        if self._kick_wait_frames == 0:
            if self._orbit_dir == -1:
                self.sendBodySector(999)
                time.sleep(1)
                self.sendBodySector(100)
                time.sleep(14)
                self.sendBodySector(29)
                time.sleep(1)
                self.action_detail = '右腳踢球 sector=100'
            else:
                self.sendBodySector(999)
                time.sleep(1)
                self.sendBodySector(200)
                time.sleep(14)
                self.sendBodySector(29)
                time.sleep(1)
                self.action_detail = '左腳踢球 sector=200'

        self._kick_wait_frames += 1
        if self._kick_wait_frames >= KICK_WAIT_FRAMES:
            self._initialize()
            self.action_detail = '踢球完成 ✅ → 初始化 → find_ball'

    def _initialize(self):
        """每次撥開關啟動或踢球完成後執行：頭部歸中、停走、站穩後重新校正 IMU、重置狀態。"""
        self.head_h = HEAD_H_CENTER
        self.head_v = HEAD_V_CENTER
        self.sendHeadMotor(1, self.head_h, HEAD_SPEED)
        self.sendHeadMotor(2, self.head_v, HEAD_SPEED)
        self.sendbodyAuto(0)
        time.sleep(1)
        self.sendBodySector(29)
        time.sleep(0.5)   # 等身體確實站穩，再校正 IMU 零點
        self.sendSensorReset(True)   # 踢球後姿態可能偏移，重新校正 Yaw/Roll/Pitch
        time.sleep(0.05)

        self.ball_lost_count = 0
        self.ball_centered_count = 0

        self.search_dir = 'right'
        self.search_v_idx = 0

        self.ball_head_h = HEAD_H_CENTER
        self.ball_head_v = HEAD_V_FOOT

        self.pole_search_dir = 'right'
        self.pole_search_v_idx = 0
        self.pole_found_h = HEAD_H_CENTER
        self.pole_found_v = HEAD_V_CENTER
        self._pole_confirm_frames = 0
        self._return_head_frames = 0

        self._orbit_dir = 1
        self._orbit_frames = 0
        self._orbit_target_frames = 0
        self._kick_wait_frames = 0

        self.goal_search_dir = 'right'
        self.goal_search_v_idx = 0
        self.goal_found_h = HEAD_H_CENTER
        self.goal_found_v = HEAD_V_CENTER
        self._goal_confirm_frames = 0

        self.state         = 'find_ball'
        self.sub_state     = ''
        self.action_detail = '初始化完成 → find_ball'

    # -----------------------------------------------------------------------
    # 主迴圈
    # -----------------------------------------------------------------------

    def main(self):
        if not self.is_start:
            if self.initialized:
                self.sendbodyAuto(0)
                self.initialized   = False
                self.action_detail = '=== 停止 ==='
            return

        if not self.initialized:
            self._initialize()
            self.initialized = True
            return

        self.ball.update()

        if self.state == 'find_ball':
            self._state_find_ball()
        elif self.state == 'approach_ball':
            self._state_approach_ball()
        elif self.state == 'find_pole':
            self._state_find_pole()
        elif self.state == 'adjust_position':
            self._state_adjust_position()
        elif self.state == 'kick':
            self._state_kick()
        elif self.state == 'find_goal':
            self._state_find_goal()

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