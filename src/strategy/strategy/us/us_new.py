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

import json
from std_msgs.msg import String

# ===========================================================================
# 調參區
# ===========================================================================

USE_REFEREE_COMM = False   # 是否使用裁判通訊，False=不使用，直接進行策略測試
# --- 策略開關 ---
# 'KICK_OBSTACLE' : 踢向障礙物（find_pole → adjust_position → kick）
# 'SHOOT'         : 射門（find_goal）
# 'PENALTY_KICK'  : 直接射門，不找球、不找柱、不繞球
STRATEGY_MODE = 'KICK_OBSTACLE'

COLOR_BALL = 'yellow'
COLOR_POLE = 'blue'
COLOR_GOAL = 'red'
COLOR_LINE = 'white'

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
HEAD_SPEED    = 40

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
HEAD_SEARCH_STEP_H  = 70
HEAD_SEARCH_V_LEVELS = [1300, 1450, 1600, 1750,1950]
#HEAD_SEARCH_V_LEVELS = [1300, 1450, 1600, 1750,1950]
FIND_BALL_STOP_SETTLE_FRAMES = 3

# --- find_pole 掃描 ---
POLE_SEARCH_V_LEVELS   = [1450, 1600, 1750, 1900, 2020]
POLE_SEARCH_STEP_H     = 60

# --- 球門柱過濾條件 ---
POLE_MIN_AREA    = 80
POLE_ASPECT_MIN  = 0.1   # 柱子是細長形，不限制比例
POLE_ASPECT_MAX  = 5.0

# --- find_ball 邊走邊找時的白線避開 ---
# 白線偵測不再只靠 area，而是看 bbox 形狀：
# 1. 位於畫面下半部
# 2. 寬度夠大
# 3. 高度不要太高
# 4. w/h 像橫向長條
# 5. 下緣接近畫面底部
LINE_MIN_Y = 130
LINE_MIN_WIDTH = 45
LINE_MAX_HEIGHT = 45
LINE_MIN_ASPECT = 2.5
LINE_MIN_BOTTOM_Y = 165
LINE_CONFIRM_FRAMES = 2

# IMU 轉 90 度避線
LINE_TURN_ANGLE_DEG = 90.0
LINE_YAW_TOL_DEG = 5.0
LINE_IMU_TURN_THETA_FAST = 5
LINE_IMU_TURN_THETA_SLOW = 2
LINE_IMU_TURN_TIMEOUT_FRAMES = 45     # main=0.1s，45 約 4.5 秒
LINE_TURN_COOLDOWN_FRAMES = 8         # 轉完後冷卻，避免同一條線馬上重複觸發

# 方向設定：
# LINE_TURN_BY_LINE_POSITION=True 時：
#   白線在左邊 → target_yaw = yaw - 90
#   白線在右邊 → target_yaw = yaw + 90
# 如果實測方向反了，把 LINE_TURN_SIGN 改成 -1。
LINE_TURN_BY_LINE_POSITION = True
LINE_TURN_SIGN = 1

# 如果 yaw error 越轉越大，代表 theta 正負方向跟 IMU yaw 定義相反，
# 把 LINE_IMU_THETA_SIGN 改成 -1。
LINE_IMU_THETA_SIGN = -1

# IMU 讀不到時的備援：退回固定幀數轉彎
LINE_TURN_THETA = 5
LINE_BACK_X = -100
LINE_AVOID_FRAMES = 12


# --- find_pole: 頭部回球位 ---
POLE_CONFIRM_FRAMES     = 5    # 連續看到藍柱幾幀才算找到（可調）
RETURN_HEAD_WAIT_FRAMES = 15   # 等待馬達移動到位的幀數（0.1s × 15 ≈ 1.5s）

# --- adjust_position: 繞球軌道修正 ---
ORBIT_Y_LEFT      = 900   # 往左繞（orbit_dir= 1）側向步長（可調）
ORBIT_Y_RIGHT     = -800   # 往右繞（orbit_dir=-1）側向步長（可調）
ORBIT_THETA_LEFT  = -4     # 往左繞旋轉步長（可調）
ORBIT_THETA_RIGHT = 4     # 往右繞旋轉步長（可調）
ORBIT_V_GAIN      = 60.0   # head_v 偏差 → x 步長係數（可調）
ORBIT_X_MAX       = 300   # x 步長上限
ORBIT_TICK_GAIN   = 0.18  # pole_h 誤差刻度 → 目標幀數係數（可調，影響繞球總量）

# --- kick: 踢球動作 ---
KICK_WAIT_FRAMES  = 30   # 等待踢球動作完成的幀數（0.1s × 30 = 3s，可調）

# --- kick: 固定頭部重新判斷左右腳 ---
# --- kick: 慢速掃描判斷左右腳 ---
KICK_SCAN_H = 2048

# kick 前球通常在腳邊，所以 V 不建議掃太高
KICK_SCAN_V_LEVELS = [1150, 1200, 1250, 1300, 1350, 1450]

# 每一層 V 停幾個 main frame；main 是 0.1s，所以 4 = 0.4 秒
KICK_SCAN_WAIT_FRAMES = 4

# [新增] kick 進入掃描前，先等 H=2048 到位；main 是 0.1s，所以 10 = 1 秒
KICK_SCAN_SETTLE_FRAMES = 10

# [新增] kick 掃描用的頭部速度；只在 prepare 送 H，scan 只動 V
KICK_SCAN_HEAD_SPEED = 30

# 至少要看到幾次球，才相信掃描結果
# 建議先用 1，因為球在腳邊不一定每個 V 層都看得到。
KICK_SCAN_MIN_SAMPLES = 1

# 左右腳判斷死區，避免 cx 靠近中心時亂跳
KICK_SIDE_DEADZONE = 2

# 看不到球或太靠中心時的預設腳
KICK_DEFAULT_SECTOR = 200

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
TURN_DONE_DEG  = 3.0    # 度，可調整
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
WALK_THETA_DEAD_TICKS = 60
# 球夠近後停止 theta 修正，直接直走到底
# head_v 低於此值（球已很近）就強制 theta=0
WALK_THETA_STOP_V = 1600


# ===========================================================================
# 【FAILURE_DIAG】失敗診斷參數（專題）
# ===========================================================================
# 這一版先只做「診斷與紀錄」，不直接改變原本策略流程
FAIL_NONE = 'NONE'
FAIL_BALL_LOST = 'BALL_LOST'
FAIL_TARGET_LOST = 'TARGET_LOST'
FAIL_KICK_NO_BALL = 'KICK_NO_BALL'
FAIL_YAW_ERROR_BEFORE_KICK = 'YAW_ERROR_BEFORE_KICK'
FAIL_TOO_CLOSE = 'TOO_CLOSE'
FAIL_TOO_FAR = 'TOO_FAR'

# [新增] kick 前位置與繞球品質診斷
FAIL_BALL_SIDE_OFFSET = 'BALL_SIDE_OFFSET' # 踢球前球位置偏移 需要左右 平移修正
FAIL_ORBIT_NOT_ENOUGH = 'ORBIT_NOT_ENOUGH' # 繞球不足，球門柱誤差太大
FAIL_ORBIT_OVER = 'ORBIT_OVER'             # 繞球過度，球門柱誤差太大

FAILURE_HISTORY_MAX = 20

# [新增] kick 前位置判斷門檻
# 只做診斷，不做前進 / 後退 / 平移修正。
KICK_TOO_CLOSE_V = 1150
KICK_TOO_FAR_V = 1350
KICK_SIDE_OFFSET_TOL = 45

# [新增] 繞球品質判斷門檻
# 第一版先用 _orbit_target_frames 粗略判斷，不額外重新找球門。
ORBIT_MIN_FRAMES = 8
ORBIT_MAX_FRAMES = 45

# ===========================================================================
# 【RECOVERY】kick 前位置修正參數（專題）
# ===========================================================================
# 這一版先只針對 kick 前位置問題做一次小修正：
# KICK_NO_BALL / TOO_CLOSE  → 後退重掃
# TOO_FAR                  → 前進重掃
# BALL_SIDE_OFFSET          → 左右平移重掃
#
# 注意：最多先修正 1 次，避免機器人一直修正越走越亂。
KICK_RECOVERY_RETRY_MAX = 1

# 修正完後停幾個 main frame，讓身體穩定再重新掃球。
KICK_RECOVERY_SETTLE_FRAMES = 10

# KICK_NO_BALL / TOO_CLOSE：後退一小步
KICK_RECOVERY_BACK_X = -800
KICK_RECOVERY_BACK_FRAMES = 40

# TOO_FAR：前進一小步
KICK_RECOVERY_FORWARD_X = 300
KICK_RECOVERY_FORWARD_FRAMES = 35

# BALL_SIDE_OFFSET：左右平移一小步
# y 正負方向要真機測：
#   如果球越修越偏，把 SIDE_LEFT / SIDE_RIGHT 的 y 正負方向對調即可。
KICK_RECOVERY_SIDE_Y = 800
KICK_RECOVERY_SIDE_FRAMES = 30

# kick 前若 yaw 和進入 kick 當下差太多，先記錄成姿態異常。
# 注意：這一版只診斷，不在 kick 前強制旋轉，避免球已經在腳邊時被碰走。
PRE_KICK_YAW_CHECK = True
PRE_KICK_YAW_MAX_ERROR = 2

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
                    f"#=========== kick 判斷狀態 ============#\n"
                    f" kick.ref_visible: {n.kick_ref_visible}\n"
                    f" kick.ref_cx/cy : {n.kick_ref_cx} / {n.kick_ref_cy}\n"
                    f" kick.ref_head  : h={n.kick_ref_head_h} v={n.kick_ref_head_v}\n"
                    f" kick.visible   : {n.kick_debug_visible}\n"
                    f" kick.cx        : {n.kick_debug_cx}  IMG_CX={IMG_CX}\n"
                    f" kick.side      : {n.kick_debug_side}\n"
                    f" kick.sector    : {n.kick_debug_sector}\n"

                    f" kick.best_v    : {n.kick_debug_best_v}\n"
                    f" kick.pos_judge : {n.kick_position_judge}\n"
                    f" kick.recovery  : {n.kick_recovery_action} "
                    f"count={n.kick_recovery_count}/{KICK_RECOVERY_RETRY_MAX} "
                    f"frames={n.kick_recovery_frames} settle={n.kick_recovery_settle}\n"
                    f"#=========== failure 診斷 ============#\n"
                    f" failure_type  : {n.debug_failure_type}\n"
                    f" failure_detail: {n.debug_failure_detail}\n"
                    f" failure_count : {n.failure_count}\n"
                    f" last_state    : {n.last_failure_state}\n"
                    f" kick_yaw_err : {n.kick_yaw_error:+.1f}°\n"
                    f" orbit_judge  : {n.orbit_quality_judge}\n"
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

        self.find_ball_walk_search = False
        self.find_ball_stop_settle_frames = 0

        # [新增] 邊走邊找球時，看到白線要避開
        self.line_avoid_frames = 0
        self.line_avoid_dir = 1   # 1=左轉，-1=右轉
        self.line_confirm_count = 0

        # [新增] 白線 IMU 旋轉 90 度避線
        self.line_turn_active = False
        self.line_turn_target_yaw = 0.0
        self.line_turn_start_yaw = 0.0
        self.line_turn_timeout = 0
        self.line_turn_cooldown_frames = 0

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

        # [新增] kick 前慢速掃描判斷左右腳
        self._kick_phase = 'prepare'
        self._kick_scan_idx = 0
        self._kick_scan_wait = 0
        self._kick_scan_samples = []
        self._kick_selected_cx = -1
        self._kick_selected_area = 0
        self._kick_prepare_from_h = HEAD_H_CENTER

        # adjust_position 最後看到球的位置，給 kick 判斷左右腳用
        self.kick_ref_visible = False
        self.kick_ref_cx = 0
        self.kick_ref_cy = 0
        self.kick_ref_head_h = HEAD_H_CENTER
        self.kick_ref_head_v = HEAD_V_FOOT

        # kick 判斷 debug 顯示
        self.kick_debug_visible = False
        self.kick_debug_cx = -1
        self.kick_debug_side = 'none'
        self.kick_debug_sector = 0

        # find_goal 掃描方向與層索引（SHOOT 策略）
        self.goal_search_dir   = 'right'
        self.goal_search_v_idx = 0

        # find_goal 找到球門時記錄的頭部刻度
        self.goal_found_h = HEAD_H_CENTER
        self.goal_found_v = HEAD_V_CENTER

        # find_goal 對準確認計數器
        self._goal_confirm_frames = 0

        self.initialized   = False
        self.state = 'penalty_kick' if STRATEGY_MODE == 'PENALTY_KICK' else 'find_ball'
        self.sub_state     = ''
        self.action_detail = '等待開始'

        # [新增] failure 診斷狀態（專題 v1：先紀錄，不改流程）
        self.debug_failure_type = FAIL_NONE
        self.debug_failure_detail = ''
        self.failure_count = 0
        self.failure_history = []
        self.last_failure_state = ''

        # [新增] kick 前 yaw 診斷用
        self.kick_target_yaw = 0.0
        self.kick_yaw_error = 0.0

        # [新增] kick 前位置診斷顯示用
        self.kick_debug_best_v = 0
        self.kick_position_judge = 'UNKNOWN'

        # [新增] 繞球品質診斷顯示用
        self.orbit_quality_judge = 'UNKNOWN'

        # [新增] kick 前位置 recovery 狀態
        self.kick_recovery_count = 0
        self.kick_recovery_action = 'NONE'
        self.kick_recovery_frames = 0
        self.kick_recovery_settle = 0
        self.kick_recovery_from_failure = FAIL_NONE

        # referee comm 狀態
        self.ref_state = 0
        self.ref_state_name = 'NO_REF'
        self.ref_play = False
        self.ref_self_enabled = False
        self.ref_time_left = 0

        # 訂閱通訊節點送出的 referee 狀態
        self.create_subscription(
            String,
            '/comm/game_state',
            self._comm_game_state_cb,
            10
        )

        self._printer = StatusPrinter(self)
        self._printer.start()

        self._reset_head()
        self.create_timer(0.1, self.main)




    def _comm_game_state_cb(self, msg):
        """
        接收 us_comm.py 發出的 /comm/game_state

        msg.data 內容大概是：
        {
            "state": 5,
            "state_name": "Play: Ball in play",
            "time_left": xxx,
            "self_enabled": true,
            "play": true,
            ...
        }
        """
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.action_detail = f'comm json 解析失敗: {e}'
            return

        self.ref_state = data.get('state', 0)
        self.ref_state_name = data.get('state_name', 'UNKNOWN')
        self.ref_play = bool(data.get('play', False))
        self.ref_self_enabled = bool(data.get('self_enabled', False))
        self.ref_time_left = data.get('time_left', 0)

    # -----------------------------------------------------------------------
    # [新增] 失敗診斷工具函式（專題 v1）
    # -----------------------------------------------------------------------

    def _set_failure(self, failure_type, detail=''):
        """
        記錄目前偵測到的失敗原因。
        這一版只負責顯示與存 history，不直接改變原本動作流程。
        """
        self.debug_failure_type = failure_type
        self.debug_failure_detail = detail
        self.last_failure_state = self.state
        self.failure_count += 1

        self.failure_history.append({
            'time': time.time(),
            'state': self.state,
            'sub_state': self.sub_state,
            'failure_type': failure_type,
            'detail': detail,
            'ball_visible': self.ball.visible,
            'ball_cx': self.ball.cx,
            'ball_cy': self.ball.cy,
            'head_h': self.head_h,
            'head_v': self.head_v,
            'kick_sector': self.kick_debug_sector,
            'kick_yaw_error': self.kick_yaw_error,
            'kick_best_v': self.kick_debug_best_v,
            'kick_position_judge': self.kick_position_judge,
            'orbit_target_frames': self._orbit_target_frames,
            'orbit_quality_judge': self.orbit_quality_judge,
            'kick_recovery_action': self.kick_recovery_action,
            'kick_recovery_count': self.kick_recovery_count,
            'kick_recovery_from_failure': self.kick_recovery_from_failure,
        })

        if len(self.failure_history) > FAILURE_HISTORY_MAX:
            self.failure_history.pop(0)

    def _clear_current_failure(self):
        """清除目前畫面上的 failure，但保留 failure_history 與 failure_count。"""
        self.debug_failure_type = FAIL_NONE
        self.debug_failure_detail = ''
        self.last_failure_state = ''
        self.kick_yaw_error = 0.0
        self.kick_position_judge = 'UNKNOWN'
        self.kick_debug_best_v = 0
        self.orbit_quality_judge = 'UNKNOWN'

    def _diagnose_pre_kick_yaw(self):
        """
        kick 前 yaw 診斷：
        比較目前 yaw 與進入 kick 當下記錄的 kick_target_yaw。
        若偏差過大，只記錄 YAW_ERROR_BEFORE_KICK，不強制修正。
        """
        if not PRE_KICK_YAW_CHECK:
            return

        yaw = self._read_imu_yaw()
        if yaw is None:
            return

        self.kick_yaw_error = self._normalize_yaw_error(self.kick_target_yaw, yaw)

        if abs(self.kick_yaw_error) > PRE_KICK_YAW_MAX_ERROR:
            self._set_failure(
                FAIL_YAW_ERROR_BEFORE_KICK,
                f'kick前yaw偏差過大 err={self.kick_yaw_error:+.1f}° '
                f'target={self.kick_target_yaw:.1f} current={yaw:.1f}'
            )

    def _diagnose_kick_position(self, best):
        """
        [新增] kick 前球位置診斷：
        使用 kick 掃描中 area 最大的球樣本 best 判斷距離與左右偏移。
        這一版只紀錄 failure_type，不做前進、後退或平移修正。
        """
        best_v = best.get('head_v', self.kick_ref_head_v)
        best_cx = best.get('cx', self.kick_ref_cx)

        self.kick_debug_best_v = best_v
        self.kick_position_judge = 'OK'

        # 優先判斷距離，再判斷左右偏移，避免同一幀連續跳多個狀態。
        if best_v <= KICK_TOO_CLOSE_V:
            self.kick_position_judge = 'TOO_CLOSE'
            self._set_failure(
                FAIL_TOO_CLOSE,
                f'kick前球太近 best_v={best_v} <= {KICK_TOO_CLOSE_V}, cx={best_cx}'
            )
            return FAIL_TOO_CLOSE

        if best_v >= KICK_TOO_FAR_V:
            self.kick_position_judge = 'TOO_FAR'
            self._set_failure(
                FAIL_TOO_FAR,
                f'kick前球太遠 best_v={best_v} >= {KICK_TOO_FAR_V}, cx={best_cx}'
            )
            return FAIL_TOO_FAR

        if best_cx < IMG_CX - KICK_SIDE_OFFSET_TOL:
            self.kick_position_judge = 'BALL_OFFSET_LEFT'
            self._set_failure(
                FAIL_BALL_SIDE_OFFSET,
                f'kick前球太偏左 cx={best_cx} < {IMG_CX - KICK_SIDE_OFFSET_TOL}, best_v={best_v}'
            )
            return FAIL_BALL_SIDE_OFFSET

        if best_cx > IMG_CX + KICK_SIDE_OFFSET_TOL:
            self.kick_position_judge = 'BALL_OFFSET_RIGHT'
            self._set_failure(
                FAIL_BALL_SIDE_OFFSET,
                f'kick前球太偏右 cx={best_cx} > {IMG_CX + KICK_SIDE_OFFSET_TOL}, best_v={best_v}'
            )
            return FAIL_BALL_SIDE_OFFSET

        return FAIL_NONE

    def _diagnose_orbit_quality(self, source_name, target_frames):
        """
        [新增] 繞球品質診斷：
        第一版先用計算出的 target_frames 粗略判斷。
        target_frames 太少 → 可能繞球不足
        target_frames 太多 → 可能繞球過頭
        這一版只紀錄，不改變 adjust_position 動作。
        """
        self.orbit_quality_judge = 'OK'

        if target_frames < ORBIT_MIN_FRAMES:
            self.orbit_quality_judge = 'ORBIT_NOT_ENOUGH'
            self._set_failure(
                FAIL_ORBIT_NOT_ENOUGH,
                f'{source_name} 繞球目標幀數太少 target={target_frames} < {ORBIT_MIN_FRAMES}'
            )
            return FAIL_ORBIT_NOT_ENOUGH

        if target_frames > ORBIT_MAX_FRAMES:
            self.orbit_quality_judge = 'ORBIT_OVER'
            self._set_failure(
                FAIL_ORBIT_OVER,
                f'{source_name} 繞球目標幀數過多 target={target_frames} > {ORBIT_MAX_FRAMES}'
            )
            return FAIL_ORBIT_OVER

        return FAIL_NONE

    def _reset_kick_scan(self):
        """
        [新增] 重置 kick 掃描流程。
        recovery 完成後呼叫，讓 kick 從 prepare 重新開始掃球。
        """
        self._kick_phase = 'prepare'
        self._kick_scan_idx = 0
        self._kick_scan_wait = 0
        self._kick_scan_samples = []
        self._kick_selected_cx = -1
        self._kick_selected_area = 0

        self.kick_debug_visible = False
        self.kick_debug_cx = -1
        self.kick_debug_side = 're_scan'
        self.kick_debug_sector = 0

        self.kick_debug_best_v = 0
        self.kick_position_judge = 'RE_SCAN'

    def _start_kick_recovery(self, action, failure_type, detail='', record_failure=True):
        """
        [新增] 啟動 kick 前位置修正。
        這裡只設定 recovery 狀態，不直接走路。

        action:
          'BACK'       : 後退一小步
          'FORWARD'    : 前進一小步
          'SIDE_LEFT'  : 往一側平移
          'SIDE_RIGHT' : 往另一側平移

        record_failure:
          True  = 啟動 recovery 時順便呼叫 _set_failure()
          False = failure 已經在診斷函式中記錄過，避免重複累計
        """
        if self.kick_recovery_count >= KICK_RECOVERY_RETRY_MAX:
            self.action_detail = (
                f'kick recovery 已達上限 '
                f'{self.kick_recovery_count}/{KICK_RECOVERY_RETRY_MAX}，'
                f'不再修正，保留原本流程'
            )
            return False

        self.kick_recovery_count += 1
        self.kick_recovery_action = action
        self.kick_recovery_from_failure = failure_type
        self.kick_recovery_settle = 0

        if action == 'BACK':
            self.kick_recovery_frames = KICK_RECOVERY_BACK_FRAMES
        elif action == 'FORWARD':
            self.kick_recovery_frames = KICK_RECOVERY_FORWARD_FRAMES
        elif action == 'SIDE_LEFT':
            self.kick_recovery_frames = KICK_RECOVERY_SIDE_FRAMES
        elif action == 'SIDE_RIGHT':
            self.kick_recovery_frames = KICK_RECOVERY_SIDE_FRAMES
        else:
            self.kick_recovery_frames = 0
            return False

        if record_failure:
            self._set_failure(failure_type, detail)

        self._kick_phase = 'recover'

        self.action_detail = (
            f'啟動 kick recovery：action={action} '
            f'failure={failure_type} '
            f'count={self.kick_recovery_count}/{KICK_RECOVERY_RETRY_MAX}'
        )
        return True

    def _process_kick_recovery(self):
        """
        [新增] 執行 kick 前位置修正。
        修正完成後停幾幀，再重新進 kick scan。
        """
        self.sendbodyAuto(1)

        # --------------------------------------------------
        # recovery 動作還沒做完：持續送一個小步伐命令
        # --------------------------------------------------
        if self.kick_recovery_frames > 0:
            x = 0
            y = 0
            theta = 0

            if self.kick_recovery_action == 'BACK':
                x = KICK_RECOVERY_BACK_X
            elif self.kick_recovery_action == 'FORWARD':
                x = KICK_RECOVERY_FORWARD_X
            elif self.kick_recovery_action == 'SIDE_LEFT':
                y = KICK_RECOVERY_SIDE_Y
            elif self.kick_recovery_action == 'SIDE_RIGHT':
                y = -KICK_RECOVERY_SIDE_Y

            self.sendContinuousValue(x=x, y=y, theta=theta)
            self.kick_recovery_frames -= 1

            self.action_detail = (
                f'kick recovery 執行中 action={self.kick_recovery_action} '
                f'x={x} y={y} theta={theta} '
                f'剩餘={self.kick_recovery_frames}'
            )
            return

        # --------------------------------------------------
        # recovery 動作做完，先停下等待穩定
        # --------------------------------------------------
        self.sendContinuousValue(x=0, y=0, theta=0)
        self.sendbodyAuto(0)

        self.kick_recovery_settle += 1
        self.action_detail = (
            f'kick recovery 完成，停下穩定中 '
            f'{self.kick_recovery_settle}/{KICK_RECOVERY_SETTLE_FRAMES}'
        )

        if self.kick_recovery_settle < KICK_RECOVERY_SETTLE_FRAMES:
            return

        # --------------------------------------------------
        # 穩定完成，重新 kick scan
        # --------------------------------------------------
        self.kick_recovery_action = 'NONE'
        self.kick_recovery_frames = 0
        self.kick_recovery_settle = 0

        self._reset_kick_scan()
        self.action_detail = 'kick recovery 完成 ✅ → 重新 kick 掃描'

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
        scan_finished = False

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
                    scan_finished = True

        self.sendHeadMotor(1, self.head_h, HEAD_SPEED)
        return scan_finished
    
    def _detect_white_line(self):
        """
        [修改] 偵測畫面下方白線。
        不只用 area，而是用 bbox 形狀判斷：
          1. 在畫面下半部
          2. 寬度夠大
          3. 高度不要太大
          4. aspect_ratio = w / h 夠大，像橫線
          5. 下緣 y + h 靠近畫面底部
          6. 連續 LINE_CONFIRM_FRAMES 幀看到才觸發
        回傳：
          visible: 是否確認看到白線
          cx: 白線中心 x
          area: 白線面積
        """
        objs = self.get_objects(COLOR_LINE)

        if not objs:
            self.line_confirm_count = 0
            return False, 0, 0

        candidates = []

        for o in objs:
            try:
                cx = o['centroid'][0]
                cy = o['centroid'][1]
                area = o['area']
                x, y, w, h = o['bbox']
            except Exception:
                continue

            if h <= 0:
                continue

            aspect = float(w) / float(h)
            bottom_y = y + h

            if (
                cy > LINE_MIN_Y
                and w > LINE_MIN_WIDTH
                and h < LINE_MAX_HEIGHT
                and aspect > LINE_MIN_ASPECT
                and bottom_y > LINE_MIN_BOTTOM_Y
            ):
                candidates.append({
                    'cx': cx,
                    'cy': cy,
                    'area': area,
                    'w': w,
                    'h': h,
                    'aspect': aspect,
                    'bottom_y': bottom_y,
                })

        if not candidates:
            self.line_confirm_count = 0
            return False, 0, 0

        # 優先選最靠近畫面底部的白線，不是單純選最大面積
        best = max(candidates, key=lambda o: o['bottom_y'])

        self.line_confirm_count += 1

        if self.line_confirm_count < LINE_CONFIRM_FRAMES:
            return False, best['cx'], best['area']

        return True, best['cx'], best['area']

    def _read_imu_yaw(self):
        """
        讀取 API callback 更新的 IMU yaw。
        API 常見資料格式：
          self.imu_rpy = [roll, pitch, yaw]
          或 self.yaw
        """
        try:
            return float(self.imu_rpy[2])
        except Exception:
            pass

        try:
            return float(self.yaw)
        except Exception:
            pass

        return None

    def _normalize_yaw_error(self, target_yaw, current_yaw):
        """
        把 yaw 誤差限制在 -180 ~ +180。
        避免 179 度到 -179 度時誤差爆掉。
        """
        err = target_yaw - current_yaw

        while err > 180.0:
            err -= 360.0

        while err < -180.0:
            err += 360.0

        return err

    def _normalize_yaw_angle(self, yaw):
        """把 yaw 角度整理到 -180 ~ +180。"""
        while yaw > 180.0:
            yaw -= 360.0

        while yaw < -180.0:
            yaw += 360.0

        return yaw

    def _start_line_imu_turn(self, line_cx):
        """
        看到白線後，啟動 IMU 旋轉 90 度。
        回傳 True：成功啟動 IMU 避線
        回傳 False：IMU 讀不到，已改用固定幀數 fallback
        """
        yaw = self._read_imu_yaw()

        if yaw is None:
            # IMU 讀不到，就退回原本固定幀數避線
            self.line_avoid_frames = LINE_AVOID_FRAMES

            if line_cx < IMG_CX:
                self.line_avoid_dir = -1
            else:
                self.line_avoid_dir = 1

            self.action_detail = '看到白線，但讀不到 IMU yaw → 改用固定幀數避線'
            return False

        self.line_turn_start_yaw = yaw

        if LINE_TURN_BY_LINE_POSITION:
            # 白線在左 → 右轉；白線在右 → 左轉
            # 如果實測方向反了，改 LINE_TURN_SIGN = -1
            raw_sign = -1 if line_cx < IMG_CX else 1
            target = yaw + raw_sign * LINE_TURN_SIGN * LINE_TURN_ANGLE_DEG
        else:
            target = yaw + LINE_TURN_SIGN * LINE_TURN_ANGLE_DEG

        self.line_turn_target_yaw = self._normalize_yaw_angle(target)
        self.line_turn_timeout = 0
        self.line_turn_active = True
        self.line_turn_cooldown_frames = 0

        self.sendbodyAuto(1)
        self.sendContinuousValue(x=0, y=0, theta=0)

        self.action_detail = (
            f'看到白線，啟動 IMU 90度避線：'
            f'start_yaw={self.line_turn_start_yaw:.1f} '
            f'target_yaw={self.line_turn_target_yaw:.1f} '
            f'line_cx={line_cx}'
        )

        return True

    def _process_line_imu_turn(self):
        """
        執行白線避開：靠 IMU yaw 轉到 target_yaw。
        回傳 True：目前仍在避線，要直接 return
        回傳 False：避線完成或沒有避線，可繼續原本流程
        """
        if not self.line_turn_active:
            return False

        yaw = self._read_imu_yaw()

        if yaw is None:
            self.sendContinuousValue(x=0, y=0, theta=0)
            self.sendbodyAuto(0)

            self.line_turn_active = False
            self.line_turn_timeout = 0
            self.action_detail = '白線 IMU 避線失敗：讀不到 yaw，停止'
            return False

        err = self._normalize_yaw_error(self.line_turn_target_yaw, yaw)
        abs_err = abs(err)

        # 已經轉到目標
        if abs_err <= LINE_YAW_TOL_DEG:
            self.sendContinuousValue(x=0, y=0, theta=0)
            self.sendbodyAuto(0)

            self.line_turn_active = False
            self.line_turn_timeout = 0
            self.line_turn_cooldown_frames = LINE_TURN_COOLDOWN_FRAMES

            # 轉完後重置搜尋層，避免頭停在奇怪位置
            self.search_dir = 'right'
            self.search_v_idx = 0

            self.action_detail = (
                f'白線 IMU 避線完成 ✅ '
                f'yaw={yaw:.1f} target={self.line_turn_target_yaw:.1f} '
                f'err={err:+.1f}'
            )
            return False

        # 超時保護，避免 IMU 沒動或卡住時一直轉
        self.line_turn_timeout += 1
        if self.line_turn_timeout >= LINE_IMU_TURN_TIMEOUT_FRAMES:
            self.sendContinuousValue(x=0, y=0, theta=0)
            self.sendbodyAuto(0)

            self.line_turn_active = False
            self.line_turn_timeout = 0
            self.line_turn_cooldown_frames = LINE_TURN_COOLDOWN_FRAMES

            self.action_detail = (
                f'白線 IMU 避線超時 ⚠️ '
                f'yaw={yaw:.1f} target={self.line_turn_target_yaw:.1f} '
                f'err={err:+.1f}'
            )
            return False

        # 依 yaw 誤差決定轉速
        if abs_err > 25:
            theta = LINE_IMU_TURN_THETA_FAST
        else:
            theta = LINE_IMU_TURN_THETA_SLOW

        if err < 0:
            theta = -theta

        theta *= LINE_IMU_THETA_SIGN

        self.sendbodyAuto(1)
        self.sendContinuousValue(x=0, y=0, theta=theta)

        # 轉向時頭還是繼續掃球
        self._search_head()

        self.action_detail = (
            f'白線 IMU 90度避線中 '
            f'yaw={yaw:.1f} target={self.line_turn_target_yaw:.1f} '
            f'err={err:+.1f} theta={theta} '
            f'timeout={self.line_turn_timeout}/{LINE_IMU_TURN_TIMEOUT_FRAMES}'
        )

        return True

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
            # 有看到球，停止邊走邊找
            if self.find_ball_walk_search:
                self.sendContinuousValue(x=0, y=0, theta=0)
                self.sendbodyAuto(0)
                self.find_ball_walk_search = False

                # 若找到球時正在避線，立即取消避線狀態
                self.line_turn_active = False
                self.line_turn_timeout = 0
                self.line_avoid_frames = 0
                self.line_turn_cooldown_frames = 0

                self.search_v_idx = 0
                self.ball_centered_count = 0
                self.find_ball_stop_settle_frames = FIND_BALL_STOP_SETTLE_FRAMES

            if self.find_ball_stop_settle_frames > 0:
                self.sendContinuousValue(x=0, y=0, theta=0)
                self.sendbodyAuto(0)
                self.find_ball_stop_settle_frames -= 1
                self.action_detail = (
                    f'找到球，停步穩定中 '
                    f'{self.find_ball_stop_settle_frames}/{FIND_BALL_STOP_SETTLE_FRAMES}'
                )
                return

            self.ball_lost_count = 0
            centered = self._track_object(self.ball.cx, self.ball.cy)

            if centered:
                self.ball_centered_count += 1
                self.action_detail = (
                    f'球對準中心 ✅  '
                    f'確認中 {self.ball_centered_count}/{BALL_CENTERED_FRAMES}'
                )

                if self.ball_centered_count >= BALL_CENTERED_FRAMES:
                    self.ball_centered_count = 0
                    self.sub_state = 'turn_to_ball'
                    self.state = 'approach_ball'
                    self.sendbodyAuto(1)
                    self.action_detail = '進入 approach_ball → turn_to_ball'

            else:
                self.ball_centered_count = 0
                self.action_detail = (
                    f'追蹤球中 cx={self.ball.cx} cy={self.ball.cy}'
                )

        else:
            self.ball_centered_count = 0
            self.ball_lost_count += 1

            # --------------------------------------------------
            # 已經進入「邊走邊找」模式
            # --------------------------------------------------
            if self.find_ball_walk_search:
                # --------------------------------------------------
                # [新增] 若正在執行白線 IMU 轉 90 度，優先處理
                # --------------------------------------------------
                if self._process_line_imu_turn():
                    return

                # 轉完後冷卻幾幀，避免同一條白線立刻又觸發一次
                if self.line_turn_cooldown_frames > 0:
                    self.line_turn_cooldown_frames -= 1

                # --------------------------------------------------
                # [新增] IMU 讀不到時的 fallback：固定幀數避線
                # --------------------------------------------------
                if self.line_avoid_frames > 0:
                    self.sendbodyAuto(1)

                    theta = self.line_avoid_dir * LINE_TURN_THETA
                    self.sendContinuousValue(
                        x=LINE_BACK_X,
                        y=0,
                        theta=theta
                    )

                    self.line_avoid_frames -= 1
                    self._search_head()

                    self.action_detail = (
                        f'白線避線 fallback 中 theta={theta} '
                        f'剩餘 {self.line_avoid_frames}/{LINE_AVOID_FRAMES} '
                        f'head_h={self.head_h}'
                    )
                    return

                # --------------------------------------------------
                # [新增] 邊走邊找球時，若看到白線，啟動 IMU 旋轉 90 度
                # --------------------------------------------------
                if self.line_turn_cooldown_frames <= 0:
                    line_visible, line_cx, line_area = self._detect_white_line()

                    if line_visible:
                        started = self._start_line_imu_turn(line_cx)

                        # IMU 轉向成功啟動，下一幀開始持續轉到 90 度
                        if started:
                            return

                        # IMU 讀不到時，_start_line_imu_turn 會設定 line_avoid_frames，
                        # 下一幀會進入 fallback；這一幀先停一下。
                        self.sendbodyAuto(1)
                        self.sendContinuousValue(x=0, y=0, theta=0)
                        return

                # --------------------------------------------------
                # 沒看到白線，正常邊走邊找球
                # --------------------------------------------------
                self.sendbodyAuto(1)
                self.sendContinuousValue(x=200, y=0, theta=0)

                self._search_head()

                self.action_detail = (
                    f'邊走邊找球中 x=200  '
                    f'dir={self.search_dir}  '
                    f'層={self.search_v_idx}  '
                    f'V={HEAD_SEARCH_V_LEVELS[self.search_v_idx]}  '
                    f'head_h={self.head_h}'
                )
                return

            # --------------------------------------------------
            # 還沒進入邊走邊找：
            # 先原地分層掃描
            # --------------------------------------------------
            self.sendContinuousValue(x=0, y=0, theta=0)
            self.sendbodyAuto(0)

            scan_finished = self._search_head()

            if scan_finished:
                self.find_ball_walk_search = True
                self.ball_lost_count = 0

                self.sendbodyAuto(1)
                self.sendContinuousValue(x=200, y=0, theta=0)

                self.action_detail = (
                    f'分層掃描完整輪仍沒看到球 ✅ '
                    f'→ 開始邊走邊找 x=200'
                )
            else:
                self.action_detail = (
                    f'原地分層掃描中 dir={self.search_dir}  '
                    f'層={self.search_v_idx}/{len(HEAD_SEARCH_V_LEVELS)-1}  '
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
                self._set_failure(FAIL_BALL_LOST, f'approach_ball 球消失太久 lost={self.ball_lost_count}')
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
                self._set_failure(FAIL_TARGET_LOST, 'find_goal 對準過程中球門追丟')
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

                # [新增] 只診斷繞球量是否可能不足或過多，不改變原本流程
                self._diagnose_orbit_quality('find_goal', self._orbit_target_frames)

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
                self._set_failure(FAIL_TARGET_LOST, 'find_pole 對準過程中藍柱追丟')
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

                # [新增] 只診斷繞球量是否可能不足或過多，不改變原本流程
                self._diagnose_orbit_quality('find_pole', self._orbit_target_frames)

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
        - 達到 _orbit_target_frames 後進入 kick

        重點：
        - adjust_position 過程中頭本來就在追球
        - 所以每次看得到球，就把當下球的 cx/cy 和頭部 h/v 記錄起來
        - kick 時直接用這個最後記錄的位置判斷左右腳，不再重新找球
        """
        # 到達目標幀數 → 完成修正
        if self._orbit_frames >= self._orbit_target_frames:
            self.sendContinuousValue(x=0, y=0, theta=0)
            self.sendbodyAuto(0)
            self._kick_wait_frames = 0
            # [新增] 每次進入 kick 都從 prepare 開始，避免沿用上一次掃描狀態
            self._kick_phase = 'prepare'
            self._kick_scan_idx = 0
            self._kick_scan_wait = 0
            self._kick_scan_samples = []
            self._kick_selected_cx = -1
            self._kick_selected_area = 0
            self.kick_debug_best_v = 0
            self.kick_position_judge = 'SCANNING'

            # [新增] 每次重新進入 kick 時，重置 recovery 狀態。
            self.kick_recovery_count = 0
            self.kick_recovery_action = 'NONE'
            self.kick_recovery_frames = 0
            self.kick_recovery_settle = 0
            self.kick_recovery_from_failure = FAIL_NONE

            # [新增] 進入 kick 當下記錄 yaw，後面用來診斷踢前姿態是否又偏掉。
            yaw = self._read_imu_yaw()
            if yaw is not None:
                self.kick_target_yaw = yaw
            self.kick_yaw_error = 0.0

            self.state = 'kick'
            self.action_detail = (
                f'軌道修正完成 ✅ → kick  '
                f'最後球位置 visible={self.kick_ref_visible} '
                f'cx={self.kick_ref_cx} cy={self.kick_ref_cy}'
            )
            return

        # 頭部追蹤球（H+V），同時用 head_v 判斷距離
        if self.ball.visible:
            # 先記錄「這一幀」看到球的位置，給 kick 判斷左右腳用
            self.kick_ref_visible = True
            self.kick_ref_cx = self.ball.cx
            self.kick_ref_cy = self.ball.cy
            self.kick_ref_head_h = self.head_h
            self.kick_ref_head_v = self.head_v

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
            f'head_v={self.head_v}  v_err={v_err}  '
            f'kick_ref cx={self.kick_ref_cx} cy={self.kick_ref_cy}'
        )

    def _state_kick(self):
        """
        踢球：
        [只掃 Vertical 的慢速掃描版]

        流程：
        1. 停止走路
        2. 先把 Horizontal 固定到 KICK_SCAN_H = 2048
        3. 等 KICK_SCAN_SETTLE_FRAMES 幀，讓水平頭部真的到位
        4. 掃描時 Horizontal 不再動，只掃 Vertical
        5. 每個 V 層等待 KICK_SCAN_WAIT_FRAMES 幀，再讀一次球的位置
        6. 收集看到的 ball.cx / area
        7. 用 area 最大的那筆判斷左腳或右腳
        """

        # ------------------------------------------------------------
        # phase 0.5：kick 前位置 recovery
        # ------------------------------------------------------------
        if self._kick_phase == 'recover':
            self._process_kick_recovery()
            return

        # ------------------------------------------------------------
        # phase 1：準備，停止走路，H 拉到 2048，V 到第一層
        # ------------------------------------------------------------
        if self._kick_phase == 'prepare':
            self.sendContinuousValue(x=0, y=0, theta=0)
            self.sendbodyAuto(0)

            self._kick_scan_idx = 0
            self._kick_scan_wait = 0
            self._kick_scan_samples = []
            self._kick_selected_cx = -1
            self._kick_selected_area = 0
            self._kick_prepare_from_h = self.head_h
            self.kick_debug_best_v = 0
            self.kick_position_judge = 'SCANNING'

            # [重點] 只在 prepare 先把 Horizontal 拉回 2048。
            # settle/scan 會持續補送同一個 H 目標，避免實體頭部還停在上一狀態。
            self.head_h = KICK_SCAN_H
            self.head_v = KICK_SCAN_V_LEVELS[self._kick_scan_idx]
            self.sendHeadMotor(1, self.head_h, KICK_SCAN_HEAD_SPEED)
            self.sendHeadMotor(2, self.head_v, KICK_SCAN_HEAD_SPEED)

            self.kick_debug_visible = False
            self.kick_debug_cx = -1
            self.kick_debug_side = 'settle_head'
            self.kick_debug_sector = 0
            self.kick_ref_head_h = self.head_h
            self.kick_ref_head_v = self.head_v

            self.action_detail = (
                f'kick準備：H from {self._kick_prepare_from_h} → {self.head_h}，'
                f'Vertical 從 V={self.head_v} 開始，等待頭部到位'
            )

            self._kick_phase = 'settle'
            return

        # ------------------------------------------------------------
        # phase 1.5：等待 H=2048 到位
        # ------------------------------------------------------------
        if self._kick_phase == 'settle':
            # 這裡不掃描，只是等頭部穩定。每幀補送 H=2048，
            # 避免上一個狀態留下的 H 實體位置或延遲指令影響 kick。
            self.head_h = KICK_SCAN_H
            self.head_v = KICK_SCAN_V_LEVELS[0]
            self.sendHeadMotor(1, self.head_h, KICK_SCAN_HEAD_SPEED)
            self.sendHeadMotor(2, self.head_v, KICK_SCAN_HEAD_SPEED)

            self._kick_scan_wait += 1

            self.kick_ref_head_h = self.head_h
            self.kick_ref_head_v = self.head_v

            self.action_detail = (
                f'kick等待頭部到位：H={self.head_h} '
                f'from={self._kick_prepare_from_h} V={self.head_v} '
                f'等待 {self._kick_scan_wait}/{KICK_SCAN_SETTLE_FRAMES}'
            )

            if self._kick_scan_wait < KICK_SCAN_SETTLE_FRAMES:
                return

            self._kick_scan_wait = 0
            self._kick_phase = 'scan'
            return

        # ------------------------------------------------------------
        # phase 2：只掃 Vertical，慢慢收集球的位置
        # ------------------------------------------------------------
        if self._kick_phase == 'scan':
            # scan 只改 Vertical，但仍固定補送 H=2048，避免頭部被上一狀態或動作包帶走。
            self.head_h = KICK_SCAN_H
            self.sendHeadMotor(1, self.head_h, KICK_SCAN_HEAD_SPEED)

            target_v = KICK_SCAN_V_LEVELS[self._kick_scan_idx]

            # [重點] scan 階段 Horizontal 固定在 KICK_SCAN_H，只掃 Vertical。
            if self.head_v != target_v:
                self.head_v = target_v
                self.sendHeadMotor(2, self.head_v, KICK_SCAN_HEAD_SPEED)
                self._kick_scan_wait = 0
                return

            self._kick_scan_wait += 1

            self.action_detail = (
                f'kick慢速掃描中：H固定={self.head_h}  '
                f'V層 {self._kick_scan_idx + 1}/{len(KICK_SCAN_V_LEVELS)} '
                f'V={target_v}  '
                f'等待 {self._kick_scan_wait}/{KICK_SCAN_WAIT_FRAMES}'
            )

            if self._kick_scan_wait < KICK_SCAN_WAIT_FRAMES:
                return

            # 等夠後，讀一次球的位置。
            self.ball.update()

            if self.ball.visible:
                self._kick_scan_samples.append({
                    'cx': self.ball.cx,
                    'cy': self.ball.cy,
                    'area': self.ball.area,
                    'head_h': self.head_h,
                    'head_v': self.head_v,
                })

                self.kick_debug_visible = True
                self.kick_debug_cx = self.ball.cx
                self.kick_ref_head_h = self.head_h
                self.kick_ref_head_v = self.head_v

                self.action_detail = (
                    f'kick掃描看到球：H固定={self.head_h} V={self.head_v} '
                    f'cx={self.ball.cx} cy={self.ball.cy} area={self.ball.area}'
                )

            # 換下一層 V
            self._kick_scan_idx += 1
            self._kick_scan_wait = 0

            if self._kick_scan_idx >= len(KICK_SCAN_V_LEVELS):
                self._kick_phase = 'decide'
                return

            # 只改 Vertical
            self.head_v = KICK_SCAN_V_LEVELS[self._kick_scan_idx]
            self.sendHeadMotor(2, self.head_v, KICK_SCAN_HEAD_SPEED)
            return

        # ------------------------------------------------------------
        # phase 3：根據掃描結果決定左右腳
        # ------------------------------------------------------------
        if self._kick_phase == 'decide':
            # [新增] kick 前姿態診斷，只記錄，不在球旁強制補轉。
            # 若後續同時發生距離/位置問題，畫面會優先顯示距離/位置診斷。
            self._diagnose_pre_kick_yaw()

            if len(self._kick_scan_samples) >= KICK_SCAN_MIN_SAMPLES:
                # 選 area 最大的那筆，通常代表球看得最清楚 / 最近
                best = max(self._kick_scan_samples, key=lambda s: s['area'])

                self._kick_selected_cx = best['cx']
                self._kick_selected_area = best['area']

                self.kick_debug_visible = True
                self.kick_debug_cx = best['cx']
                self.kick_ref_cx = best['cx']
                self.kick_ref_cy = best['cy']
                self.kick_ref_head_h = best['head_h']
                self.kick_ref_head_v = best['head_v']

                # --------------------------------------------------
                # 先保留原本左右腳判斷
                # --------------------------------------------------
                if best['cx'] < IMG_CX - KICK_SIDE_DEADZONE:
                    self.kick_debug_side = 'left_scan_v_only'
                    self.kick_debug_sector = 200
                    self.action_detail = (
                        f'kick掃描判斷：best H={best["head_h"]} V={best["head_v"]} '
                        f'cx={best["cx"]} < {IMG_CX - KICK_SIDE_DEADZONE} '
                        f'→ 左腳 sector=200'
                    )

                elif best['cx'] > IMG_CX + KICK_SIDE_DEADZONE:
                    self.kick_debug_side = 'right_scan_v_only'
                    self.kick_debug_sector = 100
                    self.action_detail = (
                        f'kick掃描判斷：best H={best["head_h"]} V={best["head_v"]} '
                        f'cx={best["cx"]} > {IMG_CX + KICK_SIDE_DEADZONE} '
                        f'→ 右腳 sector=100'
                    )

                else:
                    self.kick_debug_side = 'center_default_scan_v_only'
                    self.kick_debug_sector = KICK_DEFAULT_SECTOR
                    self.action_detail = (
                        f'kick掃描判斷：best H={best["head_h"]} V={best["head_v"]} '
                        f'cx={best["cx"]} 接近中心 IMG_CX={IMG_CX} '
                        f'→ 預設 sector={KICK_DEFAULT_SECTOR}'
                    )

                # --------------------------------------------------
                # [新增] kick 前球位置診斷 + recovery
                # 只針對第一次位置異常做一次小修正，修正後重新 scan。
                # --------------------------------------------------
                pos_failure = self._diagnose_kick_position(best)

                if pos_failure == FAIL_TOO_CLOSE:
                    started = self._start_kick_recovery(
                        'BACK',
                        FAIL_TOO_CLOSE,
                        (
                            f'kick前球太近 best_v={best["head_v"]} <= {KICK_TOO_CLOSE_V} '
                            f'→ 後退重掃'
                        ),
                        record_failure=False
                    )
                    if started:
                        return

                elif pos_failure == FAIL_TOO_FAR:
                    started = self._start_kick_recovery(
                        'FORWARD',
                        FAIL_TOO_FAR,
                        (
                            f'kick前球太遠 best_v={best["head_v"]} >= {KICK_TOO_FAR_V} '
                            f'→ 前進重掃'
                        ),
                        record_failure=False
                    )
                    if started:
                        return

                elif pos_failure == FAIL_BALL_SIDE_OFFSET:
                    if best['cx'] < IMG_CX:
                        action = 'SIDE_LEFT'
                        side_text = '球太偏左 → 側移重掃'
                    else:
                        action = 'SIDE_RIGHT'
                        side_text = '球太偏右 → 側移重掃'

                    started = self._start_kick_recovery(
                        action,
                        FAIL_BALL_SIDE_OFFSET,
                        f'{side_text} cx={best["cx"]}, IMG_CX={IMG_CX}',
                        record_failure=False
                    )
                    if started:
                        return

            else:
                # --------------------------------------------------
                # [新增] KICK_NO_BALL recovery：第一次沒看到球先後退重掃。
                # 若 recovery 已達上限，才保留原本預設腳踢的流程。
                # --------------------------------------------------
                self.kick_debug_visible = False
                self.kick_debug_cx = -1
                self.kick_debug_side = 'no_ball_scan_v_only'
                self.kick_debug_sector = KICK_DEFAULT_SECTOR

                detail = f'kick掃描沒看到球 samples={len(self._kick_scan_samples)} → 後退重掃'
                started = self._start_kick_recovery(
                    'BACK',
                    FAIL_KICK_NO_BALL,
                    detail,
                    record_failure=True
                )
                if started:
                    return

                # 已經修正過仍然看不到球，保留原本流程：記錄後用預設腳。
                self._set_failure(
                    FAIL_KICK_NO_BALL,
                    f'kick掃描仍沒看到球，recovery已達上限，samples={len(self._kick_scan_samples)}'
                )
                self.action_detail = (
                    f'kick掃描全部沒看到球，samples={len(self._kick_scan_samples)} '
                    f'→ recovery已達上限，預設 sector={KICK_DEFAULT_SECTOR}'
                )

            self._kick_phase = 'execute'
            return

        # ------------------------------------------------------------
        # phase 4：執行踢球動作，只執行一次
        # ------------------------------------------------------------
        if self._kick_phase == 'execute':
            sector = self.kick_debug_sector

            self.sendContinuousValue(x=0, y=0, theta=0)
            self.sendbodyAuto(0)

            time.sleep(2)
            self.sendBodySector(999)

            # 左腳你原本想等久一點，保留 4 秒；右腳維持 2 秒
            if sector == 200:
                time.sleep(4)
            else:
                time.sleep(2)

            self.sendBodySector(sector)
            time.sleep(14)
            self.sendBodySector(29)
            time.sleep(1)

            self.sendBodySector(123)
            time.sleep(1)


            self._kick_wait_frames = 0
            self._kick_phase = 'wait_done'
            return

        # ------------------------------------------------------------
        # phase 5：踢完後初始化
        # ------------------------------------------------------------
        if self._kick_phase == 'wait_done':
            self._kick_wait_frames += 1

            if self._kick_wait_frames >= KICK_WAIT_FRAMES:
                self._initialize()
                self.action_detail = '踢球完成 ✅ → 初始化 → find_ball'


    def _state_penalty_kick(self):
        """
        PENALTY_KICK 策略：
        只做射門動作，不找球、不找柱、不繞球。
        執行一次後停在 penalty_done，避免一直重複踢。
        """
        self.sendContinuousValue(x=0, y=0, theta=0)
        self.sendbodyAuto(0)

        self.action_detail = 'PENALTY_KICK：左腳射門 sector={200}'

        time.sleep(2)
        self.sendBodySector(999)   # 踢球前預備
        time.sleep(2)
        self.sendBodySector(200)
        time.sleep(14)
        self.sendBodySector(29)    # 回初始站姿
        time.sleep(1)

        self.sendBodySector(123)
        time.sleep(1)

        self.state = 'penalty_done'
        self.action_detail = 'PENALTY_KICK 完成 ✅ 停在 penalty_done'


    def _initialize(self):
        """每次撥開關啟動或踢球完成後執行：頭部歸中、停走、站穩後重新校正 IMU、重置狀態。"""
        self.sendContinuousValue(x=0, y=0, theta=0)
        self.sendbodyAuto(0)

        self.head_h = HEAD_H_CENTER
        self.head_v = HEAD_V_CENTER
        self.sendHeadMotor(1, HEAD_H_CENTER, HEAD_SPEED)
        self.sendHeadMotor(2, HEAD_V_CENTER, HEAD_SPEED)
        time.sleep(1)
        self.sendBodySector(29)
        time.sleep(1)
        self.sendBodySector(123)
        self.sendContinuousValue(x=0, y=0, theta=0)
        self.sendbodyAuto(0)
        time.sleep(0.5)   # 等身體確實站穩，再校正 IMU 零點
        self.sendSensorReset(True)   # 踢球後姿態可能偏移，重新校正 Yaw/Roll/Pitch
        time.sleep(0.5)

        # [新增] 每次重新初始化時清除目前 failure 顯示，但保留 failure_history 統計。
        self._clear_current_failure()

        self.ball.visible = False
        self.pole.visible = False
        self.goal.visible = False

        self.ball_lost_count = 0
        self.ball_centered_count = 0

        self.find_ball_walk_search = False
        self.find_ball_stop_settle_frames = 0

        self.line_avoid_frames = 0
        self.line_avoid_dir = 1
        self.line_confirm_count = 0

        self.line_turn_active = False
        self.line_turn_target_yaw = 0.0
        self.line_turn_start_yaw = 0.0
        self.line_turn_timeout = 0
        self.line_turn_cooldown_frames = 0

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

        self._kick_phase = 'prepare'
        self._kick_scan_idx = 0
        self._kick_scan_wait = 0
        self._kick_scan_samples = []
        self._kick_selected_cx = -1
        self._kick_selected_area = 0
        self._kick_prepare_from_h = HEAD_H_CENTER

        # [新增] 初始化時重置 kick recovery 狀態
        self.kick_recovery_count = 0
        self.kick_recovery_action = 'NONE'
        self.kick_recovery_frames = 0
        self.kick_recovery_settle = 0
        self.kick_recovery_from_failure = FAIL_NONE



        self.kick_ref_visible = False
        self.kick_ref_cx = 0
        self.kick_ref_cy = 0
        self.kick_ref_head_h = HEAD_H_CENTER
        self.kick_ref_head_v = HEAD_V_FOOT

        self.kick_debug_visible = False
        self.kick_debug_cx = -1
        self.kick_debug_side = 'none'
        self.kick_debug_sector = 0

        self.goal_search_dir = 'right'
        self.goal_search_v_idx = 0
        self.goal_found_h = HEAD_H_CENTER
        self.goal_found_v = HEAD_V_CENTER
        self._goal_confirm_frames = 0

        if STRATEGY_MODE == 'PENALTY_KICK':
            self.state = 'penalty_kick'
            self.action_detail = '初始化完成 → penalty_kick'
        else:
            self.state = 'find_ball'
            self.action_detail = '初始化完成 → find_ball'

        self.sub_state = ''

    # -----------------------------------------------------------------------
    # 主迴圈
    # -----------------------------------------------------------------------

    def main(self):
        # ------------------------------------------------------------
        # 啟動條件：
        # USE_REFEREE_COMM = False → 用原本網頁 Start / Stop
        # USE_REFEREE_COMM = True  → 網頁 Start + referee PLAY + self_enabled
        # ------------------------------------------------------------
        if USE_REFEREE_COMM:
            allow_run = self.is_start and self.ref_play and self.ref_self_enabled
        else:
            allow_run = self.is_start

        if not allow_run:
            if self.initialized:
                self.sendContinuousValue(x=0, y=0, theta=0)
                self.sendbodyAuto(0)
                self.initialized = False

            if USE_REFEREE_COMM:
                self.action_detail = (
                    f'等待裁判指令 state={self.ref_state} '
                    f'{self.ref_state_name} '
                    f'play={self.ref_play} '
                    f'enabled={self.ref_self_enabled}'
                )
            else:
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
        elif self.state == 'penalty_kick':
            self._state_penalty_kick()

        elif self.state == 'penalty_done':
            self.sendContinuousValue(x=0, y=0, theta=0)
            self.sendbodyAuto(0)

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