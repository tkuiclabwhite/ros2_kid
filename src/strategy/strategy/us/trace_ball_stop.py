#!/usr/bin/env python3
# coding=utf-8
"""
United Soccer 策略 v2 — find_ball + approach_ball
狀態機:
  find_ball    : 頭部掃描搜尋球，對準中心連續 10 幀後進入下一狀態
  approach_ball:
    turn_to_ball : 身體旋轉對正球（頭 H 偏角歸零）
    walk_to_ball : 直走靠近，小幅 theta 修正，head_v <= APPROACH_STOP_V 停止
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

COLOR_BALL = 'yellow'

IMG_W  = 320
IMG_H  = 240
IMG_CX = IMG_W // 2
IMG_CY = IMG_H // 2

# --- 頭部馬達 ---
HEAD_H_CENTER = 2048
HEAD_V_CENTER = 1600
HEAD_H_MAX    = 3026   # 往左 90 度
HEAD_H_MIN    = 1024   # 往右 90 度
HEAD_V_MAX    = 2048
HEAD_V_MIN    = 1200
HEAD_V_FOOT   = 1250   # 球到腳邊的 V 刻度
HEAD_SPEED    = 30

# 1 度對應的刻度數：(3026-1024) / 180 ≈ 11.1
HEAD_DEG_PER_TICK = (HEAD_H_MAX - HEAD_H_MIN) / 180.0   # ≈ 11.1

# --- 追蹤 P 控制器 ---
HEAD_KP_H       = 1.0
HEAD_KP_V       = 1.0
HEAD_MAX_STEP_H = 80
HEAD_MAX_STEP_V = 60
HEAD_TOL_X      = 12
HEAD_TOL_Y      = 20

# --- 搜尋 ---
HEAD_SEARCH_STEP_H  = 60
HEAD_SEARCH_V_LEVELS = [1300, 1450, 1600, 1750]

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
APPROACH_STOP_V  = 1250   # head_v 到達此值視為夠靠近（等於 HEAD_V_FOOT）
WALK_X_NORMAL    = 1200    # 正常前進步長
WALK_X_SLOW      = 200    # 靠近時慢速步長
# 直走時小幅 theta 修正：頭 H 偏角換算後乘以此係數
WALK_THETA_GAIN  = 1.15   # 可調整，值越大修正越積極
WALK_THETA_MAX   = 3      # theta 修正上限，避免走太斜
# 直走時 theta 死區：head_h 偏離中心小於此刻度數時 theta=0，讓機器人安心前進
# 建議從 150 開始（約 13 度），走太斜再調小
WALK_THETA_DEAD_TICKS = 200
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
                    f" state         : {n.state}\n"
                    f" sub_state     : {n.sub_state}\n"
                    f" action_detail : {n.action_detail}\n"
                    f"#============== 視覺狀態 ===============#\n"
                    f" ball.visible  : {n.ball.visible}"
                    f"  lost={n.ball_lost_count}/{BALL_LOST_FRAMES}\n"
                    f" ball.cx / cy  : {n.ball.cx} / {n.ball.cy}\n"
                    f" ball.area     : {n.ball.area}\n"
                    f" centered_cnt  : {n.ball_centered_count}/{BALL_CENTERED_FRAMES}\n"
                    f"#============== 頭部狀態 ===============#\n"
                    f" head_h        : {n.head_h}  偏角={h_deg:+.1f}°\n"
                    f" head_v        : {n.head_v}"
                    f"  (停止門檻={APPROACH_STOP_V})\n"
                    f" search_v_idx  : {n.search_v_idx}/{len(HEAD_SEARCH_V_LEVELS)-1}"
                    f"  V={HEAD_SEARCH_V_LEVELS[n.search_v_idx]}\n"
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

        self.head_h = HEAD_H_CENTER
        self.head_v = HEAD_V_CENTER

        self.search_dir    = 'right'
        self.search_v_idx  = 0

        self.ball_lost_count     = 0
        self.ball_centered_count = 0   # 對準中心的連續幀計數

        self.state      = 'find_ball'
        self.sub_state  = ''
        self.action_detail = '初始化'

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
        if self.head_v <= APPROACH_STOP_V:
            self.sendContinuousValue(x=0, y=0, theta=0)   # 步長歸零
            self.sendbodyAuto(0)                           # 停止步態
            self.action_detail = (
                f'抵達球旁 head_v={self.head_v} ✅ → 下一狀態'
            )
            # TODO: 之後在這裡切換到 find_pole
            # self.state = 'find_pole'
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
    # 主迴圈
    # -----------------------------------------------------------------------

    def main(self):
        if not self.is_start:
            self.sendbodyAuto(0)
            return

        self.ball.update()

        if self.state == 'find_ball':
            self._state_find_ball()
        elif self.state == 'approach_ball':
            self._state_approach_ball()

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