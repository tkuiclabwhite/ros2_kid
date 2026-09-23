#!/usr/bin/env python3
# coding=utf-8
"""
守門員策略 v1 — find_ball + align_to_ball + defend_ball

狀態機:
  find_ball    : 頭部原地分層掃描搜尋球，找到後 P-controller 對準畫面中心，
                 連續穩定幀數後視為完成（守門員固定站位，不走動，只靠頭部追蹤）
  align_to_ball: 身體左右平移（不轉身、不前進），頭部持續追球，
                 直到頭部水平刻度 head_h 回到中心，代表身體已站在球正前方，
                 對齊完成後進入 defend_ball
  defend_ball  : 頭部持續追球，身體只做小幅平移微調維持在球正前方，
                 持續監看球的面積（面積變大＝球在靠近），超過閾值即觸發撲球
                 sendBodySector(5555)，之後停在 dive_done 不再動作
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
HEAD_V_MAX    = 2200
HEAD_V_MIN    = 1150
HEAD_SPEED    = 30

# --- 追蹤 P 控制器 ---
HEAD_KP_H       = 1.0
HEAD_KP_V       = 1.0
HEAD_MAX_STEP_H = 40
HEAD_MAX_STEP_V = 30
HEAD_TOL_X      = 8
HEAD_TOL_Y      = 12

# --- 搜尋 ---
HEAD_SEARCH_STEP_H   = 70
HEAD_SEARCH_V_LEVELS = [1600, 1750, 1950, 2048]

# --- 球過濾條件 ---
BALL_MIN_AREA       = 50
BALL_ASPECT_MIN     = 0.5
BALL_ASPECT_MAX     = 2.0
BALL_MIN_Y_CENTROID = 20

# --- 球消失容忍 ---
BALL_LOST_FRAMES = 8

# --- find_ball 對準判斷 ---
BALL_CENTERED_FRAMES = 5   # 連續對準中心幾幀才算完成

# --- align_to_ball: 平移對齊 ---
# head_h 判斷方向（已用 HEAD_H_MAX/MIN 的左右定義驗證）：
#   head_h > HEAD_H_CENTER → 頭偏左 → 球在左 → 身體向左平移對齊（y > 0）
#   head_h < HEAD_H_CENTER → 頭偏右 → 球在右 → 身體向右平移對齊（y < 0）
ALIGN_HEAD_TOL_TICKS = 20    # head_h 與中心的誤差在此範圍內視為「已對齊」（可調）
ALIGN_Y_GAIN         = 50   # head_h 誤差刻度 → y 步長係數（可調）
ALIGN_Y_MAX          = 1500   # y 步長上限（可調）

# --- defend_ball: 等待球靠近，平移微調 + 面積觸發撲球 ---
# y 方向判斷與 align_to_ball 相同（head_h 相對中心的誤差），但增益小很多，只做小幅微調
DEFEND_Y_GAIN              = 20     # head_h 誤差刻度 → y 步長係數（比 align 小很多，可調）
DEFEND_Y_MAX               = 500    # y 步長上限（可調）
DEFEND_BALL_AREA_THRESHOLD = 70   # 球面積超過此值視為已逼近，觸發撲球（可調，需依實測調整）

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
                sys.stdout.write("\033[H\033[J")
                sys.stdout.write(
                    f"#========= 守門員策略 v1 ==========#\n"
                    f" state         : {n.state}\n"
                    f" action_detail : {n.action_detail}\n"
                    f"#============== 視覺狀態 ===============#\n"
                    f" ball.visible  : {n.ball.visible}"
                    f"  lost={n.ball_lost_count}/{BALL_LOST_FRAMES}\n"
                    f" ball.cx / cy  : {n.ball.cx} / {n.ball.cy}\n"
                    f" ball.area     : {n.ball.area}\n"
                    f" centered_cnt  : {n.ball_centered_count}/{BALL_CENTERED_FRAMES}\n"
                    f"#============== 頭部狀態 ===============#\n"
                    f" head_h        : {n.head_h}\n"
                    f" head_v        : {n.head_v}\n"
                    f" search_v_idx  : {n.search_v_idx}/{len(HEAD_SEARCH_V_LEVELS)-1}"
                    f"  V={HEAD_SEARCH_V_LEVELS[n.search_v_idx]}\n"
                    f"#=========== align_to_ball 狀態 ==========#\n"
                    f" head_h_err    : {n.head_h - HEAD_H_CENTER}"
                    f"  (容忍={ALIGN_HEAD_TOL_TICKS})\n"
                    f" align_y       : {n.align_debug_y}\n"
                    f"#============ defend_ball 狀態 ===========#\n"
                    f" ball.area     : {n.ball.area}"
                    f"  (閾值={DEFEND_BALL_AREA_THRESHOLD})\n"
                    f"#=======================================#\n"
                )
                sys.stdout.flush()
            except Exception:
                pass
            time.sleep(0.1)

# ===========================================================================
# 主策略節點
# ===========================================================================

class Goalkeeper(API):
    def __init__(self):
        super().__init__('goalkeeper')

        self.ball = BallInfo(self)

        self.head_h = HEAD_H_CENTER
        self.head_v = HEAD_V_CENTER

        self.search_dir   = 'right'
        self.search_v_idx = 0

        self.ball_lost_count     = 0
        self.ball_centered_count = 0   # 對準中心的連續幀計數

        self.align_debug_y = 0   # align_to_ball / defend_ball 目前送出的 y 步長（debug 顯示用）

        self.initialized   = False
        self.state         = 'find_ball'
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
        """分層水平掃描，掃完一輪回傳 True"""
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

    # -----------------------------------------------------------------------
    # find_ball
    # -----------------------------------------------------------------------

    def _state_find_ball(self):
        """
        頭部原地分層掃描搜尋球；看到球後改用 P-controller 對準中心，
        連續 BALL_CENTERED_FRAMES 幀對準即完成。
        守門員固定站位，此狀態全程不移動身體，只靠頭部搜尋與追蹤。
        """
        if self.ball.visible:
            self.ball_lost_count = 0
            centered = self._track_object(self.ball.cx, self.ball.cy)

            if centered:
                self.ball_centered_count += 1
                self.action_detail = (
                    f'球對準中心 ✅  確認中 {self.ball_centered_count}/{BALL_CENTERED_FRAMES}'
                )
                if self.ball_centered_count >= BALL_CENTERED_FRAMES:
                    self.ball_centered_count = 0
                    self.ball_lost_count = 0
                    self.state = 'align_to_ball'
                    self.sendbodyAuto(1)
                    self.action_detail = '球已置中完成 ✅ → align_to_ball'
            else:
                self.ball_centered_count = 0
                self.action_detail = f'追蹤球中 cx={self.ball.cx} cy={self.ball.cy}'
        else:
            self.ball_centered_count = 0
            self.ball_lost_count += 1
            self._search_head()
            self.action_detail = (
                f'原地分層掃描中 dir={self.search_dir}  '
                f'層={self.search_v_idx}/{len(HEAD_SEARCH_V_LEVELS)-1}  '
                f'V={HEAD_SEARCH_V_LEVELS[self.search_v_idx]}  '
                f'head_h={self.head_h}'
            )

    # -----------------------------------------------------------------------
    # align_to_ball
    # -----------------------------------------------------------------------

    def _state_align_to_ball(self):
        """
        身體只做左右平移（x=0, theta=0），頭部持續追球，
        直到 head_h 回到 HEAD_H_CENTER 附近，代表身體已站在球正前方，
        完成後進入 defend_ball。
        球消失超過容忍幀數 → 退回 find_ball。
        """
        if not self.ball.visible:
            self.ball_lost_count += 1
            if self.ball_lost_count <= BALL_LOST_FRAMES:
                self.action_detail = (
                    f'[align_to_ball] 球暫時消失 lost={self.ball_lost_count}/{BALL_LOST_FRAMES}'
                )
                return
            else:
                self.sendContinuousValue(x=0, y=0, theta=0)
                self.sendbodyAuto(0)
                self._reset_head()
                self.state = 'find_ball'
                self.ball_centered_count = 0
                self.action_detail = '球消失太久，退回 find_ball'
                return

        self.ball_lost_count = 0
        self._track_object(self.ball.cx, self.ball.cy)   # 頭部持續追球

        head_h_err = self.head_h - HEAD_H_CENTER

        if abs(head_h_err) < ALIGN_HEAD_TOL_TICKS:
            self.align_debug_y = 0
            self.sendContinuousValue(x=0, y=0, theta=0)
            self.state = 'defend_ball'
            self.action_detail = (
                f'對齊完成 ✅ 站在球正前方 → defend_ball  '
                f'head_h={self.head_h}（誤差={head_h_err}）'
            )
            return

        y = int(head_h_err * ALIGN_Y_GAIN)
        y = max(-ALIGN_Y_MAX, min(ALIGN_Y_MAX, y))
        self.align_debug_y = y

        self.sendContinuousValue(x=-400, y=y, theta=0)
        self.action_detail = (
            f'平移對齊球正前方中  head_h={self.head_h}（誤差={head_h_err}）  y={y}'
        )

    # -----------------------------------------------------------------------
    # defend_ball
    # -----------------------------------------------------------------------

    def _state_defend_ball(self):
        """
        頭部持續追球（H+V），身體只做小幅平移微調維持在球正前方
        （增益比 align_to_ball 小很多，屬於微調而非大幅對齊）。
        持續監看 ball.area，超過 DEFEND_BALL_AREA_THRESHOLD（球靠近）就觸發撲球動作，
        之後停在 dive_done，不再重複觸發。
        球消失超過容忍幀數 → 退回 find_ball。
        """
        if not self.ball.visible:
            self.ball_lost_count += 1
            if self.ball_lost_count <= BALL_LOST_FRAMES:
                self.action_detail = (
                    f'[defend_ball] 球暫時消失 lost={self.ball_lost_count}/{BALL_LOST_FRAMES}'
                )
                return
            else:
                self.sendContinuousValue(x=0, y=0, theta=0)
                self.sendbodyAuto(0)
                self._reset_head()
                self.state = 'find_ball'
                self.ball_centered_count = 0
                self.action_detail = '球消失太久，退回 find_ball'
                return

        self.ball_lost_count = 0
        self._track_object(self.ball.cx, self.ball.cy)   # 頭部持續追球

        if self.ball.area > DEFEND_BALL_AREA_THRESHOLD:
            self.sendContinuousValue(x=0, y=0, theta=0)
            self.sendbodyAuto(0)
            time.sleep(2)
            self.sendBodySector(9797)
            time.sleep(2)
            self.state = 'dive_done'
            self.action_detail = (
                f'球面積 {self.ball.area} 超過閾值 {DEFEND_BALL_AREA_THRESHOLD} ✅ '
                f'→ 撲球 sendBodySector(9797)'
            )
            return

        head_h_err = self.head_h - HEAD_H_CENTER
        y = int(head_h_err * DEFEND_Y_GAIN)
        y = max(-DEFEND_Y_MAX, min(DEFEND_Y_MAX, y))
        self.align_debug_y = y

        self.sendContinuousValue(x=-450, y=y, theta=0)
        self.action_detail = (
            f'等待球靠近中  area={self.ball.area}/{DEFEND_BALL_AREA_THRESHOLD}  '
            f'head_h={self.head_h}（誤差={head_h_err}）  y={y}'
        )

    # -----------------------------------------------------------------------
    # 主迴圈
    # -----------------------------------------------------------------------

    def main(self):
        if not self.is_start:
            if self.initialized:
                self.sendbodyAuto(0)
                self.initialized = False
            self.action_detail = '=== 停止 ==='
            return

        if not self.initialized:
            self._reset_head()
            self.ball_lost_count     = 0
            self.ball_centered_count = 0
            self.align_debug_y       = 0
            self.state = 'find_ball'
            self.initialized = True
            self.action_detail = '初始化完成 → find_ball'
            return

        self.ball.update()

        if self.state == 'find_ball':
            self._state_find_ball()
        elif self.state == 'align_to_ball':
            self._state_align_to_ball()
        elif self.state == 'defend_ball':
            self._state_defend_ball()
        elif self.state == 'dive_done':
            self.sendContinuousValue(x=0, y=0, theta=0)
            self.sendbodyAuto(0)

# ===========================================================================
# 進入點
# ===========================================================================

def main(args=None):
    rclpy.init(args=args)
    node = Goalkeeper()
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
