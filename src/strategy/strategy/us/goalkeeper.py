#!/usr/bin/env python3
# coding=utf-8
"""
測試腳本：尋找紅色球門（ㄇ字型）
獨立於主策略之外，測試好再合併進 trace_ball_stop.py 的 find_goal 狀態。

狀態機:
  scan        : 分層水平掃描搜尋紅色球門
  center_goal : P-controller 將頭對準球門，連續穩定幀數後記錄 goal_found_h/v
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
HEAD_V_MAX    = 2200
HEAD_V_MIN    = 1200
HEAD_SPEED    = 30

HEAD_DEG_PER_TICK = (HEAD_H_MAX - HEAD_H_MIN) / 180.0   # ≈ 11.1

# --- 追蹤 P 控制器 ---
HEAD_KP_H       = 1.0
HEAD_KP_V       = 1.0
HEAD_MAX_STEP_H = 80
HEAD_MAX_STEP_V = 60
HEAD_TOL_X      = 12
HEAD_TOL_Y      = 20

# --- 球門搜尋（V 偏高，因為要抬頭看橫桿） ---
GOAL_SEARCH_STEP_H   = 60
GOAL_SEARCH_V_LEVELS = [1700, 1850, 2000, 2200]

# --- 球門過濾條件（形狀變化大，先只卡面積，不卡 aspect_ratio） ---
GOAL_MIN_AREA = 80

# --- 對準確認 ---
GOAL_CONFIRM_FRAMES = 5

# ===========================================================================
# 視覺：球門
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
                h_deg = (HEAD_H_CENTER - n.head_h) / HEAD_DEG_PER_TICK
                sys.stdout.write("\033[H\033[J")
                sys.stdout.write(
                    f"#========= find_goal 測試 ==========#\n"
                    f" sub_state     : {n.sub_state}\n"
                    f" action_detail : {n.action_detail}\n"
                    f"#============ 視覺狀態 =============#\n"
                    f" goal.visible  : {n.goal.visible}\n"
                    f" goal.cx / cy  : {n.goal.cx} / {n.goal.cy}\n"
                    f" goal.area     : {n.goal.area}\n"
                    f" goal.aspect   : {n.goal.aspect:.2f}\n"
                    f" goal.x_min/max: {n.goal.x_min} / {n.goal.x_max}\n"
                    f"#============ 頭部狀態 =============#\n"
                    f" head_h        : {n.head_h}  偏角={h_deg:+.1f}°\n"
                    f" head_v        : {n.head_v}\n"
                    f" search_v_idx  : {n.search_v_idx}/{len(GOAL_SEARCH_V_LEVELS)-1}"
                    f"  V={GOAL_SEARCH_V_LEVELS[n.search_v_idx]}\n"
                    f" confirm_frames: {n._confirm_frames}/{GOAL_CONFIRM_FRAMES}\n"
                    f"#====================================#\n"
                    f" goal_found_h  : {n.goal_found_h}\n"
                    f" goal_found_v  : {n.goal_found_v}\n"
                    f"#====================================#\n"
                )
                sys.stdout.flush()
            except Exception:
                pass
            time.sleep(0.1)

# ===========================================================================
# 測試節點
# ===========================================================================

class FindGoalTest(API):
    def __init__(self):
        super().__init__('find_goal_test')

        self.goal = GoalInfo(self)

        self.head_h = HEAD_H_CENTER
        self.head_v = HEAD_V_CENTER

        self.search_dir   = 'right'
        self.search_v_idx = 0

        self.goal_found_h = HEAD_H_CENTER
        self.goal_found_v = HEAD_V_CENTER

        self._confirm_frames = 0
        self.initialized      = False

        self.sub_state     = 'scan'
        self.action_detail = '等待開始'

        self._printer = StatusPrinter(self)
        self._printer.start()

        self.create_timer(0.1, self.main)

    # -----------------------------------------------------------------------

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

    def _search_goal_head(self):
        """分層水平掃描搜尋紅色球門"""
        target_v = GOAL_SEARCH_V_LEVELS[self.search_v_idx]
        if self.head_v != target_v:
            self.head_v = target_v
            self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

        if self.search_dir == 'right':
            self.head_h -= GOAL_SEARCH_STEP_H
            if self.head_h <= HEAD_H_MIN:
                self.head_h = HEAD_H_MIN
                self.search_dir = 'left'
        else:
            self.head_h += GOAL_SEARCH_STEP_H
            if self.head_h >= HEAD_H_MAX:
                self.head_h = HEAD_H_MAX
                self.search_dir = 'right'
                self.search_v_idx += 1
                if self.search_v_idx >= len(GOAL_SEARCH_V_LEVELS):
                    self.search_v_idx = 0   # 循環重掃
        self.sendHeadMotor(1, self.head_h, HEAD_SPEED)

    # -----------------------------------------------------------------------

    def _state_find_goal(self):
        """
        sub_state = 'scan'        : 分層水平掃描搜尋紅色球門
        sub_state = 'center_goal' : P-controller 將頭對準球門中心，穩定後計幀數
        """
        if self.sub_state == 'scan':
            self.goal.update()
            if self.goal.visible:
                self._confirm_frames = 0
                self.sub_state = 'center_goal'
                self.action_detail = f'發現球門，切換對準模式 cx={self.goal.cx}'
            else:
                self._search_goal_head()
                self.action_detail = (
                    f'掃描球門中 dir={self.search_dir}  '
                    f'層={self.search_v_idx}  '
                    f'V={GOAL_SEARCH_V_LEVELS[self.search_v_idx]}  '
                    f'head_h={self.head_h}'
                )

        elif self.sub_state == 'center_goal':
            self.goal.update()
            if not self.goal.visible:
                self._confirm_frames = 0
                self.sub_state = 'scan'
                self.action_detail = '球門追丟，回掃描'
            else:
                centered = self._track_object(self.goal.cx, self.goal.cy)
                if centered:
                    self._confirm_frames += 1
                    self.action_detail = (
                        f'球門對準中，確認幀 ({self._confirm_frames}/{GOAL_CONFIRM_FRAMES})  '
                        f'cx={self.goal.cx}  head_h={self.head_h}'
                    )
                    if self._confirm_frames >= GOAL_CONFIRM_FRAMES:
                        self.goal_found_h = self.head_h
                        self.goal_found_v = self.head_v
                        self.action_detail = (
                            f'找到球門 ✅ goal_h={self.goal_found_h} goal_v={self.goal_found_v} '
                            f'cx={self.goal.cx} cy={self.goal.cy}  x_min/max={self.goal.x_min}/{self.goal.x_max}'
                        )
                else:
                    self._confirm_frames = 0
                    self.action_detail = (
                        f'對準球門中 cx={self.goal.cx} head_h={self.head_h}'
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
            self.head_h = HEAD_H_CENTER
            self.head_v = HEAD_V_CENTER
            self.sendHeadMotor(1, self.head_h, HEAD_SPEED)
            self.sendHeadMotor(2, self.head_v, HEAD_SPEED)
            self.sub_state       = 'scan'
            self._confirm_frames = 0
            self.initialized = True
            return

        self._state_find_goal()

# ===========================================================================
# 進入點
# ===========================================================================

def main(args=None):
    rclpy.init(args=args)
    node = FindGoalTest()
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