#!/usr/bin/env python3
# coding=utf-8
"""
United Soccer 新策略 - 第一階段：踢傳球桿得分
狀態機: find_ball → approach_ball → find_pole → orbit_ball → kick

Sector 動作對照表（需自行錄製）:
  29  : 基礎站姿（API 原本定義，不需錄製）
  101 : 左腳踢球
  102 : 右腳踢球
"""
import sys
import time
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from strategy.API import API

# ===========================================================================
# 調參區 — 所有需要現場測試調整的數值都集中在這裡
# ===========================================================================

# --- 顏色索引 ---
COLOR_BALL = 1   # 橘色 (Orange)
COLOR_POLE = 2   # 藍色 (Blue)

# --- 畫面解析度 ---
IMG_W = 320
IMG_H = 240
IMG_CX = IMG_W // 2   # 160
IMG_CY = IMG_H // 2   # 120

# --- 頭部馬達範圍 ---
HEAD_H_CENTER  = 2048   # 水平中心刻度
HEAD_V_CENTER  = 2048   # 垂直中心刻度（正視前方）
HEAD_H_MAX     = 3072   # 水平最大（向右）
HEAD_H_MIN     = 1024   # 水平最小（向左）
HEAD_V_MAX     = 2048   # 垂直最大（向下）
HEAD_V_MIN     = 1200   # 垂直最小（向上），低於此值避免硬體碰撞
HEAD_SPEED     = 80     # 頭部馬達移動速度

# --- 頭部追蹤參數 ---
# 畫面偏差轉頭部刻度的增益，數值越大頭轉越快
HEAD_TRACK_GAIN_H = 3.0
HEAD_TRACK_GAIN_V = 3.0
# 判定「物體在畫面中央」的容忍像素範圍
HEAD_CENTER_TOL_X = 12   # 水平容忍 ±12px
HEAD_CENTER_TOL_Y = 20   # 垂直容忍 ±20px

# --- 搜尋參數 ---
HEAD_SEARCH_STEP  = 60   # 每次搜尋移動的刻度步長
HEAD_SEARCH_V_MAX = 1300  # 搜尋時垂直最低（往下看球）
HEAD_SEARCH_V_MIN = 1800  # 搜尋時垂直不往太高找
# 找不到球超過幾圈後開始原地轉身
SEARCH_SPIN_AFTER = 2

# --- approach_ball 停止條件 ---
# 頭部垂直刻度越大代表球越近（球在畫面越下方）
# 此數值需現場調整：讓機器人停在球在腳邊、但不踩到球的距離
APPROACH_STOP_V   = 1750  # 達到此垂直刻度視為「球已到腳邊」
# 球在畫面水平方向的容忍範圍（對齊球用）
APPROACH_H_TOL    = 20

# --- 對準柱子的角度容忍（度） ---
POLE_ALIGN_TOL = 15   # yaw 誤差在此範圍內視為對準

# --- orbit_ball 繞行參數 ---
# 繞行時身體側移速度（正=左移，負=右移）
ORBIT_Y_SPEED  = 400
# 繞行時輔助旋轉角速度（幫助身體轉向）
ORBIT_THETA    = 5
# 繞行時前後微調速度（維持與球的距離）
ORBIT_X_HOLD   = 150
# 頭部追球的垂直目標刻度（繞行時希望球維持在這個高度）
ORBIT_BALL_V_TARGET = 1700
# 判定繞行完成需要連續幾幀符合條件
ORBIT_CONFIRM_COUNT = 5

# --- kick 參數 ---
# 球在畫面 x 座標，小於此值用左腳，大於此值用右腳
KICK_FOOT_THRESHOLD = IMG_CX   # 160
SECTOR_KICK_LEFT  = 200   # 左腳踢球 Sector
SECTOR_KICK_RIGHT = 100   # 右腳踢球 Sector
KICK_WAIT         = 20.0   # 踢球動作等待秒數（需配合動作長度調整）

# --- 步態速度 ---
WALK_X_FAST   =  600   # 快速前進
WALK_X_NORMAL =  350   # 正常前進
WALK_X_SLOW   =  150   # 慢速靠近
WALK_Y_LEFT   =  400   # 左平移
WALK_Y_RIGHT  = -400   # 右平移
WALK_THETA_L  =  4     # 左轉
WALK_THETA_R  = -4     # 右轉

# --- IMU 換算 ---
SCALE2DEGREE = 360 / 4096   # 頭部刻度轉角度

# ===========================================================================
# 輔助：視覺物件
# ===========================================================================

class BallInfo:
    """從 API 抓取橘色球資訊，挑出最接近（面積最大）的那顆"""

    def __init__(self, api: API):
        self.api = api
        self.visible  = False
        self.cx = 0   # 畫面中心 x
        self.cy = 0   # 畫面中心 y
        self.area = 0

    def update(self):
        objs = self.api.get_objects('orange')
        if not objs:
            self.visible = False
            self.cx = self.cy = self.area = 0
            return
        # 取面積最大的（通常是最近的）
        best = max(objs, key=lambda o: o['area'])
        self.visible = True
        self.cx   = best['centroid'][0]
        self.cy   = best['centroid'][1]
        self.area = best['area']


class PoleInfo:
    """從 API 抓取藍色傳球桿資訊，挑出面積最大的"""

    def __init__(self, api: API):
        self.api = api
        self.visible  = False
        self.cx = 0
        self.cy = 0
        self.area = 0

    def update(self):
        objs = self.api.get_objects('blue')
        if not objs:
            self.visible = False
            self.cx = self.cy = self.area = 0
            return
        best = max(objs, key=lambda o: o['area'])
        self.visible = True
        self.cx   = best['centroid'][0]
        self.cy   = best['centroid'][1]
        self.area = best['area']


# ===========================================================================
# 狀態機 debug 印表執行緒
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
                    f"#============= United Soccer 新策略 =============#\n"
                    f" state          : {n.state}\n"
                    f" action_detail  : {n.action_detail}\n"
                    f"#================= 視覺狀態 ======================#\n"
                    f" ball  visible={n.ball.visible}  cx={n.ball.cx}  cy={n.ball.cy}  area={n.ball.area}\n"
                    f" pole  visible={n.pole.visible}  cx={n.pole.cx}  cy={n.pole.cy}  area={n.pole.area}\n"
                    f"#================= 頭部狀態 ======================#\n"
                    f" head_h={n.head_h}  head_v={n.head_v}\n"
                    f" search_dir={n.search_dir}  search_count={n.search_count}\n"
                    f"#================= IMU / 對準 ====================#\n"
                    f" yaw={n.imu_rpy[2]:.2f}  pole_target_yaw={n.pole_target_yaw:.2f}\n"
                    f" orbit_confirm={n.orbit_confirm_count}/{ORBIT_CONFIRM_COUNT}\n"
                    f"#=================================================#\n"
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
        super().__init__('us')

        # 視覺物件
        self.ball = BallInfo(self)
        self.pole = PoleInfo(self)

        # 頭部刻度（本地追蹤用）
        self.head_h = HEAD_H_CENTER
        self.head_v = HEAD_V_CENTER

        # 搜尋狀態
        self.search_dir   = 'right'
        self.search_count = 0

        # 柱子對準目標 yaw（find_pole 階段計算）
        self.pole_target_yaw = 0.0

        # 繞行確認計數
        self.orbit_confirm_count = 0

        # 狀態機
        self.state         = 'find_ball'
        self.action_detail = '初始化'

        # debug 印表
        self._printer = StatusPrinter(self)
        self._printer.start()

        # 重設頭部到初始位置
        self._reset_head()

        # 主迴圈 10Hz
        self.create_timer(0.1, self.main)

    # -----------------------------------------------------------------------
    # 頭部控制
    # -----------------------------------------------------------------------

    def _reset_head(self):
        """頭部歸位到正視前方"""
        self.head_h = HEAD_H_CENTER
        self.head_v = HEAD_V_CENTER
        self.sendHeadMotor(1, self.head_h, HEAD_SPEED)
        self.sendHeadMotor(2, self.head_v, HEAD_SPEED)
        self.search_dir   = 'right'
        self.search_count = 0

    def _clamp_head(self):
        self.head_h = max(HEAD_H_MIN, min(HEAD_H_MAX, self.head_h))
        self.head_v = max(HEAD_V_MIN, min(HEAD_V_MAX, self.head_v))

    def _send_head(self):
        self._clamp_head()
        self.sendHeadMotor(1, self.head_h, HEAD_SPEED)
        self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

    def _track_object(self, cx, cy):
        """
        根據畫面上物體的位置，調整頭部馬達追蹤目標。
        回傳 True 表示物體已在畫面中央。
        """
        err_x = cx - IMG_CX   # 正值 = 目標在右邊 → 頭要往右轉（head_h 減小）
        err_y = cy - IMG_CY   # 正值 = 目標在下方 → 頭要往下轉（head_v 增大）

        self.head_h -= int(err_x * HEAD_TRACK_GAIN_H * 4096 / 360 / 10)
        self.head_v += int(err_y * HEAD_TRACK_GAIN_V * 4096 / 360 / 10)
        self._send_head()

        return abs(err_x) < HEAD_CENTER_TOL_X and abs(err_y) < HEAD_CENTER_TOL_Y

    def _search_head(self, v_target=None):
        """
        頭部掃描搜尋：右→下→左→下→右 循環。
        v_target 可指定垂直鎖定刻度（搜尋球時往地面方向固定）。
        """
        if v_target is not None:
            self.head_v = v_target
            self.sendHeadMotor(2, self.head_v, HEAD_SPEED)

        if self.search_dir == 'right':
            self.head_h -= HEAD_SEARCH_STEP
            if self.head_h <= HEAD_H_MIN:
                self.head_h = HEAD_H_MIN
                self.search_dir = 'left'
                self.search_count += 1
        else:  # 'left'
            self.head_h += HEAD_SEARCH_STEP
            if self.head_h >= HEAD_H_MAX:
                self.head_h = HEAD_H_MAX
                self.search_dir = 'right'
                self.search_count += 1

        self._send_head()

    # -----------------------------------------------------------------------
    # 步態控制
    # -----------------------------------------------------------------------

    def _walk(self, x=0, y=0, theta=0):
        """啟動步態並發送步長"""
        self.sendBodyAutoCmd(x=x, y=y, theta=theta)

    def _stop_walk(self):
        """停止步態並回站姿"""
        self.sendbodyAuto(0)
        time.sleep(0.5)
        self.sendBodySector(29)
        time.sleep(0.8)

    # -----------------------------------------------------------------------
    # 狀態機各狀態
    # -----------------------------------------------------------------------

    def _state_find_ball(self):
        """
        搜尋橘色球。
        找到後進入 approach_ball。
        找不到時頭部掃描；超過 SEARCH_SPIN_AFTER 圈後原地左轉幫助搜尋。
        """
        if self.ball.visible:
            self.action_detail = '發現球！→ approach_ball'
            self.search_count = 0
            self.state = 'approach_ball'
            return

        self.action_detail = f'掃描中 dir={self.search_dir} count={self.search_count}'

        if self.search_count >= SEARCH_SPIN_AFTER:
            # 原地轉身輔助搜尋
            self._walk(x=0, y=0, theta=WALK_THETA_L)
        else:
            self._walk(x=WALK_X_SLOW, y=0, theta=0)

        # 垂直鎖定在往地面方向（球在地上）
        self._search_head(v_target=HEAD_SEARCH_V_MAX)

    def _state_approach_ball(self):
        """
        走向球直到球到達腳邊。
        - 頭部持續追球
        - 用頭部水平偏移做左右修正
        - 用頭部垂直刻度判斷距離
        """
        if not self.ball.visible:
            self.action_detail = '球消失！→ find_ball'
            self.state = 'find_ball'
            return

        centered = self._track_object(self.ball.cx, self.ball.cy)

        # 距離判斷：head_v 越大代表球越近（在畫面越下方）
        dist_ok = self.head_v >= APPROACH_STOP_V
        # 水平對齊判斷：球接近畫面正中央
        align_ok = abs(self.ball.cx - IMG_CX) < APPROACH_H_TOL

        if dist_ok and align_ok:
            self.action_detail = '球已到腳邊 → find_pole'
            self._stop_walk()
            self._reset_head()
            time.sleep(0.3)
            self.state = 'find_pole'
            return

        # 根據頭部水平偏移決定轉向修正
        h_err = self.head_h - HEAD_H_CENTER
        if h_err > 150:
            theta = WALK_THETA_L   # 球偏左，身體左轉
        elif h_err < -150:
            theta = WALK_THETA_R   # 球偏右，身體右轉
        else:
            theta = 0

        # 根據距離決定前進速度
        if self.head_v < APPROACH_STOP_V - 200:
            x = WALK_X_NORMAL
            self.action_detail = f'前進靠近 V={self.head_v} theta={theta}'
        else:
            x = WALK_X_SLOW
            self.action_detail = f'慢速靠近 V={self.head_v} theta={theta}'

        self._walk(x=x, y=0, theta=theta)

    def _state_find_pole(self):
        """
        停在原地，頭部掃描找藍色柱子。
        找到後計算柱子對應的世界 yaw 角度，存入 pole_target_yaw。
        進入 orbit_ball。
        """
        if self.pole.visible:
            # 先讓頭部對準柱子，再讀取角度
            centered = self._track_object(self.pole.cx, self.pole.cy)
            if centered:
                # 用「當前 yaw + 頭部水平偏移轉換角度」估算柱子方向
                h_offset_deg = (self.head_h - HEAD_H_CENTER) * SCALE2DEGREE
                self.pole_target_yaw = self.imu_rpy[2] + h_offset_deg * 0.8
                self.action_detail = f'柱子鎖定！target_yaw={self.pole_target_yaw:.1f} → orbit_ball'
                self._reset_head()
                self.orbit_confirm_count = 0
                self.state = 'orbit_ball'
            else:
                self.action_detail = f'對準柱子中 cx={self.pole.cx}'
            return

        # 找不到柱子，掃描（垂直看向前方高度）
        self.action_detail = f'掃描柱子 dir={self.search_dir} count={self.search_count}'
        self._search_head(v_target=HEAD_V_CENTER)

    def _state_orbit_ball(self):
        """
        繞著球旋轉，直到機器人朝向柱子方向，且球在正確腳位。

        繞行方向決策：
        - pole_target_yaw > 當前 yaw → 柱子在左邊 → 往左繞（y 正值）
        - pole_target_yaw < 當前 yaw → 柱子在右邊 → 往右繞（y 負值）

        完成條件：
        1. yaw 誤差 < POLE_ALIGN_TOL
        2. 球在正確腳位（左腳對左柱、右腳對右柱）
        兩個條件連續 ORBIT_CONFIRM_COUNT 幀都滿足才算完成。
        """
        if not self.ball.visible:
            self.action_detail = '繞行中球消失！→ find_ball'
            self._stop_walk()
            self.state = 'find_ball'
            return

        # 頭部持續追球
        self._track_object(self.ball.cx, self.ball.cy)

        current_yaw = self.imu_rpy[2]
        yaw_err = self.pole_target_yaw - current_yaw

        # 決定繞行方向
        if yaw_err > 0:
            # 柱子在左邊 → 往左繞，球最終到右腳側
            orbit_y     = ORBIT_Y_SPEED
            orbit_theta = WALK_THETA_L
            target_foot = 'right'   # 用右腳踢
        else:
            # 柱子在右邊 → 往右繞，球最終到左腳側
            orbit_y     = -ORBIT_Y_SPEED
            orbit_theta = WALK_THETA_R
            target_foot = 'left'    # 用左腳踢

        # 前後微調：讓球保持在腳邊距離
        v_err = self.head_v - ORBIT_BALL_V_TARGET
        if v_err > 80:
            orbit_x = WALK_X_SLOW    # 球太近，稍微後退
        elif v_err < -80:
            orbit_x = -WALK_X_SLOW   # 球太遠，稍微前進
        else:
            orbit_x = 0

        # 繞行
        self._walk(x=orbit_x, y=orbit_y, theta=orbit_theta)

        # 完成條件檢查
        yaw_aligned = abs(yaw_err) < POLE_ALIGN_TOL
        if target_foot == 'right':
            foot_ok = self.ball.cx > IMG_CX   # 球在畫面右側 → 右腳
        else:
            foot_ok = self.ball.cx < IMG_CX   # 球在畫面左側 → 左腳

        if yaw_aligned and foot_ok:
            self.orbit_confirm_count += 1
            self.action_detail = (
                f'對準確認中 {self.orbit_confirm_count}/{ORBIT_CONFIRM_COUNT} '
                f'yaw_err={yaw_err:.1f} foot={target_foot}'
            )
        else:
            self.orbit_confirm_count = 0
            self.action_detail = (
                f'繞行中 yaw_err={yaw_err:.1f} foot_ok={foot_ok} '
                f'orbit_y={orbit_y}'
            )

        if self.orbit_confirm_count >= ORBIT_CONFIRM_COUNT:
            self.action_detail = f'對準完成！準備用 {target_foot} 腳踢 → kick'
            self._stop_walk()
            # 把踢球腳資訊帶入 kick 狀態
            self._kick_foot = target_foot
            self.state = 'kick'

    def _state_kick(self):
        """
        踢球。根據 orbit_ball 決定的腳選擇 Sector。
        踢完後重置回 find_ball 繼續下一顆。
        """
        foot = getattr(self, '_kick_foot', 'right')

        if foot == 'left':
            sector = SECTOR_KICK_LEFT
            self.action_detail = f'左腳踢球 (Sector {sector})'
        else:
            sector = SECTOR_KICK_RIGHT
            self.action_detail = f'右腳踢球 (Sector {sector})'

        self.sendBodySector(sector)
        time.sleep(KICK_WAIT)

        # 踢完後回站姿，重置狀態
        self.sendBodySector(29)
        time.sleep(0.5)
        self._reset_head()
        self.orbit_confirm_count = 0
        self.state = 'find_ball'
        self.action_detail = '踢球完畢，重新尋球'

    # -----------------------------------------------------------------------
    # 更新視覺資料
    # -----------------------------------------------------------------------

    def _update_objects(self):
        self.ball.update()
        self.pole.update()

    # -----------------------------------------------------------------------
    # 主迴圈
    # -----------------------------------------------------------------------

    def main(self):
        if not self.is_start:
            self.sendbodyAuto(0)
            return

        self._update_objects()

        if self.state == 'find_ball':
            self._state_find_ball()
        elif self.state == 'approach_ball':
            self._state_approach_ball()
        elif self.state == 'find_pole':
            self._state_find_pole()
        elif self.state == 'orbit_ball':
            self._state_orbit_ball()
        elif self.state == 'kick':
            self._state_kick()


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