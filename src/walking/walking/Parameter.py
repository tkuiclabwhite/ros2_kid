# Parameter.py
# 只放「設定值/可調參數」，不要放步態迴圈的狀態

import math
from typing import Dict

# ---- IK計算的站姿 (實際上走路演算主要看這裡的9-21，但為了初始化，全身都要有) ----
STAND_GP: Dict[int, int] = {
    # --- 上半身 (ttyUSB0) ---
    1: 3072, 2: 1925, 3: 2048, 4: 2048,            # 左手 (請自行填入設定值)
    5: 1024, 6: 2171, 7: 2048, 8: 2048,            # 右手 (請自行填入設定值)
    
    # --- 下半身 (ttyUSB2) ---
    9: 2048,                                       # 腰
    10: 2066, 11: 2001, 12: 1647, 13: 2712, 14: 2406, 15: 2004,  # 左腳
    16: 2039, 17: 1950, 18: 2501, 19: 1364, 20: 1652, 21: 2060,  # 右腳
    
    # --- 頭部/配件 (ttyUSB1) ---
    22: 2048, 23: 2048                             # 頭部 (請自行填入設定值)
}


DIR: Dict[int, int] = {
    10:  1,   # L_HIP_YAW
    11:  1 ,  # L_HIP_ROLL
    12:  1,   # L_HIP_PITCH
    13:  1,   # L_KNEE
    14: -1,   # L_ANKLE_PITCH
    15: -1,   # L_ANKLE_ROLL

    16:  1,   # R_HIP_YAW
    17:  1,   # R_HIP_ROLL
    18: -1,   # R_HIP_PITCH
    19: -1,   # R_KNEE
    20:  1,   # R_ANKLE_PITCH
    21: -1,   # R_ANKLE_ROLL
}

# ---- 時序參數 ----
period_t   = 360   # 單步週期 (ms)
sample_time = 20    # 取樣時間 (ms)
Tdsp        = 0.0   # 雙支撐比例: 0 <= Tdsp < 1

# ---- 幾何/物理 ----
COM_HEIGHT   = 29.5  # 質心高度 (cm)
STAND_HEIGHT = 0.0   # 站姿高度「調整量」(cm)：相對調好的 stand，+ 往上站高，- 往下蹲低
LENGTH_PELVIS= 9.1   # 骨盆寬 (cm)
G            = 981.0 # 重力 (cm/s^2)
Tc_          = math.sqrt(COM_HEIGHT / G)  # LIPM 時間常數
L1 = 12.5   # 大腿長度 (cm)：URDF 髖 pitch 軸 → 膝軸
L2 = 12.5   # 小腿長度 (cm)：URDF 膝軸 → 踝 pitch 軸

# ---- 絕對零點：URDF 關節角 0（雙腳垂直全直、腳底平貼）對應的馬達刻度 ----
# 預設 2048（與 rl_deploy/joint_map.py 相同假設，尚未在實機驗證）
# 驗證方式：腿部 10~21 全下 2048，雙腳應垂直全直、腳底板平貼；不是的話改這張表
LEG_ZERO_TICKS: Dict[int, int] = {mid: 2048 for mid in range(10, 22)}

# 網頁送出站姿 (width_size / STAND_HEIGHT) 時的移動時間 (秒)。
# 各馬達依自己的移動量算速度，所有腿部馬達同時到位；0 = 不限速（最快）
STAND_MOVE_TIME = 1.5
# 站姿移動的最低 Profile Velocity（1 = 0.229 rpm）。太慢的話，腳底板在地上推不動（靜摩擦），
# 承重較多的腳（目前是左腳）的 roll 會停在半路；motion 的 stand 用 100
STAND_MIN_VELOCITY = 30

# 雙腳全直時左右腳底板內側距離 (cm)。
SOLE_GAP_STRAIGHT = 2.6

# ---- 基準站姿 = 你用 motion 調好的 stand ----
# 網頁的 width_size / STAND_HEIGHT 都是「相對這個 stand 的調整量」：
#   width_size   : + 雙腳往外開，- 雙腳往內縮 (cm，左右腳間距的變化量；兩腳各分一半)
#   STAND_HEIGHT : + 站高，- 蹲低 (cm)
#   兩個都 0 = 完全維持調好的 stand，不會動馬達
# STAND_BASE_WIDTH / STAND_BASE_HEIGHT 由 walking_node 用 stand 的馬達刻度做正運動學 (FK) 算出來，
# 執行時自動更新；下面只是還沒抓到 stand 之前的預設值，不用手動改
STAND_BASE_WIDTH  = 0.0    # stand 的雙腳間距，相對雙腳全直的偏移量 (cm，內部單位)
STAND_BASE_HEIGHT = 23.5   # stand 的髖到腳踝高度 (cm)

# 步態 / IK 內部用的「相對雙腳全直的偏移量」(全寬) = stand + 調整量
def width_offset(width_delta):
    return STAND_BASE_WIDTH + float(width_delta)

# 實際的腳底板內側間距 (cm)，只用來顯示 / 警告
def sole_gap(width_delta):
    return SOLE_GAP_STRAIGHT + width_offset(width_delta)

# 步態 / IK 內部用的絕對站姿高度 (cm) = stand + 調整量
def stand_height_abs(height_delta):
    return STAND_BASE_HEIGHT + float(height_delta)

# ---- 步態形狀 ----
step_length  = 0      # x
shift_length = 0.0    # y
theta_       = 0.0    # theta
# width_size   = 4.5    # 半步寬 (cm)
width_size   = 0.0    # 腳寬「調整量」(cm)：相對調好的 stand，+ 往外開，- 往內縮
lift_height  = 2.5
com_y_swing  = 0      # 質心側擺幅度 (cm)
COM_Y_shift  = 0      # 質心側向偏移 (cm)
compensation_swing_hip   = 0.0  # 擺動髖補償 (cm)
compensation_swing_ankle = 0.0
SPEED_SCALE = 1


# ---- 新增：樓梯與姿態補償參數 ----
walking_mode = 0     # 0:平地, 1:上樓, 2:下樓
Board_High   = 0.0   # 階梯高度 (cm)
Clearance    = 3.0   # 越障安全餘裕 (cm)
Hip_roll     = 0.0   # 髖關節側向補償 (Degree)
Ankle_roll   = 0.0   # 踝關節側向補償 (Degree)