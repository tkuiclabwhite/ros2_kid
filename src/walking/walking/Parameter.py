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
STAND_HEIGHT = 23.5  # 站姿高度 (cm)
LENGTH_PELVIS= 9.1   # 骨盆寬 (cm)
G            = 981.0 # 重力 (cm/s^2)
Tc_          = math.sqrt(COM_HEIGHT / G)  # LIPM 時間常數
L1 = 12.5   # 大腿長度 (cm)
L2 = 12.5   # 小腿長度 (cm)

# ---- 步態形狀 ----
step_length  = 0      # x
shift_length = 0.0    # y
theta_       = 0.0    # theta
# width_size   = 4.5    # 半步寬 (cm)
width_size   = 0    # 半步寬 (cm)
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