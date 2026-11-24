"""模擬關節 ↔ Dynamixel 馬達的對應與單位換算。

這個檔案是「模擬」與「真機」之間唯一的翻譯層，所有魔術數字都集中在這裡。

── 刻度換算的依據 ──────────────────────────────────────────────────────

    tick  = ZERO[id] + DIR[id] × TPR × q          (q = 模擬關節角, rad)
    q     = DIR[id] × (tick − ZERO[id]) / TPR
    TPR   = 4096 / 2π = 651.8986 ticks/rad

`DIR` 直接取自原系統 `walking/Parameter.py` —— 那張表是傳統步態控制器在用
的、已經驗證過的方向表，不另外量。

`ZERO = 2048` 的驗證（2026-10-05）：拿使用者給的「理想站姿刻度」反推
「模擬關節角 = 0 對應哪個刻度」，六個 pitch/knee 關節**全部精確等於
2048.0**：

    ID 關節              理想刻度   q_sim      → zero
    12 LthightJoint        1716   −0.5093     2048.0
    13 LcalvesJoint        2684   +0.9756     2048.0
    14 LankleJoint         2366   −0.4878     2048.0
    18 RthightJoint        2380   −0.5093     2048.0
    19 RcalvesJoint        1412   +0.9756     2048.0
    20 RankleJoint         1730   −0.4878     2048.0

⚠️ 但這個驗證是**循環的** —— 那些理想刻度是使用者給的，而
   `STAND_JOINT_POS` 是從它們推導出來的，所以它只證明前後一致，沒有證明
   符合真機。真正的驗證是 `calib_node --stage pose`：下這些刻度，看機器人
   站起來的姿態對不對、IMU 的 pitch/roll 是否接近 0。

── 為什麼 policy 要用「偏差空間」──────────────────────────────────────

訓練時 `use_default_offset=True`，動作是相對 `STAND_JOINT_POS` 的偏移。
真機每顆馬達有機械零點誤差，所以部署時**觀測與動作共用同一個基準**：

    觀測  q_obs    = q_real − ref
    動作  q_target = ref + ACTION_SCALE × action

`ref` 從 `stand.ini` 讀（每次開機讀，誤差自動吸收），或用模擬的站姿。
哪一個才對要靠 `--stage pose` 的 IMU 讀數決定 —— 兩者的 FK 差異很大：

                       雙腳高度差  腳掌vs軀幹  站距      雙腳前後差
    模擬 STAND_JOINT_POS   +0.0 mm   1.2°     143.0 mm   −0.0 mm
    真機 stand.ini         −2.7 mm  16.8°     138.1 mm  −16.9 mm

腳掌相對軀幹差 17°。真機站著時腳掌必然平貼地面，所以要嘛軀幹真的傾
17°（那模擬站姿在真機上會翹腳），要嘛某個 DIR / URDF 轉軸跟假設不同。
**跑 policy 之前必須先用 IMU 分辨。**
"""

from __future__ import annotations

import configparser
import math
import os

# ──────────────────────────────────────────────────────────────────────────
# 常數
# ──────────────────────────────────────────────────────────────────────────

TPR = 4096.0 / (2.0 * math.pi)  # 651.8986 ticks/rad
TICK_ZERO = 2048  # 模擬關節角 0 對應的刻度（見檔頭的驗證與警告）

# Dynamixel Present Velocity 的單位是 0.229 rev/min。
# 原系統的 driver 把**原始暫存器值**放在 /joint_velocities 的 `velocity`
# 欄位（不是 position），沒有做單位換算。
VEL_RAW_TO_RADS = 0.229 * 2.0 * math.pi / 60.0  # = 0.0239808

# 訓練時的動作設定（從 6600 的 params/env.yaml 確認）
ACTION_SCALE = 0.25  # rad
USE_DEFAULT_OFFSET = True
CLIP_ACTION = 3.0  # 訓練時 clip_actions=None，這是**部署刻意加上**的保護

# 方向表：原封不動取自 walking/Parameter.py
DIR: dict[int, int] = {
    10: 1,   # L_HIP_YAW
    11: 1,   # L_HIP_ROLL
    12: 1,   # L_HIP_PITCH
    13: 1,   # L_KNEE
    14: -1,  # L_ANKLE_PITCH
    15: -1,  # L_ANKLE_ROLL
    16: 1,   # R_HIP_YAW
    17: 1,   # R_HIP_ROLL
    18: -1,  # R_HIP_PITCH
    19: -1,  # R_KNEE
    20: 1,   # R_ANKLE_PITCH
    21: -1,  # R_ANKLE_ROLL
}

# 模擬的關節名 → 馬達 ID。
# 對應依據：README 的「ID 10–15：左腳 (HipYaw/HipRoll/HipPitch/Knee/
# AnklePitch/AnkleRoll)，16–21：右腳」，與 URDF 的關節鏈順序一致。
SIM_TO_ID: dict[str, int] = {
    "Lhip_jointJoint": 10,
    "LassJoint": 11,
    "LthightJoint": 12,
    "LcalvesJoint": 13,
    "LankleJoint": 14,
    "Lsoles_of_feetJoint": 15,
    "Rhip_jointJoint": 16,
    "RassJoint": 17,
    "RthightJoint": 18,
    "RcalvesJoint": 19,
    "RankleJoint": 20,
    "Rsoles_of_feetJoint": 21,
}
ID_TO_SIM = {v: k for k, v in SIM_TO_ID.items()}
LEG_IDS = sorted(SIM_TO_ID.values())

# 全身的馬達（stand.ini 的 motordata 有 21 個值，對應 ID 1..21；頭部 22/23
# 由 Head_Topic 另外控制）。
# ⚠️ 站姿要下**全部 21 個**，不是只有 12 個腿關節 —— 手臂也要到位、靠自己的
#    PD 夾住。這樣就不需要起原系統的 camera.launch.py（那支其實是「完整系統
#    launch」，會帶起未修補的 driver 跟同樣會發 /joint_commands 的
#    walking_node）。
ALL_STAND_IDS = list(range(1, 22))

# 模擬的站姿（robot_cfg.STAND_JOINT_POS，只取 12 個腿關節）。
# ⚠️ 這是**寫死的複本**。改模擬的站姿之後要手動同步，不然部署會用舊值。
#    calib_node --stage pose 會把兩邊印出來讓你核對。
SIM_STAND: dict[str, float] = {
    "Lhip_jointJoint": 0.0,
    "RassJoint": 0.1222,
    "LassJoint": 0.1222,
    "Rhip_jointJoint": 0.0,
    "LthightJoint": -0.5093,
    "RthightJoint": -0.5093,
    "LcalvesJoint": 0.9756,
    "RcalvesJoint": 0.9756,
    "LankleJoint": -0.4878,
    "RankleJoint": -0.4878,
    "Lsoles_of_feetJoint": -0.1222,
    "Rsoles_of_feetJoint": -0.1222,
}

# 安全行程（刻度）。踝 roll 照 sanitize_urdf 的 CLAMP_LIMITS ±0.30 rad。
ANKLE_ROLL_IDS = (15, 21)
ANKLE_ROLL_TICK_LIMIT = int(round(0.30 * TPR))  # ±196
SAFE_TICK_SPAN = 600  # 其餘關節：離參考姿態最多 ±600 ticks（≈ ±0.92 rad）


# ──────────────────────────────────────────────────────────────────────────
# 換算
# ──────────────────────────────────────────────────────────────────────────


def tick_to_q(mid: int, tick: float) -> float:
    """馬達刻度 → 模擬關節角 (rad)。"""
    return DIR[mid] * (tick - TICK_ZERO) / TPR


def q_to_tick(mid: int, q: float) -> float:
    """模擬關節角 (rad) → 馬達刻度。"""
    return TICK_ZERO + DIR[mid] * TPR * q


def vel_raw_to_q_dot(mid: int, raw: float) -> float:
    """Present Velocity 原始值 → 模擬關節角速度 (rad/s)。方向要乘 DIR。"""
    return DIR[mid] * raw * VEL_RAW_TO_RADS


# ──────────────────────────────────────────────────────────────────────────
# stand.ini
# ──────────────────────────────────────────────────────────────────────────

# 原系統的站姿檔。motionpackage 寫入、Stand 按鈕讀取的就是這一份。
STAND_INI = os.path.expanduser(
    "~/ros2_kid/src/strategy/strategy/Parameter/stand.ini"
)

# stand.ini 的 section 結構（從 motionpackage.py 的 motionstate 對照）：
#   motionstate 1 → 偏移 / trim
#   motionstate 2 → 速度 (Profile Velocity)
#   motionstate 3 → **位置**（就是站姿刻度，21 個值對應 ID 1..21）
#   motionstate 4 → 加速度 (Profile Acceleration)
_MOTIONSTATE_POSITION = 3


def read_stand_ini(
    path: str = STAND_INI, ids: list[int] | None = None
) -> dict[int, int]:
    """讀 stand.ini 的站姿刻度，回傳 {motor_id: tick}。

    `ids` 預設是 12 個腿關節；要下完整站姿就傳 `ALL_STAND_IDS`。

    每次開機讀，所以手調過站姿之後不用改任何程式。
    找不到檔案或格式不符就丟例外 —— **不要默默退回預設值**，那會讓機器人
    用錯誤的基準跑 policy。
    """
    if not os.path.exists(path):
        raise FileNotFoundError(f"找不到站姿檔：{path}")
    cp = configparser.ConfigParser()
    cp.read(path)
    for sec in cp.sections():
        if cp[sec].getint("motionstate", -1) != _MOTIONSTATE_POSITION:
            continue
        data = [int(x) for x in cp[sec]["motordata"].split(",")]
        want = ids or LEG_IDS
        if len(data) < max(want):
            raise ValueError(
                f"{path} 的 motordata 只有 {len(data)} 個值，需要至少 {max(want)} 個"
            )
        return {mid: data[mid - 1] for mid in (ids or LEG_IDS)}
    raise ValueError(f"{path} 裡找不到 motionstate={_MOTIONSTATE_POSITION} 的段落")


def sim_stand_ticks() -> dict[int, int]:
    """模擬站姿換算出的刻度，{motor_id: tick}。"""
    return {mid: int(round(q_to_tick(mid, SIM_STAND[ID_TO_SIM[mid]]))) for mid in LEG_IDS}


# ──────────────────────────────────────────────────────────────────────────
# 安全檢查
# ──────────────────────────────────────────────────────────────────────────


def clamp_tick(mid: int, tick: float, ref_tick: float) -> tuple[int, bool]:
    """把目標刻度夾在安全範圍內。回傳 (夾過的刻度, 是否被夾)。

    兩層保護：
      1. 踝 roll 的絕對行程 ±196 ticks（對應 sanitize_urdf 的 ±0.30 rad）——
         真機的踝 roll 行程比 URDF 宣告的小，超過會打到機構。
      2. 其餘關節離參考姿態不超過 ±600 ticks —— 擋掉 policy 發瘋或觀測
         組錯時的大幅指令。
    """
    if mid in ANKLE_ROLL_IDS:
        lo, hi = TICK_ZERO - ANKLE_ROLL_TICK_LIMIT, TICK_ZERO + ANKLE_ROLL_TICK_LIMIT
    else:
        lo, hi = ref_tick - SAFE_TICK_SPAN, ref_tick + SAFE_TICK_SPAN
    t = min(max(tick, lo), hi)
    return int(round(t)), (abs(t - tick) > 0.5)
