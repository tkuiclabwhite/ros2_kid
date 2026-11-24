"""部署的核心運算：觀測組裝 + 動作換算。**不依賴 ROS，只用 numpy。**

為什麼要抽出來：`policy_node`（真機，ROS，python 3.10）和
`kid_rl/mujoco/sim2sim_mujoco.py`（MuJoCo，python 3.12）要跑**同一份**程式碼。
重寫兩份就測不到真正的部署路徑了。

    真機:    /joint_states, /package/sensorpackage  →  DeployCore  →  /joint_commands
    MuJoCo:  d.qpos/qvel, sensordata                →  DeployCore  →  d.ctrl 的 PD 目標

兩邊餵進來的都是**感測器式的量**（刻度、暫存器原始值、感測器座標的四元數／
陀螺），所以刻度換算、DIR 方向、imu_rotation、站姿基準這些轉換都會被測到，
而不是繞過去。

── 真機缺陷開關 ────────────────────────────────────────────────────────

`DeployCfg` 的那幾個旗標是為了 sim2sim 的「一次加一個缺陷」而存在的：
先全部關掉確認管線乾淨，再一個一個打開，找出哪一個殺死 policy。

    quantize_pos     編碼器量化（1 tick = 1.534 mrad）
    quantize_vel     速度暫存器量化（0.229 rpm = 24.0 mrad/s）
    pose_reference   "sim"（模擬站姿）或 "stand_ini"（真機站姿，差最多 12.8°）
    clamp_ticks      踝 roll 的 ±196 tick 行程夾限（真機實測 97% 週期在夾）
    obs_latency      觀測延遲幾個控制週期
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import numpy as np

from . import joint_map as jm

# ── 步態時鐘（與 kid_rl/gait.py 一致）────────────────────────────────
GAIT_PERIOD = 0.60
GAIT_OFFSET = 0.5
STANCE_RATIO = 0.55
# 站立模式時相位快照到雙支撐中點（gait.STAND_PHASE）
STAND_PHASE = 0.025
STAND_VEL_THRESH = 0.064   # m/s
STAND_ANG_THRESH = 0.10    # rad/s

# ── 指令範圍（= 訓練時 commands.base_velocity.ranges）────────────────
#
# ⚠️ 真機上**一定要夾**。policy 沒見過範圍外的指令，而 /rl_deploy/cmd_vel 是
#    任何人都能 publish 的 topic —— 一句 `linear: {x: 1.0}` 就餵進 2.5 倍訓練
#    上限的指令。極速天花板實測只有 ~0.31 m/s，給更大的指令不會更快，
#    只會讓 policy 在沒見過的輸入上輸出沒見過的動作。
CMD_LIMITS = ((-0.25, 0.40), (-0.20, 0.20), (-1.0, 1.0))   # vx, vy, wz

# 觀測項與維度（宣告順序 = KIDDeployObservationsCfg.PolicyCfg）
TERMS_47 = [
    ("base_ang_vel", 3), ("projected_gravity", 3), ("velocity_commands", 3),
    ("joint_pos", 12), ("joint_vel", 12), ("actions", 12), ("gait_phase", 2),
]
TERMS_48 = TERMS_47 + [("standing_mode", 1)]

DEG2RAD = math.pi / 180.0


@dataclass
class DeployCfg:
    obs_dim: int = 48
    action_scale: float = jm.ACTION_SCALE
    action_clip: float = 10.0          # 訓練時無裁切；這個值高到不會綁住
    # policy 的 12 個輸出對應的關節（模擬器內部順序，inspect_usd.py 取得）
    joint_order: list[str] = field(default_factory=lambda: [
        "Lhip_jointJoint", "Rhip_jointJoint", "LassJoint", "RassJoint",
        "LthightJoint", "RthightJoint", "LcalvesJoint", "RcalvesJoint",
        "LankleJoint", "RankleJoint", "Lsoles_of_feetJoint", "Rsoles_of_feetJoint",
    ])
    # IMU 感測器座標 → base_link（calib --stage imu 實測 (+x,−y,−z)）
    imu_rotation: tuple = (1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0)
    gravity_rereference: bool = True
    # 再參考的目標：模擬裡「PD 夾住 STAND_JOINT_POS、靜止」時的
    # projected_gravity（stand_test 2026-10-21 實測）。理由見
    # set_gravity_reference()。
    gravity_reference: tuple = (-0.033, 0.0, -0.999)
    clamp_cmd: bool = True
    # ── ⚠️ 站姿是**兩個**基準，不是一個 ────────────────────────────
    #
    # `pose_reference`       policy 偏差空間的原點（觀測與動作用）
    # `stand_pose_reference` 機器人實際站起來的姿態（TO_STAND 用）
    #
    # 2026-11-06 拆開。在此之前是同一個參數，而兩份都成立的實測互相矛盾：
    #   · calib --stage pose（真機）：stand.ini 腳底平貼 ✅；
    #     模擬 STAND_JOINT_POS 腳底不平、機身後傾 7.63° ❌
    #   · MuJoCo sim2sim：pose_reference=stand_ini → 傾角 87.5°，倒。
    #     必須用模擬 STAND_JOINT_POS（兩者差最多 12.8°，等於把 policy 的
    #     偏差空間原點整個平移）
    # 它們要求的是不同的東西：**站起來的姿態要腳底平貼，偏差原點要跟訓練
    # 一致**。同一個參數不可能同時滿足，所以不要二選一，拆開。
    #
    # 代價：兩個姿態之間有落差，進 policy 前要**走過去**而不是跳過去
    # （policy_node 的 TO_POLICY 狀態）。真機的機械零點誤差最終要用一次性
    # 校正表吸收，校正表做好之後兩者會合而為一，這個落差就自然歸零。
    pose_reference: str = "sim"              # "sim" | "stand_ini"
    stand_pose_reference: str = "stand_ini"  # "sim" | "stand_ini"
    stand_ini_path: str = jm.STAND_INI
    # ── 真機缺陷開關（sim2sim 用）──────────────────────────────────
    quantize_pos: bool = False
    quantize_vel: bool = False
    clamp_ticks: bool = False
    obs_latency: int = 0

    def terms(self):
        return TERMS_48 if self.obs_dim == 48 else TERMS_47


def quat_to_mat(q) -> np.ndarray:
    w, x, y, z = np.asarray(q, dtype=float) / max(np.linalg.norm(q), 1e-12)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


class DeployCore:
    """真機與 MuJoCo 共用的觀測／動作運算。"""

    def __init__(self, cfg: DeployCfg | None = None) -> None:
        self.cfg = cfg or DeployCfg()
        self.R = np.asarray(self.cfg.imu_rotation, dtype=float).reshape(3, 3)
        self.ids = [jm.SIM_TO_ID[n] for n in self.cfg.joint_order]

        # 站姿基準（刻度）。觀測與動作共用同一組 —— policy 在「相對站姿的
        # 偏差空間」工作，真機每顆馬達的機械零點誤差因此自動被吸收。
        self.ref_all = self.pose_ticks(self.cfg.pose_reference, self.cfg.stand_ini_path)
        self.ref = {mid: self.ref_all[mid] for mid in jm.LEG_IDS}
        # 實際站起來的姿態（可以跟上面不同，理由見 DeployCfg）
        self.stand_ref_all = self.pose_ticks(
            self.cfg.stand_pose_reference, self.cfg.stand_ini_path
        )

        self.prev_action = np.zeros(12, dtype=np.float32)
        self.g_ref: np.ndarray | None = None
        self._obs_queue: list[np.ndarray] = []

    @staticmethod
    def pose_ticks(which: str, path: str = jm.STAND_INI) -> dict[int, int]:
        """站姿刻度 {motor_id: tick}，21 顆。

        "stand_ini" 整份用真機的站姿檔；"sim" 的 12 個腿關節換成模擬的
        STAND_JOINT_POS，其餘（手臂／腰／頭）仍用 stand.ini —— 模擬的手臂
        是 ±15° 但部署的 DIR 表沒有手臂的方向，換算不了，列在硬體清單上。
        """
        ini = jm.read_stand_ini(path=os.path.expanduser(path), ids=jm.ALL_STAND_IDS)
        return ini if which == "stand_ini" else {**ini, **jm.sim_stand_ticks()}

    # ── 指令 ──────────────────────────────────────────────────────────
    def clamp_cmd(self, cmd):
        """把指令夾進訓練範圍。回傳 (夾過的指令, 是否被夾)。"""
        c = np.asarray(cmd, dtype=np.float32).reshape(3)
        if not self.cfg.clamp_cmd:
            return c.copy(), False
        out = np.array(
            [min(max(float(c[i]), lo), hi) for i, (lo, hi) in enumerate(CMD_LIMITS)],
            dtype=np.float32,
        )
        return out, bool(np.any(np.abs(out - c) > 1e-6))

    # ── 站立模式 ──────────────────────────────────────────────────────
    def is_standing(self, cmd) -> bool:
        c = np.asarray(cmd, dtype=float)
        return bool(
            np.linalg.norm(c[:2]) < STAND_VEL_THRESH and abs(c[2]) < STAND_ANG_THRESH
        )

    def gait_phase(self, t: float, cmd) -> float:
        """步態相位 [0,1)。站立模式快照到 STAND_PHASE（gait._phase 的行為）。"""
        if self.is_standing(cmd):
            return STAND_PHASE
        return (t % GAIT_PERIOD) / GAIT_PERIOD

    # ── 重力 ──────────────────────────────────────────────────────────
    def set_gravity_reference(self, acc_g) -> np.ndarray:
        """用靜止時的加速度計定出站姿的重力方向。回傳實測的 g_base。

        ⚠️ 只能在**靜止站姿**時呼叫。靜止時加速度計量的是比力 = −g。
        """
        a = np.asarray(acc_g, dtype=float)
        n = np.linalg.norm(a)
        if n < 1e-6:
            raise ValueError("加速度計讀值為 0")
        g0 = -(self.R @ (a / n))
        # gravity_rereference：把「真機夾住站姿」映到「**模擬**夾住站姿」，
        # 而不是映到鉛垂。關節已經走相對站姿的偏差空間，姿態用同一套基準
        # 才一致。代價：絕對傾角會被低/高估這個偏差量（保護用的傾角另外算）。
        #
        # ⚠️ 2026-11-06 修正。原本寫 g_ref = (0, 0, −1)，註解引用的「訓練時
        #    站姿 projected_gravity ≈ (0, 0.019, −1)」是**舊 STAND_ROT 時代**
        #    的值。修正 STAND_ROT（改用腳底法向判準）之後，模擬裡 PD 夾住
        #    STAND_JOINT_POS 靜止時實測是 (−0.033, +0.000, −0.999)，傾角 1.89°。
        #    映到鉛垂等於塞給 policy 一個 1.89° 的永久姿態偏差。
        #
        # ⚠️ **不要拿 STAND_ROT 直接算**。那會得到 (+0.021, 0, −1.000) ——
        #    傾角差不多但**符號相反**，兩者相距 3.1°。STAND_ROT 是零重力的
        #    幾何姿態；要對齊的是重力下 PD 平衡點的姿態（同一個現象讓機器人
        #    比幾何值高 11.3 mm：膝在平衡點比目標站姿伸直 2.93°）。
        #    HOLD_POLICY 的物理情境就是 stand_test，所以用 stand_test 的實測值。
        if self.cfg.gravity_rereference:
            r = np.asarray(self.cfg.gravity_reference, dtype=float)
            self.g_ref = r / max(np.linalg.norm(r), 1e-12)
        else:
            self.g_ref = g0
        return g0

    def gravity(self, quat) -> np.ndarray:
        """base_link 座標的 projected_gravity（單位向量）。

        quat 是 IMU 相對**歸零姿態**的四元數（感測器座標）。
            g_base(t) = R_mount · R_q(t)ᵀ · R_mountᵀ · g_ref
        """
        if self.g_ref is None:
            raise RuntimeError("先呼叫 set_gravity_reference()")
        return self.R @ quat_to_mat(quat).T @ self.R.T @ self.g_ref

    # ── 觀測 ──────────────────────────────────────────────────────────
    def build_obs(
        self, *, pos_ticks, vel_raw, quat, gyr_dps, cmd, t, vel_rads=None
    ) -> np.ndarray:
        """組出 obs_dim 維觀測。輸入全部是**感測器式**的量。

        pos_ticks  {motor_id: 刻度}
        vel_raw    {motor_id: Present Velocity 原始值}（單位 0.229 rpm）
        quat       (w,x,y,z) 感測器座標、相對歸零姿態
        gyr_dps    (x,y,z) 感測器座標、deg/s
        cmd        (vx, vy, wz)
        t          從進入 POLICY 起算的秒數（步態時鐘用）
        vel_rads   {motor_id: rad/s}。給了就取代 vel_raw（真機的
                   joint_vel_source=diff 用位置微分算，已經是 rad/s）。
                   走這條路 quantize_vel 不作用。
        """
        c = self.cfg
        ang = self.R @ (np.asarray(gyr_dps, dtype=float) * DEG2RAD)
        grav = self.gravity(quat)
        qp, qv = [], []
        for mid in self.ids:
            tick = pos_ticks[mid]
            if c.quantize_pos:
                tick = round(tick)
            qp.append(jm.tick_to_q(mid, tick) - jm.tick_to_q(mid, self.ref[mid]))
            if vel_rads is not None:
                qv.append(float(vel_rads[mid]))
            else:
                raw = vel_raw[mid]
                if c.quantize_vel:
                    raw = round(raw)
                qv.append(jm.vel_raw_to_q_dot(mid, raw))
        phase = self.gait_phase(t, cmd)
        parts = [
            ang, grav, np.asarray(cmd, dtype=float),
            np.asarray(qp), np.asarray(qv), self.prev_action,
            [math.sin(2 * math.pi * phase), math.cos(2 * math.pi * phase)],
        ]
        if c.obs_dim == 48:
            parts.append([1.0 if self.is_standing(cmd) else 0.0])
        obs = np.concatenate(parts).astype(np.float32)
        if obs.shape[0] != c.obs_dim:
            raise ValueError(f"觀測維度 {obs.shape[0]} ≠ {c.obs_dim}")

        # ── 觀測延遲 ─────────────────────────────────────────────────
        #
        # ⚠️ **只延遲感測器來的項**，不要延遲全部。真機上：
        #       base_ang_vel / projected_gravity / joint_pos / joint_vel
        #           ← 來自匯流排與 IMU，會舊（/joint_states 50 Hz → 最多 20 ms）
        #       velocity_commands / actions / gait_phase / standing_mode
        #           ← 控制器自己手上的量，永遠是新的
        #
        # 把全部一起延遲會高估延遲的傷害（prev_action 變舊 = policy 不知道
        # 自己剛剛做了什麼，那是真機沒有的問題）。
        if c.obs_latency > 0:
            self._obs_queue.append(obs.copy())
            if len(self._obs_queue) > c.obs_latency:
                old = self._obs_queue.pop(0)
                i = 0
                for name, dim in c.terms():
                    if name in ("base_ang_vel", "projected_gravity",
                                "joint_pos", "joint_vel"):
                        obs[i:i + dim] = old[i:i + dim]
                    i += dim
        return obs

    # ── 動作 ──────────────────────────────────────────────────────────
    def action_to_targets(self, raw_action, ramp: float = 1.0):
        """policy 原始輸出 → {motor_id: (目標刻度, 目標關節角)}，並記 prev_action。

        ⚠️ `prev_action` 存的是**原始值**。訓練時 clip_actions=None，`actions`
           那 12 維觀測吃的是未裁切的輸出；裁切只作用在下給馬達的值。
        """
        c = self.cfg
        raw = np.asarray(raw_action, dtype=np.float32).reshape(-1)
        self.prev_action = raw.copy()
        a = np.clip(raw, -c.action_clip, c.action_clip)
        out, n_clamp = {}, 0
        for i, mid in enumerate(self.ids):
            q = jm.tick_to_q(mid, self.ref[mid]) + c.action_scale * ramp * float(a[i])
            tick = jm.q_to_tick(mid, q)
            if c.clamp_ticks:
                tick, was = jm.clamp_tick(mid, tick, self.ref[mid])
                n_clamp += int(was)
            out[mid] = (tick, jm.tick_to_q(mid, tick))
        return out, n_clamp

    def reset(self) -> None:
        self.prev_action[:] = 0.0
        self.g_ref = None
        self._obs_queue.clear()
