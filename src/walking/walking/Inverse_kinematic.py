# -*- coding: utf-8 -*-
"""
Inverse_kinematic.py (Unified Baseline Version)
-----------------------------------------------
- 不分軟/硬體模式
- 由外部指定 baseline（角度與刻度）
- 每次 IK 計算產生絕對刻度 → 減 baseline → 取得相對轉動量
"""

import math
import time
import threading
from dataclasses import dataclass
from typing import Dict, List

from . import Walkinggait as wg
from . import Parameter as parameter
from .Parameter import STAND_GP, DIR

# ---------- 基本參數 ----------
@dataclass
class Parameters:
    l1: float = 12.5
    l2: float = 12.5

PI = math.pi
TPR = 4096.0 / (2.0 * PI)

LEFT_IDS = [10, 11, 12, 13, 14, 15]
RIGHT_IDS = [16, 17, 18, 19, 20, 21]
IK_IDS = LEFT_IDS + RIGHT_IDS
ALL_IDS = list(STAND_GP.keys())

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def ticks_diff_signed(a: float, b: float, mod: int = 4096) -> float:
    """回傳有號差值，確保差距在 (-mod/2, +mod/2) 範圍內"""
    d = (a - b) % mod
    if d > mod / 2:
        d -= mod
    return d

# ---------- IK 計算 ----------
def compute_leg_ik(Lx, Ly, Lz, Lt, Rx, Ry, Rz, Rt,
                   rotate_body_l_=0.0, flag_=0) -> List[float]:
    l1 = float(getattr(parameter, "L1", Parameters.l1))
    l2 = float(getattr(parameter, "L2", Parameters.l2))
    l1_2, l2_2 = l1 * l1, l2 * l2
    base_L = base_R = PI / 2.0

    # ===== 嚴格模式防護罩 (隔離 Mode 0) =====
    current_mode = int(getattr(parameter, "walking_mode", 0))
    if current_mode in [1, 2]:
        hip_roll_rad = math.radians(float(getattr(parameter, "Hip_roll", 0.0)))
        ankle_roll_rad = math.radians(float(getattr(parameter, "Ankle_roll", 0.0)))
    else:
        hip_roll_rad = 0.0
        ankle_roll_rad = 0.0

    # ---- 左腿 ----
    T_Lt = Lt + base_L
    T_Ly = (math.atan2(Lz, Ly) if Ly != 0 else PI / 2.0) + hip_roll_rad
    Lxyz = min(math.sqrt(Lx**2 + Ly**2 + Lz**2), l1 + l2)
    valL = clamp((l1_2 + Lxyz**2 - l2_2) / (2 * l1 * Lxyz), -1.0, 1.0)
    kneeL = PI - math.acos(clamp((l1_2 + l2_2 - Lxyz**2) / (2 * l1 * l2), -1.0, 1.0))
    if Lx >= 0:
        hipL = PI/2 - math.acos(valL) - math.atan2(Lx, math.sqrt(Ly**2 + Lz**2))
    else:
        hipL = PI - math.acos(valL) - math.atan2(math.sqrt(Ly**2 + Lz**2), -Lx)
    ankleL = PI - hipL - kneeL
    rollL = PI - T_Ly - (rotate_body_l_ if flag_ else 0.0) + ankle_roll_rad

    # ---- 右腿 ----
    T_Rt = Rt + base_R
    T_Ry = (math.atan2(Rz, Ry) if Ry != 0 else PI / 2.0) - hip_roll_rad
    Rxyz = min(math.sqrt(Rx**2 + Ry**2 + Rz**2), l1 + l2)
    valR = clamp((l1_2 + Rxyz**2 - l2_2) / (2 * l1 * Rxyz), -1.0, 1.0)
    kneeR = PI - math.acos(clamp((l1_2 + l2_2 - Rxyz**2) / (2 * l1 * l2), -1.0, 1.0))
    if Rx >= 0:
        hipR = PI/2 - math.acos(valR) - math.atan2(Rx, math.sqrt(Ry**2 + Rz**2))
    else:
        hipR = PI - math.acos(valR) - math.atan2(math.sqrt(Ry**2 + Rz**2), -Rx)
    ankleR = PI - hipR - kneeR
    rollR = PI - T_Ry - (rotate_body_l_ if flag_ else 0.0) - ankle_roll_rad

    return [T_Lt, T_Ly, hipL, kneeL, ankleL, rollL,
            T_Rt, T_Ry, hipR, kneeR, ankleR, rollR]

# ---------- IKService ----------
class IKService:
    def __init__(self, baseline_ticks=None, baseline_ang=None, min_pv=0, speed_scale=1.0):
        """
        baseline_ticks: Dict[int,int]  → 馬達 baseline 絕對刻度
        baseline_ang:   List[float]    → 對應 baseline 的角度
        """
        self.min_pv = min_pv
        self.speed_scale = float(speed_scale)

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

        self._base_ticks = dict(baseline_ticks) if baseline_ticks else dict(STAND_GP)
        self._base_ang = list(baseline_ang) if baseline_ang else [0.0]*len(IK_IDS)
        self._prev_ang = list(self._base_ang)

        self._latest_gp = dict(self._base_ticks)
        self._latest_pv = {mid: 0 for mid in ALL_IDS}
        self._rel_ticks = {mid: 0 for mid in ALL_IDS}

    # ====== 啟動 / 停止 ======
    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    # ====== 主循環 ======
    def _loop(self):
        walking = getattr(wg, "walking", wg.WalkingGaitByLIPM())
        while not self._stop.is_set():
            dt = float(getattr(walking, "sample_time_", 30)) / 1000.0
            if hasattr(walking, "process"):
                walking.process()

            Lx, Ly, Lz, Lt, Rx, Ry, Rz, Rt = (
                walking.end_point_lx_, walking.end_point_ly_, walking.end_point_lz_, walking.end_point_lthta_,
                walking.end_point_rx_, walking.end_point_ry_, walking.end_point_rz_, walking.end_point_rthta_
            )
            ang_now = compute_leg_ik(Lx, Ly, Lz, Lt, Rx, Ry, Rz, Rt)

            gp_abs = self._calc_gp(ang_now, self._base_ang)
            rel = {mid: ticks_diff_signed(gp_abs[mid], self._base_ticks[mid]) for mid in gp_abs}
            pv = self._calc_pv(ang_now, self._prev_ang, dt)
		
            with self._lock:
                self._latest_gp.update(gp_abs)
                self._latest_pv.update(pv)
                self._rel_ticks.update(rel)
                self._prev_ang = list(ang_now)
            
            time.sleep(dt)

    # ====== 工具 ======
    def _calc_gp(self, ang_now, base_ang):
        gp = {}
        for idx, mid in enumerate(IK_IDS):
            delta = ang_now[idx] - base_ang[idx]
            delta_ticks = DIR[mid] * TPR * delta
            ticks = int(round(self._base_ticks[mid] + delta_ticks)) % 4096
            gp[mid] = ticks
        return gp

    def _calc_pv(self, ang_now, ang_prev, dt):
        PV_UNIT = 60.0 / (4096.0 * 0.229)
        pv = {}
        for idx, mid in enumerate(IK_IDS):
            d = abs(ang_now[idx] - ang_prev[idx])
            if d <= 1e-9:
                v = 0
            else:
                ticks_per_s = d / dt * TPR
                v = int(round(ticks_per_s * PV_UNIT * self.speed_scale))
                v = max(v, self.min_pv)
            pv[mid] = clamp(v, 0, 32767)
        return pv

    # ====== 介面 ======
    def latest_gp(self):
        with self._lock:
            return dict(self._latest_gp)

    def latest_rel_gp(self):
        with self._lock:
            return dict(self._rel_ticks)

    def latest_pv(self):
        with self._lock:
            return dict(self._latest_pv)

    def latest_rel_deg(self):
        with self._lock:
            tick_to_deg = 360.0 / 4096.0
            return {mid: self._rel_ticks[mid] * tick_to_deg for mid in self._rel_ticks}
