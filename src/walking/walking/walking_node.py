#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
WalkingNode.py — [Snapshot Baseline + Landing Fix + Dynamic Stand Update]
功能：
1. Generate ON: 抓取當前位置作為 Baseline。
2. Generate OFF: 執行落地緩衝，確保雙腳踩平，並回復到 Baseline。
3. Idle (Stand): 若調整 width_size/height，即時重算 IK 並驅動馬達，同時更新 Baseline。
"""

import sys
import time
import threading
import math
import json
from collections import deque
from typing import Dict, List, Tuple, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, String, Bool
from sensor_msgs.msg import JointState
from tku_msgs.msg import Interface

# ==== 專案內部匯入 ====
try:
    from . import Parameter as parameter
    from .Parameter import DIR 
    from .Inverse_kinematic import compute_leg_ik, IK_IDS, TPR
    from .Walkinggait import WalkingGaitByLIPM, StopStep, StartStep, FirstStep, Repeat
except ImportError as e:
    print(f"[Error] Import failed: {e}. Make sure Parameter.py exists.")
    sys.exit(1)

# ============================================================
# 馬達 ID 設定
# ============================================================
IDS_ARM  = [1, 2, 3, 4, 5, 6, 7, 8]
IDS_HEAD = [22, 23]
IDS_LEG  = [9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21]
LEG_IDS = IDS_LEG 
DEFAULT_TICKS = 2048

# ============================================================
# 全域變數：儲存從 Driver 收到的馬達資訊
# ============================================================
current_motor_state: Dict[int, int] = {}
motor_state_lock = threading.Lock()
# 每顆馬達最後一次在 /joint_commands 上的目標刻度（motion / walking 送出的 goal，不含重力下垂）
commanded_ticks: Dict[int, int] = {}

# IK 自動相容層
import inspect
try:
    _IK_NPARAM = len(inspect.signature(compute_leg_ik).parameters)
except Exception:
    _IK_NPARAM = 8

def compute_ik_auto(lx, ly, lz, lyaw, rx, ry, rz, ryaw, lroll=0.0, lpitch=0.0, rroll=0.0, rpitch=0.0):
    if _IK_NPARAM >= 12:
        return compute_leg_ik(lx, ly, lz, lroll, lpitch, lyaw, rx, ry, rz, rroll, rpitch, ryaw)
    else:
        return compute_leg_ik(lx, ly, lz, lyaw, rx, ry, rz, ryaw)

walking = WalkingGaitByLIPM()
# _gait_dt = float(getattr(walking, "sample_time_", 30)) / 1000.0
_gait_dt = float(getattr(walking, "sample_time_", 20)) / 1000.0
DT = _gait_dt if _gait_dt > 0 else 1.0 / 33.333
CONTROL_RATE_HZ = 1.0 / DT

STATE_NAME = {StartStep: "StartStep", FirstStep: "FirstStep", Repeat: "Repeat", StopStep: "StopStep"}

# ============================================================
# 輔助函式
# ============================================================
def apply_ankle_compensation(ang_list: List[float]) -> List[float]:
    comp_deg = float(getattr(parameter, "compensation_swing_ankle", 0.0))
    if abs(comp_deg) < 1e-9: return ang_list
    comp_rad = math.radians(comp_deg)
    new_ang = list(ang_list)
    new_ang[5]  += comp_rad
    new_ang[11] -= comp_rad
    return new_ang

def calc_rel_gp_from_ang(ang_now: List[float], baseline_ang: List[float]) -> Dict[int, int]:
    """
    計算 IK 角度的變化量 (Delta)，並轉換成 Ticks
    """
    rel: Dict[int, int] = {}
    for idx, mid in enumerate(IK_IDS):
        delta = float(ang_now[idx]) - float(baseline_ang[idx])
        rel[mid] = int(round(DIR[mid] * TPR * delta))
    return rel

# 絕對零點 = URDF 關節角 0：雙腳全直 (內部偏移量 0 = 腳底板間距 SOLE_GAP_STRAIGHT、膝蓋全直 → 高度 = L1 + L2)
ANCHOR_WIDTH  = 0.0   # 內部偏移量，不是 width_size
ANCHOR_HEIGHT = float(getattr(parameter, "L1", 12.5)) + float(getattr(parameter, "L2", 12.5))
LEG_ZERO_TICKS: Dict[int, int] = dict(getattr(parameter, "LEG_ZERO_TICKS", {}))
SOLE_GAP_STRAIGHT = float(getattr(parameter, "SOLE_GAP_STRAIGHT", 2.6))

STAND_MOVE_TIME = float(getattr(parameter, "STAND_MOVE_TIME", 0.0))
STAND_MIN_VELOCITY = max(1, int(getattr(parameter, "STAND_MIN_VELOCITY", 1)))
PV_TICKS_PER_S = 0.229 * 4096.0 / 60.0   # Profile Velocity 1 單位 = 0.229 rpm ≈ 15.63 ticks/s

def sync_velocity(target: Dict[int, int], current: Dict[int, int], move_time: float) -> Dict[int, int]:
    """讓每顆馬達在 move_time 秒內同時到位的 Profile Velocity。
    最小給 STAND_MIN_VELOCITY（太慢推不動腳底板的靜摩擦），最大 1023（driver 的上限）。"""
    vel = {}
    for mid, tgt in target.items():
        cur = current.get(mid, tgt)
        d = abs(ticks_diff_signed(tgt, cur))
        vel[mid] = max(STAND_MIN_VELOCITY, min(1023, int(math.ceil(d / move_time / PV_TICKS_PER_S))))
    return vel

def ticks_diff_signed(a: float, b: float, mod: int = 4096) -> float:
    d = (a - b) % mod
    return d - mod if d > mod / 2 else d

# URDF 關節角 0（LEG_ZERO_TICKS，雙腳全直）在 IK 輸出裡對應的角度
STRAIGHT_ANG = compute_ik_auto(0, 0, ANCHOR_HEIGHT, 0, 0, 0, ANCHOR_HEIGHT, 0)

def ticks_to_ang(ticks: Dict[int, int]) -> List[float]:
    """腿部馬達刻度 → IK 角度（以 LEG_ZERO_TICKS = 全直為零點）。"""
    return [STRAIGHT_ANG[idx] + DIR[mid] * ticks_diff_signed(ticks[mid], LEG_ZERO_TICKS.get(mid, DEFAULT_TICKS)) / TPR
            for idx, mid in enumerate(IK_IDS)]

def _fk_leg(roll: float, hip: float, knee: float):
    """compute_leg_ik 的反運算（單腳）：hip roll / hip pitch / knee 角度 → 腳踝相對髖的 (x, y, z)。"""
    l1 = float(getattr(parameter, "L1", 12.5)); l2 = float(getattr(parameter, "L2", 12.5))
    r = math.sqrt(max(l1*l1 + l2*l2 + 2*l1*l2*math.cos(knee), 1e-9))
    val = max(-1.0, min(1.0, (l1*l1 + r*r - l2*l2) / (2*l1*r)))
    alpha = math.pi/2 - math.acos(val) - hip
    x = r * math.sin(alpha); sxy = r * math.cos(alpha)
    return x, sxy * math.cos(roll), sxy * math.sin(roll)

def stand_pose_from_ticks(ticks: Dict[int, int]):
    """stand 刻度 → 雙腳腳踝位置 (lx, ly, lz, rx, ry, rz)。"""
    ang = ticks_to_ang(ticks)
    hr = 0.0
    if int(getattr(parameter, "walking_mode", 0)) in [1, 2]:
        hr = math.radians(float(getattr(parameter, "Hip_roll", 0.0)))
    lx, ly, lz = _fk_leg(ang[1] - hr, ang[2], ang[3])
    rx, ry, rz = _fk_leg(ang[7] + hr, ang[8], ang[9])
    return (lx, ly, lz, rx, ry, rz)

def pose_ik(pose, dw: float, dh: float) -> List[float]:
    """stand 姿勢 + 調整量 (dw 全寬，兩腳各 dw/2；dh 高度) 的 IK 角度。"""
    lx, ly, lz, rx, ry, rz = pose
    return apply_ankle_compensation(compute_ik_auto(lx, ly + dw/2, lz + dh, 0,
                                                    rx, ry - dw/2, rz + dh, 0))

def ticks_from_anchor(target_ang: List[float], anchor_ang: List[float],
                      anchor_ticks: Dict[int, int]) -> Dict[int, int]:
    """以錨點(刻度+角度)為基準，換算目標角度的腿部絕對刻度。"""
    rel = calc_rel_gp_from_ang(target_ang, anchor_ang)
    return {mid: int((anchor_ticks.get(mid, DEFAULT_TICKS) + rel.get(mid, 0)) % 4096)
            for mid in LEG_IDS}

# ============================================================
# Parameter 管理
# ============================================================
PARAM_KEYS = ["period_t","Tdsp","COM_HEIGHT","STAND_HEIGHT","lift_height",
              "com_y_swing","COM_Y_shift","width_size","step_length","shift_length",
              "theta","THTA","compensation_swing_ankle",
              "Board_High", "Clearance", "Hip_roll", "Ankle_roll"]

def get_param_dict():
    out = {}
    for k in PARAM_KEYS:
        if hasattr(parameter, k): out[k] = float(getattr(parameter, k))
        # print(f"[Param] {k} = {out[k]}")
    return out

def apply_param_dict(d):
    # 1. 先嘗試套用所有傳進來的參數
    for k, v in d.items():
        # ★ 修正：只要原本就有(hasattr)，或者在你的合法名單(PARAM_KEYS)裡，都准許通過！
        if hasattr(parameter, k) or k in PARAM_KEYS: 
            try: setattr(parameter, k, float(v))
            except Exception: pass
    
    # 2. 處理特殊計算 (Tc_)
    if hasattr(parameter, "COM_HEIGHT") and hasattr(parameter, "G"):
        try:
            if float(parameter.COM_HEIGHT) > 0.0 and float(parameter.G) > 0.0:
                parameter.Tc_ = math.sqrt(float(parameter.COM_HEIGHT) / float(parameter.G))
        except Exception: pass
    
    # 3. 更新 walking 物件的參數
    if hasattr(walking, "period_t"): walking.period_t = float(parameter.period_t)
    if hasattr(walking, "T_DSP_"):
        try: walking.T_DSP_ = max(0.0, min(1.0, float(parameter.Tdsp)))
        except Exception: pass
    if hasattr(walking, "width_size_"): walking.width_size_ = parameter.width_offset(getattr(parameter, "width_size", 0.0))
    if hasattr(walking, "lift_height_"): walking.lift_height_ = float(getattr(parameter, "lift_height", 0.0))
    if hasattr(walking, "stand_height"): walking.stand_height = parameter.stand_height_abs(getattr(parameter, "STAND_HEIGHT", 0.0))
    if hasattr(walking, "com_height"): walking.com_height = float(getattr(parameter, "COM_HEIGHT", 0.0))
    if hasattr(walking, "step_length_"): walking.step_length_ = float(getattr(parameter, "step_length", 0.0))
    if hasattr(walking, "shift_length_"): walking.shift_length_ = float(getattr(parameter, "shift_length", 0.0))
    
    # =========================================================================
    # ★ 關鍵修改：強制鎖定 sample_time 為 30
    # =========================================================================
    # 不管上面讀到了什麼 (0 或亂碼)，這裡直接覆寫回 30
    # FIXED_SAMPLE_TIME = 30.0
    FIXED_SAMPLE_TIME = 20.0
    
    # 更新 Parameter 物件
    if hasattr(parameter, "sample_time"):
        parameter.sample_time = FIXED_SAMPLE_TIME
        
    # 更新 WalkingGait 物件 (這是防止 ZeroDivisionError 的關鍵)
    if hasattr(walking, "sample_time_"):
        walking.sample_time_ = FIXED_SAMPLE_TIME
        # print(f"[DEBUG] sample_time enforced to {FIXED_SAMPLE_TIME}") # 需要除錯時可打開
    # =========================================================================

    if hasattr(walking, "var_theta_"):
        theta_val = None
        if "THTA" in d: 
            try: theta_val = float(d["THTA"])
            except: pass
        elif "theta" in d:
            try: theta_val = float(d["theta"])
            except: pass
        if theta_val is None: theta_val = float(getattr(parameter, "THTA", 0.0))
        try: parameter.THTA = float(theta_val)
        except: pass
        walking.var_theta_ = float(theta_val)

# ============================================================
# Node 類別
# ============================================================
class WalkingNode(Node):
    def __init__(self):
        super().__init__('walking_strategy_node')
        self.cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)
        self.create_subscription(JointState, '/joint_commands', self._joint_cmd_cb, 10)

        # Web Interface Topics
        self.create_subscription(Int16, '/ContinousMode_Topic', self._mode_cb, 10)
        self.create_subscription(Interface, '/SendBodyAuto_Topic', self._cmd_cb, 10)
        self.create_subscription(Interface, '/ChangeContinuousValue_Topic', self._cmd_cb, 10)
        self.param_pub = self.create_publisher(String, '/walking_params', 10)
        self.create_subscription(String, '/walking_params_update', self._param_update_cb, 10)
        self.create_subscription(Bool, '/walking_params_request', self._param_request_cb, 10)

        self.req_reset_anchor: bool = False
        self.create_subscription(Bool, '/walking_reset_anchor', self._reset_anchor_cb, 10)

        self.walk_active: bool = False
        self.last_width_size: float = float(getattr(parameter, "width_size", 0.0))
        self.width_dirty: bool = False
        self._own_stamps = deque(maxlen=200)   # walking 自己送出的 /joint_commands stamp

        self.get_logger().info("Walking Node Initialized. Waiting for Driver Data...")
        self.publish_params()

    def _joint_cmd_cb(self, msg: JointState):
        """記錄 motion 最後一次送出的目標刻度（抓 stand 錨點用）；walking 自己送的不算"""
        st = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if st in self._own_stamps:
            return
        with motor_state_lock:
            for i, name in enumerate(msg.name):
                try:
                    commanded_ticks[int(name)] = int(round(msg.position[i]))
                except (ValueError, IndexError):
                    pass

    def _joint_state_cb(self, msg: JointState):
        with motor_state_lock:
            for i, name in enumerate(msg.name):
                try:
                    mid = int(name)
                    pos = int(msg.position[i])
                    current_motor_state[mid] = pos
                except ValueError:
                    pass

    def publish_command(self, target_ticks: Dict[int, int], velocity: Dict[int, int] = None):
        """velocity=None → 不帶速度（driver 收到 0 = 最快）；有給就逐顆帶 Profile Velocity。"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        self._own_stamps.append((msg.header.stamp.sec, msg.header.stamp.nanosec))
        msg.name = []
        msg.position = []
        for mid, tick in target_ticks.items():
            msg.name.append(str(mid))
            msg.position.append(float(tick))
            if velocity is not None:
                msg.velocity.append(float(velocity.get(mid, 0)))
        self.cmd_pub.publish(msg)

    def _mode_cb(self, msg: Int16):
        self.walk_active = (msg.data == 1)
        self.get_logger().info(f'recv /ContinousMode_Topic={msg.data} -> walk_active={self.walk_active}')

    # Callback 函式
    def _reset_anchor_cb(self, msg: Bool):
        if msg.data:
            self.req_reset_anchor = True
            # self.get_logger().warn("[Request] Received Anchor Reset Flag! Next 'Generate' will update baseline.")

    def _cmd_cb(self, msg: Interface):
        scale = 0.001; theta_scale = 1.0
        parameter.step_length  = float(msg.x) * scale
        parameter.shift_length = float(msg.y) * scale
        parameter.THTA         = float(math.radians(msg.theta)) * theta_scale
        parameter.X = float(msg.x) * scale; parameter.Y = float(msg.y) * scale; parameter.theta = float(msg.theta) * theta_scale
        walking.step_length_  = float(msg.x) * scale
        walking.shift_length_ = float(msg.y) * scale
        walking.var_theta_    = float(math.radians(msg.theta)) * theta_scale
        parameter.walking_mode = msg.walking_mode

    def publish_params(self):
        d = get_param_dict()
        msg = String()
        msg.data = json.dumps(d)
        self.param_pub.publish(msg)

    def _param_request_cb(self, msg: Bool):
        if msg.data: self.publish_params()

    def _param_update_cb(self, msg: String):
        try: d = json.loads(msg.data)
        except Exception as e: return
        # width_size / STAND_HEIGHT = 相對調好 stand 的調整量，可正可負
        if "width_size" in d:
            try:
                gap = parameter.sole_gap(d["width_size"])
                if gap < 0.0:
                    self.get_logger().warn(f"width_size={d['width_size']}：腳底板間距約 {gap:.2f} cm，可能會互相碰到")
            except (TypeError, ValueError):
                d.pop("width_size")
        # height：站高超過腿長（膝蓋反折）或蹲到 <= 0 → 拒絕
        if "STAND_HEIGHT" in d:
            try:
                h = parameter.stand_height_abs(d["STAND_HEIGHT"])
                h_max = ANCHOR_HEIGHT
                if not (0.0 < h < h_max):
                    self.get_logger().error(
                        f"拒絕 STAND_HEIGHT={d['STAND_HEIGHT']}：調整後高度 {h:.2f} cm 超出 (0, {h_max:.2f}) "
                        f"(stand 高度 {parameter.STAND_BASE_HEIGHT} cm)，"
                        f"保留 {getattr(parameter, 'STAND_HEIGHT', 0.0)}")
                    d.pop("STAND_HEIGHT")
            except (TypeError, ValueError):
                d.pop("STAND_HEIGHT")
        apply_param_dict(d)
        if bool(d.get("generate", False)):
            self.walk_active = not self.walk_active
            self.get_logger().info(f"[generate] toggle walk_active -> {self.walk_active}")
        if "width_size" in d:
            try:
                new_w = float(d["width_size"])
                if abs(new_w - self.last_width_size) > 1e-6:
                    self.last_width_size = new_w
                    self.width_dirty = True
            except: pass
        # 網頁按 Send 帶有站姿參數 → 一律重算站姿（即使數值跟啟動預設一樣，也要從全直移過去）
        if any(k in d for k in ("width_size", "STAND_HEIGHT", "COM_HEIGHT")):
            self.width_dirty = True
        self.publish_params()

# ============================================================
# Main Loop
# ============================================================
def main():
    print(f"[info] Control Loop DT={DT*1000:.1f} ms ({CONTROL_RATE_HZ:.2f} Hz)")
    rclpy.init(args=None)
    node = WalkingNode()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # 變數宣告
    baseline_ticks: Dict[int, int] = {} # 這是目前的基準 (走路用的)
    baseline_ang: List[float] = []      # 這是目前的角度 (走路用的)
    
    
    was_walking = False 
    stopping_active = False       
    saved_lift_height = None
    
    landing_mode = False       
    landing_counter = 0        
    LANDING_LIMIT = 20         

    # 參數追蹤
    # ★ 設定初始預設值 (請確保這裡跟 Parameter.py 預設一樣)
    # start_w = 4.5 
    # 基準站姿 = 調好的 stand：刻度 + FK 算出的腳位置 + 對應 IK 角度
    stand_ticks: Dict[int, int] = {}
    stand_pose = None
    stand_ang: List[float] = []

    def capture_stand(tag: str):
        """抓 stand 刻度：motion 送出的目標刻度優先，其次實際刻度，都沒有 → LEG_ZERO_TICKS（全直）。
        再用 FK 算出 stand 的寬度 / 高度，更新 parameter.STAND_BASE_*。"""
        nonlocal stand_ticks, stand_pose, stand_ang
        with motor_state_lock:
            snap = dict(current_motor_state)
            cmd = dict(commanded_ticks)
        src = []
        ticks = {}
        for mid in LEG_IDS:
            if mid in cmd:
                ticks[mid] = int(cmd[mid]); src.append("cmd")
            elif int(snap.get(mid, 0)) > 0:
                ticks[mid] = int(snap[mid]); src.append("now")
            else:
                ticks[mid] = LEG_ZERO_TICKS.get(mid, DEFAULT_TICKS); src.append("zero")
        stand_ticks = ticks
        stand_pose = stand_pose_from_ticks(ticks)
        stand_ang = pose_ik(stand_pose, 0.0, 0.0)
        lx, ly, lz, rx, ry, rz = stand_pose
        parameter.STAND_BASE_WIDTH = ly - ry
        parameter.STAND_BASE_HEIGHT = (lz + rz) / 2
        print(f"[{tag}] Stand 錨點 ({'/'.join(sorted(set(src)))}): "
              f"L=({lx:.2f},{ly:.2f},{lz:.2f}) R=({rx:.2f},{ry:.2f},{rz:.2f}) "
              f"→ 高度 {parameter.STAND_BASE_HEIGHT:.2f} cm，腳底板間距 {parameter.sole_gap(0):.2f} cm")

    def stand_target(dw: float, dh: float) -> Dict[int, int]:
        """stand 刻度 + (調整量造成的 IK 角度變化)"""
        return ticks_from_anchor(pose_ik(stand_pose, dw, dh), stand_ang, stand_ticks)

    def sync_walking_stand():
        walking.width_size_ = parameter.width_offset(getattr(parameter, "width_size", 0.0))
        walking.stand_height = parameter.stand_height_abs(getattr(parameter, "STAND_HEIGHT", 0.0))

    last_applied_dw = None
    last_applied_dh = None
    last_applied_com = float(getattr(parameter, "COM_HEIGHT", 0.0))
    WIDTH_EPS = 1e-6

    t_next = time.perf_counter() + DT
    last_dbg_t = 0.0

    try:
        print("[setup] Waiting for motor data...")
        while rclpy.ok():
            with motor_state_lock:
                if len([mid for mid in LEG_IDS if mid in current_motor_state]) == len(LEG_IDS):
                    print(f"[setup] Connection Established!")
                    break
            time.sleep(0.1)
        
        # ---------------------------------------------------------
        # ★ 修改：這裡不再抓取 initial_ticks 了！
        # 我們留到按下 Generate 之後才抓，這樣你才有機會手動扶正機器人
        # ---------------------------------------------------------
        # print(f"[setup] Ready. Please enable 'Generate' to set the Anchor Pose.")

        while rclpy.ok():
            is_walking = node.walk_active
            now = time.perf_counter()
            ws = getattr(walking, "walking_state", None)

            # Debug Log
            # if now - last_dbg_t > 1.0:
            #     last_dbg_t = now
            #     mode_str = "LANDING_Override" if landing_mode else ("WALKING" if (is_walking or stopping_active) else "IDLE")
            #     state_name = STATE_NAME.get(ws, str(ws))
            #     current_w = float(getattr(walking, "width_size_", 0.0))
            #     # 顯示目前是否已經定錨
            #     anchor_status = "SET" if initial_ticks else "NOT_SET"
            #     # print(f"[Loop] Mode={mode_str} | Anchor={anchor_status} | Width={current_w:.2f}", flush=True)

            # ==========================================================
            # 1. Generate ON (走路開始) --> ★ 在這裡定錨！
            # ==========================================================
            if is_walking and not was_walking:
                print("\n[Generate ON] STARTING...")
                
                # 錨點 = 調好的 stand（motion 執行過 → 重抓；還沒抓過 → 抓一次）
                if node.req_reset_anchor or stand_pose is None:
                    capture_stand("Generate ON")
                node.req_reset_anchor = False
                sync_walking_stand()

                # 移到 stand + 網頁調整量（調整量都 0 → 刻度就是 stand）
                dw = float(getattr(parameter, "width_size", 0.0))
                dh = float(getattr(parameter, "STAND_HEIGHT", 0.0))
                baseline_ticks = stand_target(dw, dh)
                node.publish_command(baseline_ticks)
                last_applied_dw, last_applied_dh = dw, dh
                last_applied_com    = float(getattr(parameter, "COM_HEIGHT", 0.0))
                node.width_dirty = False

                # =========================================================
                # ★ 修正重點：這一段必須「跳出」else，向左縮排！
                # 確保無論是 Reset 還是 Resume，這裡都會被執行到
                # =========================================================
                walking.final_step()
                b_ang = compute_ik_auto(
                    walking.end_point_lx_, walking.end_point_ly_, walking.end_point_lz_, walking.end_point_lthta_,
                    walking.end_point_rx_, walking.end_point_ry_, walking.end_point_rz_, walking.end_point_rthta_,
                    lroll=getattr(walking, "end_point_lroll_", 0.0),
                    rroll=getattr(walking, "end_point_rroll_", 0.0),
                )
                baseline_ang = apply_ankle_compensation(b_ang)
                stopping_active = False
                landing_mode = False
                landing_counter = 0
                if saved_lift_height is not None:
                    try: walking.lift_height_ = float(saved_lift_height)
                    except: pass
                    saved_lift_height = None
                    
            # ==========================================================
            # 2. Generate OFF (走路停止)
            # ==========================================================
            if not is_walking and was_walking:
                # ... (維持原樣) ...
                print("[Generate OFF] Stopping requested...")
                stopping_active = True
                landing_mode = False
                try:
                    walking.ready_to_stop_ = True
                    walking.step_ = walking.now_step_ + 1
                except: pass
                node.width_dirty = False

            was_walking = is_walking

            # 3. Landing Logic ... (維持原樣) ...
            if stopping_active and not landing_mode:
                rts = bool(getattr(walking, "ready_to_stop_", False))
                if current_mode in [1, 2] or ((ws == StartStep) and (not rts)):
                    landing_mode = True
                    landing_counter = 0

            # ==========================================================
            # 分支 A: 走路中
            # ==========================================================
            if (is_walking or stopping_active) and not landing_mode:
                if stopping_active and saved_lift_height is None:
                     saved_lift_height = float(getattr(walking, "lift_height_", 0.0))

                walking.process()

                current_mode = getattr(parameter, "walking_mode", 0)
                if current_mode in [1, 2]:
                    if not hasattr(node, "trajectory_log"): node.trajectory_log = []
                    t_sec = walking.time_point_ / 1000.0
                    node.trajectory_log.append(f"{t_sec:.3f},{walking.now_step_},{walking.lpx_:.3f},{walking.lpz_:.3f},{walking.rpx_:.3f},{walking.rpz_:.3f},{walking.pz_:.3f}")
                    if getattr(walking, "ready_to_stop_", False): node.walk_active = False

                ang_now = compute_ik_auto(
                    walking.end_point_lx_, walking.end_point_ly_, walking.end_point_lz_, walking.end_point_lthta_,
                    walking.end_point_rx_, walking.end_point_ry_, walking.end_point_rz_, walking.end_point_rthta_,
                    lroll=getattr(walking, "end_point_lroll_", 0.0), rroll=getattr(walking, "end_point_rroll_", 0.0)
                )
                ang_now = apply_ankle_compensation(ang_now)
                
                # 走路時使用動態更新的 baseline
                if baseline_ang:
                    rel_cmd = calc_rel_gp_from_ang(ang_now, baseline_ang)
                    gp_abs = {}
                    for mid in LEG_IDS:
                        base = baseline_ticks.get(mid, DEFAULT_TICKS)
                        delta = rel_cmd.get(mid, 0)
                        gp_abs[mid] = int((base + delta) % 4096)
                    node.publish_command(gp_abs)

            # ==========================================================
            # 分支 B: 落地模式
            # ==========================================================
            elif landing_mode:
                if baseline_ticks:
                    node.publish_command(baseline_ticks)
                landing_counter += 1
                if landing_counter >= LANDING_LIMIT:
                    print(f"[state] Landing finished. Restored Baseline.")
                    stopping_active = False
                    landing_mode = False

                    if getattr(parameter, "walking_mode", 0) in [1, 2]:
                        parameter.walking_mode = 0
                        print("[state] LC Step Completed. Auto-reset to Mode 0.")
                    
                    if hasattr(node, "trajectory_log") and len(node.trajectory_log) > 0:
                        import os
                        save_path = os.path.expanduser("~/trajectory_log.csv")
                        try:
                            with open(save_path, "w", encoding="utf-8") as f:
                                f.write("time,step,lpx,lpz,rpx,rpz,com_z\n")
                                f.write("\n".join(node.trajectory_log))
                            print(f"\033[92m[Debug] CSV saved to {save_path}\033[0m")
                        except Exception as e: pass
                        node.trajectory_log.clear()

                    if saved_lift_height is not None:
                        try: walking.lift_height_ = float(saved_lift_height)
                        except: pass
                        saved_lift_height = None

            # ==========================================================
            # 分支 C: Idle 站姿調整
            # ==========================================================
            else:
                dw = float(getattr(parameter, "width_size", 0.0))
                dh = float(getattr(parameter, "STAND_HEIGHT", 0.0))
                cur_com = float(getattr(parameter, "COM_HEIGHT", 0.0))
                is_dirty = node.width_dirty

                changed = (last_applied_dw is None or abs(dw - last_applied_dw) > WIDTH_EPS
                           or abs(dh - last_applied_dh) > WIDTH_EPS
                           or abs(cur_com - last_applied_com) > WIDTH_EPS)

                if changed or is_dirty:
                    node.width_dirty = False
                    # 錨點 = 調好的 stand（motion 執行過 → 重抓；還沒抓過 → 抓一次）
                    if node.req_reset_anchor or stand_pose is None:
                        capture_stand("Idle")
                    node.req_reset_anchor = False
                    sync_walking_stand()

                    new_ticks = stand_target(dw, dh)
                    print(f"[Idle] Stand + (width {dw:+.2f}, height {dh:+.2f}) → "
                          f"高度 {walking.stand_height:.2f} cm，腳底板間距 {parameter.sole_gap(dw):.2f} cm")

                    # Debug：每顆腿部馬達 目標(相對 stand) / 目前實際刻度
                    with motor_state_lock:
                        cur_snapshot = dict(current_motor_state)
                    for side, ids in (("L", IK_IDS[:6]), ("R", IK_IDS[6:])):
                        print(f"  [{side}] " + "  ".join(
                            f"{mid}:{new_ticks[mid]}({new_ticks[mid]-stand_ticks[mid]:+d})/now {cur_snapshot.get(mid, -1)}"
                            for mid in ids))

                    # 發送指令（限速：所有腿部馬達在 STAND_MOVE_TIME 秒內同時到位）
                    if STAND_MOVE_TIME > 0:
                        node.publish_command(new_ticks,
                                             sync_velocity(new_ticks, cur_snapshot, STAND_MOVE_TIME))
                    else:
                        node.publish_command(new_ticks)

                    # 更新 Baseline (為了讓下次走路是從這個新姿勢開始)
                    baseline_ticks = dict(new_ticks)
                    last_applied_dw, last_applied_dh = dw, dh
                    last_applied_com = cur_com

            # # --- dt / phase debug ---
            if not hasattr(node, "_last_loop_now"):
                node._last_loop_now = time.perf_counter()
                node._last_tnext = t_next
            else:
                _now = time.perf_counter()
                loop_dt = _now - node._last_loop_now
                lag = _now - t_next   # >0 代表落後
                if _now - last_dbg_t > 0.5:  # 每 0.5 秒印一次
                    last_dbg_t = _now
                    print(f"[LOOP] loop_dt={loop_dt*1000:.2f}ms  lag={lag*1000:.2f}ms  DT={DT*1000:.2f}ms")
                node._last_loop_now = _now

            # 頻率鎖定
            now = time.perf_counter()
            if now < t_next: time.sleep(t_next - now); t_next += DT
            else: skipped = int((now - t_next) // DT) + 1; t_next += DT * skipped

    except KeyboardInterrupt:
        print("\n[Ctrl+C]")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("[final] done.")

if __name__ == "__main__":
    main()
