"""跑 Isaac Lab 訓練出來的盲走 policy（ONNX）於真機。

    ros2 launch rl_deploy policy.launch.py
    ros2 topic pub --once /rl_deploy/enable std_msgs/Bool '{data: true}'   # → 站姿
    ros2 topic pub --once /rl_deploy/go     std_msgs/Bool '{data: true}'   # → 跑 policy
    ros2 topic pub /rl_deploy/cmd_vel geometry_msgs/Twist \\
        '{linear: {x: 0.2}}'                                              # 下指令
    ros2 topic pub --once /rl_deploy/enable std_msgs/Bool '{data: false}'  # 退回站姿

── 狀態機（照 unitree_rl_gym deploy_real.py 的分段）────────────────────

    IDLE        扭力關、不發指令。等 /rl_deploy/enable
    TO_STAND    3 秒插值到站姿（全身 21 顆）
    HOLD_STAND  夾住站姿。進來時觸發 IMU 歸零。等 /rl_deploy/go
    POLICY      50 Hz 推論。任一保護觸發就退回 HOLD_STAND

退回 HOLD_STAND 的條件：
    · /joint_states 或 IMU 超過 stale_timeout_ms 沒更新
    · 傾角超過 max_tilt_deg
    · /rl_deploy/enable 變 false（會一路退到 IDLE 並關扭力）

── 觀測（47 維，順序照 KIDDeployObservationsCfg 的宣告）────────────────

    base_ang_vel       3   gyr(deg/s)→rad/s，套 imu_rotation
    projected_gravity  3   **從 QUAT 算**，不是加速度計（理由見 config）
    velocity_commands  3   /rl_deploy/cmd_vel
    joint_pos         12   (q_real − q_ref)，policy 順序
    joint_vel         12   Present Velocity → rad/s，policy 順序
    actions           12   上一步的原始輸出（未縮放）
    gait_phase         2   sin/cos(2π·φ)，φ = (t mod 0.60)/0.60

動作：q_target = q_ref + 0.25 × clip(action)，再換成刻度。

── ⚠️ 開始之前一定要填 policy_joint_order ──────────────────────────────

policy 的 12 個輸出對應哪 12 個關節，是由**模擬器內部的關節順序**決定的
（PhysX 的順序，不是 LEG_JOINTS 的列表順序）。填錯等於把動作接到錯誤的
關節上 —— 機器人會做出完全無關的動作。

    cd ~/IsaacLab && ./isaaclab.sh -p ~/isaac_projects/kid_rl/inspect_usd.py

把它印出來的關節順序填進 config/rl_deploy.yaml 的 `policy_joint_order`。
**沒填（還是預設的佔位值）這個節點會拒絕啟動。**
"""

from __future__ import annotations

import math
import os
import time

import numpy as np
import onnxruntime as ort
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int16MultiArray, String

from tku_msgs.msg import SensorPackage, SensorSet

from . import joint_map as jm

GAIT_PERIOD = 0.60  # 跟 kid_rl.gait.GAIT_PERIOD 一致
DEG2RAD = math.pi / 180.0

IDLE, TO_STAND, HOLD_STAND, POLICY = "IDLE", "TO_STAND", "HOLD_STAND", "POLICY"

# config 裡的佔位值 —— 看到這個就拒絕啟動
PLACEHOLDER_ORDER = ["FILL_ME"]


class PolicyNode(Node):
    def __init__(self) -> None:
        super().__init__("rl_policy")

        # ── 參數 ──────────────────────────────────────────────────────
        p = self.declare_parameters(
            "",
            [
                ("model", "kid_6600.onnx"),
                ("obs_dim", 47),
                ("control_hz", 50.0),
                ("action_scale", jm.ACTION_SCALE),
                ("action_clip", jm.CLIP_ACTION),
                ("policy_joint_order", PLACEHOLDER_ORDER),
                ("imu_rotation", [1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0]),
                ("gravity_source", "quat"),
                ("gravity_rereference", True),
                ("joint_vel_source", "register"),
                ("pose_reference", "stand_ini"),
                ("zero_imu_at_stand", True),
                ("max_tilt_deg", 30.0),
                # 100 ms 太緊：馬達出力時 RS485 偶發掉包，SDK 的 LATENCY_TIMER
                # 修補後單次停頓約 20 ms（修補前 110 ms）。200 ms = 10 個控制
                # 週期，仍是有效的保護界線（期間馬達維持上一個指令不動）。
                ("stale_timeout_ms", 200.0),
                # 超過這個就記一筆（不中止），用來看停頓的分布
                ("stale_warn_ms", 50.0),
                ("stand_ramp_s", 3.0),
                # 進 POLICY 時把 action_scale 從 0 線性升到滿值的秒數。
                # LSTM 的隱藏狀態在進 POLICY 時被清零，第一步的輸出是「冷啟動」
                # 的，直接全幅套用會讓吊著的機器人猛然一踢（實測一進 POLICY
                # 就量到 55° 傾角）。0.5 s = 25 步，只影響起步。
                ("action_ramp_s", 0.5),
                # > 0 時以這個頻率把觀測與動作逐項印出來，用來對帳是不是
                # 哪一段的尺度/符號組錯了。0 = 關。
                ("log_obs_hz", 0.0),
            ],
        )
        g = {q.name: q.value for q in p}
        self.hz = float(g["control_hz"])
        self.dt = 1.0 / self.hz
        self.obs_dim = int(g["obs_dim"])
        self.a_scale = float(g["action_scale"])
        self.a_clip = float(g["action_clip"])
        self.R = np.asarray(g["imu_rotation"], dtype=np.float64).reshape(3, 3)
        self.grav_src = str(g["gravity_source"])
        self.grav_reref = bool(g["gravity_rereference"])
        self.vel_src = str(g["joint_vel_source"])
        self.zero_imu = bool(g["zero_imu_at_stand"])
        self.max_tilt = float(g["max_tilt_deg"])
        self.stale_s = float(g["stale_timeout_ms"]) / 1000.0
        self.warn_s = float(g["stale_warn_ms"]) / 1000.0
        self.ramp_s = float(g["stand_ramp_s"])
        self.a_ramp_s = float(g["action_ramp_s"])
        self.log_obs_dt = (
            0.0 if float(g["log_obs_hz"]) <= 0 else 1.0 / float(g["log_obs_hz"])
        )
        self.t_log = 0.0

        # ── policy 的關節順序（必填）────────────────────────────────
        order = list(g["policy_joint_order"])
        if order == PLACEHOLDER_ORDER or len(order) != len(jm.LEG_IDS):
            raise SystemExit(
                "❌ policy_joint_order 還沒填（或長度不是 12）。\n"
                "   policy 的 12 個輸出對應哪 12 個關節是由模擬器內部的關節順序\n"
                "   決定的（PhysX 順序，不是 LEG_JOINTS 的列表順序）。填錯等於把\n"
                "   動作接到錯誤的關節上。\n\n"
                "   取得方式（在訓練電腦上）：\n"
                "     cd ~/IsaacLab && ./isaaclab.sh -p "
                "~/isaac_projects/kid_rl/inspect_usd.py\n\n"
                "   把印出來的 12 個腿關節名稱依序填進 config/rl_deploy.yaml。"
            )
        unknown = [n for n in order if n not in jm.SIM_TO_ID]
        if unknown or len(set(order)) != len(order):
            raise SystemExit(f"❌ policy_joint_order 有未知或重複的關節名：{unknown or order}")
        self.order_ids = [jm.SIM_TO_ID[n] for n in order]
        self.get_logger().info(f"policy 關節順序 → 馬達 ID {self.order_ids}")

        # ── 站姿基準 ──────────────────────────────────────────────────
        if str(g["pose_reference"]) == "stand_ini":
            self.ref_all = jm.read_stand_ini(ids=jm.ALL_STAND_IDS)
            src = "stand.ini"
        else:
            self.ref_all = {**jm.read_stand_ini(ids=jm.ALL_STAND_IDS), **jm.sim_stand_ticks()}
            src = "模擬 STAND_JOINT_POS"
        self.ref_leg = {mid: self.ref_all[mid] for mid in jm.LEG_IDS}
        self.get_logger().info(f"站姿基準：{src}（{len(self.ref_all)} 顆）")

        # ── ONNX ──────────────────────────────────────────────────────
        mp = str(g["model"])
        if not os.path.isabs(mp):
            mp = os.path.join(get_package_share_directory("rl_deploy"), "models", mp)
        self.sess = ort.InferenceSession(mp, providers=["CPUExecutionProvider"])
        ins = {i.name: i.shape for i in self.sess.get_inputs()}
        if ins.get("obs", [None, None])[1] != self.obs_dim:
            raise SystemExit(f"❌ 模型的 obs 維度是 {ins.get('obs')}，設定是 {self.obs_dim}")
        self.h = np.zeros((1, 1, 64), dtype=np.float32)
        self.c = np.zeros((1, 1, 64), dtype=np.float32)
        self.get_logger().info(f"模型 {os.path.basename(mp)}  輸入 {ins}")

        # ── 狀態 ──────────────────────────────────────────────────────
        self.state = IDLE
        self.enable = False
        self.go = False
        self.cmd = np.zeros(3, dtype=np.float32)
        self.prev_action = np.zeros(len(jm.LEG_IDS), dtype=np.float32)
        self.pos: dict[int, float] = {}
        self.vel_raw: dict[int, float] = {}
        self.prev_pos: dict[int, float] = {}
        self.vel_diff: dict[int, float] = {}
        self.imu: SensorPackage | None = None
        self.t_pos = 0.0
        self.t_imu = 0.0
        self.g_ref: np.ndarray | None = None  # 歸零時的 base 重力（再參考用）
        self.state_t = time.monotonic()
        self.stand_from: dict[int, float] | None = None
        self.phase_t0 = time.monotonic()
        self.n_clamp = 0
        self.max_age_pos = 0.0
        self.max_age_imu = 0.0

        # ── 介面 ──────────────────────────────────────────────────────
        self.create_subscription(JointState, "/joint_states", self._cb_pos, 20)
        self.create_subscription(JointState, "/joint_velocities", self._cb_vel, 20)
        self.create_subscription(SensorPackage, "/package/sensorpackage", self._cb_imu, 20)
        self.create_subscription(Bool, "/rl_deploy/enable", self._cb_enable, 10)
        self.create_subscription(Bool, "/rl_deploy/go", self._cb_go, 10)
        self.create_subscription(Twist, "/rl_deploy/cmd_vel", self._cb_cmd, 10)
        self.cmd_pub = self.create_publisher(JointState, "/joint_commands", 10)
        self.torque_pub = self.create_publisher(Int16MultiArray, "/set_torque", 10)
        self.zero_pub = self.create_publisher(SensorSet, "/sensorset", 10)
        self.status_pub = self.create_publisher(String, "/rl_deploy/status", 10)

        self.timer = self.create_timer(self.dt, self._loop)
        self.get_logger().info(
            f"就緒（{self.hz:.0f} Hz）。發 /rl_deploy/enable true 進站姿。"
        )

    # ── 回呼 ──────────────────────────────────────────────────────────
    def _cb_pos(self, msg: JointState) -> None:
        now = time.monotonic()
        new = {}
        for n, v in zip(msg.name, msg.position):
            try:
                new[int(n)] = float(v)
            except ValueError:
                pass
        if self.prev_pos and self.t_pos:
            dt = now - self.t_pos
            if dt > 1e-4:
                self.vel_diff = {
                    k: (new[k] - self.prev_pos[k]) / dt
                    for k in new
                    if k in self.prev_pos
                }
        self.prev_pos = dict(new)
        self.pos.update(new)
        self.t_pos = now

    def _cb_vel(self, msg: JointState) -> None:
        src = msg.position if len(msg.position) else msg.velocity
        for n, v in zip(msg.name, src):
            try:
                self.vel_raw[int(n)] = float(v)
            except ValueError:
                pass

    def _cb_imu(self, msg: SensorPackage) -> None:
        self.imu = msg
        self.t_imu = time.monotonic()

    def _cb_enable(self, msg: Bool) -> None:
        self.enable = bool(msg.data)
        if not self.enable:
            self._to(IDLE)

    def _cb_go(self, msg: Bool) -> None:
        self.go = bool(msg.data)

    def _cb_cmd(self, msg: Twist) -> None:
        self.cmd = np.array(
            [msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32
        )

    # ── 工具 ──────────────────────────────────────────────────────────
    def _to(self, state: str) -> None:
        if state == self.state:
            return
        self.get_logger().info(f"狀態 {self.state} → {state}")
        self.state = state
        self.state_t = time.monotonic()
        if state == IDLE:
            self.go = False
            self._torque(False)
        elif state == TO_STAND:
            self._torque(True)
            self.stand_from = {
                mid: self.pos.get(mid, self.ref_all[mid]) for mid in self.ref_all
            }
        elif state == HOLD_STAND:
            # QUAT 是相對歸零姿態的，所以在站姿歸零 —— 之後的姿態偏差才有意義
            if self.zero_imu:
                m = SensorSet()
                m.reset = True
                self.zero_pub.publish(m)
                self.get_logger().info("已要求 IMU 歸零（站姿 = 參考姿態）")
            self.g_ref = None
            self.h[:] = 0.0
            self.c[:] = 0.0  # 重新進 policy 前清掉 LSTM 記憶
            self.prev_action[:] = 0.0
        elif state == POLICY:
            self.phase_t0 = time.monotonic()
            self.n_clamp = 0
            self.max_age_pos = 0.0
            self.max_age_imu = 0.0

    def _torque(self, on: bool) -> None:
        m = Int16MultiArray()
        m.data = [0, 1 if on else 0]
        self.torque_pub.publish(m)

    def _send(self, ticks: dict[int, int], profile_vel: int = 0) -> None:
        msg = JointState()
        msg.name = [str(k) for k in ticks]
        msg.position = [float(v) for v in ticks.values()]
        msg.velocity = [float(profile_vel)] * len(ticks)
        self.cmd_pub.publish(msg)

    def _gravity(self) -> np.ndarray | None:
        """base_link 座標的 projected_gravity（單位向量）。"""
        m = self.imu
        if m is None:
            return None
        if self.grav_src == "accel":
            a = np.array([m.acc_x, m.acc_y, m.acc_z])
            n = np.linalg.norm(a)
            return None if n < 1e-6 else -(self.R @ (a / n))
        # QUAT：R_q 把感測器座標映到歸零時的參考座標
        q = np.array([m.quat_w, m.quat_x, m.quat_y, m.quat_z])
        n = np.linalg.norm(q)
        if n < 1e-6:
            return None
        w, x, y, z = q / n
        Rq = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ])
        if self.g_ref is None:
            # 第一次：用加速度計定出參考姿態的重力方向（此刻機器人是靜止站著的）
            a = np.array([m.acc_x, m.acc_y, m.acc_z])
            na = np.linalg.norm(a)
            if na < 1e-6:
                return None
            g0 = -(self.R @ (a / na))
            # gravity_rereference：把站姿當成「直立」餵給 policy。
            # 訓練時站姿的 projected_gravity ≈ (0, 0.019, -1)，而真機 stand.ini
            # 的 base 座標是 pitch +6.6°（量到 (+0.116, +0.021, -0.993)）——
            # 那 6.6° 是姿態差不是感測誤差。policy 的關節也是走「相對站姿」的
            # 偏差空間，所以姿態也用同一套基準最一致。
            # 代價：絕對傾角會被低/高估最多 6.6°（保護用的傾角另外用未再參考的值）。
            self.g_ref = np.array([0.0, 0.0, -1.0]) if self.grav_reref else g0
            self.g_ref_raw = g0
            self.get_logger().info(
                f"重力參考：站姿實測 ({g0[0]:+.3f}, {g0[1]:+.3f}, {g0[2]:+.3f})"
                f" → 餵給 policy 的參考 ({self.g_ref[0]:+.3f}, {self.g_ref[1]:+.3f},"
                f" {self.g_ref[2]:+.3f})"
            )
        # g_base(t) = R_m · R_q(t)ᵀ · R_mᵀ · g_ref
        return self.R @ Rq.T @ self.R.T @ self.g_ref

    def _tilt_acc(self) -> float:
        """加速度計算出的傾角。**運動中不可信** —— 它量的是 (a − g)。"""
        m = self.imu
        if m is None:
            return float("nan")
        a = np.array([m.acc_x, m.acc_y, m.acc_z])
        n = np.linalg.norm(a)
        if n < 1e-6:
            return float("nan")
        g = -(self.R @ (a / n))
        return math.degrees(math.acos(float(np.clip(-g[2], -1.0, 1.0))))

    def _tilt_deg(self) -> float:
        """保護用的傾角 —— **從 QUAT 算**，相對站姿。

        ⚠️ 2026-10-06 修正。原本用加速度計，結果一進 POLICY 就跳 55°／59°
           而觸發保護。加速度計量的是比力 (a − g)：吊著的機器人一踢腿就有
           大的水平加速度，55° 只需要 tan(55°)·g = 1.43 g 的水平分量。
           DMP 的 QUAT 是姿態估計，不受線加速度影響。

        相對站姿而不是絕對鉛垂，是刻意的：判準變成「我偏離站姿多少」，
        站姿本身就是 0°，門檻的意義更直接。絕對值仍可用 _tilt_acc()
        交叉比對（status 兩個都發）。
        """
        g = self._gravity()
        if g is None:
            return float("nan")
        return math.degrees(math.acos(float(np.clip(-g[2], -1.0, 1.0))))

    # ── 主迴圈 ────────────────────────────────────────────────────────
    def _loop(self) -> None:
        now = time.monotonic()
        age_pos, age_imu = now - self.t_pos, now - self.t_imu
        if self.state == POLICY:
            if age_pos > self.max_age_pos and age_pos > self.warn_s:
                self.get_logger().warn(
                    f"/joint_states 停頓 {age_pos*1000:.0f} ms"
                    f"（門檻 {self.stale_s*1000:.0f} ms）",
                    throttle_duration_sec=1.0,
                )
            self.max_age_pos = max(self.max_age_pos, age_pos)
            self.max_age_imu = max(self.max_age_imu, age_imu)
        fresh = age_pos < self.stale_s and age_imu < self.stale_s

        if self.state == IDLE:
            if self.enable and fresh and len(self.pos) >= len(jm.ALL_STAND_IDS):
                self._to(TO_STAND)
        elif self.state == TO_STAND:
            a = min(1.0, (now - self.state_t) / self.ramp_s)
            src = self.stand_from or self.ref_all
            self._send(
                {
                    mid: int(round(src[mid] * (1 - a) + self.ref_all[mid] * a))
                    for mid in self.ref_all
                },
                profile_vel=30,
            )
            if a >= 1.0:
                self._to(HOLD_STAND)
        elif self.state == HOLD_STAND:
            self._send(self.ref_all, profile_vel=30)
            if self.go and fresh and self._gravity() is not None:
                self._to(POLICY)
        elif self.state == POLICY:
            fault = self._step_policy(fresh)
            if fault:
                self.get_logger().warn(
                    f"保護觸發（{fault}）→ 退回站姿\n"
                    f"    /joint_states 現在 {age_pos*1000:.0f} ms / 本次最大 "
                    f"{self.max_age_pos*1000:.0f} ms\n"
                    f"    IMU           現在 {age_imu*1000:.0f} ms / 本次最大 "
                    f"{self.max_age_imu*1000:.0f} ms\n"
                    f"    （門檻 {self.stale_s*1000:.0f} ms）"
                )
                self.go = False
                self._to(HOLD_STAND)

        m = self.imu
        gy = (
            float(np.linalg.norm([m.gyr_x, m.gyr_y, m.gyr_z])) if m is not None else float("nan")
        )
        s = String()
        s.data = (
            f"{self.state} en={int(self.enable)} go={int(self.go)} fresh={int(fresh)} "
            f"tilt_quat={self._tilt_deg():5.1f} tilt_acc={self._tilt_acc():5.1f} "
            f"|gyr|={gy:6.1f}d/s "
            f"|a|max={float(np.abs(self.prev_action).max()):.2f} "
            f"clamp={self.n_clamp} "
            f"age={age_pos*1000:.0f}/{age_imu*1000:.0f}ms "
            f"max={self.max_age_pos*1000:.0f}/{self.max_age_imu*1000:.0f}ms "
            f"cmd=({self.cmd[0]:+.2f},{self.cmd[1]:+.2f},{self.cmd[2]:+.2f})"
        )
        self.status_pub.publish(s)

    def _step_policy(self, fresh: bool) -> str | None:
        if not fresh:
            which = []
            if time.monotonic() - self.t_pos >= self.stale_s:
                which.append("/joint_states")
            if time.monotonic() - self.t_imu >= self.stale_s:
                which.append("IMU")
            return "資料過期：" + "+".join(which)
        tilt = self._tilt_deg()
        if not (tilt == tilt) or tilt > self.max_tilt:
            return f"傾角 {tilt:.1f}° > {self.max_tilt:.0f}°"
        grav = self._gravity()
        if grav is None:
            return "拿不到重力"
        m = self.imu
        assert m is not None

        # ── 組 47 維觀測 ──────────────────────────────────────────────
        ang = self.R @ (np.array([m.gyr_x, m.gyr_y, m.gyr_z]) * DEG2RAD)
        qp, qv = [], []
        for mid in self.order_ids:
            if mid not in self.pos:
                return f"缺 ID {mid} 的位置"
            q = jm.tick_to_q(mid, self.pos[mid])
            q_ref = jm.tick_to_q(mid, self.ref_leg[mid])
            qp.append(q - q_ref)
            if self.vel_src == "register":
                qv.append(jm.vel_raw_to_q_dot(mid, self.vel_raw.get(mid, 0.0)))
            else:
                qv.append(jm.DIR[mid] * self.vel_diff.get(mid, 0.0) / jm.TPR)
        phase = ((time.monotonic() - self.phase_t0) % GAIT_PERIOD) / GAIT_PERIOD
        obs = np.concatenate([
            ang, grav, self.cmd, np.array(qp), np.array(qv), self.prev_action,
            [math.sin(2 * math.pi * phase), math.cos(2 * math.pi * phase)],
        ]).astype(np.float32).reshape(1, -1)
        if obs.shape[1] != self.obs_dim:
            return f"觀測維度 {obs.shape[1]} ≠ {self.obs_dim}"

        # ── 推論 ──────────────────────────────────────────────────────
        act, self.h, self.c = self.sess.run(
            ["actions", "h_out", "c_out"],
            {"obs": obs, "h_in": self.h, "c_in": self.c},
        )
        # ⚠️ **餵回觀測的必須是原始值，不是裁切過的。**
        #    訓練時 clip_actions=None，`actions` 那 12 維觀測吃的是 policy 的
        #    原始輸出。這個 policy 的輸出常態就到 |a|≈3~6.5（ONNX 離線探測：
        #    乾淨的站姿觀測也給 3.9，連續 60 步平均 3.78、最大 6.46），
        #    所以裁切到 3.0 再餵回去會讓 12/47 維觀測跟訓練不一致。
        #    裁切只作用在實際下給馬達的值。
        raw = act[0].astype(np.float32)
        a = np.clip(raw, -self.a_clip, self.a_clip)

        # ── 逐項對帳（log_obs_hz > 0 才開）───────────────────────────
        if self.log_obs_dt and time.monotonic() - self.t_log >= self.log_obs_dt:
            self.t_log = time.monotonic()
            names = [jm.ID_TO_SIM[i] for i in self.order_ids]
            self.get_logger().info(
                "\n"
                f"  base_ang_vel      {np.round(ang, 3)}  rad/s  |·|={np.linalg.norm(ang):.3f}\n"
                f"  projected_gravity {np.round(grav, 3)}  傾角 {tilt:.1f}°\n"
                f"  velocity_commands {np.round(self.cmd, 3)}\n"
                f"  gait_phase        φ={phase:.3f}  "
                f"(sin {math.sin(2*math.pi*phase):+.3f}, cos {math.cos(2*math.pi*phase):+.3f})\n"
                + "".join(
                    f"  {i:>2} {n:<21} q-q_ref {qp[i]:+7.4f}  q̇ {qv[i]:+8.4f}  "
                    f"a {float(act[0][i]):+7.3f}\n"
                    for i, n in enumerate(names)
                )
                + f"  |q-q_ref|max {np.abs(qp).max():.4f} rad  "
                f"|q̇|max {np.abs(qv).max():.4f} rad/s  "
                f"|a|max {np.abs(act[0]).max():.3f}"
            )
        self.prev_action = raw   # 原始值，見上

        ramp = (
            1.0
            if self.a_ramp_s <= 0
            else min(1.0, (time.monotonic() - self.state_t) / self.a_ramp_s)
        )

        # ── 動作 → 刻度 ───────────────────────────────────────────────
        ticks, clamped = dict(self.ref_all), []
        for i, mid in enumerate(self.order_ids):
            q = jm.tick_to_q(mid, self.ref_leg[mid]) + self.a_scale * ramp * float(a[i])
            t, was = jm.clamp_tick(mid, jm.q_to_tick(mid, q), self.ref_leg[mid])
            ticks[mid] = t
            if was:
                clamped.append(mid)
        if clamped:
            self.n_clamp += 1
            self.get_logger().warn(
                f"刻度被夾限：ID {clamped}（累計 {self.n_clamp} 次）",
                throttle_duration_sec=2.0,
            )
        self._send(ticks, profile_vel=0)  # 0 = 不限速，policy 自己決定軌跡
        return None


def main() -> None:
    rclpy.init()
    try:
        node = PolicyNode()
    except SystemExit as e:
        print(e)
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._torque(False)
        rclpy.shutdown()
