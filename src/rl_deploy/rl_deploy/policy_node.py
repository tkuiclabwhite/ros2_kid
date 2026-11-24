"""跑 Isaac Lab 訓練出來的盲走 policy（ONNX）於真機。

    ros2 launch rl_deploy policy.launch.py
    ros2 topic pub --once /rl_deploy/enable std_msgs/Bool '{data: true}'   # → 站姿
    ros2 topic pub --once /rl_deploy/go     std_msgs/Bool '{data: true}'   # → 跑 policy
    ros2 topic pub /rl_deploy/cmd_vel geometry_msgs/Twist \\
        '{linear: {x: 0.2}}'                                              # 下指令
    ros2 topic pub --once /rl_deploy/enable std_msgs/Bool '{data: false}'  # 退回站姿

── 狀態機（照 unitree_rl_gym deploy_real.py 的分段）────────────────────

    IDLE        扭力關、不發指令。等 /rl_deploy/enable
    TO_STAND    3 秒插值到**站立姿態**（全身 21 顆）
    HOLD_STAND  夾住站立姿態。等 /rl_deploy/go
    TO_POLICY   3 秒插值到**policy 基準姿態**（兩者不同時才有，見下）
    HOLD_POLICY 夾住 policy 基準姿態，歸零 IMU，靜置 settle_s 後定出重力參考
    POLICY      50 Hz 推論。任一保護觸發就退回 HOLD_STAND

退回 HOLD_STAND 的條件：
    · /joint_states 或 IMU 超過 stale_timeout_ms 沒更新
    · /joint_states 的**內容**凍結超過 frozen_cycles 個週期
    · 傾角超過 max_tilt_deg
    · /rl_deploy/enable 變 false（會一路退到 IDLE 並關扭力）

── ⚠️ 站姿是兩個基準，不是一個 ────────────────────────────────────────

2026-11-06 拆開。在此之前 `pose_reference` 一個參數同時決定「站起來的姿態」
與「policy 偏差空間的原點」，而兩份都成立的實測互相矛盾：

    calib --stage pose（真機）  stand.ini 腳底平貼 ✅
                               模擬 STAND_JOINT_POS 腳底不平、後傾 7.63° ❌
    MuJoCo sim2sim             stand_ini 當原點 → 傾角 87.5°，倒 ❌
                               必須用模擬 STAND_JOINT_POS（差最多 12.8°）

兩者要求的是不同的東西，所以不二選一：

    stand_pose_reference = stand_ini   站起來要腳底平貼
    pose_reference       = sim         偏差原點要跟訓練一致

落差由 TO_POLICY 走過去（12.8° 瞬間跳過去會踢）。真機的機械零點誤差最終
要用一次性校正表吸收，校正表做好之後兩者合而為一，TO_POLICY 自動跳過。

── 觀測（48 維，順序照 KIDDeployObservationsCfg 的宣告）────────────────

    base_ang_vel       3   gyr(deg/s)→rad/s，套 imu_rotation
    projected_gravity  3   **從 QUAT 算**，不是加速度計（理由見 config）
    velocity_commands  3   /rl_deploy/cmd_vel，**夾進訓練範圍**
    joint_pos         12   (q_real − q_ref)，policy 順序
    joint_vel         12   Present Velocity → rad/s，policy 順序
    actions           12   上一步的原始輸出（未縮放、未裁切）
    gait_phase         2   sin/cos(2π·φ)，站立模式時 φ 快照到 0.025
    standing_mode      1   指令在死區內 = 1.0

⚠️ 觀測與動作的運算**全部在 `deploy_core.DeployCore`**，不在這支節點裡。
   2026-11-06 之前這裡自己 inline 組 47 維觀測，於是 `standing_mode` 與
   相位快照兩項只有 MuJoCo sim2sim 那條路有，真機這條沒有 —— 同一份模型
   在兩條路上收到不同的輸入，而模擬裡測不出來。這支節點現在只負責 ROS
   介面、狀態機與保護。

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
from .deploy_core import CMD_LIMITS, DeployCfg, DeployCore
from .dxl_bus import DxlBus

IDLE, TO_STAND, HOLD_STAND = "IDLE", "TO_STAND", "HOLD_STAND"
TO_POLICY, HOLD_POLICY, POLICY = "TO_POLICY", "HOLD_POLICY", "POLICY"

# config 裡的佔位值 —— 看到這個就拒絕啟動
PLACEHOLDER_ORDER = ["FILL_ME"]


class PolicyNode(Node):
    def __init__(self) -> None:
        super().__init__("rl_policy")

        # ── 參數 ──────────────────────────────────────────────────────
        p = self.declare_parameters(
            "",
            [
                # ── ⚠️ I/O 後端 —— 這是 sim2real 的阻斷項所在 ──────────
                #
                # "direct" 讀→推論→寫在**同一個 tick**，由這支節點直接持有
                #          馬達（宇樹 deploy_real.py 的做法）。
                #          ⚠️ 不要同時起 driver_patched，序列埠不能共用。
                # "topic"  舊的雙迴圈架構，留著對照與除錯用。
                #          ⚠️ **已知會倒** —— driver 先寫後讀且寫的是上一則
                #             /joint_commands，動作延遲至少 20 ms。MuJoCo
                #             sim2sim 實測：其他缺陷全開都活，加上這 20 ms
                #             就 3 秒倒（0.2 m/s，傾角 96°）。
                ("io_backend", "direct"),
                ("dxl_ports", ["/dev/U2D2_P1", "/dev/U2D2_P2", "/dev/U2D2_P3"]),
                ("dxl_baud", 1000000),
                ("model", "kid_7498.onnx"),
                ("obs_dim", 48),
                ("control_hz", 50.0),
                ("action_scale", jm.ACTION_SCALE),
                ("action_clip", jm.CLIP_ACTION),
                ("policy_joint_order", PLACEHOLDER_ORDER),
                ("imu_rotation", [1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0]),
                ("gravity_source", "quat"),
                ("gravity_rereference", True),
                # 再參考的目標。模擬裡 PD 夾住 STAND_JOINT_POS 靜止時的
                # projected_gravity（stand_test 2026-10-21 實測）。
                # ⚠️ 不是 (0,0,−1)，也不是從 STAND_ROT 算的 (+0.021,0,−1)
                #    —— 後者符號相反，差 3.1°。理由見 deploy_core。
                ("gravity_reference", [-0.033, 0.0, -0.999]),
                ("joint_vel_source", "register"),
                # ⚠️ 兩個站姿基準，不是一個。理由見檔頭。
                ("pose_reference", "sim"),            # policy 偏差原點
                ("stand_pose_reference", "stand_ini"),  # 實際站起來的姿態
                # ⚠️ 2026-11-06 接起來。這個 key 本來就在 rl_deploy.yaml 裡，
                #    但節點從來沒宣告它，所以 ROS 安靜地忽略，read_stand_ini
                #    一直用模組裡寫死的路徑 —— 改 yaml 不會有任何效果。
                ("stand_ini_path", jm.STAND_INI),
                ("zero_imu_at_stand", True),
                # 把 /rl_deploy/cmd_vel 夾進訓練範圍。關掉只該用於診斷。
                ("clamp_cmd", True),
                ("max_tilt_deg", 30.0),
                # 100 ms 太緊：馬達出力時 RS485 偶發掉包，SDK 的 LATENCY_TIMER
                # 修補後單次停頓約 20 ms（修補前 110 ms）。200 ms = 10 個控制
                # 週期，仍是有效的保護界線（期間馬達維持上一個指令不動）。
                ("stale_timeout_ms", 200.0),
                # 超過這個就記一筆（不中止），用來看停頓的分布
                ("stale_warn_ms", 50.0),
                # ⚠️ **內容凍結偵測**（時間戳檢查抓不到的失效模式）
                #
                # 2026-10-06 實測：policy 跑約 6 秒後，/joint_states 的**內容**
                # 完全凍結（三幀相隔 1 秒，24 個值到小數第四位全部相同），
                # 但 topic 還在以 50 Hz 發 —— 所以時間戳是新鮮的，保護沒觸發。
                # 同期 base_ang_vel 從 1.17 掉到 0.15 = 機身幾乎停住，
                # 代表寫也進不去，整條匯流排失效。
                #
                # 原因在 driver：它發布的是 joint_data[i]['present']，而那個值
                # **只在 SyncRead 成功時更新**。讀失敗時它保留舊值並照樣發布。
                # 所以「topic 活著」不代表「資料是新的」。
                #
                # policy 在凍結的觀測上繼續推論 = 完全盲飛，比停下來危險。
                ("frozen_cycles", 10),
                ("stand_ramp_s", 3.0),
                # HOLD_POLICY 的靜置時間。IMU 歸零是非同步的（發 /sensorset
                # 給 driver），而重力參考必須在**靜止**時用加速度計定出來
                # （靜止時加速度計量的是比力 = −g，運動中不成立）。
                ("settle_s", 1.0),
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
        self.grav_src = str(g["gravity_source"])
        self.vel_src = str(g["joint_vel_source"])
        self.zero_imu = bool(g["zero_imu_at_stand"])
        self.max_tilt = float(g["max_tilt_deg"])
        self.stale_s = float(g["stale_timeout_ms"]) / 1000.0
        self.warn_s = float(g["stale_warn_ms"]) / 1000.0
        self.frozen_max = int(g["frozen_cycles"])
        self.ramp_s = float(g["stand_ramp_s"])
        self.settle_s = float(g["settle_s"])
        self.a_ramp_s = float(g["action_ramp_s"])
        self.log_obs_dt = (
            0.0 if float(g["log_obs_hz"]) <= 0 else 1.0 / float(g["log_obs_hz"])
        )
        self.t_log = 0.0
        self.R = np.asarray(g["imu_rotation"], dtype=np.float64).reshape(3, 3)
        self.backend = str(g["io_backend"])
        if self.backend not in ("direct", "topic"):
            raise SystemExit(f"❌ io_backend 只能是 direct 或 topic，收到 {self.backend}")

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

        # ── 核心運算（與 MuJoCo sim2sim 共用同一份）──────────────────
        self.core = DeployCore(DeployCfg(
            obs_dim=self.obs_dim,
            action_scale=float(g["action_scale"]),
            action_clip=float(g["action_clip"]),
            joint_order=order,
            imu_rotation=tuple(g["imu_rotation"]),
            gravity_rereference=bool(g["gravity_rereference"]),
            gravity_reference=tuple(g["gravity_reference"]),
            clamp_cmd=bool(g["clamp_cmd"]),
            pose_reference=str(g["pose_reference"]),
            stand_pose_reference=str(g["stand_pose_reference"]),
            stand_ini_path=str(g["stand_ini_path"]),
            # 真機永遠要夾踝 roll 的機構行程（sim2sim 才會關掉來做 A/B）
            clamp_ticks=True,
        ))
        self.order_ids = self.core.ids
        self.ref_all = self.core.stand_ref_all      # 站起來的姿態
        self.policy_ref_all = self.core.ref_all     # policy 的偏差原點
        self.get_logger().info(f"policy 關節順序 → 馬達 ID {self.order_ids}")

        # 兩個姿態的落差。0 的話 TO_POLICY 跳過。
        self.pose_jump = {
            mid: abs(
                jm.tick_to_q(mid, self.policy_ref_all[mid])
                - jm.tick_to_q(mid, self.ref_all[mid])
            )
            for mid in jm.LEG_IDS
        }
        jump_max = max(self.pose_jump.values())
        self.need_to_policy = jump_max > 1e-4
        self.get_logger().info(
            f"站立姿態 = {g['stand_pose_reference']}，"
            f"policy 偏差原點 = {g['pose_reference']}\n"
            f"    兩者最大落差 {math.degrees(jump_max):.2f}°"
            f"（ID {max(self.pose_jump, key=self.pose_jump.get)}）"
            + (
                f" → 進 policy 前用 {self.ramp_s:.0f} s 走過去"
                if self.need_to_policy
                else " → 相同，TO_POLICY 跳過"
            )
        )
        lim = " ".join(f"[{lo:+.2f},{hi:+.2f}]" for lo, hi in CMD_LIMITS)
        self.get_logger().info(
            f"指令夾限 {'開' if g['clamp_cmd'] else '**關**'}：vx/vy/wz {lim}"
        )

        # ── I/O 後端 ──────────────────────────────────────────────────
        self.bus: DxlBus | None = None
        if self.backend == "direct":
            self.get_logger().info("I/O 後端 = direct（讀→推論→寫同一 tick）")
            self.bus = DxlBus(
                ports=list(g["dxl_ports"]),
                baud=int(g["dxl_baud"]),
                ids=jm.ALL_STAND_IDS,
                log=lambda m: self.get_logger().info(str(m)),
            )
            lost = [m for m in self.order_ids if m not in self.bus.id_port]
            if lost:
                raise SystemExit(
                    f"❌ 這些**腿**關節沒掃到：{lost}\n"
                    "   腿關節缺一顆就不能跑 policy。檢查接線與 /dev/U2D2_P*，\n"
                    "   或先用 `ros2 run rl_deploy dxl --scan` 確認。\n"
                    "   ⚠️ 也要確認沒有別的行程佔著序列埠（driver_patched /\n"
                    "      motor_control driver_node 都會佔）。"
                )
            miss = [m for m in jm.ALL_STAND_IDS if m not in self.bus.id_port]
            if miss:
                self.get_logger().warn(
                    f"非腿關節 {miss} 沒掃到 —— 站姿會少夾這幾顆，可以跑但會影響慣量"
                )
        else:
            self.get_logger().warn(
                "I/O 後端 = topic（舊的雙迴圈架構）。\n"
                "    ⚠️ 這個後端的動作延遲至少 20 ms，sim2sim 實測 0.2 m/s 下\n"
                "       3 秒就倒。只適合不著地的除錯，不要放下來走。"
            )

        # ── ONNX ──────────────────────────────────────────────────────
        mp = str(g["model"])
        if not os.path.isabs(mp):
            mp = os.path.join(get_package_share_directory("rl_deploy"), "models", mp)
        if not os.path.exists(mp):
            raise SystemExit(
                f"❌ 找不到模型 {mp}\n"
                "   從訓練電腦複製 48 維的匯出檔過來（在套件原始碼的 models/ 下，\n"
                "   再 colcon build）：\n"
                "     cp <run>/exported/policy.onnx \\\n"
                "        ~/ros2_kid/src/rl_deploy/models/kid_7498.onnx"
            )
        self.sess = ort.InferenceSession(mp, providers=["CPUExecutionProvider"])
        ins = {i.name: i.shape for i in self.sess.get_inputs()}
        if ins.get("obs", [None, None])[1] != self.obs_dim:
            raise SystemExit(f"❌ 模型的 obs 維度是 {ins.get('obs')}，設定是 {self.obs_dim}")
        nh = int(ins["h_in"][2])
        self.h = np.zeros((1, 1, nh), dtype=np.float32)
        self.c = np.zeros((1, 1, nh), dtype=np.float32)
        self.get_logger().info(f"模型 {os.path.basename(mp)}  輸入 {ins}")

        # ── 狀態 ──────────────────────────────────────────────────────
        self.state = IDLE
        self.enable = False
        self.go = False
        self.cmd = np.zeros(3, dtype=np.float32)
        self.pos: dict[int, float] = {}
        self.vel_raw: dict[int, float] = {}
        self.prev_pos: dict[int, float] = {}
        self.vel_diff: dict[int, float] = {}
        self.imu: SensorPackage | None = None
        self.t_pos = 0.0
        self.t_imu = 0.0
        self.state_t = time.monotonic()
        self.pose_from: dict[int, float] | None = None
        self.phase_t0 = time.monotonic()
        self.n_clamp = 0
        self.n_cmd_clamp = 0
        self.n_write_fail = 0
        self.max_age_pos = 0.0
        self.max_age_imu = 0.0
        self._frozen = 0
        self._last_sig = None
        self.t_loop = 0.0

        # ── 介面 ──────────────────────────────────────────────────────
        # IMU 不管哪個後端都走 topic。只有 20 ms **觀測**延遲，sim2sim 實測
        # 和其他缺陷一起開都撐得過（撐完 19 s，傾角 5.7°）。
        self.create_subscription(SensorPackage, "/package/sensorpackage", self._cb_imu, 20)
        self.create_subscription(Bool, "/rl_deploy/enable", self._cb_enable, 10)
        self.create_subscription(Bool, "/rl_deploy/go", self._cb_go, 10)
        self.create_subscription(Twist, "/rl_deploy/cmd_vel", self._cb_cmd, 10)
        self.zero_pub = self.create_publisher(SensorSet, "/sensorset", 10)
        self.status_pub = self.create_publisher(String, "/rl_deploy/status", 10)
        self.cmd_pub = self.torque_pub = None
        if self.backend == "topic":
            self.create_subscription(JointState, "/joint_states", self._cb_pos, 20)
            self.create_subscription(JointState, "/joint_velocities", self._cb_vel, 20)
            self.cmd_pub = self.create_publisher(JointState, "/joint_commands", 10)
            self.torque_pub = self.create_publisher(Int16MultiArray, "/set_torque", 10)
        else:
            # 直接持有馬達時**不訂閱** /joint_commands 的對應發布端，但照樣
            # 發布讀到的狀態，供記錄與其他節點觀察。
            self.state_pub = self.create_publisher(JointState, "/joint_states", 10)
            self.vel_pub = self.create_publisher(JointState, "/joint_velocities", 10)

        self.timer = self.create_timer(self.dt, self._loop)
        # ⚠️ 扭力在 TO_STAND 才開 —— 沒叫它動就不通電，是刻意的。
        #    所以啟動後機器人不會有任何反應，要先發 enable。
        self.get_logger().info(
            f"就緒（{self.hz:.0f} Hz）。扭力還沒開，機器人現在不會動。\n"
            "    ① 進站姿（會開扭力、3 秒插值到站姿）：\n"
            "       ros2 topic pub --once /rl_deploy/enable std_msgs/Bool '{data: true}'\n"
            "    ② 跑 policy：\n"
            "       ros2 topic pub --once /rl_deploy/go std_msgs/Bool '{data: true}'\n"
            "    ③ 看狀態（age= 會告訴你 馬達/IMU 哪個沒來）：\n"
            "       ros2 topic echo /rl_deploy/status"
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
        """⚠️ 夾進訓練範圍。/rl_deploy/cmd_vel 是任何人都能 publish 的 topic，
        而 policy 沒見過範圍外的指令。極速天花板實測 ~0.31 m/s，給更大的
        指令不會更快（已實測確認收窄範圍也不會提高天花板）。"""
        raw = np.array(
            [msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32
        )
        self.cmd, was = self.core.clamp_cmd(raw)
        if was:
            self.n_cmd_clamp += 1
            self.get_logger().warn(
                f"指令超出訓練範圍，已夾："
                f"({raw[0]:+.2f},{raw[1]:+.2f},{raw[2]:+.2f}) → "
                f"({self.cmd[0]:+.2f},{self.cmd[1]:+.2f},{self.cmd[2]:+.2f})"
                f"（累計 {self.n_cmd_clamp} 次）",
                throttle_duration_sec=2.0,
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
            self.pose_from = {
                mid: self.pos.get(mid, self.ref_all[mid]) for mid in self.ref_all
            }
        elif state == HOLD_STAND:
            self.core.reset()          # prev_action / g_ref / 延遲佇列
        elif state == TO_POLICY:
            self.pose_from = {
                mid: self.pos.get(mid, self.ref_all[mid]) for mid in self.ref_all
            }
        elif state == HOLD_POLICY:
            # QUAT 是相對歸零姿態的，所以在**policy 基準姿態**歸零 ——
            # 之後的姿態偏差才跟訓練的偏差空間同一個原點。
            if self.zero_imu:
                m = SensorSet()
                m.reset = True
                self.zero_pub.publish(m)
                self.get_logger().info("已要求 IMU 歸零（policy 基準姿態 = 參考姿態）")
            self.core.g_ref = None
        elif state == POLICY:
            self.phase_t0 = time.monotonic()
            self.h[:] = 0.0
            self.c[:] = 0.0  # 重新進 policy 前清掉 LSTM 記憶
            self.core.prev_action[:] = 0.0
            self.n_clamp = 0
            self.max_age_pos = 0.0
            self.max_age_imu = 0.0
            self._frozen = 0
            self._last_sig = None

    def _torque(self, on: bool) -> None:
        if self.bus is not None:
            ok, fail = self.bus.set_torque(on)
            self.get_logger().info(
                f"扭力 {'開' if on else '關'}：{len(ok)}/{len(self.bus.ids)} 顆"
            )
            if fail:
                self.get_logger().error(
                    "❌ 這幾顆設定失敗："
                    + "、".join(f"ID {m}({r})" for m, r in sorted(fail.items()))
                )
            # 腿關節缺一顆就不能跑 —— 其他關節失敗只影響站姿夾持。
            lost = [m for m in self.order_ids if m in fail]
            if on and lost:
                self.get_logger().error(
                    f"❌❌ **腿**關節 {lost} 的扭力沒開成功。不要發 go，"
                    "先查這幾顆的接線與狀態。"
                )
            return
        m = Int16MultiArray()
        m.data = [0, 1 if on else 0]
        self.torque_pub.publish(m)

    def _send(self, ticks: dict[int, int], profile_vel: int = 0) -> None:
        """下目標刻度。direct 後端**在同一個 tick 內**寫出去。"""
        if self.bus is not None:
            bad = self.bus.write(ticks, profile_vel)
            if bad:
                self.n_write_fail += 1
                self.get_logger().warn(
                    f"寫入失敗的埠：{bad}（累計 {self.n_write_fail} 次）。"
                    "馬達維持上一個指令；持續失敗會由讀取的過期保護攔下。",
                    throttle_duration_sec=1.0,
                )
            return
        msg = JointState()
        msg.name = [str(k) for k in ticks]
        msg.position = [float(v) for v in ticks.values()]
        msg.velocity = [float(profile_vel)] * len(ticks)
        self.cmd_pub.publish(msg)

    def _bus_read(self, now: float) -> None:
        """direct 後端：在迴圈開頭同步讀一次，並發布出來供記錄。

        ⚠️ **讀到的值合併進 self.pos（保留舊值），但 t_pos 只在 12 個腿關節
           這次全部讀到時才前進。** 這樣單次掉包不會立刻中止 policy（它只是
           用 20 ms 舊的資料，sim2sim 證明撐得過），而持續失效會在
           stale_timeout_ms 之後被過期保護攔下。
        """
        assert self.bus is not None
        pos, vel, bad = self.bus.read()
        if bad:
            self.get_logger().warn(
                f"讀取失敗的埠：{bad}", throttle_duration_sec=1.0
            )
        if not pos:
            return
        if self.prev_pos and self.t_pos:
            dt = now - self.t_pos
            if dt > 1e-4:
                self.vel_diff = {
                    k: (pos[k] - self.prev_pos[k]) / dt
                    for k in pos
                    if k in self.prev_pos
                }
        self.prev_pos = dict(pos)
        self.pos.update(pos)
        self.vel_raw.update(vel)
        if all(mid in pos for mid in self.order_ids):
            self.t_pos = now

        stamp = self.get_clock().now().to_msg()
        for pub, data in ((self.state_pub, pos), (self.vel_pub, vel)):
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = [str(k) for k in sorted(data)]
            msg.position = [float(data[k]) for k in sorted(data)]
            pub.publish(msg)

    def _ramp_to(self, target: dict[int, int]) -> float:
        """朝 target 插值並發出去，回傳進度 [0,1]。"""
        a = min(1.0, (time.monotonic() - self.state_t) / self.ramp_s)
        src = self.pose_from or target
        self._send(
            {
                mid: int(round(src[mid] * (1 - a) + target[mid] * a))
                for mid in target
            },
            profile_vel=30,
        )
        return a

    def _set_gravity_ref(self) -> bool:
        """靜止站姿時用加速度計定出重力參考。成功回 True。"""
        m = self.imu
        if m is None:
            return False
        try:
            g0 = self.core.set_gravity_reference([m.acc_x, m.acc_y, m.acc_z])
        except ValueError:
            return False
        gr = self.core.g_ref
        self.get_logger().info(
            f"重力參考：站姿實測 ({g0[0]:+.3f}, {g0[1]:+.3f}, {g0[2]:+.3f})"
            f" → 餵給 policy 的參考 ({gr[0]:+.3f}, {gr[1]:+.3f}, {gr[2]:+.3f})"
        )
        return True

    def _gravity(self) -> np.ndarray | None:
        """base_link 座標的 projected_gravity（單位向量）。"""
        m = self.imu
        if m is None:
            return None
        if self.grav_src == "accel":
            a = np.array([m.acc_x, m.acc_y, m.acc_z])
            n = np.linalg.norm(a)
            return None if n < 1e-6 else -(self.R @ (a / n))
        if self.core.g_ref is None:
            return None
        q = (m.quat_w, m.quat_x, m.quat_y, m.quat_z)
        if np.linalg.norm(q) < 1e-6:
            return None
        return self.core.gravity(q)

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
        """保護用的傾角 —— **從 QUAT 算**，相對 policy 基準姿態。

        ⚠️ 2026-10-06 修正。原本用加速度計，結果一進 POLICY 就跳 55°／59°
           而觸發保護。加速度計量的是比力 (a − g)：吊著的機器人一踢腿就有
           大的水平加速度，55° 只需要 tan(55°)·g = 1.43 g 的水平分量。
           DMP 的 QUAT 是姿態估計，不受線加速度影響。

        相對基準姿態而不是絕對鉛垂，是刻意的：判準變成「我偏離 policy 的
        基準姿態多少」，基準本身就是 0°，門檻的意義更直接。絕對值仍可用
        _tilt_acc() 交叉比對（status 兩個都發）。
        """
        g = self._gravity()
        if g is None:
            return float("nan")
        return math.degrees(math.acos(float(np.clip(-g[2], -1.0, 1.0))))

    # ── 主迴圈 ────────────────────────────────────────────────────────
    def _loop(self) -> None:
        now = time.monotonic()
        # ⚠️ direct 後端的**順序**就是修掉動作延遲的全部關鍵：
        #    讀 → 狀態機／推論 → 寫，全部在這一次回呼裡完成。
        #    原本的 driver 是「寫上一則指令 → 讀 → 發布」，所以 policy 算出的
        #    動作至少要等它下一圈（20 ms）才到馬達。
        if self.bus is not None:
            self._bus_read(now)
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
            # 腿關節全到就可以站起來。非腿關節缺顆只會少夾、不影響 policy
            # （direct 後端啟動時已經警告過哪幾顆沒掃到）。
            if self.enable and fresh and all(m in self.pos for m in self.order_ids):
                self._to(TO_STAND)
        elif self.state == TO_STAND:
            if self._ramp_to(self.ref_all) >= 1.0:
                self._to(HOLD_STAND)
        elif self.state == HOLD_STAND:
            self._send(self.ref_all, profile_vel=30)
            if self.go and fresh:
                self._to(TO_POLICY if self.need_to_policy else HOLD_POLICY)
        elif self.state == TO_POLICY:
            if self._ramp_to(self.policy_ref_all) >= 1.0:
                self._to(HOLD_POLICY)
        elif self.state == HOLD_POLICY:
            self._send(self.policy_ref_all, profile_vel=30)
            if not self.go:
                self._to(HOLD_STAND)
            # 靜置夠久（IMU 歸零生效、機身不動）之後才定重力參考
            elif now - self.state_t >= self.settle_s and fresh:
                if self.core.g_ref is not None or self._set_gravity_ref():
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

        # 迴圈耗時 —— direct 後端必須 < 20 ms，不然寫入會拖到下一個週期而
        # 把動作延遲又帶回來。修補後 3 埠讀寫實測約 11 ms，加推論約 12 ms。
        self.t_loop = time.monotonic() - now
        # ⚠️ 只在 POLICY 才報超時。TO_STAND 進入時要逐顆寫扭力（21 次往返，
        #    實測約 45 ms），那一個 tick 必然超時但完全無害 —— 那時機器人
        #    還沒在跑 policy。在 POLICY 裡超時才是真的會把動作延遲帶回來。
        if self.bus is not None and self.state == POLICY and self.t_loop > self.dt:
            self.get_logger().warn(
                f"迴圈超時 {self.t_loop*1000:.1f} ms > {self.dt*1000:.0f} ms"
                " —— 動作延遲會回來，檢查是不是有別的行程在搶序列埠",
                throttle_duration_sec=2.0,
            )
        m = self.imu
        gy = (
            float(np.linalg.norm([m.gyr_x, m.gyr_y, m.gyr_z])) if m is not None else float("nan")
        )
        # 拆兩行：一行塞不下會被終端機截掉，而 loop= 正好在被截掉的那半。
        # ⚠️ HOLD_STAND 時 tilt_quat 是 nan 是**正常的** —— QUAT 的重力參考
        #    要到 HOLD_POLICY 才定出來（見 _gravity）。那之前只有 tilt_acc
        #    可信，而它量的是絕對傾角：stand.ini 應該在 6.7° 附近。
        s = String()
        s.data = (
            f"{self.state} loop={self.t_loop*1000:5.1f}ms io={self.backend} "
            f"en={int(self.enable)} go={int(self.go)} fresh={int(fresh)} "
            f"age={age_pos*1000:.0f}/{age_imu*1000:.0f}ms "
            f"max={self.max_age_pos*1000:.0f}/{self.max_age_imu*1000:.0f}ms\n"
            f"  tilt_quat={self._tilt_deg():5.1f} tilt_acc={self._tilt_acc():5.1f} "
            f"|gyr|={gy:6.1f}d/s |a|max={float(np.abs(self.core.prev_action).max()):.2f} "
            f"clamp={self.n_clamp} cmd_clamp={self.n_cmd_clamp} "
            f"frozen={self._frozen} wfail={self.n_write_fail} "
            f"cmd=({self.cmd[0]:+.2f},{self.cmd[1]:+.2f},{self.cmd[2]:+.2f}) "
            f"stand={int(self.core.is_standing(self.cmd))}"
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
        # 內容凍結偵測（理由見參數宣告處）
        sig = tuple(self.pos.get(m, float("nan")) for m in self.order_ids)
        if self._last_sig is not None and sig == self._last_sig:
            self._frozen += 1
            if self._frozen >= self.frozen_max:
                return (
                    f"/joint_states 內容凍結 {self._frozen} 個週期"
                    f"（{self._frozen * self.dt * 1000:.0f} ms）—— topic 還在發但資料沒更新，"
                    "檢查 driver 有沒有印 I/O 異常"
                )
        else:
            self._frozen = 0
        self._last_sig = sig

        tilt = self._tilt_deg()
        if not (tilt == tilt) or tilt > self.max_tilt:
            return f"傾角 {tilt:.1f}° > {self.max_tilt:.0f}°"
        missing = [mid for mid in self.order_ids if mid not in self.pos]
        if missing:
            return f"缺 ID {missing} 的位置（/joint_states）"
        if self.vel_src == "register":
            missing = [mid for mid in self.order_ids if mid not in self.vel_raw]
            if missing:
                return f"缺 ID {missing} 的速度（/joint_velocities）"
        m = self.imu
        assert m is not None

        # ── 組觀測（全部交給 DeployCore，與 MuJoCo sim2sim 同一份）────
        #
        # ⚠️ 相位的 t 從**進入 POLICY**起算，而 DeployCore 會在站立模式時
        #    把它快照到 0.025（雙支撐中點）。這兩件事少做任一項，policy
        #    就收到從沒見過的輸入，而模擬裡測不出來。
        kw = {}
        if self.vel_src != "register":
            # 位置微分已經是 rad/s，不走暫存器換算
            kw["vel_rads"] = {
                mid: jm.DIR[mid] * self.vel_diff.get(mid, 0.0) / jm.TPR
                for mid in self.order_ids
            }
        try:
            obs = self.core.build_obs(
                pos_ticks=self.pos,
                vel_raw=self.vel_raw,
                quat=(m.quat_w, m.quat_x, m.quat_y, m.quat_z),
                gyr_dps=(m.gyr_x, m.gyr_y, m.gyr_z),
                cmd=self.cmd,
                t=time.monotonic() - self.phase_t0,
                **kw,
            )
        except (KeyError, ValueError, RuntimeError) as e:
            return f"組觀測失敗：{e}"

        # ── 推論 ──────────────────────────────────────────────────────
        act, self.h, self.c = self.sess.run(
            ["actions", "h_out", "c_out"],
            {"obs": obs.reshape(1, -1), "h_in": self.h, "c_in": self.c},
        )
        raw = act[0].astype(np.float32)

        # ── 逐項對帳（log_obs_hz > 0 才開）───────────────────────────
        #
        # 從**實際餵進去的 obs 向量**切出來，不是重算 —— 這樣連延遲、夾限、
        # 站立快照都一起對到帳。吊起來（腳不著地）先跑這個確認 48 維正確，
        # 再放下來走。
        if self.log_obs_dt and time.monotonic() - self.t_log >= self.log_obs_dt:
            self.t_log = time.monotonic()
            seg, i = {}, 0
            for name, dim in self.core.cfg.terms():
                seg[name] = obs[i:i + dim]
                i += dim
            sin_p, cos_p = seg["gait_phase"]
            phase = (math.atan2(sin_p, cos_p) / (2 * math.pi)) % 1.0
            qp, qv = seg["joint_pos"], seg["joint_vel"]
            names = [jm.ID_TO_SIM[i] for i in self.order_ids]
            self.get_logger().info(
                "\n"
                f"  base_ang_vel      {np.round(seg['base_ang_vel'], 3)}  rad/s  "
                f"|·|={np.linalg.norm(seg['base_ang_vel']):.3f}\n"
                f"  projected_gravity {np.round(seg['projected_gravity'], 3)}  "
                f"傾角 {tilt:.1f}°\n"
                f"  velocity_commands {np.round(seg['velocity_commands'], 3)}\n"
                f"  gait_phase        φ={phase:.3f}  "
                f"(sin {sin_p:+.3f}, cos {cos_p:+.3f})"
                + ("  ← 站立快照" if self.core.is_standing(self.cmd) else "")
                + "\n"
                f"  standing_mode     {float(seg['standing_mode'][0]):.0f}\n"
                + "".join(
                    f"  {i:>2} {n:<21} q-q_ref {qp[i]:+7.4f}  q̇ {qv[i]:+8.4f}  "
                    f"a {float(raw[i]):+7.3f}\n"
                    for i, n in enumerate(names)
                )
                + f"  |q-q_ref|max {np.abs(qp).max():.4f} rad  "
                f"|q̇|max {np.abs(qv).max():.4f} rad/s  "
                f"|a|max {np.abs(raw).max():.3f}  obs {obs.shape[0]} 維"
            )

        ramp = (
            1.0
            if self.a_ramp_s <= 0
            else min(1.0, (time.monotonic() - self.state_t) / self.a_ramp_s)
        )

        # ── 動作 → 刻度 ───────────────────────────────────────────────
        #
        # ⚠️ **餵回觀測的必須是原始值，不是裁切過的。** 訓練時
        #    clip_actions=None，`actions` 那 12 維觀測吃的是 policy 的原始
        #    輸出。這個 policy 的輸出常態就到 |a|≈3~6.5（ONNX 離線探測：
        #    乾淨的站姿觀測也給 3.9，連續 60 步平均 3.78、最大 6.46），
        #    所以裁切到 3.0 再餵回去會讓 12/48 維觀測跟訓練不一致。
        #    裁切只作用在實際下給馬達的值 —— DeployCore.action_to_targets
        #    記的 prev_action 是原始值，這點已經在那邊處理。
        #
        # 非腿關節（手臂／腰／頭）維持 policy 基準姿態，靠自己的 PD 夾住。
        targets, n_clamp = self.core.action_to_targets(raw, ramp=ramp)
        ticks = dict(self.policy_ref_all)
        ticks.update({mid: t for mid, (t, _) in targets.items()})
        if n_clamp:
            self.n_clamp += 1
            self.get_logger().warn(
                f"刻度被夾限：{n_clamp} 顆（累計 {self.n_clamp} 個週期）",
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
        if node.bus is not None:
            node.bus.close()
        rclpy.shutdown()
