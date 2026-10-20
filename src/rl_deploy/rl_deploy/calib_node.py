"""真機部署前的校正與量測工具。**不修改原系統任何檔案。**

    ros2 run rl_deploy calib --stage link    # 純讀取：通訊品質、速度來源比較
    ros2 run rl_deploy calib --stage imu     # 純讀取：自動解出 IMU 安裝座標
    ros2 run rl_deploy calib --stage pose    # 會動！比較 stand.ini 與模擬站姿

跑完每個 stage 會印出一段 yaml，貼進 config/rl_deploy.yaml 即可。

── 為什麼要有這支（而不是直接跑 policy）────────────────────────────────

policy 的 47 維觀測裡有 6 維來自 IMU（`base_ang_vel` 3 + `projected_gravity`
3）。`projected_gravity` 是它唯一的平衡感測 —— **軸排錯或正負號反了，policy
會主動把自己推倒**，不是走得醜而是立刻倒。其他參數錯了頂多走不好。

所以上機順序是：先確認通訊、再確認 IMU、再確認站姿，最後才跑 policy。
前三關都不跑神經網路，壞了也只是站著不動。

── 用到的原系統介面（全部唯讀或既有 topic）────────────────────────────

    /joint_states      JointState       name = 馬達 ID 字串, position = 刻度
    /joint_velocities  JointState       velocity 欄位放的是**原始暫存器值**
    /package/sensorpackage  SensorPackage   quat/acc/gyr（Arduino 已經有輸出）
    /joint_commands    JointState       name = ID 字串, position = 刻度
    /set_torque        Int16MultiArray  [target_id, state]，0 = 全體
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray

from tku_msgs.msg import SensorPackage

from . import joint_map as jm


# ──────────────────────────────────────────────────────────────────────────
# 24 種軸對齊的安裝姿態
# ──────────────────────────────────────────────────────────────────────────


def _axis_rotations() -> list[tuple[str, np.ndarray]]:
    """所有軸對齊的帶正負號排列中，行列式為 +1 的 24 個（才是真正的旋轉）。

    板載 IMU 幾乎一定是其中之一 —— 所以不用量角度，窮舉就好。
    """
    out = []
    names = ["x", "y", "z"]
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            R = np.zeros((3, 3))
            for row, (col, s) in enumerate(zip(perm, signs)):
                R[row, col] = s
            if abs(np.linalg.det(R) - 1.0) < 1e-9:
                label = ", ".join(
                    f"{'+' if signs[r] > 0 else '-'}{names[perm[r]]}" for r in range(3)
                )
                out.append((f"({label})", R))
    return out


AXIS_ROTATIONS = _axis_rotations()


# ──────────────────────────────────────────────────────────────────────────
# 節點
# ──────────────────────────────────────────────────────────────────────────


class CalibNode(Node):
    def __init__(self) -> None:
        super().__init__("rl_calib")

        self._lock = threading.Lock()
        # /joint_states
        self.pos: dict[int, float] = {}
        self.pos_stamp: list[float] = []  # 到達時刻，用來量實際頻率
        # /joint_velocities（原始暫存器值）
        self.vel_raw: dict[int, float] = {}
        self.vel_stamp: list[float] = []
        # 位置微分（在回呼裡算，見 _cb_pos 的註解）
        self._prev_pos: dict[int, float] | None = None
        self._prev_t: float = 0.0
        self.vel_diff: dict[int, float] = {}
        self.diff_samples: list[tuple[dict[int, float], dict[int, float]]] = []
        # IMU
        self.imu: SensorPackage | None = None
        self.imu_stamp: list[float] = []

        self.create_subscription(JointState, "/joint_states", self._cb_pos, 20)
        self.create_subscription(JointState, "/joint_velocities", self._cb_vel, 20)
        self.create_subscription(
            SensorPackage, "/package/sensorpackage", self._cb_imu, 20
        )
        self.cmd_pub = self.create_publisher(JointState, "/joint_commands", 10)
        self.torque_pub = self.create_publisher(Int16MultiArray, "/set_torque", 10)

    # ── 回呼 ──────────────────────────────────────────────────────────
    def _cb_pos(self, msg: JointState) -> None:
        """收到位置就順手算微分速度。

        ⚠️ 2026-10-06 修正。原本在主迴圈用 5 ms 輪詢算微分，但 /joint_states
           只有 36 ms 更新一次 —— 7.27 次輪詢才有一筆新資料，而那一筆的 dt
           用了 5 ms 而不是 36 ms，等於放大 7.27 倍，其餘樣本都是 0。
           混合後的 RMS 會被誇大 √7.27 = 2.70 倍。

           實測的微分/暫存器比值是 2.71~2.81，跟 2.70 的預測吻合 —— 所以那
           是取樣假象，不是硬體問題。**微分一定要在資料真正更新時才算。**
        """
        now = time.monotonic()
        with self._lock:
            self.pos_stamp.append(now)
            new_pos: dict[int, float] = {}
            for n, p in zip(msg.name, msg.position):
                try:
                    new_pos[int(n)] = float(p)
                except ValueError:
                    pass
            if self._prev_pos is not None:
                dt = now - self._prev_t
                if dt > 1e-4:
                    self.vel_diff = {
                        mid: (new_pos[mid] - self._prev_pos[mid]) / dt
                        for mid in new_pos
                        if mid in self._prev_pos
                    }
                    self.diff_samples.append((dict(self.vel_diff), dict(self.vel_raw)))
            self._prev_pos, self._prev_t = dict(new_pos), now
            self.pos.update(new_pos)

    def _cb_vel(self, msg: JointState) -> None:
        with self._lock:
            self.vel_stamp.append(time.monotonic())
            # driver 把原始暫存器值放在 velocity 欄位（driver_node 的發布段：
            # vel_msg.velocity = [float(joint_data[i]['present_vel']) ...]）
            src = msg.position if len(msg.position) else msg.velocity
            for n, v in zip(msg.name, src):
                try:
                    self.vel_raw[int(n)] = float(v)
                except ValueError:
                    pass

    def _cb_imu(self, msg: SensorPackage) -> None:
        with self._lock:
            self.imu_stamp.append(time.monotonic())
            self.imu = msg

    # ── 工具 ──────────────────────────────────────────────────────────
    def snapshot(self):
        with self._lock:
            return dict(self.pos), dict(self.vel_raw), self.imu

    def reset_stamps(self) -> None:
        with self._lock:
            self.pos_stamp.clear()
            self.vel_stamp.clear()
            self.imu_stamp.clear()
            self.diff_samples.clear()

    def wait_for_data(self, need_imu: bool = True, timeout: float = 10.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            p, _, imu = self.snapshot()
            if len(p) >= len(jm.LEG_IDS) and (imu is not None or not need_imu):
                return True
            time.sleep(0.1)
        return False

    def send_ticks(self, ticks: dict[int, int], profile_vel: int = 0) -> None:
        """發 /joint_commands。profile_vel 0 = 不限速。"""
        msg = JointState()
        msg.name = [str(mid) for mid in ticks]
        msg.position = [float(t) for t in ticks.values()]
        msg.velocity = [float(profile_vel)] * len(ticks)
        self.cmd_pub.publish(msg)

    def set_torque(self, on: bool) -> None:
        m = Int16MultiArray()
        m.data = [0, 1 if on else 0]  # 0 = 全體
        self.torque_pub.publish(m)


# ──────────────────────────────────────────────────────────────────────────
# stage: link
# ──────────────────────────────────────────────────────────────────────────


def _rate_stats(stamps: list[float], label: str) -> float:
    if len(stamps) < 3:
        print(f"  {label:<22} ❌ 只收到 {len(stamps)} 筆")
        return 0.0
    d = np.diff(np.asarray(stamps)) * 1000.0
    hz = 1000.0 / d.mean()
    print(
        f"  {label:<22} {hz:6.1f} Hz   間隔 平均 {d.mean():6.2f} / "
        f"中位 {np.median(d):6.2f} / 最大 {d.max():7.2f} ms   抖動σ {d.std():5.2f}"
    )
    return hz


def stage_link(node: CalibNode, seconds: float) -> None:
    print("\n" + "=" * 74)
    print("stage link — 通訊品質與速度來源比較（純讀取，馬達不會動）")
    print("=" * 74)
    if not node.wait_for_data(need_imu=False):
        print("❌ 收不到 /joint_states。確認 driver_node 有在跑。")
        return

    print(
        f"\n量測 {seconds:.0f} 秒 —— **期間請用手把腿前後搖晃**。\n"
        "（馬達靜止時 Present Velocity 會量化成 0，不晃就比不出速度來源好壞。）"
    )
    # 微分在 _cb_pos 裡做（見那邊的註解），這裡只要等資料累積
    node.reset_stamps()
    time.sleep(seconds)
    with node._lock:
        samples = [(0.0, d, r) for d, r in node.diff_samples]

    print("\n① 實際更新頻率（50 Hz 的預算是 20.0 ms）")
    hz_pos = _rate_stats(node.pos_stamp, "/joint_states")
    _rate_stats(node.vel_stamp, "/joint_velocities")
    _rate_stats(node.imu_stamp, "/package/sensorpackage")

    print("\n   判準：/joint_states < 45 Hz → 先做這兩件事（都不用改程式）")
    print("     · 3 個 U2D2 都設 FTDI latency_timer = 1")
    print("       echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB*/latency_timer")
    print("     · 23 顆馬達的 Return Delay Time (addr 9) 設 0（預設 250 = 0.5 ms/顆）")
    print("   這兩項的影響是 10~50 倍，遠大於「要不要讀速度」的 0.92 ms。")

    print("\n② 速度來源比較（暫存器 vs 位置微分，單位都換成 rad/s）")
    if not samples:
        print("   ❌ 沒收集到樣本")
    else:
        got_reg = any(len(s[2]) for s in samples)
        print(f"   {'ID':>3} {'關節':<21}{'微分 RMS':>10}{'暫存器 RMS':>12}{'相關':>8}")
        for mid in jm.LEG_IDS:
            d = np.array([s[1].get(mid, np.nan) for s in samples], dtype=float)
            d = d[~np.isnan(d)] / jm.TPR * jm.DIR[mid]  # ticks/s → rad/s（含方向）
            if got_reg:
                r = np.array(
                    [jm.vel_raw_to_q_dot(mid, s[2].get(mid, np.nan)) for s in samples],
                    dtype=float,
                )
                n = min(len(d), len(r))
                corr = (
                    float(np.corrcoef(d[:n], r[:n])[0, 1])
                    if n > 5 and d[:n].std() > 1e-9 and r[:n].std() > 1e-9
                    else float("nan")
                )
                print(
                    f"   {mid:>3} {jm.ID_TO_SIM[mid]:<21}{np.sqrt((d**2).mean()):>10.4f}"
                    f"{np.sqrt(np.nanmean(r**2)):>12.4f}{corr:>8.3f}"
                )
            else:
                print(
                    f"   {mid:>3} {jm.ID_TO_SIM[mid]:<21}{np.sqrt((d**2).mean()):>10.4f}"
                    f"{'(未開)':>12}{'—':>8}"
                )
        if got_reg and all(
            abs(s[2].get(mid, 0.0)) < 1e-9 for s in samples for mid in jm.LEG_IDS
        ):
            # ⚠️ 全是 0 **通常不是故障** —— Present Velocity 的單位是
            #    0.229 rpm = 0.0240 rad/s，低於這個就量化成 0。機器人靜止不動
            #    時暫存器本來就回 0，而位置微分的那 0.01~0.07 rad/s 只是
            #    編碼器抖動，剛好落在同一個門檻附近。
            #    所以這一段要**用手搖晃腿部**才有意義。
            print("\n   ℹ️ /joint_velocities 全是 0 —— 最可能是**馬達沒在動**。")
            print(f"      Present Velocity 的量化階是 {jm.VEL_RAW_TO_RADS:.4f} rad/s，")
            print("      靜止時回 0 是正常的，不代表 read_velocity 沒生效。")
            print("      先確認： ros2 param get /dynamixel_driver read_velocity")
            print("      是 True 的話 → **重跑這個 stage，期間用手把腿前後搖晃**，")
            print("      才比較得出暫存器與位置微分哪個可用。")
        if not got_reg:
            print("\n   /joint_velocities 沒有資料 → driver 的 read_velocity 是關的。")
            print("   要比較的話重開 driver 並加參數：")
            print("     ros2 run motor_control driver_node --ros-args -p read_velocity:=true")
            print("   或執行中切換： ros2 topic pub --once /set_motor_read "
                  "std_msgs/Int16MultiArray '{data: [0, 1]}'")

    # ── ③ IMU 內容新鮮度 ────────────────────────────────────────────
    #
    # `/package/sensorpackage` 的**發布率**（switch.py 的 pub_hz）跟 IMU 的
    # **實際更新率**（Arduino 的 dmpBegin FIFO 頻率）是兩件事。pub_hz 設 50
    # 但 DMP 還是 20 Hz 的話，會有 2~3 次連續發布帶著一模一樣的資料 ——
    # policy 看到的就是過期 2 步的姿態。數「有幾次內容真的變了」才看得出來。
    print("\n③ IMU 內容新鮮度（發布率 ≠ 實際更新率）")
    imus = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0:
        _, _, m = node.snapshot()
        if m is not None:
            imus.append(
                (m.quat_w, m.quat_x, m.quat_y, m.quat_z,
                 m.acc_x, m.acc_y, m.acc_z, m.gyr_x, m.gyr_y, m.gyr_z,
                 m.roll, m.pitch, m.yaw)
            )
        time.sleep(0.004)
    if len(imus) < 20:
        print("   ❌ IMU 樣本不足")
    else:
        A = np.asarray(imus)
        allzero = {
            "quat": not np.any(A[:, 0:4]),
            "acc": not np.any(A[:, 4:7]),
            "gyr": not np.any(A[:, 7:10]),
            "ypr": not np.any(A[:, 10:13]),
        }
        changed = (np.abs(np.diff(A, axis=0)) > 1e-9).any(axis=1)
        fresh_hz = changed.sum() / 3.0
        print(f"   取樣 {len(imus)} 筆 / 3 秒；內容真正改變 {changed.sum()} 次"
              f" → **實際更新 ≈ {fresh_hz:.1f} Hz**")
        for k, z in allzero.items():
            print(f"   {k:<5} {'❌ 全是 0' if z else '✅ 有資料'}")
        if allzero["acc"] or allzero["gyr"] or allzero["quat"]:
            print(
                "\n   ❌ acc/gyr/quat 沒有資料 → **policy 跑不起來**（它要這 6 維）。\n"
                "      switch.py 本身是對的（on_timer 有填這些欄位），所以是上游：\n"
                "      **Arduino 燒的是舊版 sketch，只送 #YPR= 沒送 #ACC=/#GYR=/#QUAT=。**\n"
                "\n      3 秒驗證（先 Ctrl+C 掉 switch 節點，它佔著 /dev/ttyACM0）：\n"
                "        python3 - <<'EOF'\n"
                "        import serial, time\n"
                "        s = serial.Serial('/dev/ttyACM0', 115200, timeout=1)\n"
                "        t = time.time()\n"
                "        while time.time() - t < 3:\n"
                "            l = s.readline().decode(errors='ignore').strip()\n"
                "            if l: print(l)\n"
                "        EOF\n"
                "\n      只看到 #YPR= → 燒 ~/ros2_kid/Arduino/IMU.ino，\n"
                "      順便把 dmpBegin(..., 20) 的 20 改成 50（一次解決兩件事）。"
            )
        elif fresh_hz < 40:
            print(
                f"\n   ⚠️ 實際只更新 {fresh_hz:.1f} Hz，但 policy 要 50 Hz。\n"
                "      改 Arduino/IMU.ino 的 dmpBegin(..., 20) → 50。\n"
                "      頻寬檢查：6 行約 156 bytes/sample，50 Hz = 7800 B/s，\n"
                "      115200 baud 可用 11520 B/s → 68% 佔用，可行。"
            )

    print("\n   判準：")
    print("     · 相關係數 > 0.9 → 兩者等價，用哪個都行，建議用暫存器（已濾波）")
    print("     · 相關係數低或暫存器 RMS 異常大 → 用位置微分")
    print(f"     · 位置微分的量化雜訊是 {1/jm.TPR/0.02:.4f} rad/s，"
          f"訓練時的觀測雜訊是 ±0.3 rad/s（佔 {1/jm.TPR/0.02/0.3*100:.0f}%），所以微分完全可用")

    print("\n── 貼進 config/rl_deploy.yaml ──")
    print("/**:\n  ros__parameters:")
    print(f"    measured_joint_state_hz: {hz_pos:.1f}")
    print("    joint_vel_source: register   # 或 difference")


# ──────────────────────────────────────────────────────────────────────────
# stage: imu
# ──────────────────────────────────────────────────────────────────────────


def stage_imu(node: CalibNode) -> None:
    print("\n" + "=" * 74)
    print("stage imu — 自動解出 IMU 安裝座標（純讀取，馬達不會動）")
    print("=" * 74)
    print(
        "\npolicy 的 projected_gravity 是它唯一的平衡感測。軸排錯或正負號反了，\n"
        "它會主動把自己推倒 —— 所以這一關不能跳過。但不用量角度：板載 IMU\n"
        "幾乎一定是 24 種軸對齊姿態之一，窮舉即可。"
    )
    if not node.wait_for_data():
        print("❌ 收不到 /package/sensorpackage。確認 switch 節點有在跑、Arduino 有插。")
        return

    # ── ① 靜態：用重力定出安裝的 roll / pitch ───────────────────────
    input("\n① 把機器人擺成站姿並**保持靜止不動**，然後按 Enter（會量 5 秒）…")
    acc = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < 5.0:
        _, _, m = node.snapshot()
        if m is not None:
            acc.append([m.acc_x, m.acc_y, m.acc_z])
        time.sleep(0.02)
    A = np.asarray(acc)
    if len(A) < 20:
        print(f"❌ 只收到 {len(A)} 筆加速度")
        return
    a_hat = A.mean(axis=0)
    norm = np.linalg.norm(a_hat)
    print(f"   平均加速度 = ({a_hat[0]:+.4f}, {a_hat[1]:+.4f}, {a_hat[2]:+.4f}) g，"
          f"模長 {norm:.4f}")
    if norm < 0.2:
        print(
            "\n   ❌ 加速度全是 0 —— SensorPackage 的 acc/gyr/quat 沒有資料。\n"
            "      switch.py 本身是對的（它有填這些欄位），所以問題在上游：\n"
            "      **Arduino 燒的是舊版 sketch，只送 #YPR= 沒送 #ACC=/#GYR=。**\n"
            "\n      3 秒驗證（要先停掉 switch 節點，它佔著 port）：\n"
            "        python3 -c \"import serial,time; s=serial.Serial('/dev/ttyACM0',115200,timeout=1);\\\n"
            "          t=time.time()\n"
            "          while time.time()-t<3:\n"
            "              l=s.readline().decode(errors='ignore').strip()\n"
            "              print(l) if l else None\"\n"
            "\n      只看到 #YPR= → 把 Arduino/IMU.ino 燒進去（順便把\n"
            "      dmpBegin(..., 20) 改成 50，一次解決兩件事）。"
        )
        return
    if abs(norm - 1.0) > 0.15:
        print(f"   ⚠️ 模長 {norm:.3f} 離 1 g 太遠 —— 可能單位不是 g，或機器人沒靜止。")
    a_hat = a_hat / norm

    # 靜止時加速度計量到的是比力 = −g（感測器座標）。
    # 我們要的是 g 在 base_link 座標 ≈ (0, 0, −1)：
    #     g_base = R · g_sensor = R · (−a_hat)  ≈ (0,0,−1)
    #  ⟺  R · a_hat ≈ (0, 0, +1)
    up = np.array([0.0, 0.0, 1.0])
    scored = []
    for label, R in AXIS_ROTATIONS:
        v = R @ a_hat
        err = math.degrees(math.acos(float(np.clip(np.dot(v, up), -1, 1))))
        scored.append((err, label, R))
    scored.sort(key=lambda t: t[0])
    best_err = scored[0][0]
    cands = [s for s in scored if s[0] < best_err + 15.0]
    print(f"\n   重力篩選後剩 {len(cands)} 個候選（殘差 {best_err:.2f}° ~ "
          f"{cands[-1][0]:.2f}°）：")
    for err, label, _ in cands:
        print(f"     {label:<16} 殘差 {err:6.2f}°")
    if best_err > 10.0:
        print("   ⚠️ 最佳殘差超過 10° —— IMU 可能不是軸對齊安裝，或站姿本身是歪的。")

    # ── ② 動態：用前後搖晃定出剩下的 yaw 自由度 ─────────────────────
    print(
        "\n② 重力看不到「繞重力軸的安裝旋轉」，所以要一個動作：\n"
        "   **用手把機器人前後搖晃**（繞左右軸、像點頭），持續 8 秒。"
    )
    input("   準備好按 Enter 開始…")
    gyr = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < 8.0:
        _, _, m = node.snapshot()
        if m is not None:
            gyr.append([m.gyr_x, m.gyr_y, m.gyr_z])
        time.sleep(0.02)
    G = np.asarray(gyr) * math.pi / 180.0  # deg/s → rad/s
    if len(G) < 40:
        print(f"❌ 只收到 {len(G)} 筆角速度")
        return
    G = G - G.mean(axis=0)  # 去掉零偏
    print(f"   原始角速度 RMS = ({np.sqrt((G**2).mean(0))[0]:.3f}, "
          f"{np.sqrt((G**2).mean(0))[1]:.3f}, {np.sqrt((G**2).mean(0))[2]:.3f}) rad/s")
    if np.sqrt((G**2).mean()) < 0.05:
        print("   ⚠️ 幾乎沒偵測到轉動 —— 搖晃幅度要再大一點，重跑這一段。")
        return

    print(f"\n   在 {len(cands)} 個候選裡挑「前後搖晃落在 base_link 的 y 軸」的那個：")
    print(f"   {'排列':<16}{'x 能量':>9}{'y 能量':>9}{'z 能量':>9}   判讀")
    ranked = []
    for err, label, R in cands:
        W = G @ R.T  # 每一列轉到 base 座標
        e = (W**2).mean(axis=0)
        frac = e / e.sum()
        ranked.append((frac[1], err, label, R, frac))
    ranked.sort(key=lambda t: -t[0])
    for frac_y, err, label, _, frac in ranked:
        tag = "← 最佳" if frac_y == ranked[0][0] else ""
        print(f"   {label:<16}{frac[0]:>9.3f}{frac[1]:>9.3f}{frac[2]:>9.3f}   {tag}")

    frac_y, err, label, R, frac = ranked[0]
    if frac_y < 0.6:
        print(
            f"\n   ⚠️ 最佳候選的 y 軸只佔 {frac_y:.1%} 能量 —— 搖晃方向可能不純，"
            "或 IMU 不是軸對齊。重做 ② 並盡量只做前後點頭。"
        )

    # ── ③ 正負號：印即時值讓你目視確認 ──────────────────────────────
    print("\n③ 最後確認正負號。接下來 10 秒會即時印出換算後的值：")
    print("   把機器人**慢慢往前傾**，應該看到  pitch 率 > 0、重力 x 分量變正。")
    print("   如果方向相反，把最佳排列的第二列正負號反過來（y → −y）。")
    input("   按 Enter 開始…")
    t0 = time.monotonic()
    while time.monotonic() - t0 < 10.0:
        _, _, m = node.snapshot()
        if m is not None:
            a = np.array([m.acc_x, m.acc_y, m.acc_z])
            w = np.array([m.gyr_x, m.gyr_y, m.gyr_z]) * math.pi / 180.0
            g_base = -(R @ a) / max(np.linalg.norm(a), 1e-9)
            w_base = R @ w
            tilt = math.degrees(math.acos(float(np.clip(-g_base[2], -1, 1))))
            print(
                f"\r   projected_gravity = ({g_base[0]:+.3f}, {g_base[1]:+.3f}, "
                f"{g_base[2]:+.3f})  傾角 {tilt:5.1f}°   "
                f"ang_vel = ({w_base[0]:+.3f}, {w_base[1]:+.3f}, {w_base[2]:+.3f})",
                end="",
                flush=True,
            )
        time.sleep(0.05)
    print()

    print("\n── 貼進 config/rl_deploy.yaml ──")
    print("/**:\n  ros__parameters:")
    print(f"    # gyr/acc 的感測器座標 → base_link： {label}")
    print(f"    imu_rotation: {[float(x) for x in R.flatten()]}")
    print(f"    imu_static_residual_deg: {err:.2f}")


# ──────────────────────────────────────────────────────────────────────────
# stage: pose
# ──────────────────────────────────────────────────────────────────────────


def _ramp(node: CalibNode, target: dict[int, int], seconds: float) -> None:
    """從目前位置線性插值到目標，期間持續發指令。"""
    start, _, _ = node.snapshot()
    start = {mid: start.get(mid, target[mid]) for mid in target}
    n = max(2, int(seconds * 50))
    for i in range(1, n + 1):
        a = i / n
        node.send_ticks(
            {mid: int(round(start[mid] * (1 - a) + target[mid] * a)) for mid in target},
            profile_vel=30,  # ≈ 0.72 rad/s 上限，當安全網
        )
        time.sleep(seconds / n)


def _hold_and_measure(node: CalibNode, target: dict[int, int], seconds: float):
    """維持姿態並平均 IMU 與實際到位誤差。"""
    accs, imus = [], []
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        node.send_ticks(target, profile_vel=30)
        p, _, m = node.snapshot()
        if m is not None:
            imus.append((m.roll, m.pitch, m.yaw))
            accs.append([m.acc_x, m.acc_y, m.acc_z])
        time.sleep(0.02)
    p, _, _ = node.snapshot()
    err = {mid: p.get(mid, float("nan")) - target[mid] for mid in target}
    return np.asarray(imus), np.asarray(accs), err


# imu_rotation：stage imu 解出來的（感測器座標 → base_link）。
# ⚠️ stage pose 預設用這個值判讀姿態。改過 IMU 安裝就要同步改這裡
#    （或改 config/rl_deploy.yaml 再從那裡讀）。
IMU_ROTATION = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


def _gravity_in_base(acc: np.ndarray, R: np.ndarray = IMU_ROTATION):
    """靜止時的加速度計讀值 → base_link 的 projected_gravity。

    靜止時加速度計量到的是比力 = −g，所以 g_base = −R·â。
    回傳 (g_base, 總傾角°, pitch°, roll°)。pitch > 0 = 前傾。

    ⚠️ **只在靜止時有效。** 走路時加速度計量的是 (a − g)，機身 0.11 g 的
       加速度就會讓視在重力偏 6.37°（訓練雜訊只有 2.87°）——
       跑 policy 時 projected_gravity 要用 DMP 的 QUAT，不是加速度計。
    """
    a = np.asarray(acc, dtype=float)
    n = np.linalg.norm(a)
    if n < 1e-6:
        return np.zeros(3), float("nan"), float("nan"), float("nan")
    g = -(R @ (a / n))
    clip = lambda v: float(np.clip(v, -1.0, 1.0))  # noqa: E731
    return (
        g,
        math.degrees(math.acos(clip(-g[2]))),
        math.degrees(math.asin(clip(g[0]))),
        math.degrees(math.asin(clip(-g[1]))),
    )


def stage_pose(node: CalibNode, hold: float) -> None:
    print("\n" + "=" * 74)
    print("stage pose — 比較 stand.ini 與模擬站姿（⚠️ 馬達會動）")
    print("=" * 74)

    try:
        # ⚠️ 下**全身 21 顆**（手臂也要到位、靠自己的 PD 夾住），不是只有 12 個
        #    腿關節。這樣就不用起原系統的 usb_cam/camera.launch.py ——
        #    那支其實是「完整系統 launch」，會帶起未修補的 driver（27.5 Hz、
        #    沒有 read_velocity）和同樣會發 /joint_commands 的 walking_node。
        ini_all = jm.read_stand_ini(ids=jm.ALL_STAND_IDS)
    except Exception as e:  # noqa: BLE001
        print(f"❌ 讀 stand.ini 失敗：{e}")
        return
    ini = {mid: ini_all[mid] for mid in jm.LEG_IDS}   # 比較只看腿
    sim = jm.sim_stand_ticks()
    sim_all = {**ini_all, **sim}   # 腿用模擬值，手臂/腰沿用 stand.ini

    print("\n兩組站姿刻度與換算出的關節角：")
    print(f"  {'ID':>3} {'關節':<21}{'stand.ini':>10}{'模擬':>8}{'差(tick)':>10}{'差(度)':>9}")
    for mid in jm.LEG_IDS:
        dq = math.degrees(jm.tick_to_q(mid, ini[mid]) - jm.tick_to_q(mid, sim[mid]))
        print(
            f"  {mid:>3} {jm.ID_TO_SIM[mid]:<21}{ini[mid]:>10}{sim[mid]:>8}"
            f"{ini[mid]-sim[mid]:>+10}{dq:>+9.2f}"
        )

    print(
        "\n⚠️ URDF 的 FK 顯示兩者的腳掌相對軀幹差 17°。真機站著時腳掌必然平貼\n"
        "   地面，所以要嘛軀幹真的傾 17°（模擬站姿在真機上會翹腳），要嘛某個\n"
        "   DIR / 轉軸跟假設不同。**這一關就是要用 IMU 分辨。**\n"
    )
    print(f"會下全身 {len(ini_all)} 顆（ID 1~21，含手臂與腰）。")
    print("⚠️ **不要**同時跑 usb_cam/camera.launch.py —— 那支是完整系統 launch，")
    print("   它的 walking_node 也發 /joint_commands，driver 也是未修補版。")
    print("\n流程：吊起機器人（腳微微離地）→ 開扭力 → 3 秒插值到姿態 → 量 IMU")
    print("      兩個姿態各做一次。任何時候 Ctrl+C 可中止（會關扭力）。")
    if input("\n機器人已吊起、周圍無人？輸入 yes 繼續：").strip().lower() != "yes":
        print("中止。")
        return

    if not node.wait_for_data():
        print("❌ 收不到 /joint_states 或 IMU。")
        return

    results = {}
    try:
        node.set_torque(True)
        time.sleep(0.5)
        for label, target in (
            ("stand.ini", ini_all),
            ("模擬 STAND_JOINT_POS", sim_all),
        ):
            print(f"\n── 移動到「{label}」…")
            _ramp(node, target, 3.0)
            imus, accs, err = _hold_and_measure(node, target, hold)
            if len(imus) < 10:
                print("   ❌ IMU 樣本不足")
                continue
            r, p, y = imus.mean(axis=0)
            a = accs.mean(axis=0)
            g, tilt, pitch, roll = _gravity_in_base(a)
            leg_err = {k: v for k, v in err.items() if k in jm.LEG_IDS}
            worst = max(
                leg_err.items(), key=lambda kv: abs(kv[1]) if kv[1] == kv[1] else 0
            )
            # ⚠️ DMP 的 ypr 是**相對歸零姿態**的，而歸零可能是很久以前做的，
            #    所以它的絕對值不能當傾角用（實測會出現 roll 177.95° 這種值）。
            #    姿態一律用加速度計 + imu_rotation 算。
            print(f"   projected_gravity = ({g[0]:+.4f}, {g[1]:+.4f}, {g[2]:+.4f})")
            print(f"   總傾角 {tilt:5.2f}°   pitch {pitch:+6.2f}° "
                  f"({'前傾' if pitch > 0 else '後傾'})   roll {roll:+6.2f}°")
            print(f"   （DMP 原始 ypr {y:+.1f}/{p:+.1f}/{r:+.1f}° —— 未歸零，僅供參考）")
            print(f"   最大到位誤差  ID {worst[0]} = {worst[1]:+.1f} ticks "
                  f"({math.degrees(worst[1]/jm.TPR):+.2f}°)")
            print("   → 看一眼：腳掌平貼嗎？軀幹直立嗎？")
            results[label] = (tilt, pitch, roll)
            input("   按 Enter 進行下一個姿態…")
    except KeyboardInterrupt:
        print("\n已中止。")
    finally:
        print("\n關閉扭力（機器人會軟掉，確認它還吊著）…")
        node.set_torque(False)

    if len(results) == 2:
        print("\n── 判讀 ──")
        for k, (tilt, pitch, roll) in results.items():
            print(f"  {k:<22} 總傾角 {tilt:6.2f}°   pitch {pitch:+6.2f}° "
                  f"({'前傾' if pitch > 0 else '後傾'})   roll {roll:+6.2f}°")
        ks = list(results)
        dp = results[ks[0]][1] - results[ks[1]][1]
        print(f"\n  兩者 pitch 相差 {abs(dp):.2f}°（FK 預測腳掌-軀幹差 17°）")
        best = min(results.items(), key=lambda kv: abs(kv[1][0]))
        print(f"  軀幹較直立的是：**{best[0]}**")
        print("  訓練時的傾角是 2~4°。選傾斜較小、且腳掌平貼的那個當 pose_reference。")
        print("\n── 貼進 config/rl_deploy.yaml ──")
        print("/**:\n  ros__parameters:")
        print(f"    pose_reference: {'stand_ini' if best[0] == 'stand.ini' else 'sim'}")


# ──────────────────────────────────────────────────────────────────────────


def main() -> None:
    ap = argparse.ArgumentParser(description="RL 部署前校正")
    ap.add_argument("--stage", required=True, choices=["link", "imu", "pose"])
    ap.add_argument("--seconds", type=float, default=10.0, help="link 的量測時長")
    ap.add_argument("--hold", type=float, default=4.0, help="pose 每個姿態維持秒數")
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = CalibNode()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        if args.stage == "link":
            stage_link(node, args.seconds)
        elif args.stage == "imu":
            stage_imu(node)
        elif args.stage == "pose":
            stage_pose(node, args.hold)
    except KeyboardInterrupt:
        pass
    finally:
        # ⚠️ 先 shutdown 讓 rclpy.spin() 返回，再 join 執行緒。
        #    順序顛倒的話背景執行緒還卡在 spin 裡，程式退出時會丟
        #    「terminate called without an active exception」。
        rclpy.shutdown()
        spin.join(timeout=2.0)


if __name__ == "__main__":
    sys.exit(main())
