"""控制迴圈用的 Dynamixel 匯流排。**不依賴 ROS。**

為什麼要有這支：原本的部署是兩個不同步的 50 Hz 迴圈 ——

    driver_node (50 Hz)            policy_node (50 Hz)
      寫上一則 goal  ──┐              ┌── 收 /joint_states
      SyncRead        │              │   組觀測、推論
      發 /joint_states ┘              └── 發 /joint_commands
                                           ↑ driver 要**下一圈**才取到

driver 的迴圈是**先寫後讀**，而且寫的是上一則 `/joint_commands` 的值，所以
policy 算出來的動作至少要等一個完整的 driver 週期（20 ms）才會到馬達。
調頻率繞不過去 —— 那是順序造成的，不是速度造成的。

MuJoCo sim2sim 實測這 20 ms 動作延遲是**唯一的阻斷項**（2026-11-09，新模型
model_7498）：

    量化 + 踝 roll 夾限 + action_clip 3.0 + 20 ms 觀測延遲   @0.2 → ✅ 撐完 19 s
    以上全部 + 20 ms **動作**延遲                            @0.2 → ❌ 3 秒倒 96°
    只開動作延遲的指令掃描：0.10/0.15/0.18 ✅，0.20/0.22 ❌，0.25 ✅，
      0.28 ❌，0.30 ✅，0.35/0.40 ❌ —— ≥0.20 只有 2/7 存活，是擲硬幣。

解法是宇樹 `deploy_real.py` 的做法：**讀 → 推論 → 寫在同一個 tick**。這支
就是讓 policy_node 能直接持有馬達、不經過 topic。

── ⚠️ 不能和 driver 同時跑 ─────────────────────────────────────────────

序列埠不能兩個行程同時持有。用 `io_backend=direct` 時 launch 檔**不要**起
`driver_patched`／`motor_control/driver_node`。IMU 那條（`motionpackage/switch`）
照樣要起 —— IMU 還是走 topic，只有 20 ms 觀測延遲，實測活得下來。

仍然會發布 `/joint_states` 與 `/joint_velocities` 供記錄與其他節點觀察，
但**不訂閱** `/joint_commands`（馬達由這支獨佔）。
"""

from __future__ import annotations

from dynamixel_sdk import (
    COMM_SUCCESS,
    GroupSyncRead,
    GroupSyncWrite,
    PacketHandler,
    PortHandler,
)

from . import fast_port
from .dxl_tool import DEFAULT_BAUD, DEFAULT_PORTS

PROTOCOL_VERSION = 2.0

# 控制表位址（與 motor_control/driver_node.py 一致）
ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_ACCELERATION = 108   # SyncWrite 起點：加速度(4)+速度(4)+位置(4)
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_VELOCITY = 128       # SyncRead 起點：速度(4)+位置(4)
ADDR_PRESENT_POSITION = 132
ADDR_HARDWARE_ERROR_STATUS = 70
LEN_SYNC_WRITE = 12
LEN_SYNC_READ = 8

# Hardware Error Status（位址 70）的位元意義，Protocol 2.0
_HW_BITS = {
    0: "輸入電壓",
    2: "過熱",
    3: "編碼器",
    4: "電衝擊",
    5: "過載",
}


def decode_hw_error(v: int) -> str:
    hit = [n for b, n in _HW_BITS.items() if v & (1 << b)]
    return "+".join(hit) if hit else f"未知(0x{v:02X})"


def _signed32(v: int) -> int:
    return v - 4294967296 if v > 0x7FFFFFFF else v


class DxlBus:
    """以埠為單位隔離的同步讀寫。單一埠故障不會拖垮其他埠。"""

    def __init__(
        self,
        ports: list[str] | None = None,
        baud: int = DEFAULT_BAUD,
        ids: list[int] | None = None,
        latency_timer: int = 1,
        log=print,
    ) -> None:
        self.log = log
        self.patched = fast_port.apply(latency_timer)
        self.pk = PacketHandler(PROTOCOL_VERSION)
        self.ports: dict[str, dict] = {}
        self.id_port: dict[int, str] = {}
        want = sorted(ids or range(1, 22))

        for name in ports or DEFAULT_PORTS:
            try:
                ph = PortHandler(name)
                # ⚠️ openPort() 在裝置不存在時是**丟 FileNotFoundError**，
                #    不是回 False（dxl_tool 的註解已經踩過一次）。
                if not (ph.openPort() and ph.setBaudRate(baud)):
                    self.log(f"⚠️ 開不起來：{name}")
                    continue
            except OSError as e:
                self.log(f"⚠️ 開不起來：{name}（{e}）")
                continue

            # ⚠️ 跳過已經在前一個埠找到的 ID，所以**後面的埠可能根本沒有
            #    任何 ID 要問** —— 那不是故障，是前面已經找齊了。這兩種情況
            #    一定要分開報，不然看起來像埠壞掉（實際上原本的系統在 P3
            #    讀到的是頭部 22/23，而這裡刻意不掃頭部）。
            asked, found = 0, []
            for mid in want:
                if mid in self.id_port:
                    continue
                asked += 1
                _, result, _ = self.pk.ping(ph, mid)
                if result == COMM_SUCCESS:
                    found.append(mid)
                    self.id_port[mid] = name
            if not found:
                if asked == 0:
                    self.log(
                        f"{name}：0 顆 —— 要找的 {len(want)} 顆在前面的埠已經找齊，"
                        "沒有 ID 需要問這個埠（不是故障）"
                    )
                else:
                    self.log(f"⚠️ {name}：問了 {asked} 個 ID 都沒回應")
                ph.closePort()
                continue

            gr = GroupSyncRead(ph, self.pk, ADDR_PRESENT_VELOCITY, LEN_SYNC_READ)
            for mid in found:
                gr.addParam(mid)
            self.ports[name] = {
                "ph": ph,
                "gr": gr,
                "gw": GroupSyncWrite(
                    ph, self.pk, ADDR_PROFILE_ACCELERATION, LEN_SYNC_WRITE
                ),
                "ids": found,
            }
            self.log(f"{name}：{len(found)} 顆 → ID {found}")

        self.ids = sorted(self.id_port)
        missing = [m for m in want if m not in self.id_port]
        self.log(f"馬達共 {len(self.ids)}/{len(want)} 顆"
                 + (f"，缺 {missing}" if missing else "，齊全 ✅"))

    # ── 讀 ────────────────────────────────────────────────────────────
    def read(self) -> tuple[dict[int, int], dict[int, int], list[str]]:
        """回傳 (pos_ticks, vel_raw, 失敗的埠名)。

        ⚠️ **只回傳本次真的讀到的 ID**，讀失敗就不放進 dict —— 不要沿用舊值。
           原本的 driver 是「讀失敗保留舊值並照樣發布」，所以時間戳永遠新鮮
           而內容凍結，保護抓不到（2026-10-06 實測跑 6 秒後整條匯流排失效
           但 topic 還在 50 Hz 發）。直接讀就沒有這個問題：讀不到就是讀不到。
        """
        pos: dict[int, int] = {}
        vel: dict[int, int] = {}
        bad: list[str] = []
        for name, p in self.ports.items():
            try:
                if p["gr"].txRxPacket() != COMM_SUCCESS:
                    bad.append(name)
                    continue
                for mid in p["ids"]:
                    if not p["gr"].isAvailable(
                        mid, ADDR_PRESENT_VELOCITY, LEN_SYNC_READ
                    ):
                        continue
                    pos[mid] = _signed32(
                        p["gr"].getData(mid, ADDR_PRESENT_POSITION, 4)
                    )
                    vel[mid] = _signed32(
                        p["gr"].getData(mid, ADDR_PRESENT_VELOCITY, 4)
                    )
            except Exception as e:                       # noqa: BLE001
                self.log(f"❌ {name} 讀取異常：{e}")
                bad.append(name)
        return pos, vel, bad

    # ── 寫 ────────────────────────────────────────────────────────────
    def write(
        self, ticks: dict[int, int], profile_vel: int = 0, accel: int = 0
    ) -> list[str]:
        """下目標位置。`profile_vel=0` = 不限速（policy 自己決定軌跡）。"""
        bad: list[str] = []
        for name, p in self.ports.items():
            gw = p["gw"]
            gw.clearParam()
            n = 0
            for mid in p["ids"]:
                if mid not in ticks:
                    continue
                t = max(0, min(4095, int(ticks[mid])))
                v = max(0, min(1023, int(profile_vel)))
                a = max(0, min(32767, int(accel)))
                gw.addParam(
                    mid,
                    a.to_bytes(4, "little", signed=True)
                    + v.to_bytes(4, "little", signed=True)
                    + t.to_bytes(4, "little", signed=True),
                )
                n += 1
            if not n:
                continue
            try:
                if gw.txPacket() != COMM_SUCCESS:
                    bad.append(name)
            except Exception as e:                       # noqa: BLE001
                self.log(f"❌ {name} 寫入異常：{e}")
                bad.append(name)
        return bad

    # ── 扭力 ──────────────────────────────────────────────────────────
    def set_torque(
        self, on: bool, ids: list[int] | None = None
    ) -> tuple[list[int], dict[int, str]]:
        """逐顆開關扭力。回傳 (成功的 ID, {失敗的 ID: 原因})。

        不做 SyncWrite 版本 —— 這不在控制迴圈裡，而且逐顆才知道誰失敗。

        ⚠️ `write1ByteTxRx` 回**兩個**值 (comm_result, error)，不是三個。
           `ping` 和 `read1ByteTxRx` 才是三個 (data/model, result, error)。

        ⚠️ **`error != 0` 不等於寫入失敗。** 它是狀態封包的錯誤位元組：
             bit 7 (0x80) = Alert，意思是「這顆的 Hardware Error Status
                            (位址 70) 不是 0」—— 指令**已經執行了**，馬達
                            只是順便回報它有硬體錯誤（過載／過熱／電壓…）。
             bit 0~6      = 指令層級的錯誤（CRC、範圍、長度…），這才是沒執行。
           判斷有沒有寫進去只看 `comm_result`。我原本寫成
           `result == COMM_SUCCESS and err == 0`，於是 7 顆帶著既有硬體警報
           的馬達被誤報成失敗（實際上扭力有開，機器人也確實站起來了）。
        """
        ok: list[int] = []
        fail: dict[int, str] = {}
        alert: dict[int, str] = {}
        for mid in sorted(ids or self.ids):
            name = self.id_port.get(mid)
            if name is None:
                fail[mid] = "不在任何埠上"
                continue
            ph = self.ports[name]["ph"]
            try:
                result, err = self.pk.write1ByteTxRx(
                    ph, mid, ADDR_TORQUE_ENABLE, 1 if on else 0
                )
            except Exception as e:                       # noqa: BLE001
                fail[mid] = f"例外 {e}"
                continue
            if result != COMM_SUCCESS:
                fail[mid] = self.pk.getTxRxResult(result)
                continue
            if err & 0x7F:                               # 指令層級錯誤 = 真的沒做
                fail[mid] = self.pk.getRxPacketError(err)
                continue
            ok.append(mid)
            if err & 0x80:                               # 有執行，但馬達在警報
                try:
                    hw, r2, _ = self.pk.read1ByteTxRx(
                        ph, mid, ADDR_HARDWARE_ERROR_STATUS
                    )
                    alert[mid] = (
                        decode_hw_error(hw) if r2 == COMM_SUCCESS else "讀不到狀態"
                    )
                except Exception:                        # noqa: BLE001
                    alert[mid] = "讀不到狀態"
        if alert:
            self.log(
                "⚠️ 這些馬達有既有的硬體警報（扭力**有**開，但它在抱怨）："
                + "、".join(f"ID {m}:{r}" for m, r in sorted(alert.items()))
                + "\n    過載／過熱多半是上一次卡住或長時間出力留下的。"
                " 先用 `ros2 run rl_deploy dxl --check` 看詳情，"
                "要清就 `dxl --reboot --ids all`（會清掉扭力與增益，之後要"
                " --set-pgain 380 --set-dgain 500 --ids legs 重設）。"
            )
        return ok, fail

    def close(self) -> None:
        for p in self.ports.values():
            try:
                p["ph"].closePort()
            except Exception:                            # noqa: BLE001, S110
                pass
        self.ports.clear()
