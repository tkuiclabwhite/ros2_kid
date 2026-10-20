"""Dynamixel 直連工具：讀寫設定 + 通訊瓶頸量測。**driver_node 要先停掉。**

    ros2 run rl_deploy dxl --check            # 讀 RDT / PID / 電流上限 / 行程
    ros2 run rl_deploy dxl --set-rdt 0        # 寫 Return Delay Time
    ros2 run rl_deploy dxl --bench            # 量瓶頸：是頻寬還是每顆的往返延遲
    ros2 run rl_deploy dxl --set-pgain 850    # 之後做增益辨識用

⚠️ 它直接開 /dev/U2D2_P* —— **跟 driver_node 搶 port**。跑之前先把 driver
   停掉（Ctrl+C 掉 calib.launch.py，或 `pkill -f driver_node`）。

── 為什麼需要 --bench ──────────────────────────────────────────────────

實測 `/joint_states` 只有 27.5 Hz（36.37 ms），但封包本身只佔 7.85 ms
（SyncWrite 3.11 + SyncRead 位置+速度 4.74 @ 1 Mbaud）。所以**不是頻寬**，
剩下的 28.5 ms 是別的東西。

嫌疑在 SDK 的實作裡（dynamixel_sdk 4.0.5）：

    GroupSyncRead.rxPacket():
        for dxl_id in self.data_dict:                      ← 每顆一次！
            self.data_dict[dxl_id], result, _ = self.ph.readRx(...)

每顆馬達都要獨立 `readRx` → 獨立的 `rxPacket` → 獨立的 `port.readPort()`
輪詢迴圈。23 顆就是 23 次序列埠往返，而不是一次把全部收回來。

**如果是這個原因，成本會隨「馬達顆數」線性成長而不是隨「位元組數」。**
`--bench` 就是掃顆數來分辨這兩者。

── 可能的解法（SDK 4.0.5 已經有）──────────────────────────────────────

同一個 SDK 的 PacketHandler 有 `syncReadTx(..., fast_option=True)` 與
`fastSyncReadRx()` —— 那是 **Fast Sync Read**（指令 0x8A），**所有馬達的回應
合併成一個封包**，只要一次往返。`GroupSyncRead` 沒有用到它。

`--bench` 會把兩者並排計時，並**互相驗證讀到的位置一致**，這樣才知道
改 driver 值不值得。
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time

from dynamixel_sdk import (  # type: ignore
    COMM_SUCCESS,
    GroupSyncRead,
    GroupSyncWrite,
    PacketHandler,
    PortHandler,
)

PROTOCOL = 2.0
DEFAULT_PORTS = ["/dev/U2D2_P1", "/dev/U2D2_P2", "/dev/U2D2_P3"]
DEFAULT_BAUD = 1_000_000
BROADCAST_ID = 0xFE

# X 系列（XM430 / XM540）控制表
A_MODEL = (0, 2)
A_FIRMWARE = (6, 1)
A_RETURN_DELAY = (9, 1)
A_OPERATING_MODE = (11, 1)
A_CURRENT_LIMIT = (38, 2)
A_MAX_POS_LIMIT = (48, 4)
A_MIN_POS_LIMIT = (52, 4)
A_POS_D_GAIN = (80, 2)
A_POS_I_GAIN = (82, 2)
A_POS_P_GAIN = (84, 2)
A_PRESENT_VELOCITY = (128, 4)
A_PRESENT_POSITION = (132, 4)
A_PROFILE_ACCEL = (108, 4)   # driver 的 SyncWrite 起點（accel+vel+goal 共 12 bytes）
LEN_SYNC_WRITE = 12
A_GOAL_POSITION = (116, 4)
A_TORQUE_ENABLE = (64, 1)

LEG_IDS = list(range(10, 22))
ALL_IDS = list(range(1, 24))


# ──────────────────────────────────────────────────────────────────────────


class Bus:
    """開啟多個 port 並掃出每顆馬達在哪個 port。"""

    def __init__(self, ports: list[str], baud: int) -> None:
        self.ph = PacketHandler(PROTOCOL)
        self.ports: dict[str, PortHandler] = {}
        for name in ports:
            # ⚠️ PortHandler.openPort() 在裝置不存在時是**丟 FileNotFoundError**
            #    而不是回 False（pyserial 的行為透出來了），所以要包起來。
            try:
                p = PortHandler(name)
                if not p.openPort():
                    print(f"  ⚠️ 開不起來（被佔用？）：{name}")
                    continue
                if not p.setBaudRate(baud):
                    print(f"  ⚠️ baudrate 設不起來：{name}")
                    p.closePort()
                    continue
            except FileNotFoundError:
                print(f"  ⚠️ 裝置不存在：{name}")
                continue
            except Exception as e:  # noqa: BLE001
                print(f"  ⚠️ {name}: {e}")
                continue
            self.ports[name] = p
            print(f"  ✅ {name} @ {baud}")
        if not self.ports:
            raise SystemExit(
                "❌ 一個 port 都開不起來。確認：\n"
                "   · driver_node 已經停掉（它會佔住 port）  pkill -f driver_node\n"
                "   · /dev/U2D2_P1..P3 存在            ls -l /dev/U2D2_P*\n"
                "   · 使用者在 dialout 群組            groups"
            )

    def scan(self, ids: list[str] | list[int]) -> dict[int, PortHandler]:
        """ID → port。用讀 Model Number 來探測（比 ping 可靠）。"""
        found: dict[int, PortHandler] = {}
        for mid in [int(i) for i in ids]:
            for name, p in self.ports.items():
                v, r, _ = self.ph.read2ByteTxRx(p, mid, A_MODEL[0])
                if r == COMM_SUCCESS:
                    found[mid] = p
                    break
        return found

    def read(self, p: PortHandler, mid: int, addr: tuple[int, int]) -> int | None:
        a, n = addr
        fn = {1: self.ph.read1ByteTxRx, 2: self.ph.read2ByteTxRx, 4: self.ph.read4ByteTxRx}[n]
        v, r, _ = fn(p, mid, a)
        return v if r == COMM_SUCCESS else None

    def write(self, p: PortHandler, mid: int, addr: tuple[int, int], val: int) -> bool:
        a, n = addr
        fn = {1: self.ph.write1ByteTxRx, 2: self.ph.write2ByteTxRx, 4: self.ph.write4ByteTxRx}[n]
        r, _ = fn(p, mid, a, val)
        return r == COMM_SUCCESS

    def close(self) -> None:
        for p in self.ports.values():
            p.closePort()


def _signed(v: int | None, bits: int = 32) -> int | None:
    if v is None:
        return None
    half = 1 << (bits - 1)
    return v - (1 << bits) if v >= half else v


# ──────────────────────────────────────────────────────────────────────────
# --check
# ──────────────────────────────────────────────────────────────────────────


def cmd_check(bus: Bus, ids: list[int]) -> None:
    print("\n掃描馬達…")
    found = bus.scan(ids)
    print(f"找到 {len(found)} / {len(ids)} 顆\n")
    missing = [i for i in ids if i not in found]
    if missing:
        print(f"⚠️ 沒回應：{missing}\n")

    print(f"{'ID':>3} {'型號':>7} {'FW':>4} {'RDT':>5} {'模式':>5} "
          f"{'電流上限':>9} {'P':>6} {'I':>5} {'D':>6} {'行程(min..max)':>18} {'位置':>7}")
    rdt_vals = []
    for mid in ids:
        p = found.get(mid)
        if p is None:
            print(f"{mid:>3} {'—':>7}")
            continue
        g = lambda a: bus.read(p, mid, a)  # noqa: E731
        rdt = g(A_RETURN_DELAY)
        rdt_vals.append(rdt)
        print(
            f"{mid:>3} {str(g(A_MODEL)):>7} {str(g(A_FIRMWARE)):>4} {str(rdt):>5} "
            f"{str(g(A_OPERATING_MODE)):>5} {str(g(A_CURRENT_LIMIT)):>9} "
            f"{str(g(A_POS_P_GAIN)):>6} {str(g(A_POS_I_GAIN)):>5} {str(g(A_POS_D_GAIN)):>6} "
            f"{str(_signed(g(A_MIN_POS_LIMIT))) + '..' + str(_signed(g(A_MAX_POS_LIMIT))):>18} "
            f"{str(_signed(g(A_PRESENT_POSITION))):>7}"
        )

    vals = [v for v in rdt_vals if v is not None]
    if vals:
        print(f"\nReturn Delay Time：{sorted(set(vals))}  (unit = 2 µs)")
        worst = max(vals)
        print(f"  最大 {worst} → 每顆延遲 {worst*2/1000:.3f} ms，"
              f"{len(vals)} 顆串行共 {worst*2/1000*len(vals):.2f} ms")
        if worst == 0:
            print("  ✅ 已經是 0，不是這裡的問題 → 跑 --bench 找真正的瓶頸")
        else:
            print(f"  ❌ 要設成 0： ros2 run rl_deploy dxl --set-rdt 0")

    print("\n參考值（部署時要設的）：")
    print("  Current Limit = 1251   (= 3.36 A ÷ 2.69 mA，對應 LEG_EFFORT 8.0 N·m)")
    print("  踝 roll (ID 15/21) Position Limit = 1852 .. 2244  (2048 ± 196 = ±0.30 rad)")
    print("  Operating Mode = 3     (位置控制)")


# ──────────────────────────────────────────────────────────────────────────
# --set-rdt / --set-pgain
# ──────────────────────────────────────────────────────────────────────────


def cmd_write(bus: Bus, ids: list[int], addr: tuple[int, int], val: int, label: str) -> None:
    print(f"\n把 {len(ids)} 顆的 {label} 設成 {val}…")
    found = bus.scan(ids)
    ok = bad = 0
    for mid in ids:
        p = found.get(mid)
        if p is None:
            print(f"  ID {mid:>2} ❌ 沒回應")
            bad += 1
            continue
        if bus.write(p, mid, addr, val):
            back = bus.read(p, mid, addr)
            if back == val:
                ok += 1
            else:
                print(f"  ID {mid:>2} ⚠️ 寫了但讀回 {back}")
                bad += 1
        else:
            print(f"  ID {mid:>2} ❌ 寫入失敗（扭力開著時 EEPROM 區不能寫）")
            bad += 1
    print(f"\n成功 {ok}，失敗 {bad}")
    if bad:
        print("  失敗的話先關扭力： ros2 topic pub --once /set_torque "
              "std_msgs/Int16MultiArray '{data: [0, 0]}'（需要 driver 在跑）")
    print("  ⚠️ RDT / Current Limit / Position Limit 都在 EEPROM 區，**扭力必須關**才寫得進去。")


# ──────────────────────────────────────────────────────────────────────────
# --bench
# ──────────────────────────────────────────────────────────────────────────


def _time_calls(fn, n: int = 40) -> tuple[float, float]:
    ts = []
    for _ in range(n):
        t0 = time.perf_counter()
        fn()
        ts.append((time.perf_counter() - t0) * 1000.0)
    return statistics.mean(ts), statistics.median(ts)


def cmd_bench(bus: Bus, ids: list[int]) -> None:
    print("\n掃描馬達…")
    found = bus.scan(ids)
    # 挑馬達最多的那個 port 來測（最接近真實負載）
    by_port: dict[str, list[int]] = {}
    for mid, p in found.items():
        name = next(k for k, v in bus.ports.items() if v is p)
        by_port.setdefault(name, []).append(mid)
    if not by_port:
        print("❌ 沒找到任何馬達")
        return
    for k, v in by_port.items():
        print(f"  {k}: {sorted(v)}")
    pname = max(by_port, key=lambda k: len(by_port[k]))
    port = bus.ports[pname]
    pids = sorted(by_port[pname])
    print(f"\n用 {pname}（{len(pids)} 顆）測。資料長度 8 bytes（速度+位置，跟 driver 一樣）")

    addr, length = A_PRESENT_VELOCITY[0], 8

    # ── ① 掃顆數：成本隨顆數還是隨位元組？ ──────────────────────────
    print("\n① GroupSyncRead 掃顆數")
    print(f"   {'顆數':>5}{'位元組':>8}{'平均 ms':>10}{'中位 ms':>10}{'每顆 ms':>10}")
    counts = sorted({1, 2, 4, 8, len(pids)} & set(range(1, len(pids) + 1)))
    rows = []
    for n in counts:
        gr = GroupSyncRead(port, bus.ph, addr, length)
        for mid in pids[:n]:
            gr.addParam(mid)
        mean, med = _time_calls(gr.txRxPacket)
        nbytes = 14 + n * (12 + length)
        rows.append((n, mean))
        print(f"   {n:>5}{nbytes:>8}{mean:>10.2f}{med:>10.2f}{mean/n:>10.2f}")

    if len(rows) >= 2:
        (n0, t0), (n1, t1) = rows[0], rows[-1]
        slope = (t1 - t0) / (n1 - n0)
        intercept = t0 - slope * n0
        print(f"\n   線性擬合： 時間 ≈ {intercept:.2f} + {slope:.2f} × 顆數  (ms)")
        bw_per_motor = (12 + length) * 10 / DEFAULT_BAUD * 1000
        print(f"   純頻寬的每顆成本應該只有 {bw_per_motor:.3f} ms "
              f"→ 實測 {slope:.2f} ms 是它的 {slope/bw_per_motor:.0f} 倍")
        if slope > bw_per_motor * 3:
            print("   ❌ **成本隨顆數線性成長，而且遠超頻寬** → 瓶頸是每顆一次的序列埠")
            print("      往返（GroupSyncRead.rxPacket 裡的 for dxl_id 迴圈），不是頻寬。")
        else:
            print("   ✅ 接近頻寬極限 → 真的是頻寬不夠，只能減少讀取量或提高 baud")

    # ── ② Fast Sync Read：全部回應併成一個封包 ──────────────────────
    print("\n② Fast Sync Read（指令 0x8A，SDK 4.0.5 的 fastSyncReadRx）")
    gr = GroupSyncRead(port, bus.ph, addr, length)
    for mid in pids:
        gr.addParam(mid)
    gr.txRxPacket()
    ref = {mid: gr.getData(mid, A_PRESENT_POSITION[0], 4) for mid in pids}

    param = bytes(pids)
    total = len(pids) * (length + 4)

    def fast_once():
        bus.ph.syncReadTx(port, addr, length, param, len(param), True)
        return bus.ph.fastSyncReadRx(port, BROADCAST_ID, total)

    try:
        data, result, _ = fast_once()
        if result != COMM_SUCCESS or not data:
            print(f"   ❌ 不支援或回應異常（result={result}, len={len(data) if data else 0}）")
            print("      馬達固件可能太舊。那就只剩「減少讀取顆數」或「提高 baud」。")
        else:
            mean, med = _time_calls(lambda: fast_once())
            print(f"   平均 {mean:.2f} ms   中位 {med:.2f} ms   "
                  f"（GroupSyncRead 全部 {len(pids)} 顆是 {rows[-1][1]:.2f} ms）")
            print(f"   → 快 {rows[-1][1]/max(mean,1e-9):.1f} 倍")
            # 驗證：解析出的位置要跟 GroupSyncRead 一致
            stride = length + 4
            okc = 0
            for i, mid in enumerate(pids):
                base = i * stride + 2  # 每段是 ERR + ID + Param(length) + CRC(2)
                chunk = data[base + 4 : base + 8]  # 位置在 128 起算的 offset 4
                if len(chunk) == 4:
                    v = chunk[0] | (chunk[1] << 8) | (chunk[2] << 16) | (chunk[3] << 24)
                    if abs(_signed(v) - _signed(ref[mid])) <= 10:
                        okc += 1
            print(f"   解析驗證：{okc}/{len(pids)} 顆的位置與 GroupSyncRead 一致"
                  f" {'✅' if okc == len(pids) else '⚠️ 解析偏移可能要調'}")
    except Exception as e:  # noqa: BLE001
        print(f"   ❌ 呼叫失敗：{e}")

    # ── ③④ 完整迴圈複製：把 driver 的 _control_loop 做的事照抄一次計時 ──
    #
    # 為什麼要這一段：① 量出來的匯流排 I/O 只有約 15 ms（P1 3.0 + P2 4.0 +
    # P3 2.0 的讀，加 3 次寫），對應 66 Hz；但 driver 實測只有 27.5 Hz
    # （36.37 ms）—— **有 21 ms 不在匯流排上**。
    #
    # 這一段把「純 SDK 的完整迴圈」跟「driver 實測」對比：
    #   複製 ≈ 15 ms  → 那 21 ms 是 driver 的 Python 開銷（每圈重建
    #                   GroupSyncWrite、組 4 個 JointState、rclpy publish、
    #                   跟重連執行緒搶 GIL），不是硬體極限
    #   複製 ≈ 36 ms  → 匯流排真的就這麼慢，① 的單 port 量測低估了
    def build_loop(port_subset: dict[str, list[int]]):
        """回傳一個「跑完一圈」的函式，照 driver 的做法：先寫再讀。"""
        groups = []
        for pname2, mids in port_subset.items():
            pt = bus.ports[pname2]
            gw = GroupSyncWrite(pt, bus.ph, A_PROFILE_ACCEL[0], LEN_SYNC_WRITE)
            gr2 = GroupSyncRead(pt, bus.ph, addr, length)
            cur = {}
            for mid in mids:
                v = bus.read(pt, mid, A_PRESENT_POSITION)
                cur[mid] = v if v is not None else 2048
                gr2.addParam(mid)
            groups.append((gw, gr2, mids, cur))

        def once():
            for gw, gr2, mids, cur in groups:
                gw.clearParam()
                for mid in mids:
                    # 目標寫「現在位置」→ 即使扭力開著也不會動
                    gw.addParam(
                        mid,
                        (0).to_bytes(4, "little", signed=True)
                        + (0).to_bytes(4, "little", signed=True)
                        + int(cur[mid]).to_bytes(4, "little", signed=True),
                    )
                gw.txPacket()
                gr2.txRxPacket()

        return once

    print("\n③ 完整迴圈複製 —— 3 個 port 全部（跟 driver 做的事一樣）")
    all_loop = build_loop(by_port)
    mean_all, med_all = _time_calls(all_loop, 30)
    print(f"   平均 {mean_all:.2f} ms   中位 {med_all:.2f} ms   → {1000/mean_all:.1f} Hz")
    print(f"   driver 實測是 36.37 ms (27.5 Hz)")
    gap = 36.37 - mean_all
    if gap > 5:
        print(f"   ❌ **差 {gap:.1f} ms 不在匯流排上** → 是 driver 的 Python 開銷。")
        print("      每圈它會重建 GroupSyncWrite（clearParam + 23 次 addParam）、")
        print("      組 4 個 JointState、做 4 次 rclpy publish，還跟重連執行緒搶 GIL。")
    else:
        print(f"   ✅ 跟 driver 差不多（{gap:+.1f} ms）→ 匯流排真的就是瓶頸。")

    # ── ⑤ 只跑有腿的那個 port ───────────────────────────────────────
    #
    # policy 只控制 12 個腿關節（ID 10~21）。如果它們剛好全在同一個 port 上，
    # driver 的 `ports` 參數就能把範圍縮到那一個 —— **純 launch 參數，不動程式**。
    leg_ports = {
        k: sorted(set(v) & set(LEG_IDS)) for k, v in by_port.items()
        if set(v) & set(LEG_IDS)
    }
    print(f"\n④ 腿關節 (ID 10~21) 分布在：")
    for k, v in leg_ports.items():
        print(f"   {k}: {v}  ({len(v)} 顆)")
    if len(leg_ports) == 1:
        pname2 = next(iter(leg_ports))
        print(f"\n   ✅ **12 個腿關節全在 {pname2}** → policy 只需要這一個 port。")
        only = build_loop({pname2: by_port[pname2]})
        mean1, med1 = _time_calls(only, 30)
        print(f"   只跑 {pname2}（{len(by_port[pname2])} 顆）：平均 {mean1:.2f} ms "
              f"→ **{1000/mean1:.1f} Hz**")
        print(f"\n   做法（不動任何程式，只改 launch 參數）：")
        print(f"     ros2 run motor_control driver_node --ros-args \\")
        print(f"       -p ports:=\"['{pname2}']\" -p read_velocity:=true")
        print("   ⚠️ 代價：手臂與頭不再收到指令。所以順序要是")
        print("      ① 三個 port 全開 → 下站姿（手臂到位、扭力開著自己夾住）")
        print(f"      ② 重啟 driver 只開 {pname2} → 跑 policy")
    else:
        print("\n   ⚠️ 腿關節跨多個 port，沒辦法只開一個。")

    # ── ⑤ 寫入路徑拆解 ─────────────────────────────────────────────
    #
    # 從 ①③④ 推出來：**SyncWrite 佔整個迴圈 72%**，而且其中 6.45 ms 不是頻寬
    #   P2 讀 13 顆            4.02 ms
    #   P2 完整迴圈(寫+讀)    12.27 ms  → 寫 8.25 ms（讀的 2.1 倍）
    #   但 180 bytes @1Mbaud 只需要 1.80 ms
    #
    # 所以要拆：是 Python 組封包慢、還是序列埠寫入慢、還是跟位元組數有關。
    pt = bus.ports[pname]
    mids = pids
    payload12 = (
        (0).to_bytes(4, "little", signed=True)
        + (0).to_bytes(4, "little", signed=True)
        + (2048).to_bytes(4, "little", signed=True)
    )
    payload4 = (2048).to_bytes(4, "little", signed=True)

    gw12 = GroupSyncWrite(pt, bus.ph, A_PROFILE_ACCEL[0], LEN_SYNC_WRITE)
    gw4 = GroupSyncWrite(pt, bus.ph, A_GOAL_POSITION[0], 4)
    grr = GroupSyncRead(pt, bus.ph, addr, length)
    for mid in mids:
        grr.addParam(mid)

    def only_params(gw, payload):
        def f():
            gw.clearParam()
            for mid in mids:
                gw.addParam(mid, payload)
        return f

    def full_write(gw, payload):
        def f():
            gw.clearParam()
            for mid in mids:
                gw.addParam(mid, payload)
            gw.txPacket()
        return f

    print(f"\n⑤ 寫入路徑拆解（{pname}，{len(mids)} 顆）")
    print(f"   {'項目':<34}{'平均 ms':>10}{'理論頻寬 ms':>13}")
    cases = [
        ("clearParam + addParam（純 Python）", only_params(gw12, payload12), 0.0),
        ("+ txPacket，12 bytes/顆（driver 現況）", full_write(gw12, payload12),
         (11 + len(mids) * 13) * 10 / DEFAULT_BAUD * 1000),
        ("+ txPacket，4 bytes/顆（只寫目標位置）", full_write(gw4, payload4),
         (11 + len(mids) * 5) * 10 / DEFAULT_BAUD * 1000),
        ("SyncRead 參考值", grr.txRxPacket,
         (14 + len(mids) * (12 + length)) * 10 / DEFAULT_BAUD * 1000),
    ]
    res = {}
    for label, fn, bw in cases:
        mean, _ = _time_calls(fn, 40)
        res[label] = mean
        print(f"   {label:<34}{mean:>10.2f}{bw:>13.2f}")

    py_cost = res["clearParam + addParam（純 Python）"]
    w12 = res["+ txPacket，12 bytes/顆（driver 現況）"]
    w4 = res["+ txPacket，4 bytes/顆（只寫目標位置）"]
    print(f"\n   組封包的 Python 成本          {py_cost:.2f} ms")
    print(f"   txPacket 本身（12 bytes）     {w12 - py_cost:.2f} ms")
    print(f"   12 → 4 bytes 省下             {w12 - w4:.2f} ms")
    print("\n   判讀：")
    if py_cost > w12 * 0.4:
        print("     ❌ **Python 組封包就佔了大半** → 要快只能讓 driver 把 GroupSyncWrite")
        print("        重用而不是每圈 clearParam/addParam 重建。")
    elif w12 - w4 > 1.0:
        print("     ❌ **跟位元組數強相關** → driver 每圈寫 accel+vel+goal 共 12 bytes，")
        print("        但 accel/vel 一直是 0。改成只寫 Goal Position (addr 116, 4 bytes)")
        print(f"        每個 port 省 {w12-w4:.2f} ms，三個 port 省 {(w12-w4)*3:.2f} ms。")
    else:
        print("     → 跟位元組數無關 → 是每次交易的固定成本。")
        print("        ⚠️ **這也代表提高 baud 幫不上忙** —— baud 只縮短線上傳輸時間，")
        print("           壓不掉固定成本。往下看 ⑥ 才知道那個固定成本是什麼。")

    # ── ⑥ txPacket 內部拆解 ─────────────────────────────────────────
    #
    # ⑤ 顯示寫入有約 6 ms 的固定成本，而且跟位元組數無關。翻過 SDK 原始碼，
    # 排除了三個猜測：
    #   ✗ Python 組封包慢      → ⑤ 量到 0.01 ms
    #   ✗ syncWriteTxOnly 等 timeout → txRxPacket 對 BROADCAST_ID 有提早返回
    #   ✗ 跟 baud 有關          → 12 vs 4 bytes 沒差
    # 剩下 txPacket 裡只有四件事：addStuffing、updateCRC（Python 迴圈）、
    # clearPort（= pyserial reset_input_buffer）、writePort（= ser.write）。
    # 把後兩個包起來計時就知道了。
    print(f"\n⑥ txPacket 內部拆解（{pname}，{len(mids)} 顆，12 bytes/顆）")
    acc = {"clear": 0.0, "write": 0.0, "n": 0}
    orig_clear, orig_write = pt.clearPort, pt.writePort

    def timed_clear():
        t = time.perf_counter()
        r = orig_clear()
        acc["clear"] += (time.perf_counter() - t) * 1000
        return r

    def timed_write(packet):
        t = time.perf_counter()
        r = orig_write(packet)
        acc["write"] += (time.perf_counter() - t) * 1000
        acc["n"] = len(packet)
        return r

    # ⚠️ 這裡是**實例屬性**，會遮蔽類別屬性。還原時必須用 del 而不是
    #    「把原本的綁定方法寫回去」—— 寫回去會留下一個指向舊實作的實例屬性，
    #    之後 ⑦ 在**類別**上套修補就對這個實例無效。
    #    （2026-10-06 就是這樣污染了 ⑦：單 port 寫入看起來沒改善，
    #      但 3 port 迴圈改善了 —— 因為另外兩個 port 沒被 ⑥ 碰過。）
    pt.clearPort, pt.writePort = timed_clear, timed_write
    try:
        N = 40
        t0 = time.perf_counter()
        for _ in range(N):
            gw12.clearParam()
            for mid in mids:
                gw12.addParam(mid, payload12)
            gw12.txPacket()
        total = (time.perf_counter() - t0) * 1000 / N
    finally:
        del pt.clearPort, pt.writePort   # 不是 = orig_*，理由見上

    clear_ms = acc["clear"] / N
    write_ms = acc["write"] / N
    rest = total - clear_ms - write_ms
    bw = acc["n"] * 10 / DEFAULT_BAUD * 1000
    print(f"   封包大小                      {acc['n']} bytes（線上傳輸 {bw:.2f} ms）")
    print(f"   clearPort (reset_input_buffer) {clear_ms:>7.2f} ms")
    print(f"   writePort (ser.write)          {write_ms:>7.2f} ms")
    print(f"   其餘（addStuffing + CRC）       {rest:>7.2f} ms")
    print(f"   合計                           {total:>7.2f} ms")
    print("\n   判讀：")
    if clear_ms > total * 0.5:
        print("     ❌ **clearPort 是元凶** —— pyserial 的 reset_input_buffer 在")
        print("        ftdi_sio 上很慢。driver 不用改：這是 SDK 內部呼叫的。")
        print("        解法是改用一個不呼叫 clearPort 的自訂寫入路徑。")
    elif write_ms > total * 0.5:
        print(f"     ❌ **ser.write 阻塞 {write_ms:.2f} ms**，而線上只需要 {bw:.2f} ms")
        print("        → 它在等 USB/FTDI 把資料送完。可試：")
        print("          · 把 U2D2 的 FTDI 參數再調（已設 latency_timer=1）")
        print("          · 提高 baud 縮短實際傳輸時間")
    else:
        print("     → 成本分散，沒有單一元凶。")

    # ── ⑦ 套用 clearPort 修補後重測 ─────────────────────────────────
    #
    # ⑥ 指出 clearPort（pyserial 的 reset_input_buffer → termios.tcflush）
    # 在 ftdi_sio 上要 7.4 ms。rl_deploy.fast_port 把它換成語意相同但不阻塞的
    # 「讀掉已緩衝的位元組」。這一段**先量出效果再決定要不要用**。
    from . import fast_port

    print("\n⑦ 套用 clearPort 修補後重測")
    before_all = mean_all
    fast_port.apply()
    try:
        m6, _ = _time_calls(full_write(gw12, payload12), 40)
        mr, _ = _time_calls(grr.txRxPacket, 40)
        mall, _ = _time_calls(all_loop, 30)
        print(f"   {'項目':<28}{'修補前':>10}{'修補後':>10}{'變化':>10}")
        print(f"   {'單 port 寫入':<28}{w12:>10.2f}{m6:>10.2f}{m6-w12:>+10.2f}")
        print(f"   {'單 port 讀取':<28}"
              f"{res['SyncRead 參考值']:>10.2f}{mr:>10.2f}"
              f"{mr-res['SyncRead 參考值']:>+10.2f}")
        print(f"   {'3 port 完整迴圈':<28}{before_all:>10.2f}{mall:>10.2f}"
              f"{mall-before_all:>+10.2f}")
        print(f"\n   → 控制率 {1000/before_all:.1f} Hz → **{1000/mall:.1f} Hz**")
        # 驗證資料還是對的（修補不能破壞讀取）
        grr.txRxPacket()
        now = {mid: grr.getData(mid, A_PRESENT_POSITION[0], 4) for mid in pids}
        bad = [mid for mid in pids if abs(_signed(now[mid]) - _signed(ref[mid])) > 50]
        print(f"   資料驗證：{len(pids)-len(bad)}/{len(pids)} 顆位置與修補前一致"
              f" {'✅' if not bad else f'❌ 異常 {bad}'}")
        if mall <= 20.0:
            print("\n   ✅ **低於 20 ms → 三個 port 全開也能跑 50 Hz**，不用只開一個。")
            print("      套用方式（motor_control 一個字都不用改）：")
            print("        ros2 run rl_deploy driver_patched --ros-args -p read_velocity:=true")
            print("      或直接用 ros2 launch rl_deploy calib.launch.py（已經改成用它）")
        else:
            print(f"\n   ⚠️ {mall:.2f} ms = {1000/mall:.1f} Hz，還沒到 50 Hz（需要 ≤ 20.0 ms）。")
            print("      對照 ④ 的單 port 數字（3 × 單 port 應該接近這個值）看差在哪。")
    finally:
        fast_port.revert()
        print("   （已還原 clearPort，本次量測不影響後續）")


# ──────────────────────────────────────────────────────────────────────────
# --step-test：位置環增益辨識
# ──────────────────────────────────────────────────────────────────────────


def cmd_step_test(
    bus: Bus, mid: int, amp: int, pgains: list[int], dgains: list[int],
    igain: int = 0,
) -> None:
    """單關節階躍響應 → 反推等效 ζ 與上升時間，用來定 Position P/D Gain。

    ── 為什麼需要 ──────────────────────────────────────────────────────

    模擬用 `ImplicitActuator` 之外的 `DCMotorCfg` + PD：

        τ = clip(25·e − 1.9·ω, ±8)

    真機出廠是 `P=800, I=0, D=0`。換算（e-Manual: KPP = P/128，PWM→扭矩
    用堵轉 11.42 N·m / PWM 885）：

        P=800 → 52.6 N·m/rad   （模擬 25，硬 2.1 倍）
        P=380 → 25.0 N·m/rad   ← 對上模擬
        P=200 → 13.1 N·m/rad

    但 **D=0 等於 ζ=0（完全不衰減）**，而模擬的 B=1.9 在 K=25 時是
    ζ=1.64（過阻尼）。所以 P 調對了但沒阻尼反而更糟 —— 實測 P=200 比
    P=380 好看，是用「變軟」換來的（ζ 不變還是 0，只是 ω_n 從 43 降到 31）。

    ── 為什麼要量而不是算 ──────────────────────────────────────────────

    e-Manual 給了 `KPD = PositionDGain / 16`，但**沒說 D 項的誤差微分用什麼
    時間單位**（ticks/s？ticks/ms？內部控制週期？），差一個數量級結果就全錯。
    所以 D 只能用階躍響應掃出來。

    ── 判準（目標 = 複製模擬的閉環）────────────────────────────────────

        模擬 K=25, B=1.9 → ω_n 43.1 rad/s、ζ 1.64（過阻尼）
        慢極點 14.7 rad/s → 階躍 10~90% 上升 ≈ 150 ms、**無過衝**

    所以要找的是 **P=380 且過衝 ≈ 0、上升時間 ≈ 150 ms** 的那個 D。
    """
    from . import fast_port

    found = bus.scan([mid])
    pt = found.get(mid)
    if pt is None:
        print(f"❌ ID {mid} 沒回應")
        return
    from .joint_map import ID_TO_SIM, TPR

    name = ID_TO_SIM.get(mid, "?")
    print(
        f"\n階躍響應辨識 —— ID {mid}（{name}），"
        f"振幅 {amp} ticks = {amp / TPR:.4f} rad = {math.degrees(amp / TPR):.2f}°"
    )
    print("⚠️ 只對這一顆開扭力。機器人要吊著，而且這顆關節周圍要淨空。")
    if input("   輸入 yes 繼續：").strip().lower() != "yes":
        print("中止。")
        return

    fast_port.apply()   # 不然每次讀要 7.4 ms，取樣太慢
    try:
        start = bus.read(pt, mid, A_PRESENT_POSITION)
        if start is None:
            print("❌ 讀不到位置")
            return
        start = _signed(start)
        print(f"   起始位置 {start}")
        print()
        print(
            f"   {'P':>6}{'D':>6}{'過衝%':>8}{'上升10-90':>11}{'垂度':>10}"
            f"{'ζ':>8}{'ω_n':>9}{'重力τ':>9}"
        )
        print("   （過衝與上升時間都是相對**實際穩態值**；垂度是相對指令振幅）")
        bus.write(pt, mid, A_TORQUE_ENABLE, 1)
        results = []
        for P in pgains:
            for D in dgains:
                bus.write(pt, mid, A_POS_P_GAIN, P)
                bus.write(pt, mid, A_POS_I_GAIN, igain)
                bus.write(pt, mid, A_POS_D_GAIN, D)
                # 回到起點並靜置
                bus.write(pt, mid, A_GOAL_POSITION, start)
                time.sleep(0.6)
                # 階躍
                target = start + amp
                t0 = time.perf_counter()
                bus.write(pt, mid, A_GOAL_POSITION, target)
                tr, pr = [], []
                while time.perf_counter() - t0 < 0.8:
                    v = bus.read(pt, mid, A_PRESENT_POSITION)
                    if v is not None:
                        tr.append(time.perf_counter() - t0)
                        pr.append(_signed(v))
                bus.write(pt, mid, A_GOAL_POSITION, start)
                time.sleep(0.6)
                if len(pr) < 20:
                    print(f"   {P:>6}{D:>6}   取樣不足 ({len(pr)})")
                    continue
                # ⚠️ 2026-10-06 修正：正規化要用**實際到達的穩態值**，不是
                #    指令振幅。重力會把關節往下拉，讓它永遠到不了 100%
                #    （實測 P=200 只到 55%、P=380 到 78%），以振幅正規化的話
                #    上升時間永遠是 nan，就看不到 D 的作用（D 只影響暫態）。
                y = [(v - start) / amp for v in pr]
                yf = sum(y[-8:]) / len(y[-8:])          # 穩態（最後幾點平均）
                if abs(yf) < 0.05:
                    print(f"   {P:>6}{D:>6}   幾乎沒動（穩態 {yf*100:.0f}%），跳過")
                    continue
                peak = max(y) if yf > 0 else min(y)
                over = max(0.0, (peak - yf) / abs(yf)) * 100
                ss = yf - 1.0                            # 垂度（相對指令）
                t10 = next((tr[i] for i, v in enumerate(y) if v >= 0.1 * yf), float("nan"))
                t90 = next((tr[i] for i, v in enumerate(y) if v >= 0.9 * yf), float("nan"))
                rise = (t90 - t10) if t90 == t90 and t10 == t10 else float("nan")
                # 垂度 → 重力扭矩（獨立驗證 K ∝ P 的推導）
                K = (P / 128) * TPR / 885 * 11.42
                tau_g = K * abs(ss) * amp / TPR
                if over > 1.0:
                    Mp = over / 100
                    ln = math.log(Mp)
                    zeta = -ln / math.sqrt(math.pi**2 + ln**2)
                    wn = (
                        math.pi / (rise * math.sqrt(1 - zeta**2))
                        if rise == rise and zeta < 1
                        else float("nan")
                    )
                    zs, ws = f"{zeta:.2f}", f"{wn:.0f}"
                else:
                    # 無過衝 → 過阻尼，用上升時間反推慢極點
                    zs = "≥1.0"
                    ws = f"{2.2/rise:.0f}" if rise == rise and rise > 0 else "—"
                print(
                    f"   {P:>6}{D:>6}{over:>8.1f}"
                    f"{rise*1000 if rise == rise else float('nan'):>9.0f}ms"
                    f"{ss*100:>9.1f}%{zs:>8}{ws:>9}{tau_g:>9.3f}"
                )
                results.append((P, D, over, rise))
        bus.write(pt, mid, A_GOAL_POSITION, start)
        time.sleep(0.5)
    finally:
        bus.write(pt, mid, A_TORQUE_ENABLE, 0)
        fast_port.revert()
        print()
        print("   已關閉這顆的扭力、還原 clearPort。")
        print(f"   ⚠️ **增益還是最後那組**，要設回去：")
        print(f"      ros2 run rl_deploy dxl --set-pgain <值> --set-dgain <值> --ids legs")

    print()
    print("   目標（複製模擬 K=25 / B=1.9 的閉環）：")
    print("     P = 380、**過衝 ≈ 0**、上升 10~90% ≈ 150 ms")
    print()
    print("   『重力τ』欄：同一顆關節在不同 P 下算出的重力扭矩應該**一致**。")
    print("   一致就代表 e-Manual 的 KPP = P/128 換算是對的（這是獨立驗證）。")
    print("   2026-10-06 實測 ID 13：P=200 → 0.907 N·m、P=380 → 0.843 N·m ✅")
    ok = [r for r in results if r[0] == 380 and r[2] < 2.0
          and r[3] == r[3] and 0.10 < r[3] < 0.25]
    if ok:
        print(f"   ✅ 符合的組合：{[(p, d) for p, d, *_ in ok]}")
    else:
        print("   → 沒有組合同時滿足。把 --dgains 的範圍往上/下延伸再掃一次。")


# ──────────────────────────────────────────────────────────────────────────


def main() -> None:
    ap = argparse.ArgumentParser(description="Dynamixel 直連工具（driver 要先停）")
    ap.add_argument("--ports", nargs="+", default=DEFAULT_PORTS)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--ids", default="all", help="all | legs | 以逗號分隔")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--bench", action="store_true")
    ap.add_argument("--set-rdt", type=int, default=None)
    ap.add_argument("--set-pgain", type=int, default=None)
    ap.add_argument("--set-dgain", type=int, default=None)
    ap.add_argument("--set-igain", type=int, default=None)
    ap.add_argument("--step-test", action="store_true")
    ap.add_argument("--id", type=int, default=13, help="step-test 的關節（預設 13 = L 膝）")
    ap.add_argument("--amp", type=int, default=100, help="階躍振幅（ticks）")
    ap.add_argument("--pgains", default="200,380")
    ap.add_argument("--dgains", default="0,500,1500,3000")
    args, _ = ap.parse_known_args()

    ids = (
        ALL_IDS if args.ids == "all"
        else LEG_IDS if args.ids == "legs"
        else [int(x) for x in args.ids.split(",")]
    )

    print("開啟 port…")
    bus = Bus(args.ports, args.baud)
    try:
        if args.check:
            cmd_check(bus, ids)
        if args.set_rdt is not None:
            cmd_write(bus, ids, A_RETURN_DELAY, args.set_rdt, "Return Delay Time")
        for val, addr, label in (
            (args.set_pgain, A_POS_P_GAIN, "Position P Gain"),
            (args.set_igain, A_POS_I_GAIN, "Position I Gain"),
            (args.set_dgain, A_POS_D_GAIN, "Position D Gain"),
        ):
            if val is not None:
                cmd_write(bus, ids, addr, val, label)
        if args.bench:
            cmd_bench(bus, ids)
        if args.step_test:
            cmd_step_test(
                bus, args.id, args.amp,
                [int(x) for x in args.pgains.split(",")],
                [int(x) for x in args.dgains.split(",")],
            )
        if not any([args.check, args.bench, args.set_rdt is not None,
                    args.set_pgain is not None, args.set_dgain is not None,
                    args.set_igain is not None, args.step_test]):
            ap.print_help()
    finally:
        bus.close()


if __name__ == "__main__":
    sys.exit(main())
