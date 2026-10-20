"""把 dynamixel_sdk 的 `PortHandler.clearPort` 換成不阻塞的版本。

── 為什麼 ──────────────────────────────────────────────────────────────

2026-10-06 在 Orin + U2D2 上實測（`dxl --bench` 的 ⑥）：

    封包大小                      183 bytes（線上傳輸 1.83 ms）
    clearPort (reset_input_buffer)    7.43 ms   ← 95% 的成本
    writePort (ser.write)             0.09 ms
    其餘（addStuffing + CRC）          0.31 ms
    合計                              7.82 ms

`PacketHandler.txPacket()` 每次送封包前都會呼叫 `port.clearPort()`，而
pyserial 的實作是

    def clearPort(self):
        self.ser.reset_input_buffer()      # = termios.tcflush(fd, TCIFLUSH)

`tcflush` 在 ftdi_sio 上要 7.4 ms。三個 port 各付一次（寫）再加讀，整個
控制迴圈就被頂到 36 ms = 27.5 Hz，而 policy 需要 50 Hz。

排除過的其他假設（都有數據）：
    ✗ Python 組封包慢              → 0.02 ms
    ✗ syncWriteTxOnly 等 timeout   → txRxPacket 對 BROADCAST_ID 有提早返回
    ✗ 頻寬不夠 / 提高 baud 能解決    → 12 bytes 7.88 ms vs 4 bytes 8.08 ms，
                                      減位元組反而更慢 → 跟位元組數無關
    ✗ 每顆馬達一次往返              → 線性擬合的斜率 0.17 ms/顆 ≈ 頻寬 0.20

── 換成什麼 ────────────────────────────────────────────────────────────

語意相同（清空輸入緩衝區），但用「讀掉已緩衝的位元組」而不是 tcflush：

    n = ser.in_waiting
    if n: ser.read(n)

`in_waiting` 只回報已經在緩衝區裡的量，所以 `read(n)` 不會等待 —— 微秒級。

⚠️ 競態：`in_waiting` 與 `read` 之間可能又進來一個位元組，那它會留下。
   這跟 `reset_input_buffer` 有同樣的競態，而且 SDK 的 `rxPacket()` 會自己
   搜尋封包標頭（0xFF 0xFF 0xFD）重新同步，所以可以容忍。

── 怎麼套用而不改原系統 ────────────────────────────────────────────────

`rl_deploy.driver_patched` 會先 import 這個模組，再呼叫
`motor_control.driver_node.main()`。所以 motor_control 一個字都不用改，
而且隨時可以切回去（直接跑 `ros2 run motor_control driver_node`）。
"""

from __future__ import annotations

_patched = False
_original = None
_orig_latency = None


def _fast_clear_port(self) -> None:
    """清空輸入緩衝區，但不用 tcflush。"""
    try:
        n = self.ser.in_waiting
        if n:
            self.ser.read(n)
    except Exception:  # noqa: BLE001
        # 任何意外都退回 SDK 原本的做法 —— 慢，但一定正確
        self.ser.reset_input_buffer()


def apply(latency_timer: int = 1) -> bool:
    """套用修補。回傳是否真的套用了（已套用過會回 False）。

    兩件事：
      ① clearPort → 不阻塞的版本（見檔頭）
      ② SDK 的 LATENCY_TIMER 常數 → 對上 FTDI 實際的設定

    ── ② 為什麼也要改 ────────────────────────────────────────────────

    `PortHandler.setPacketTimeout` 用的是模組常數：

        packet_timeout = tx_time_per_byte * len + LATENCY_TIMER * 2.0 + 2.0

    SDK 寫死 `LATENCY_TIMER = 16`，但我們把 FTDI 的 latency_timer 設成 1
    （`/sys/bus/usb-serial/devices/ttyUSB*/latency_timer`）。所以逾時被
    高估了 30 ms：

        LATENCY_TIMER=16  274 bytes → 逾時 36.74 ms  → 3 次失敗 110.2 ms
        LATENCY_TIMER= 1  274 bytes → 逾時  6.74 ms  → 3 次失敗  20.2 ms

    2026-10-06 實測：policy 跑到一半 `/joint_states` 單次停頓 **108 ms**
    而觸發「資料過期」保護（IMU 同期最大只有 21 ms，所以是匯流排不是 ROS）。
    110.2 ms 的理論值跟 108 ms 幾乎一樣 —— **就是三次連續讀取失敗在等逾時**。
    馬達出力時 RS485 上的電氣雜訊會偶發掉包，這是正常的；問題是每次都要
    付 36.7 ms 的罰則。改成 1 之後同樣的掉包只花 20 ms，不會觸發保護。
    """
    global _patched, _original, _orig_latency
    if _patched:
        return False
    from dynamixel_sdk import port_handler as _p  # type: ignore

    _original = _p.PortHandler.clearPort
    _p.PortHandler.clearPort = _fast_clear_port
    _orig_latency = _p.LATENCY_TIMER
    _p.LATENCY_TIMER = latency_timer
    _patched = True
    return True


def revert() -> bool:
    """還原成 SDK 原本的 clearPort 與 LATENCY_TIMER。"""
    global _patched
    if not _patched or _original is None:
        return False
    from dynamixel_sdk import port_handler as _p  # type: ignore

    _p.PortHandler.clearPort = _original
    if _orig_latency is not None:
        _p.LATENCY_TIMER = _orig_latency
    _patched = False
    return True
