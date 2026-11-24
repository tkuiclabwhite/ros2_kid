"""跑 motor_control 的 driver_node，但先套用 clearPort 修補。

    ros2 run rl_deploy driver_patched --ros-args -p read_velocity:=true

**motor_control 一個字都不用改。** 這支只是先 import 修補再呼叫它的 main()。
要切回原版就直接跑 `ros2 run motor_control driver_node`。

修補的原因與數據見 rl_deploy/fast_port.py 的 docstring（簡言之：pyserial 的
reset_input_buffer 在 ftdi_sio 上要 7.43 ms，佔了寫入成本的 95%，把控制迴圈
壓在 27.5 Hz）。
"""

from __future__ import annotations

import sys

from . import fast_port


def main() -> None:
    ok = fast_port.apply()
    import dynamixel_sdk.port_handler as _p

    print(
        f"[rl_deploy] SDK 修補 {'已套用' if ok else '先前已套用'}：\n"
        "            ① clearPort → 不阻塞的 in_waiting/read（取代 tcflush 的 7.4 ms）\n"
        f"            ② LATENCY_TIMER 16 → {_p.LATENCY_TIMER}"
        "（對上 FTDI 實際設定，單次讀取失敗的逾時 36.7 → 6.7 ms）",
        flush=True,
    )
    from motor_control.driver_node import main as driver_main  # noqa: PLC0415

    driver_main()


if __name__ == "__main__":
    sys.exit(main())
