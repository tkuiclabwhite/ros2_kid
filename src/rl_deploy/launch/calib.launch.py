"""只起校正需要的三個節點：馬達 driver、IMU/開關、（校正工具自己手動跑）。

⚠️ **不要**同時起 walking_node 或 motionpackage —— 它們也發 /joint_commands，
   三個一起發會互相覆蓋。
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            # ⚠️ 用 rl_deploy 的包裝版，不是 motor_control 的原版 ——
            #    它會先把 dynamixel_sdk 的 clearPort 換成不阻塞的實作。
            #    實測 reset_input_buffer 在 ftdi_sio 上要 7.43 ms，佔寫入
            #    成本的 95%，把控制迴圈壓在 27.5 Hz（policy 要 50 Hz）。
            #    詳見 rl_deploy/fast_port.py。要切回原版就把下面兩行換成
            #    package="motor_control", executable="driver_node"。
            package="rl_deploy",
            executable="driver_patched",
            name="dynamixel_driver",
            output="screen",
            # read_velocity 原本預設 False。開起來才有 /joint_velocities 可比較；
            # 代價只有 0.92 ms（4.6% 的 20 ms 預算）。
            parameters=[{"read_velocity": True}],
        ),
        Node(
            package="motionpackage",
            executable="switch",
            name="unified_sensor",
            output="screen",
            # 原本預設 20 Hz。policy 要 50 Hz。
            # ⚠️ 這裡設 50 只是「republish 更快」—— Arduino 端的 DMP 仍是 20 Hz，
            #    真正要提高還得改 Arduino/IMU.ino 的 dmpBegin(..., 20) → 50。
            parameters=[{"pub_hz": 50.0}],
        ),
    ])
