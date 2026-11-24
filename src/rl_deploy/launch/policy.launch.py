"""跑 policy 需要的最小節點集合。

    ros2 launch rl_deploy policy.launch.py                      # direct（預設）
    ros2 launch rl_deploy policy.launch.py io_backend:=topic     # 舊的雙迴圈

⚠️ **不要同時跑 usb_cam/camera.launch.py** —— 那支是「完整系統 launch」，
   它會起未修補的 driver（27.5 Hz、沒有 read_velocity），而且它的
   walking_node 也發 /joint_commands，會跟 policy 搶控制權。

── io_backend 決定起幾個節點 ───────────────────────────────────────────

**direct（預設，2 個節點）**

    motionpackage/switch       IMU，pub_hz 50
    rl_deploy/policy_node      自己持有馬達：讀 → 推論 → 寫同一個 tick

  ⚠️ **沒有 driver。** 序列埠不能兩個行程同時持有，policy_node 自己開
     /dev/U2D2_P1~P3。要用 dxl / calib 這些工具就得先把 launch 停掉。

**topic（2026-11-09 之前的架構，3 個節點）**

    rl_deploy/driver_patched   修補版 driver（clearPort 不阻塞 → 50 Hz）
    motionpackage/switch       IMU，pub_hz 50
    rl_deploy/policy_node      透過 /joint_states ↔ /joint_commands

  ⚠️ **已知會倒。** driver 的迴圈是「寫上一則指令 → 讀 → 發布」，所以
     policy 算出的動作至少要等它下一圈（20 ms）才到馬達。MuJoCo sim2sim
     實測（model_7498，2026-11-09）：量化 + 踝 roll 夾限 + action_clip 3.0
     + 20 ms 觀測延遲全開都撐完 19 s，再加上這 20 ms 動作延遲就 **3 秒倒**
     （0.2 m/s，傾角 96°）。只適合吊起來除錯，不要放下來走。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    cfg = os.path.join(
        get_package_share_directory("rl_deploy"), "config", "rl_deploy.yaml"
    )
    backend = LaunchConfiguration("io_backend")
    return LaunchDescription([
        DeclareLaunchArgument(
            "io_backend",
            default_value="direct",
            choices=["direct", "topic"],
            description="direct = policy_node 自己持有馬達（讀→推論→寫同一 tick）；"
                        "topic = 舊的雙迴圈架構，動作延遲 20 ms，已知會倒",
        ),
        # ⚠️ 只有 topic 後端才起 driver。direct 時起了會搶走序列埠，
        #    policy_node 會因為掃不到腿關節而拒絕啟動。
        Node(
            package="rl_deploy",
            executable="driver_patched",
            name="dynamixel_driver",
            output="screen",
            parameters=[{"read_velocity": True}],
            condition=LaunchConfigurationEquals("io_backend", "topic"),
        ),
        Node(
            package="motionpackage",
            executable="switch",
            name="unified_sensor",
            output="screen",
            parameters=[{"pub_hz": 50.0}],
        ),
        Node(
            package="rl_deploy",
            executable="policy_node",
            name="rl_policy",
            output="screen",
            # ⚠️ 順序有意義：cfg 先，launch 參數後 —— 後者才蓋得掉 yaml。
            parameters=[cfg, {"io_backend": backend}],
        ),
    ])
