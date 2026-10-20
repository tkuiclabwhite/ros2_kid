"""跑 policy 需要的最小節點集合。

⚠️ **不要同時跑 usb_cam/camera.launch.py** —— 那支是「完整系統 launch」，
   它會起未修補的 driver（27.5 Hz、沒有 read_velocity），而且它的
   walking_node 也發 /joint_commands，會跟 policy 搶控制權。

起三個：
  rl_deploy/driver_patched   修補版 driver（clearPort 不阻塞 → 50 Hz）
  motionpackage/switch       IMU，pub_hz 50
  rl_deploy/policy_node      policy 本體
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    cfg = os.path.join(
        get_package_share_directory("rl_deploy"), "config", "rl_deploy.yaml"
    )
    return LaunchDescription([
        Node(
            package="rl_deploy",
            executable="driver_patched",
            name="dynamixel_driver",
            output="screen",
            parameters=[{"read_velocity": True}],
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
            parameters=[cfg],
        ),
    ])
