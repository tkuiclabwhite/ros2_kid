#!/usr/bin/env python3
# coding=utf-8
"""
HuroCup United Soccer 通訊節點 (Phase 1:收 + 發)。

單一 UDP socket 接收裁判(ID 32)與隊友(ID 0-26)的 broadcast 封包,
解析後 publish 成 ROS2 topic 供主策略訂閱:

  /comm/game_state  std_msgs/String (JSON)  裁判賽況 + 我是否被允許移動
  /comm/teammates   std_msgs/String (JSON)  隊友 state/perception/intention

採 String+JSON,與你們 /detections/<color> 的慣例一致,Phase 1 不需動 tku_msgs。
Phase 2 再加 /comm/intention 反向打包送回 UDP。

參數:
  udp_port   (int)  UDP 監聽埠           預設 9000
  robot_id   (int)  本機 ID              預設 0
  team_side  (str)  'left' / 'right'     預設 'left'
  bind_addr  (str)  綁定位址             預設 '0.0.0.0'
"""

import json
import socket
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 讓「ros2 run」與「直接 python3」兩種跑法都能 import 到協定模組
try:
    from strategy.us import hurocup_protocol as proto
except ImportError:
    import hurocup_protocol as proto


class CommNode(Node):
    def __init__(self):
        super().__init__('us_comm')

        self.declare_parameter('udp_port', 9000)
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('team_side', 'left')
        self.declare_parameter('bind_addr', '0.0.0.0')

        self.udp_port = int(self.get_parameter('udp_port').value)
        self.robot_id = int(self.get_parameter('robot_id').value)
        self.team_side = str(self.get_parameter('team_side').value)
        self.bind_addr = str(self.get_parameter('bind_addr').value)

        # 預設 RELIABLE depth=10,與主策略預設訂閱相容,Phase 1 不會踩 QoS 不匹配
        self.game_state_pub = self.create_publisher(String, '/comm/game_state', 10)
        self.teammates_pub = self.create_publisher(String, '/comm/teammates', 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except OSError:
            pass
        self.sock.bind((self.bind_addr, self.udp_port))
        self.sock.settimeout(0.5)            # 讓收包迴圈能定期檢查關閉旗標

        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self.get_logger().info(
            f"[comm] listening UDP :{self.udp_port}  id={self.robot_id}  side={self.team_side}")

    # ----------------------------------------------------------------- 收包
    def _rx_loop(self):
        while self._running and rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break

            try:
                msg = proto.unpack_message(data)
            except ValueError as e:
                self.get_logger().warn(f"[comm] drop packet from {addr}: {e}")
                continue

            self._dispatch(msg)

    def _dispatch(self, msg):
        rid = msg['robot_id']
        if rid == proto.REFEREE_ID:
            self._handle_referee(msg)
        elif rid != self.robot_id:           # 不是自己 -> 當隊友(對手也會落這,可日後再分)
            self._handle_teammate(msg)

    # --------------------------------------------------------------- 裁判
    def _handle_referee(self, msg):
        gs = next((o for o in msg['objects'] if o['kind'] == 'referee'), None)
        if gs is None:
            return

        bit = proto.player_bit_index(self.robot_id, self.team_side)
        my_mask = gs['team_left'] if self.team_side == 'left' else gs['team_right']

        out = {
            'state': gs['state'],
            'state_name': gs['state_name'],
            'time_left': gs['time_left'],
            'self_enabled': proto.is_enabled(my_mask, bit),
            'play': gs['state'] in proto.PLAY_STATES,
            'team_left': gs['team_left'],
            'team_right': gs['team_right'],
        }
        m = String()
        m.data = json.dumps(out)
        self.game_state_pub.publish(m)

    # --------------------------------------------------------------- 隊友
    def _handle_teammate(self, msg):
        out = {'robot_id': msg['robot_id'], 'seq': msg['seq'], 'objects': msg['objects']}
        m = String()
        m.data = json.dumps(out)
        self.teammates_pub.publish(m)

    # --------------------------------------------------------------- 收尾
    def destroy_node(self):
        self._running = False
        try:
            self.sock.close()
        except OSError:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()