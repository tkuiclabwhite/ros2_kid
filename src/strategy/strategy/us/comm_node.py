#!/usr/bin/env python3
# coding=utf-8
"""
HuroCup United Soccer 通訊節點 (Phase 1:收 + 發,含終端視覺化)。

單一 UDP socket 收裁判(ID 32)+ 場上其他機器人(ID 0-10 左隊 / 16-26 右隊)的
broadcast/unicast 封包。ID 所屬 team_side 跟本機 team_side 相同才是隊友,
不同則是對手(US-21.7),解析後 publish:
  /comm/game_state  std_msgs/String (JSON)
  /comm/teammates   std_msgs/String (JSON)  同側隊友
  /comm/opponents   std_msgs/String (JSON)  對側對手

dashboard 參數開啟時(預設 True),終端即時顯示收到的訊號。
之後接主策略要安靜跑時,用 -p dashboard:=false 關掉。

參數:
  udp_port(int)  robot_id(int)  team_side(str 'left'/'right')
  bind_addr(str)  dashboard(bool)
"""

import json
import socket
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from strategy.us import hurocup_protocol as proto
except ImportError:
    import hurocup_protocol as proto

ANSI = {'reset': '\033[0m', 'red': '\033[91m', 'grn': '\033[92m',
        'yel': '\033[93m', 'cyn': '\033[96m', 'dim': '\033[2m', 'bold': '\033[1m'}
STATE_COLOR = {0: 'red', 1: 'yel', 2: 'yel', 3: 'grn', 4: 'grn', 5: 'grn'}


class CommNode(Node):
    def __init__(self):
        super().__init__('us_comm')

        self.declare_parameter('udp_port', 9000)
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('team_side', 'left')
        self.declare_parameter('bind_addr', '0.0.0.0')
        self.declare_parameter('dashboard', True)

        self.udp_port = int(self.get_parameter('udp_port').value)
        self.robot_id = int(self.get_parameter('robot_id').value)
        self.team_side = str(self.get_parameter('team_side').value)
        self.bind_addr = str(self.get_parameter('bind_addr').value)
        self.dashboard = bool(self.get_parameter('dashboard').value)

        self.game_state_pub = self.create_publisher(String, '/comm/game_state', 10)
        self.teammates_pub = self.create_publisher(String, '/comm/teammates', 10)
        self.opponents_pub = self.create_publisher(String, '/comm/opponents', 10)

        # 共享狀態(rx thread 寫,dashboard timer 讀)
        self._lock = threading.Lock()
        self._last_ref = None
        self._teammates = {}
        self._opponents = {}
        self._rx_total = self._rx_ref = self._rx_mate = self._rx_opp = self._drops = 0
        self._last_drop = '-'
        self._rate_t = time.time()
        self._rate_n = 0
        self._rate = 0.0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except OSError:
            pass
        self.sock.bind((self.bind_addr, self.udp_port))
        self.sock.settimeout(0.5)

        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        if self.dashboard:
            self.create_timer(0.1, self._draw)
        else:
            self.get_logger().info(
                f"[comm] listening UDP :{self.udp_port} id={self.robot_id} side={self.team_side}")

    # ------------------------------------------------------------ 收包
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
                with self._lock:
                    self._drops += 1
                    self._last_drop = f"{addr[0]}: {e}"
                if not self.dashboard:
                    self.get_logger().warn(f"[comm] drop {addr}: {e}")
                continue

            with self._lock:
                self._rx_total += 1
                self._rate_n += 1
            self._dispatch(msg, addr)

    def _dispatch(self, msg, addr):
        rid = msg['robot_id']
        if rid == proto.REFEREE_ID:
            self._handle_referee(msg)
            return
        if rid == self.robot_id:
            return   # 自己送出的封包,忽略

        side = proto.team_side_of(rid)
        if side == self.team_side:
            self._handle_teammate(msg, addr)
        elif side is not None:
            self._handle_opponent(msg, addr)
        # side is None: ID 不在任何已知隊伍範圍內,忽略

    def _handle_referee(self, msg):
        gs = next((o for o in msg['objects'] if o['kind'] == 'referee'), None)
        if gs is None:
            return
        bit = proto.player_bit_index(self.robot_id, self.team_side)
        my_mask = gs['team_left'] if self.team_side == 'left' else gs['team_right']
        out = {
            'state': gs['state'], 'state_name': gs['state_name'],
            'time_left': gs['time_left'],
            'self_enabled': proto.is_enabled(my_mask, bit),
            'play': gs['state'] in proto.PLAY_STATES,
            'team_left': gs['team_left'], 'team_right': gs['team_right'],
        }
        m = String()
        m.data = json.dumps(out)
        self.game_state_pub.publish(m)
        with self._lock:
            self._rx_ref += 1
            self._last_ref = out

    def _handle_teammate(self, msg, addr):
        out = {'robot_id': msg['robot_id'], 'seq': msg['seq'], 'objects': msg['objects']}
        m = String()
        m.data = json.dumps(out)
        self.teammates_pub.publish(m)
        with self._lock:
            self._rx_mate += 1
            self._teammates[msg['robot_id']] = {
                'seq': msg['seq'], 'objects': msg['objects'],
                't': time.time(), 'ip': addr[0],
            }

    def _handle_opponent(self, msg, addr):
        out = {'robot_id': msg['robot_id'], 'seq': msg['seq'], 'objects': msg['objects']}
        m = String()
        m.data = json.dumps(out)
        self.opponents_pub.publish(m)
        with self._lock:
            self._rx_opp += 1
            self._opponents[msg['robot_id']] = {
                'seq': msg['seq'], 'objects': msg['objects'],
                't': time.time(), 'ip': addr[0],
            }

    # ------------------------------------------------------------ dashboard
    @staticmethod
    def _summ(objs):
        tagmap = {'state': 's', 'perception': 'p', 'intention': 'i', 'extension': 'x'}
        parts = []
        for o in objs:
            k = o.get('kind')
            tag = tagmap.get(k, '?')
            if k in ('state', 'perception'):
                parts.append(f"{tag}:{o['name']}({o['x']},{o['y']})")
            elif k == 'intention':
                parts.append(f"{tag}:{o['name']}({o['x']},{o['y']}) off={o.get('offence')}")
            elif k == 'extension':
                parts.append(f"{tag}:type{o['type']}")
        return '  '.join(parts) if parts else '(empty)'

    def _draw(self):
        now = time.time()
        with self._lock:
            ref = self._last_ref
            mates = dict(self._teammates)
            opps = dict(self._opponents)
            rx, rr, rm, ro, dr, ld = (self._rx_total, self._rx_ref, self._rx_mate,
                                      self._rx_opp, self._drops, self._last_drop)
            dt = now - self._rate_t
            if dt >= 1.0:
                self._rate = self._rate_n / dt
                self._rate_t = now
                self._rate_n = 0
            rate = self._rate

        L = [
            f"{ANSI['bold']}#======== US Comm — RX dashboard ========#{ANSI['reset']}",
            f" listen  : {self.bind_addr}:{self.udp_port}  id={self.robot_id} side={self.team_side}",
            f" rx      : {rx}  (ref {rr} / mate {rm} / opp {ro} / drop {dr})   {rate:.1f} pkt/s",
            "#======== Referee (id 32) ========#",
        ]
        if ref is None:
            L.append(f"{ANSI['dim']} 尚未收到裁判封包...{ANSI['reset']}")
        else:
            col = ANSI[STATE_COLOR.get(ref['state'], 'cyn')]
            tag = '[PLAY]' if ref['play'] else ('[SET]' if ref['state'] in (1, 2) else '[STOP]')
            en = (ANSI['grn'] + 'True' + ANSI['reset']) if ref['self_enabled'] \
                else (ANSI['red'] + 'False' + ANSI['reset'])
            L.append(f" state       : {col}{ANSI['bold']}{ref['state']} "
                     f"{ref['state_name']:<18}{tag}{ANSI['reset']}")
            L.append(f" time_left   : {ref['time_left']}")
            L.append(f" self_enabled: {en}")
            L.append(f" masks       : L=0b{ref['team_left']:b}  R=0b{ref['team_right']:b}")

        L.append("#======== Teammates ========#")
        if not mates:
            L.append(f"{ANSI['dim']} 尚未收到隊友封包...{ANSI['reset']}")
        else:
            for rid in sorted(mates):
                t = mates[rid]
                age = now - t['t']
                fresh = ANSI['grn'] if age < 1.0 else ANSI['dim']
                L.append(f" {fresh}id {rid:2d}{ANSI['reset']}  seq {t['seq']:5d}  "
                         f"age {age:4.1f}s  | {self._summ(t['objects'])}")

        L.append("#======== Opponents ========#")
        if not opps:
            L.append(f"{ANSI['dim']} 尚未收到對手封包...{ANSI['reset']}")
        else:
            for rid in sorted(opps):
                t = opps[rid]
                age = now - t['t']
                fresh = ANSI['grn'] if age < 1.0 else ANSI['dim']
                L.append(f" {fresh}id {rid:2d}{ANSI['reset']}  seq {t['seq']:5d}  "
                         f"age {age:4.1f}s  | {self._summ(t['objects'])}")

        L.append(f"#======== last drop: {ld} ========#")
        sys.stdout.write('\033[H\033[J' + '\n'.join(L) + '\n')
        sys.stdout.flush()

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
        sys.stdout.write(ANSI['reset'])
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()