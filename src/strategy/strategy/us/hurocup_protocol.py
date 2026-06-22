#!/usr/bin/env python3
# coding=utf-8
"""
HuroCup United Soccer 通訊協定 (US-21) 封包打包 / 解析。

純 Python,無 ROS 相依,讓 comm_node 與 fake_referee / fake_teammate
共用同一份格式,測試時走的就是實機同一條解析路徑。

依據規則書 v15.0.2。所有多位元組欄位皆為 big-endian (MSB first)。
"""

import struct

# ---------------------------------------------------------------------------
# 常數
# ---------------------------------------------------------------------------

HEADER = 0xADDE
REFEREE_ID = 32

# 角度縮放。規則書本文 US-21.11 寫 RAD10000(例:-18023 = -1.8023 rad),
# 但 State / Perception / Intention 物件表格欄位寫 "RAD1000",兩者矛盾。
# 這裡採物件表格的值;若實機角度對不上,改成 10000 即可。
ORI_SCALE = 1000

# 物件 Type 對照(每個 object 固定 8 bytes,版面一致,靠 type 值判斷種類)
STATE_TYPES = {
    1: "own_position", 2: "ball", 3: "team_robot",
    4: "opponent_robot", 5: "team_goal", 6: "opponent_goal",
}
PERCEPT_TYPES = {
    33: "own_position", 34: "ball", 35: "team_robot", 36: "opponent_robot",
    37: "team_goal", 38: "opponent_goal", 39: "field_line", 40: "center_circle_arc",
}
INTENTION_TYPES = {65: "shot_on_goal", 66: "play_goal_keeper", 67: "movement"}
REFEREE_TYPES = {97: "game_state"}

GAME_STATES = {
    0: "stopped",
    1: "set_kickoff_left",
    2: "set_kickoff_right",
    3: "play_kickoff_left",
    4: "play_kickoff_right",
    5: "play_ball_in_play",
}
PLAY_STATES = {3, 4, 5}

# 每個 object:U8 type, U16 a, U16 b, I16 c, U8 d  -> 8 bytes
_OBJ_FMT = ">BHHhB"
_OBJ_SIZE = struct.calcsize(_OBJ_FMT)            # 8
# 標頭到 num_objects:U16 header, U16 len, U8 id, U16 seq, U8 num -> 8 bytes
_HDR_FMT = ">HHBHB"
_HDR_SIZE = struct.calcsize(_HDR_FMT)            # 8
# 訊息總長 = 8 + 8*N + dummy(1) + checksum(1) = 10 + 8N


# ---------------------------------------------------------------------------
# 打包 (給 fake_referee / fake_teammate / 之後 Phase 2 用)
# ---------------------------------------------------------------------------

def _checksum(buf: bytes) -> int:
    return sum(buf) & 0xFF


def pack_message(robot_id: int, seq: int, objects: list) -> bytes:
    """objects 為 (type, a, b, c, d) 5-tuple 的 list。回傳完整封包 bytes。"""
    body = b"".join(
        struct.pack(_OBJ_FMT, t & 0xFF, a & 0xFFFF, b & 0xFFFF, int(c), d & 0xFF)
        for (t, a, b, c, d) in objects
    )
    total_len = _HDR_SIZE + len(body) + 2
    hdr = struct.pack(_HDR_FMT, HEADER, total_len, robot_id & 0xFF,
                      seq & 0xFFFF, len(objects) & 0xFF)
    pre = hdr + body + bytes([0x00])             # dummy = 0x00
    return pre + bytes([_checksum(pre)])


# 便利建構子 ----------------------------------------------------------------

def make_state_object(type_id: int, x: int, y: int,
                      orientation_rad: float = 0.0, confidence: int = 255):
    return (type_id, x, y, int(orientation_rad * ORI_SCALE), confidence)


def make_referee_game_state(team_left_mask: int, team_right_mask: int,
                            time_left: int, state: int):
    # 對應 decode:a=team_left, b=team_right, c=time_left, d=state
    return (97, team_left_mask, team_right_mask, time_left, state)


# ---------------------------------------------------------------------------
# 解析 (comm_node 用)
# ---------------------------------------------------------------------------

def decode_object(t: int, a: int, b: int, c: int, d: int) -> dict:
    if t in STATE_TYPES:
        return {"kind": "state", "type": t, "name": STATE_TYPES[t],
                "x": a, "y": b, "orientation": c / ORI_SCALE, "confidence": d}
    if t in PERCEPT_TYPES:
        return {"kind": "perception", "type": t, "name": PERCEPT_TYPES[t],
                "x": a, "y": b, "orientation": c / ORI_SCALE, "confidence": d}
    if t in INTENTION_TYPES:
        return {"kind": "intention", "type": t, "name": INTENTION_TYPES[t],
                "x": a, "y": b, "orientation": c / ORI_SCALE, "offence": d}
    if t in REFEREE_TYPES:
        return {"kind": "referee", "type": t, "name": REFEREE_TYPES[t],
                "team_left": a, "team_right": b, "time_left": c,
                "state": d, "state_name": GAME_STATES.get(d, f"unknown_{d}")}
    if t >= 128:
        return {"kind": "extension", "type": t, "u1": a, "u2": b, "i3": c, "u4": d}
    return {"kind": "unknown", "type": t, "a": a, "b": b, "c": c, "d": d}


def unpack_message(data: bytes) -> dict:
    """解析一則訊息;格式錯誤拋 ValueError(呼叫端 catch 後丟棄即可)。"""
    if len(data) < _HDR_SIZE + 2:
        raise ValueError("packet too short")

    header, length, rid, seq, num = struct.unpack(_HDR_FMT, data[:_HDR_SIZE])
    if header != HEADER:
        raise ValueError(f"bad header 0x{header:04X}")

    expected = _HDR_SIZE + num * _OBJ_SIZE + 2
    if len(data) < expected:
        raise ValueError(f"truncated: need {expected}, got {len(data)}")

    if _checksum(data[:expected - 1]) != data[expected - 1]:
        raise ValueError("checksum mismatch")

    objects = []
    off = _HDR_SIZE
    for _ in range(num):
        t, a, b, c, d = struct.unpack(_OBJ_FMT, data[off:off + _OBJ_SIZE])
        objects.append(decode_object(t, a, b, c, d))
        off += _OBJ_SIZE

    return {"robot_id": rid, "seq": seq, "num": num, "objects": objects}


# ---------------------------------------------------------------------------
# bitmask 工具
# ---------------------------------------------------------------------------

def player_bit_index(robot_id: int, side: str) -> int:
    """
    規則書:左隊 ID 0-10、右隊 ID 16-26,但 bitmask 只有 UINT16(bit 0-15)。
    因此右隊用相對位元 (id - 16)。此處為合理假設,實機若不同再調。
    """
    return robot_id - 16 if side == "right" else robot_id


def is_enabled(bitmask: int, bit_index: int) -> bool:
    return bool(bitmask & (1 << bit_index))