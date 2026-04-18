from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite, GroupSyncRead
import time
import threading

ADDR_RETURN_DELAY_TIME      = 9    # 回覆延遲
ADDR_OPERATING_MODE         = 11
ADDR_TORQUE_ENABLE          = 64
ADDR_STATUS_RETURN_LEVEL    = 68   # 狀態回覆等級 (建議=2)
ADDR_PRO_ACCEL              = 108   # 4 bytes, unsigned
ADDR_GOAL_VELOCITY          = 104   # speed-mode only
ADDR_PRO_VEL                = 112   # 4 bytes, unsigned (position-mode profile vel)
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132

LEN_VELOCITY = 4
LEN_POSITION = 4
COMM_SUCCESS = 0

TPS_TO_UNITS_K = 60.0 / (4096.0 * 0.229)  # ~= 0.06397

class DynamixelBus:
    def __init__(self, port: str, baudrate: int = 1_000_000, ids=None):
        self.portHandler   = PortHandler(port)
        self.packetHandler = PacketHandler(2.0)
        if not self.portHandler.openPort():
            raise RuntimeError(f"OpenPort failed: {port}")
        if not self.portHandler.setBaudRate(baudrate):
            raise RuntimeError(f"SetBaudRate failed: {baudrate}")
        self.ids  = ids or []
        self.port = port
        self.groupSyncReadPos = None
        self._last_mode = 'unknown'

        # ✅ 全域 I/O 鎖：所有 tx/rx 都必須經過這把鎖
        self.io_lock = threading.Lock()

        # ✅ 位置模式用 Profile Velocity (112)
        self.groupSyncWriteVel = GroupSyncWrite(
            self.portHandler, self.packetHandler, ADDR_PRO_VEL, LEN_VELOCITY
        )
        self.groupSyncWritePos = GroupSyncWrite(
            self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, LEN_POSITION
        )

    def enable_torque(self, pro_accel_units: int = 200, pro_vel_units: int = 600):
        with self.io_lock:
            for mid in self.ids:
                self.packetHandler.write1ByteTxRx(self.portHandler, mid, ADDR_TORQUE_ENABLE, 0)   # Torque OFF
                self.packetHandler.write1ByteTxRx(self.portHandler, mid, ADDR_OPERATING_MODE, 3)  # Position
                self.packetHandler.write1ByteTxRx(self.portHandler, mid, ADDR_RETURN_DELAY_TIME, 0)   # ★ 回覆延遲=0
                self.packetHandler.write1ByteTxRx(self.portHandler, mid, ADDR_STATUS_RETURN_LEVEL, 2) # ★ SRL=2
                self.packetHandler.write4ByteTxRx(self.portHandler, mid, ADDR_PRO_ACCEL, max(1, int(pro_accel_units)))
                self.packetHandler.write4ByteTxRx(self.portHandler, mid, ADDR_PRO_VEL,   max(1, int(pro_vel_units)))
                self.packetHandler.write1ByteTxRx(self.portHandler, mid, ADDR_TORQUE_ENABLE, 1)   # Torque ON

    def write_pv_gp(self, pv_map, gp_map):
        with self.io_lock:
            self.groupSyncWriteVel.clearParam()
            self.groupSyncWritePos.clearParam()

            for dxl_id in self.ids:
                # Profile Velocity 是無號 4 bytes；夾到最小 1，避免 0=不限速/無效
                vel_units = max(1, int(pv_map.get(dxl_id, 600)))
                pos       = int(gp_map.get(dxl_id, 2048))

                vel_bytes = vel_units.to_bytes(4, byteorder='little', signed=False)
                pos_bytes = pos.to_bytes(4, byteorder='little', signed=False)

                if not self.groupSyncWriteVel.addParam(dxl_id, vel_bytes):
                    print(f"[ERR] addParam ProfileVel failed ID={dxl_id}")
                if not self.groupSyncWritePos.addParam(dxl_id, pos_bytes):
                    print(f"[ERR] addParam GoalPos failed ID={dxl_id}")

            rv = self.groupSyncWriteVel.txPacket()
            rp = self.groupSyncWritePos.txPacket()

        if rv != COMM_SUCCESS:
            print("[ERR] tx ProfileVel:", self.packetHandler.getTxRxResult(rv))
        if rp != COMM_SUCCESS:
            print("[ERR] tx GoalPos:", self.packetHandler.getTxRxResult(rp))
        return rv, rp

    def write_gp(self, gp_map):
        with self.io_lock:
            gsw = GroupSyncWrite(self.portHandler, self.packetHandler,
                                 ADDR_GOAL_POSITION, LEN_POSITION)
            for dxl_id in self.ids:
                pos = int(gp_map.get(dxl_id, 2048))
                pos_bytes = pos.to_bytes(4, byteorder='little', signed=False)
                if not gsw.addParam(dxl_id, pos_bytes):
                    print(f"[ERR] addParam GoalPos failed ID={dxl_id}")
            return gsw.txPacket()
        
    def init_sync_read(self):
        with self.io_lock:
            self.groupSyncReadPos = GroupSyncRead(
                self.portHandler, self.packetHandler,
                ADDR_PRESENT_POSITION, LEN_POSITION
            )
            for dxl_id in self.ids:
                ok = self.groupSyncReadPos.addParam(dxl_id)
                if not ok:
                    print(f"[warn] addParam to GroupSyncRead failed: id={dxl_id} on {self.port}")
    def last_mode(self):
        return getattr(self, '_last_mode', 'unknown')

    def read_positions(self):
        with self.io_lock:
            try:
                if self.groupSyncReadPos is not None:
                    comm = self.groupSyncReadPos.txRxPacket()
                    out, avail_cnt = {}, 0
                    if comm == COMM_SUCCESS:
                        for dxl_id in self.ids:
                            if self.groupSyncReadPos.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_POSITION):
                                out[dxl_id] = int(self.groupSyncReadPos.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_POSITION))
                                avail_cnt += 1
                            else:
                                out[dxl_id] = -1
                        if avail_cnt > 0:
                            self._last_mode = 'sync'     # ★ 群讀成功
                            return out
                    # 群讀這拍沒成功 → 落到 fallback
            except Exception:
                pass

            # ---- Fallback：逐顆單讀 ----
            out = {}
            for dxl_id in self.ids:
                try:
                    pos, cr, er = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, ADDR_PRESENT_POSITION)
                    out[dxl_id] = pos if (cr == COMM_SUCCESS and er == 0 and isinstance(pos, int)) else -1
                except Exception:
                    out[dxl_id] = -1
                time.sleep(0.0002)
            self._last_mode = 'single'                   # ★ 單讀
            return out

def open_buses(port_map: dict, baudrate=1_000_000):
    buses = {}
    for dev, ids in port_map.items():
        bus = DynamixelBus(dev, baudrate=baudrate, ids=ids)
        bus.enable_torque()
        bus.init_sync_read()   # ★ 這裡才建 GroupSyncRead 並 addParam
        buses[dev] = bus
    return buses

def close_buses(buses: dict):
    for dev, bus in (buses or {}).items():
        try:
            bus.close()   # 你的 bus 物件若不是 close()，改成對應的關閉方法
        except Exception:
            pass

def move_to_pose(buses: dict, port_map: dict, gp_target: dict,
                 pv_units: int = 600, pause_s: float = 0.3):
    # 若你手上是 ticks/s，先轉：
    # pv_units = max(1, int(ticks_per_sec * TPS_TO_UNITS_K))
    for dev, bus in buses.items():
        ids = port_map.get(dev, [])
        gp_map = {i: gp_target.get(i, 2048) for i in ids}
        pv_map = {i: max(1, pv_units) for i in ids}
        bus.write_pv_gp(pv_map, gp_map)
    time.sleep(pause_s)
