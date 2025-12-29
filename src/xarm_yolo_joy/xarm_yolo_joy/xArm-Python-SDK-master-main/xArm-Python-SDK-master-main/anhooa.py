import socket
import struct
import math
from typing import Tuple, Union, Optional

# =========================
# CẤU HÌNH KẾT NỐI & HẰNG SỐ
# =========================
ROBOT_IP = "192.168.1.165"
ROBOT_PORT = 502

SOCK_TIMEOUT         = 5.0
SOCK_TIMEOUT_SHORT   = 8.0
SOCK_TIMEOUT_MOVE    = 35.0
SOCK_RECV_TIMEOUT    = 1.0

PROTO_MAGIC          = 0x0002

# Registers (theo tài liệu robot)
REG_CLEAR_ERROR      = 0x10
REG_CLEAR_WARNING    = 0x11
REG_ENABLE_JOINTS    = 0x0B
REG_READ_TCP_POSE    = 0x29
REG_MOVE_CARTESIAN   = 0x5C
REG_GRIPPER_RW       = 0x7F
REG_RETURN_ZERO_SPD  = 0x19

# Gripper constants
GRIPPER_HOST_ID      = 0x09
GRIPPER_ADDR_U16     = 0x0A15

SPEED_TCP_MAX = 1500.0  # mm/s, theo manual

# MBAP TID tăng dần
_TID = 1

# =========================
# TIỆN ÍCH GÓI TIN
# =========================
def to_be_u16(value: int) -> bytes:
    return struct.pack(">H", value)

def _next_tid():
    global _TID
    t = _TID
    _TID = (t + 1) & 0xFFFF
    if _TID == 0:
        _TID = 1
    return t

def recv_exact(sock: socket.socket, n: int) -> bytes:
    """Đọc đúng n byte, ném TimeoutError nếu thiếu."""
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise TimeoutError(f"Socket closed while reading ({len(buf)}/{n}).")
        buf.extend(chunk)
    return bytes(buf)

def create_packet(tid: int, register: int, params: bytes = b"") -> bytes:
    """Khung Lite6: [TID:2][Proto:2=0x0002][Length:2=1+len(params)][Reg:1][Params]"""
    length = 1 + len(params)
    return to_be_u16(tid) + to_be_u16(PROTO_MAGIC) + to_be_u16(length) + bytes([register]) + params

def parse_response_status(data: bytes):
    """
    Giải mã byte state trong phản hồi từ robot.
    Trả về danh sách các trạng thái (flags).
    """
    if not data or len(data) < 8:
        return ["❌ Resp invalid"]
    state = data[7]
    flags = []
    if (state >> 7) & 1: flags.append("❌ Lỗi nghiêm trọng")
    if (state >> 6) & 1: flags.append("⚠️ Cảnh báo")
    if (state >> 5) & 1: flags.append("ℹ️ Cảnh báo nhẹ")
    if (state >> 4) & 1: flags.append("⛔ Không thể chuyển động")
    if not flags:
        flags = ["✅ OK"]
    return flags

def parse_response(resp: bytes):
    """Trả về (reg, state_hex) hoặc lỗi."""
    if not resp or len(resp) < 8:
        raise ValueError("Phản hồi không hợp lệ hoặc quá ngắn")
    reg = resp[6]
    state = resp[7]
    return reg, f"0x{state:02X}"

# =========================
# CLIENT GIỮ KẾT NỐI
# =========================
class Lite6Client:
    """Quản lý 1 socket giữ kết nối để giảm latency. KHÔNG thread-safe."""
    def set_target(self, ip: str, port: int = 502):
        self.ip = ip.strip()
        self.port = int(port)
        self.close()

    def __init__(self, ip: str, port: int):
        self.ip = ip
        self.port = port
        self.sock: Optional[socket.socket] = None

    def _connect(self, timeout: float):
        self.close()
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        try:
            s.connect((self.ip, self.port))
        except Exception:
            s.close()
            raise
        s.settimeout(SOCK_RECV_TIMEOUT)
        self.sock = s

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def send_and_recv(self, packet: bytes, timeout: float) -> bytes:
        """Gửi packet và nhận phản hồi [header+body]. Tự reconnect 1 lần nếu rớt."""
        if self.sock is None:
            self._connect(timeout)
        try:
            self.sock.settimeout(timeout)
            self.sock.sendall(packet)
            header = recv_exact(self.sock, 6)
            tid, proto, length = struct.unpack(">HHH", header)
            if proto != PROTO_MAGIC:
                raise ValueError(f"Sai PROTO: 0x{proto:04X}")
            body = recv_exact(self.sock, length)
            return header + body
        except (TimeoutError, OSError, ValueError):
            self._connect(timeout)
            self.sock.settimeout(timeout)
            self.sock.sendall(packet)
            header = recv_exact(self.sock, 6)
            tid, proto, length = struct.unpack(">HHH", header)
            if proto != PROTO_MAGIC:
                raise ValueError(f"Sai PROTO: 0x{proto:04X}")
            body = recv_exact(self.sock, length)
            return header + body

# Singleton client cho module
_client = Lite6Client(ROBOT_IP, ROBOT_PORT)

# =========================
# API TƯƠNG THÍCH GUI   
# =========================
def send_command(packet: bytes, *, return_req: bool = False):
    """Gửi gói bytes đã build sẵn. Hỗ trợ trả (req, resp) khi return_req=True."""
    resp = _client.send_and_recv(packet, SOCK_TIMEOUT)
    return (packet, resp) if return_req else resp

# Clear/Enable
def cmd_clear_error(tid: Optional[int] = None) -> bytes:
    return create_packet(_next_tid() if tid is None else tid, REG_CLEAR_ERROR)

def cmd_clear_warning(tid: Optional[int] = None) -> bytes:
    return create_packet(_next_tid() if tid is None else tid, REG_CLEAR_WARNING)

def cmd_enable_all_joints(tid: Optional[int] = None) -> bytes:
    # 0x08 = all joints, 0x01 = enable
    params = b"\x08\x01"
    return create_packet(_next_tid() if tid is None else tid, REG_ENABLE_JOINTS, params=params)

# Gripper
def cmd_gripper(channel: int, open_on: bool, tid: Optional[int] = None) -> bytes:
    if channel not in (0, 1):
        raise ValueError("Gripper channel chỉ 0 hoặc 1.")
    # Theo tài liệu: mỗi kênh có mã riêng (float giá trị tác động)
    if channel == 0:
        value = 257.0 if open_on else 256.0
    else:
        value = 514.0 if open_on else 512.0
    params = bytes([GRIPPER_HOST_ID]) + to_be_u16(GRIPPER_ADDR_U16) + struct.pack("<f", value)
    return create_packet(_next_tid() if tid is None else tid, REG_GRIPPER_RW, params=params)

def open_gripper(channel=0, *, return_req: bool = False):
    pkt = cmd_gripper(channel=channel, open_on=True)
    resp = _client.send_and_recv(pkt, SOCK_TIMEOUT)
    return (pkt, resp) if return_req else resp

def close_gripper(channel=1, *, return_req: bool = False):
    # SỬA: close phải open_on=False
    pkt = cmd_gripper(channel=channel, open_on=True)
    resp = _client.send_and_recv(pkt, SOCK_TIMEOUT)
    return (pkt, resp) if return_req else resp

def stop_gripper(*, return_req: bool = False):
    """
    STOP cả 2 kênh (coi như CLOSE ngay lập tức).
    Trả về dict {channel: (req, resp)} nếu return_req=True,
    ngược lại {channel: resp_bytes}.
    """
    results = {}
    for ch in (0, 1):
        pkt = cmd_gripper(channel=ch, open_on=False)
        resp = _client.send_and_recv(pkt, SOCK_TIMEOUT)
        results[ch] = (pkt, resp) if return_req else resp
    return results

# Biến trạng thái TCP (giữ nguyên API)
CURR_X, CURR_Y, CURR_Z = 87.0, 0.0, 154.2
CURR_RX_RAD, CURR_RY_RAD, CURR_RZ_RAD = math.pi, 0.0, 0.0
TCP_SPEED_PERCENT = 100.0

def set_speed_percent(percent: float) -> str:
    """0..100%"""
    global TCP_SPEED_PERCENT
    TCP_SPEED_PERCENT = max(0.0, min(float(percent), 100.0))
    msg = f"🚀 Cập nhật tốc độ: {TCP_SPEED_PERCENT:.1f}%"
    print(msg)
    return msg

# Đọc TCP pose
def read_cartesian_position(tid: Optional[int] = None) -> Union[Tuple[float, float, float, float, float, float], str]:
    pkt = create_packet(_next_tid() if tid is None else tid, REG_READ_TCP_POSE)
    try:
        resp = _client.send_and_recv(pkt, SOCK_TIMEOUT_SHORT)
    except Exception as e:
        return f"❌ Lỗi gửi/nhận: {e}"

    if len(resp) < 32:
        return f"❌ Phản hồi quá ngắn ({len(resp)} byte)"

    reg = resp[6]
    if reg != REG_READ_TCP_POSE:
        return f"❌ Sai register: 0x{reg:02X} (kỳ vọng 0x{REG_READ_TCP_POSE:02X})"

    raw = resp[8:8+24]
    try:
        x, y, z, rx, ry, rz = struct.unpack(">6f", raw)  # BE
    except struct.error:
        return "❌ Lỗi giải mã dữ liệu (BE)"

    def sane(vals):
        x, y, z, rx, ry, rz = vals
        return (all(math.isfinite(v) for v in vals)
                and -2000 < x < 2000 and -2000 < y < 2000 and -2000 < z < 2000
                and -2*math.pi < rx < 2*math.pi and -2*math.pi < ry < 2*math.pi and -2*math.pi < rz < 2*math.pi)

    if not sane((x, y, z, rx, ry, rz)):
        try:
            x, y, z, rx, ry, rz = struct.unpack("<6f", raw)
        except struct.error:
            return "❌ Lỗi giải mã dữ liệu (LE)"

    return (x, y, z, rx, ry, rz)

# Move Cartesian (speed-based)
def _cmd_move_xyz_speed_based(
    x_mm: float, y_mm: float, z_mm: float,
    rx_rad: float, ry_rad: float, rz_rad: float,
    speed_mm_s: float, acc_mm_s2: float,
    coord_system: int = 0,      # 0: base
    absolute_pose: int = 0,     # 0: absolute
    tid: Optional[int] = None
) -> bytes:
    params = b"".join([
        struct.pack("<f", float(x_mm)),
        struct.pack("<f", float(y_mm)),
        struct.pack("<f", float(z_mm)),
        struct.pack("<f", float(rx_rad)),
        struct.pack("<f", float(ry_rad)),
        struct.pack("<f", float(rz_rad)),
        struct.pack("<f", float(speed_mm_s)),   # mm/s
        struct.pack("<f", float(acc_mm_s2)),    # mm/s^2
        struct.pack("<f", 0.0),                 # motion_time = 0  -> speed-based
        bytes([coord_system & 0xFF]),
        bytes([absolute_pose & 0xFF]),
    ])
    return create_packet(_next_tid() if tid is None else tid, REG_MOVE_CARTESIAN, params)

def move_xyz_offset_gui(dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, *, return_req: bool = False):
    global CURR_X, CURR_Y, CURR_Z
    CURR_X += float(dx); CURR_Y += float(dy); CURR_Z += float(dz)

    speed = SPEED_TCP_MAX * (TCP_SPEED_PERCENT / 100.0)
    if speed <= 0:
        return (b"", b"") if return_req else b""

    acc = min(50000.0, max(SPEED_TCP_MAX, speed * 200.0))
    pkt = _cmd_move_xyz_speed_based(CURR_X, CURR_Y, CURR_Z,
                                    CURR_RX_RAD, CURR_RY_RAD, CURR_RZ_RAD,
                                    speed, acc, 0, 0)
    resp = _client.send_and_recv(pkt, SOCK_TIMEOUT_MOVE)
    return (pkt, resp) if return_req else resp

def return_to_zero_from_speed(acc_scale: float = 120.0, motion_time_s: float = 0.0, *, return_req: bool = False):
    speed_mm_s  = 20.0
    speed_rad_s = speed_mm_s / 57.2958
    acc_rad_s2  = max(0.0, speed_rad_s * acc_scale)
    params = struct.pack("<fff", float(speed_rad_s), float(acc_rad_s2), float(motion_time_s))
    pkt = create_packet(_next_tid(), REG_RETURN_ZERO_SPD, params)
    resp = _client.send_and_recv(pkt, SOCK_TIMEOUT_MOVE)
    # demo cập nhật pose local về 0 — nếu muốn giữ nguyên, bỏ 3 dòng sau
    global CURR_X, CURR_Y, CURR_Z
    CURR_X, CURR_Y, CURR_Z = 0.0, 0.0, 0.0
    return (pkt, resp) if return_req else resp


# =========================
# DEMO
# =========================
def demo():
    print("=== Bắt đầu điều khiển Lite6 ===")
    for name, cmd_func in [
        ("Xóa lỗi", cmd_clear_error),
        ("Xóa cảnh báo", cmd_clear_warning),
        ("Bật động cơ (enable all joints)", cmd_enable_all_joints),
    ]:
        print(f"\n--- {name} ---")
        pkt = cmd_func()
        pkt_hex = " ".join(f"{b:02X}" for b in pkt)
        print("REQ:", pkt_hex)
        req, resp = send_command(pkt, return_req=True)
        print("RESP:", parse_response(resp))

    

if __name__ == "__main__":
    demo()
