
import socket
import struct
import time
import math
from typing import Tuple, Optional

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

# =========================
# TRẠNG THÁI TOÀN CỤC
# =========================
TCP_SPEED_PERCENT = 20  # 0..100
CURR_X, CURR_Y, CURR_Z = 87.0, 0.0, 154.2
CURR_RX_RAD, CURR_RY_RAD, CURR_RZ_RAD = math.pi, 0.0, 0.0   # 180°, 0°, 0°

# =========================
# TIỆN ÍCH
# =========================
def to_be_u16(v: int) -> bytes:
    return struct.pack(">H", v)

def to_be_s16(v: int) -> bytes:
    return struct.pack(">h", v)

def to_be_u32(v: int) -> bytes:
    return struct.pack(">I", v)

def to_be_f32(v: float) -> bytes:
    return struct.pack(">f", float(v))

def recv_exact(sock: socket.socket, n: int) -> bytes:
    data = bytearray()
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            raise TimeoutError("Socket closed while receiving")
        data.extend(chunk)
    return bytes(data)

# =========================
# GIAO THỨC LITE6
# =========================
def create_lite6_packet(tid: int, register: int, params: bytes = b"") -> bytes:
    """
    Khung Lite6:
    [TID:2][Proto:2=0x0002][Length:2 = 1 + len(params)][Register:1][Params:n]
    """
    length = 1 + len(params)
    pkt = (
        to_be_u16(tid) +
        to_be_u16(PROTO_MAGIC) +
        to_be_u16(length) +
        bytes([register]) +
        params
    )
    return pkt

class Lite6Client:
    def __init__(self, ip: str, port: int):
        self.ip = ip
        self.port = port
        self.sock: Optional[socket.socket] = None
        self.last_req: bytes = b""
        self.last_resp: bytes = b""

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = None

    def _connect(self, timeout: float):
        self.close()
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect((self.ip, self.port))
        self.sock = s

    def set_target(self, ip: str, port: int = 502):
        self.ip = ip.strip()
        self.port = int(port)
        self.close()

    def get_last_io_hex(self) -> Tuple[str, str]:
        def hx(b: bytes):
            return " ".join(f"{x:02X}" for x in b[:1024])
        return hx(self.last_req), hx(self.last_resp)

    def send_and_recv(self, packet: bytes, timeout: float) -> bytes:
        if self.sock is None:
            self._connect(timeout)
        try:
            self.last_req = packet
            self.sock.settimeout(timeout)
            self.sock.sendall(packet)
            header = recv_exact(self.sock, 6)
            tid, proto, length = struct.unpack(">HHH", header)
            if proto != PROTO_MAGIC:
                raise ValueError(f"Sai PROTO: 0x{proto:04X}")
            body = recv_exact(self.sock, length)
            self.last_resp = header + body
            return header + body
        except (TimeoutError, OSError, ValueError):
            # thử reconnect 1 lần
            self._connect(timeout)
            self.last_req = packet
            self.sock.sendall(packet)
            header = recv_exact(self.sock, 6)
            tid, proto, length = struct.unpack(">HHH", header)
            if proto != PROTO_MAGIC:
                raise ValueError(f"Sai PROTO: 0x{proto:04X}")
            body = recv_exact(self.sock, length)
            self.last_resp = header + body
            return header + body

# client mặc định
_client = Lite6Client(ROBOT_IP, ROBOT_PORT)

# =========================
# API NGOÀI: TARGET & LOG IO
# =========================
def set_robot_target(ip: str, port: int = 502) -> str:
    _client.set_target(ip, port)
    return f"🔌 Target: {ip}:{port}"

def get_last_io_hex() -> Tuple[str, str]:
    return _client.get_last_io_hex()

# =========================
# CÂU LỆNH CHUẨN
# =========================
def send_command(packet: bytes, timeout: float = SOCK_TIMEOUT_SHORT) -> bytes:
    return _client.send_and_recv(packet, timeout)

def parse_response(resp: bytes) -> str:
    # Tối thiểu phải có 7 byte (6 header + 1 register/state)
    if len(resp) < 7:
        return "❌ Phản hồi quá ngắn"
    # Bỏ qua 6 byte header
    body = resp[6:]
    # byte đầu là register echo
    if len(body) == 1:
        return f"✅ ACK reg=0x{body[0]:02X}"
    else:
        return f"✅ {len(body)}B (reg=0x{body[0]:02X})"

def cmd_clear_error(tid: int = 11) -> bytes:
    return create_lite6_packet(tid, REG_CLEAR_ERROR)

def cmd_clear_warning(tid: int = 12) -> bytes:
    return create_lite6_packet(tid, REG_CLEAR_WARNING)

def cmd_enable_all_joints(tid: int = 13) -> bytes:
    return create_lite6_packet(tid, REG_ENABLE_JOINTS)

def cmd_read_tcp_pose(tid: int = 41) -> bytes:
    return create_lite6_packet(tid, REG_READ_TCP_POSE)

def cmd_move_cartesian(params: bytes, tid: int = 61) -> bytes:
    return create_lite6_packet(tid, REG_MOVE_CARTESIAN, params)

def cmd_return_zero_from_speed(speed: float, tid: int = 71) -> bytes:
    # speed mm/s -> f32
    p = to_be_f32(speed)
    return create_lite6_packet(tid, REG_RETURN_ZERO_SPD, p)

# =========================
# GRIPPER (đơn & safe)
# =========================
def _pack_gripper_rw(channel: int, open_on: bool) -> bytes:
    """
    Gói RW gripper tối giản theo host/id/addr + payload 0/1 (u16)
    params: [host:1][id_hi:1 id_lo:1][addr_hi:1 addr_lo:1][chan:1][val:2]
    """
    val = 1 if open_on else 0
    return bytes([GRIPPER_HOST_ID, 0x00, 0x00]) + to_be_u16(GRIPPER_ADDR_U16) + bytes([channel & 0x01]) + to_be_u16(val)

def cmd_gripper(channel: int = 0, open_on: bool = True, tid: int = 81) -> bytes:
    return create_lite6_packet(tid, REG_GRIPPER_RW, _pack_gripper_rw(channel, open_on))

def open_gripper(channel: int = 0, tid: int = 82) -> str:
    pkt = cmd_gripper(channel=channel, open_on=True, tid=tid)
    resp = send_command(pkt, SOCK_TIMEOUT_SHORT)
    return parse_response(resp)

def close_gripper(channel: int = 0, tid: int = 83) -> str:
    pkt = cmd_gripper(channel=channel, open_on=False, tid=tid)
    resp = send_command(pkt, SOCK_TIMEOUT_SHORT)
    return parse_response(resp)

def stop_gripper(tid: int = 84) -> str:
    # Một số dòng chỉ cần gửi lại với giá trị giữ nguyên/idle. Ở đây gửi close (=0) coi như stop.
    pkt = cmd_gripper(channel=0, open_on=False, tid=tid)
    resp = send_command(pkt, SOCK_TIMEOUT_SHORT)
    return parse_response(resp)

# State machine an toàn
_gripper_state = "idle"  # idle | opening | open | closing | closed

def gripper_state() -> str:
    return _gripper_state

def open_gripper_safe(channel: int = 0, tid: int = 112) -> str:
    global _gripper_state
    if _gripper_state in ("opening", "open"):
        return "⛔ Đang mở/đã mở – bỏ qua"
    _gripper_state = "opening"
    try:
        res = open_gripper(channel=channel, tid=tid)
        _gripper_state = "open" if "✅" in res else "idle"
        return f"{res} | state={_gripper_state}"
    except Exception as e:
        _gripper_state = "idle"
        return f"❌ Lỗi open: {e}"

def close_gripper_safe(channel: int = 0, tid: int = 113) -> str:
    global _gripper_state
    if _gripper_state in ("closing", "closed"):
        return "⛔ Đang đóng/đã đóng – bỏ qua"
    _gripper_state = "closing"
    try:
        res = close_gripper(channel=channel, tid=tid)
        _gripper_state = "closed" if "✅" in res else "idle"
        return f"{res} | state={_gripper_state}"
    except Exception as e:
        _gripper_state = "idle"
        return f"❌ Lỗi close: {e}"

def stop_gripper_safe() -> str:
    global _gripper_state
    try:
        res = stop_gripper()
        _gripper_state = "idle"
        return f"{res} | state={_gripper_state}"
    except Exception as e:
        _gripper_state = "idle"
        return f"❌ Lỗi stop: {e}"

# =========================
# MOVE CARTESIAN
# =========================
def _cmd_move_xyz_speed_based(
    x_mm: float, y_mm: float, z_mm: float,
    rx_rad: float, ry_rad: float, rz_rad: float,
    speed_mm_s: float, acc_mm_s2: float,
    coord_system: int = 0, absolute_pose: int = 0, tid: int = 61
) -> bytes:
    """
    params layout (giả định theo tài liệu mẫu):
    [x:f32][y:f32][z:f32][rx:f32][ry:f32][rz:f32][speed:f32][acc:f32][coord:u16][abs:u16]
    """
    p = (
        to_be_f32(x_mm) + to_be_f32(y_mm) + to_be_f32(z_mm) +
        to_be_f32(rx_rad) + to_be_f32(ry_rad) + to_be_f32(rz_rad) +
        to_be_f32(speed_mm_s) + to_be_f32(acc_mm_s2) +
        to_be_u16(coord_system & 0xFFFF) + to_be_u16(absolute_pose & 0xFFFF)
    )
    return create_lite6_packet(tid, REG_MOVE_CARTESIAN, p)

def move_xyz_offset_gui(dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> str:
    global CURR_X, CURR_Y, CURR_Z, CURR_RX_RAD, CURR_RY_RAD, CURR_RZ_RAD

    CURR_X += dx
    CURR_Y += dy
    CURR_Z += dz

    speed = 500.0 * (TCP_SPEED_PERCENT / 100.0)
    if speed <= 0:
        return "⚠️ Tốc độ = 0% → bỏ qua lệnh move"

    acc = min(50000.0, max(500.0, speed * 200.0))
    pkt = _cmd_move_xyz_speed_based(
        x_mm=CURR_X, y_mm=CURR_Y, z_mm=CURR_Z,
        rx_rad=CURR_RX_RAD, ry_rad=CURR_RY_RAD, rz_rad=CURR_RZ_RAD,
        speed_mm_s=speed, acc_mm_s2=acc, coord_system=0, absolute_pose=0, tid=95
    )
    try:
        resp = send_command(pkt, SOCK_TIMEOUT_MOVE)
        return parse_response(resp)
    except Exception as e:
        return f"❌ Lỗi move: {e}"

def set_speed_percent(val: int) -> str:
    global TCP_SPEED_PERCENT
    TCP_SPEED_PERCENT = max(0, min(100, int(val)))
    return f"🚀 TCP speed: {TCP_SPEED_PERCENT}%"

def return_to_zero_from_speed(tid: int = 72) -> str:
    speed = 500.0 * (TCP_SPEED_PERCENT / 100.0)
    if speed <= 0:
        return "⚠️ Tốc độ = 0% → bỏ qua lệnh zero"
    pkt = cmd_return_zero_from_speed(speed, tid=tid)
    try:
        resp = send_command(pkt, SOCK_TIMEOUT_MOVE)
        return parse_response(resp)
    except Exception as e:
        return f"❌ Lỗi zero: {e}"

# =========================
# ĐỌC POSE
# =========================
def read_cartesian_position(tid: int = 41):
    """
    Đọc tọa độ Cartesian từ robot (Register 0x29)
    Trả về (x, y, z, rx, ry, rz) hoặc str lỗi
    """
    try:
        pkt = cmd_read_tcp_pose(tid=tid)
        resp = send_command(pkt, SOCK_TIMEOUT_SHORT)

        if len(resp) < 8 + 1 + 6 * 4:
            return "❌ Phản hồi quá ngắn"

        data = resp[8:]
        x, y, z, rx, ry, rz = struct.unpack(">6f", data[:24])
        return (x, y, z, rx, ry, rz)
    except Exception as e:
        return f"❌ Lỗi đọc pose: {e}"
