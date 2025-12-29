import socket
import struct
import time
import math

# ================== Cấu hình ==================
ROBOT_IP = "192.168.1.165"
ROBOT_PORT = 502

SOCK_TIMEOUT_SHORT = 8.0
SOCK_TIMEOUT_MOVE  = 35.0

# ================== Giới hạn (theo manual) ==================
TCP_SPEED_MAX_MM_S = 500.0      # 0..500 mm/s
TCP_ACC_MAX_MM_S2  = 50000.0    # 0..50000 mm/s^2

# ================== Tọa độ/định hướng mặc định (theo ảnh) ==================
CURR_X, CURR_Y, CURR_Z = 87.0, 0.0, 154.2
CURR_RX_RAD, CURR_RY_RAD, CURR_RZ_RAD = math.pi, 0.0, 0.0   # 180°, 0°, 0°

# ================== Tiện ích ==================
def to_be_u16(v: int) -> bytes:
    return struct.pack(">H", v)

def recv_all(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            break
        buf.extend(chunk)
    return bytes(buf)

def create_packet(tid: int, register: int, params: bytes = b"") -> bytes:
    proto = 0x0002
    length = 1 + len(params)
    return to_be_u16(tid) + to_be_u16(proto) + to_be_u16(length) + bytes([register]) + params

def send_cmd(packet: bytes, timeout: float) -> bytes:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(timeout)
        s.connect((ROBOT_IP, ROBOT_PORT))
        s.sendall(packet)
        
        # Nhận header (6 byte)
        header = recv_all(s, 6)
        if len(header) != 6:
            raise TimeoutError("Không nhận đủ 6 byte header.")
        
        # Giải mã thông tin từ header
        _, _, length = struct.unpack(">HHH", header)
        
        # Nhận body
        body = recv_all(s, length)
        if len(body) != length:
            raise TimeoutError(f"Không nhận đủ body ({len(body)}/{length}).")
        
        # Xử lý phản hồi
        req_b = header + packet  # Gửi gói lệnh đi, lấy lại `header` và `packet` để làm `req_b`
        resp_b = header + body   # Gọi body để lấy `resp_b`

        # Debug: In ra request và response
        print(f"Request: {req_b}")
        print(f"Response: {resp_b}")
        
        return req_b, resp_b


def parse_resp(data: bytes) -> str:
    if len(data) < 8:
        return "Phản hồi không hợp lệ"
    reg = data[6]
    state = data[7]
    flags = []
    if (state >> 6) & 1: flags.append("❌ Lỗi")
    if (state >> 5) & 1: flags.append("⚠️ Cảnh báo")
    if (state >> 4) & 1: flags.append("⛔ Không thể chuyển động")
    if not flags: flags = ["✅ OK"]
    extra = ""
    if len(data) >= 10:
        param = struct.unpack(">H", data[8:10])[0]
        extra = f", Param: 0x{param:04X}"
    return f"Reg 0x{reg:02X} | State 0x{state:02X} ({', '.join(flags)}){extra}"

# ================== Lệnh hệ thống ==================
def cmd_clear_error(tid: int = 1) -> bytes:   # Reg 0x10
    return create_packet(tid, 0x10)

def cmd_clear_warning(tid: int = 2) -> bytes: # Reg 0x11
    return create_packet(tid, 0x11)

def cmd_enable_all(tid: int = 3) -> bytes:    # Reg 0x0B
    return create_packet(tid, 0x0B, b"\x08\x01")  # all joints, enable

def cmd_enter_motion_mode(tid: int = 4) -> bytes: # Reg 0x0C
    return create_packet(tid, 0x0C, b"\x00")      # 0: enter motion mode

# ================== Cartesian Move (Reg 0x5C) ==================
def cmd_move_xyz_speed_based(
    x_mm: float, y_mm: float, z_mm: float,
    rx_rad: float, ry_rad: float, rz_rad: float,
    speed_mm_s: float, acc_mm_s2: float,
    coord_system: int = 0,      # 0: base
    absolute_pose: int = 0,     # 0: absolute
    tid: int = 60
) -> bytes:
    params = b"".join([
        struct.pack("<f", float(x_mm)),
        struct.pack("<f", float(y_mm)),
        struct.pack("<f", float(z_mm)),
        struct.pack("<f", float(rx_rad)),
        struct.pack("<f", float(ry_rad)),
        struct.pack("<f", float(rz_rad)),
        struct.pack("<f", float(speed_mm_s)),     # mm/s
        struct.pack("<f", float(acc_mm_s2)),      # mm/s^2
        struct.pack("<f", 0.0),                   # motion_time = 0  -> speed-based
        bytes([coord_system & 0xFF]),
        bytes([absolute_pose & 0xFF]),
    ])
    return create_packet(tid, 0x5C, params)



# ================== Luồng chính ==================
def main():
    print("=== Lite6 | Move XYZ (speed-based, nhập % tốc độ) ===")
    # 1) Clear lỗi/cảnh báo, enable, vào motion mode
    for name, fn in [("Clear Error", cmd_clear_error),
                     ("Clear Warning", cmd_clear_warning),
                     ("Enable All Joints", cmd_enable_all),
                     ("Enter Motion Mode", cmd_enter_motion_mode)]:
        try:
            resp = send_cmd(fn(), SOCK_TIMEOUT_SHORT)
            print(f"{name}: {parse_resp(resp)}")
        except Exception as e:
            print(f"{name}: lỗi -> {e}")
            if name in ("Enable All Joints", "Enter Motion Mode"):
                return
        time.sleep(0.2)

    # 2) Nhập tọa độ đích (giữ mặc định nếu để trống)
    def ask_float(prompt: str, default: float) -> float:
        s = input(f"{prompt} [{default}]: ").strip()
        return default if s == "" else float(s)

    print("\nNhập tọa độ đích (mm). Bỏ trống để giữ mặc định:")
    target_x = ask_float("X (mm)", CURR_X)
    target_y = ask_float("Y (mm)", CURR_Y)
    target_z = ask_float("Z (mm)", CURR_Z)
    speed = 200
    acc = 50000

    print(f"\n→ Đi tới: X={target_x}  Y={target_y}  Z={target_z} (mm)")
    print(f"→ Giữ orientation: Rx=180°, Ry=0°, Rz=0°")
    print(f"→ Speed={speed:.1f} mm/s , Acc={acc:.1f} mm/s²")

    # 4) Gửi lệnh move (absolute, base frame)
    try:
        pkt = cmd_move_xyz_speed_based(
            x_mm=target_x, y_mm=target_y, z_mm=target_z,
            rx_rad=CURR_RX_RAD, ry_rad=CURR_RY_RAD, rz_rad=CURR_RZ_RAD,
            speed_mm_s=speed, acc_mm_s2=acc,
            coord_system=0, absolute_pose=0, tid=71
        )
        resp = send_cmd(pkt, SOCK_TIMEOUT_MOVE)
        print("Move:", parse_resp(resp))
    except Exception as e:
        print("Move: lỗi ->", e)

if __name__ == "__main__":
    main()
