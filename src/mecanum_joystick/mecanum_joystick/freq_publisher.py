import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import math

class MecanumJoyControl(Node):
    def __init__(self):
        super().__init__('mecanum_joy_control')
        
        # --- 1. CẤU HÌNH GIAO TIẾP ---
        # Timer: Gửi lệnh cố định 20Hz (0.05s/lần) -> KHẮC PHỤC DELAY
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # Publisher gửi lệnh xuống vi điều khiển
        self.publisher_ = self.create_publisher(String, 'freq_cmd', 10)
        
        # Subscriber nhận tín hiệu từ tay cầm
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # --- 2. CẤU HÌNH ROBOT ---
        self.MAX_FREQ = 2500  # Tần số tối đa (Hz)
        self.DEADZONE = 0.05  # Vùng chết (chống trôi cần analog)
        
        # Biến lưu trạng thái tay cầm mới nhất
        self.current_joy_msg = None
        self.last_cmd_str = ""

        self.get_logger().info("Mecanum Joy Node Started. (Anti-Delay Mode)")

    def val_to_freq_dir(self, val):
        """
        Chuyển đổi giá trị tính toán (-1.0 đến 1.0) thành Frequency và Direction
        Quy ước: Giá trị Dương (>=0) -> Dir 0 | Giá trị Âm (<0) -> Dir 1
        """
        # Scale giá trị thành tần số
        freq = abs(val) * self.MAX_FREQ
        freq = min(freq, self.MAX_FREQ) # Giới hạn max
        
        # Xác định chiều (0 hoặc 1)
        direction = 0 if val >= 0 else 1
        
        return int(freq), direction

    def joy_callback(self, msg):
        """
        Callback này chỉ làm nhiệm vụ: LƯU TRẠNG THÁI MỚI NHẤT.
        Không tính toán hay gửi lệnh ở đây để tránh tắc nghẽn.
        """
        self.current_joy_msg = msg

    def timer_callback(self):
        """
        Hàm này chạy định kỳ 20Hz.
        Chỉ gửi lệnh xuống Robot khi lệnh có sự thay đổi so với lần trước.
        """
        if self.current_joy_msg is None:
            return

        msg = self.current_joy_msg

        # --- 3. ĐỌC DỮ LIỆU TAY CẦM ---
        raw_x = msg.axes[1] 
        raw_y = msg.axes[0]
        raw_z = msg.axes[3] if len(msg.axes) > 3 else 0.0

        # Áp dụng Deadzone
        vx = raw_x if abs(raw_x) > self.DEADZONE else 0.0
        vy = raw_y if abs(raw_y) > self.DEADZONE else 0.0
        wz = raw_z if abs(raw_z) > self.DEADZONE else 0.0

        # --- 4. CHUẨN HÓA DẤU ---
        vx = vx
        vy = -vy 
        wz = -wz 

        # --- 5. TÍNH TOÁN KINEMATICS ---
        v1 = vx + vy + wz  
        v2 = vx - vy + wz  
        v3 = vx - vy - wz  
        v4 = vx + vy - wz  

        # --- 6. CHUYỂN ĐỔI SANG LỆNH ---
        f1, d1 = self.val_to_freq_dir(v1)
        f2, d2 = self.val_to_freq_dir(v2)
        f3, d3 = self.val_to_freq_dir(v3)
        f4, d4 = self.val_to_freq_dir(v4)

        # Xử lý dừng tuyệt đối
        if vx == 0 and vy == 0 and wz == 0:
            f1 = f2 = f3 = f4 = 0
            d1 = d2 = d3 = d4 = 0

        # Tạo chuỗi lệnh
        cmd_str = f"SET {f1} {d1} {f2} {d2} {f3} {d3} {f4} {d4}"
        
        # --- [QUAN TRỌNG] LOGIC CHỐNG SPAM LỆNH TRÙNG ---
        if cmd_str != self.last_cmd_str:
            # Nếu lệnh KHÁC lệnh cũ -> Gửi đi
            msg_out = String()
            msg_out.data = cmd_str
            self.publisher_.publish(msg_out)
            
            # Cập nhật lại lệnh cũ
            self.last_cmd_str = cmd_str
            
            # Log ra để kiểm tra
            self.get_logger().info(f"Sent: {cmd_str}")
            
        else:
            # Nếu lệnh GIỐNG lệnh cũ (bao gồm cả 0 0 0 0 liên tiếp) -> Không làm gì cả
            pass

def main(args=None):
    rclpy.init(args=args)
    node = MecanumJoyControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()