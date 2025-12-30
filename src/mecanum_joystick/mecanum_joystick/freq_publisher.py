import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import math

class MecanumJoyControl(Node):
    def __init__(self):
        super().__init__('mecanum_joy_control')
        
        # Publisher gửi lệnh xuống vi điều khiển (String: SET f1 d1...)
        self.publisher_ = self.create_publisher(String, 'freq_cmd', 10)
        
        # Subscriber nhận tín hiệu từ joy_node
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)

        # --- CẤU HÌNH ROBOT ---
        self.ppr = 5000       # số xung/vòng
        self.MAX_FREQ = 3000  # Tần số tối đa (Hz) - Tăng lên để chạy nhanh hơn nếu muốn
        self.R = 0.05         # Bán kính bánh xe (m)
        
        # Deadzone (Vùng chết): Nếu cần gạt nhích nhẹ dưới mức này sẽ coi là 0
        self.DEADZONE = 0.01 

        self.get_logger().info("Mecanum Joy Node Started (Matrix Corrected).")

    def val_to_freq_dir(self, val):
        """
        Chuyển đổi giá trị vận tốc tính toán sang Tần số và Hướng
        Dựa trên ma trận của bạn: 
        - Forward = 0000 => Giá trị Dương (>=0) là Dir 0
        - Backward = 1111 => Giá trị Âm (<0) là Dir 1
        """
        freq = abs(val)
        
        # Map giá trị (-1.0 đến 1.0) sang tần số (0 đến MAX_FREQ)
        # Vì công thức bên dưới cộng gộp vx, vy, wz nên giá trị có thể > 1.0
        # Ta nhân hệ số để scale lên tần số mong muốn
        scale = self.MAX_FREQ 
        target_freq = freq * scale
        
        # Clamp (kẹp) tần số không vượt quá MAX
        target_freq = min(target_freq, self.MAX_FREQ)
        
        # Xác định hướng: Dương là 0, Âm là 1
        direction = 0 if val >= 0 else 1
        
        return int(target_freq), direction

    def joy_callback(self, msg):
        # --- 1. MAPPING TAY CẦM ---
        # Joystick Trái: Axes 0 (Trái/Phải), Axes 1 (Lên/Xuống)
        # ROS Standard: 
        #   Axis 1: Lên = +1.0, Xuống = -1.0
        #   Axis 0: Trái = +1.0, Phải = -1.0
        
        raw_x = msg.axes[1] # Lên/Xuống
        raw_y = msg.axes[0] # Trái/Phải
        
        # Joystick Phải: Axes 3 (Trái/Phải), Axes 4 (Lên/Xuống)
        # Ta dùng Axis 3 để xoay
        raw_z = msg.axes[3] # Xoay Trái/Phải

        # --- 2. XỬ LÝ DEADZONE ---
        vx = raw_x if abs(raw_x) > self.DEADZONE else 0.0
        vy = raw_y if abs(raw_y) > self.DEADZONE else 0.0
        wz = raw_z if abs(raw_z) > self.DEADZONE else 0.0

        # --- 3. CHUẨN HÓA HƯỚNG ---
        # Muốn:
        # - Đẩy tới (raw_x > 0) -> vx > 0 (Forward)
        # - Gạt Phải (raw_y < 0) -> vy > 0 (Move Right) -> Cần đảo dấu Axis 0
        # - Gạt Xoay Phải (raw_z < 0) -> wz > 0 (Rotate Right) -> Cần đảo dấu Axis 3
        
        vx = vx                 # Giữ nguyên (Lên là dương)
        vy = -vy                # Đảo dấu (Để gạt sang phải là dương)
        wz = -wz                # Đảo dấu (Để gạt sang phải là dương - xoay phải)

        # --- 4. CÔNG THỨC MECANUM (DỰA TRÊN MA TRẬN CỦA BẠN) ---
        # Ma trận yêu cầu:
        # Move Right (vy>0): 0 1 1 0 => (+ - - +)
        # Rotate Right (wz>0): 0 0 1 1 => (+ + - -)
        
        # Công thức tổng hợp:
        v1 = vx + vy + wz  # Bánh 1 (Trước Trái): + + +
        v2 = vx - vy + wz  # Bánh 2 (Trước Phải): + - +
        v3 = vx - vy - wz  # Bánh 3 (Sau Phải)  : + - -
        v4 = vx + vy - wz  # Bánh 4 (Sau Trái)  : + + -

        # --- 5. CHUYỂN ĐỔI SANG FREQ & DIR ---
        f1, d1 = self.val_to_freq_dir(v1)
        f2, d2 = self.val_to_freq_dir(v2)
        f3, d3 = self.val_to_freq_dir(v3)
        f4, d4 = self.val_to_freq_dir(v4)

        # --- 6. GỬI LỆNH ---
        # Nếu tất cả joystick về 0, gửi lệnh dừng tuyệt đối (1Hz, Dir 0 để giữ torque hoặc 0Hz để thả trôi)
        if vx == 0 and vy == 0 and wz == 0:
            f1 = f2 = f3 = f4 = 0 # Hoặc để 1 nếu muốn giữ cứng bánh
            d1 = d2 = d3 = d4 = 0

        msg_str = String()
        msg_str.data = f"SET {f1} {d1} {f2} {d2} {f3} {d3} {f4} {d4}"
        self.publisher_.publish(msg_str)
        
        # Debug (Bật lên để kiểm tra nếu chạy sai)
        # self.get_logger().info(f"Vx:{vx:.2f} Vy:{vy:.2f} Wz:{wz:.2f} -> {msg_str.data}")

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