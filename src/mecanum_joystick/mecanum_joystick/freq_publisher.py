import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import math

class MecanumJoyControl(Node):
    def __init__(self):
        super().__init__('mecanum_joy_control')
        
        # Publisher gửi lệnh xuống vi điều khiển
        self.publisher_ = self.create_publisher(String, 'freq_cmd', 10)
        
        # Subscriber nhận tín hiệu từ joy_node
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)

        # Thông số robot
        self.L = 0.2      # khoảng cách tâm -> trục trước/sau (m)
        self.W = 0.15     # khoảng cách tâm -> trục trái/phải (m)
        self.R = 0.05     # bán kính bánh xe (m)
        self.ppr = 5000   # số xung mỗi vòng
        self.MAX_FREQ = 7000  # Tần số tối đa (Hz)
        
        # Tốc độ xoay tại chỗ
        self.rotate_speed_fixed = 1.0

        self.get_logger().info("Mecanum Joy Node Started. Waiting for /joy messages...")

    def omega_to_freq_dir(self, omega):
        freq = abs(omega) * self.ppr / (2 * math.pi)
        freq = min(freq, self.MAX_FREQ)
        direction = 1 if omega >= 0 else 0
        return int(freq), direction

    def joy_callback(self, msg):
        # --- Mapping Tay cầm (ROS standard mapping) ---
        # msg.axes thường là: [Left-LR, Left-UD, L2, Right-LR, Right-UD, R2, Dpad-LR, Dpad-UD]
        # Lưu ý: Trong ROS Joy, đẩy cần lên (Up) thường là +1.0, Trái (Left) là +1.0
        
        # Axis 1: Left Stick Up/Down (Forward/Backward)
        axis_x_linear = msg.axes[1] 
        
        # Axis 0: Left Stick Left/Right (Strafe)
        # Pygame cũ: Right=+1. ROS: Left=+1. Nên ta đảo dấu để Right là dương (nếu muốn)
        # Tuy nhiên, chuẩn Robot: Y dương là sang trái. Giữ nguyên msg.axes[0] là sang trái.
        axis_y_linear = msg.axes[0] 

        # Axis 3: Right Stick Left/Right (Rotation)
        axis_z_angular = msg.axes[3] if len(msg.axes) > 3 else 0.0

        # Gán vận tốc
        vx = axis_x_linear      # Đi tới
        vy = axis_y_linear      # Đi ngang (trái)
        wz = axis_z_angular     # Xoay (trái)

        # --- Xử lý nút bấm L1/R1 để xoay tại chỗ ---
        # Mapping nút thường: msg.buttons[4] là L1, msg.buttons[5] là R1
        # Kiểm tra index tránh lỗi index out of range
        rotate_left = msg.buttons[4] if len(msg.buttons) > 4 else 0
        rotate_right = msg.buttons[5] if len(msg.buttons) > 5 else 0

        if rotate_left:
            vx = vy = 0.0
            wz = self.rotate_speed_fixed
        elif rotate_right:
            vx = vy = 0.0
            wz = -self.rotate_speed_fixed

        # --- Tính toán độ học Mecanum ---
        # Quy ước bánh xe:
        # 1: Trước Trái, 2: Trước Phải, 3: Sau Phải, 4: Sau Trái (Theo thứ tự code cũ của bạn)
        # Công thức Mecanum cơ bản (có thể cần đảo dấu tùy cách lắp động cơ thực tế)
        Lw = self.L + self.W
        
        # Lưu ý: Code cũ: v1 = vx - vy - Lw*wz.
        # Nếu vx dương (tới), vy dương (trái), wz dương (xoay trái)
        v1 = (vx - vy - Lw * wz)
        v2 = (vx + vy + Lw * wz)
        v3 = (vx + vy - Lw * wz)
        v4 = (vx - vy + Lw * wz)

        omega1 = v1 / self.R
        omega2 = v2 / self.R
        omega3 = v3 / self.R
        omega4 = v4 / self.R

        f1, d1 = self.omega_to_freq_dir(omega1)
        f2, d2 = self.omega_to_freq_dir(omega2)
        f3, d3 = self.omega_to_freq_dir(omega3)
        f4, d4 = self.omega_to_freq_dir(omega4)

        # --- Xử lý Deadzone / Idle (Logic từ code cũ) ---
        idle_freqs = [146, 21, 21, 102] 
        offset = 15
        
        # Kiểm tra nếu cần gạt về 0 (hoặc gần 0) thì áp dụng logic idle
        # Ta kiểm tra đầu vào joy gần 0 thay vì output freq để chính xác hơn, 
        # nhưng để tôn trọng logic phần cứng cũ của bạn, tôi giữ nguyên logic so sánh freq.
        if all(abs(f - idle) <= offset for f, idle in zip([f1, f2, f3, f4], idle_freqs)):
            f1 = f2 = f3 = f4 = 1
            d1 = d2 = d3 = d4 = 0
        
        # Nếu tay cầm không bấm gì (tất cả axes/buttons = 0), force stop để an toàn
        if vx == 0 and vy == 0 and wz == 0:
             # Logic idle cũ của bạn có thể vẫn trả về tần số nhiễu (146, 21...), 
             # đoạn code trên đã xử lý việc gán về 1.
             pass 

        msg_str = String()
        msg_str.data = f"SET {f1} {d1} {f2} {d2} {f3} {d3} {f4} {d4}"
        self.publisher_.publish(msg_str)
        # self.get_logger().info(f'[Xuất]: {msg_str.data}') # Uncomment nếu muốn debug

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