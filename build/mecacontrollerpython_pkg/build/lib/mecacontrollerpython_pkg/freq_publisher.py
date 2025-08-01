import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import pygame
import time

class MecanumJoystickPublisher(Node):
    def __init__(self):
        super().__init__('mecanum_joystick_publisher')
        self.publisher_ = self.create_publisher(String, 'freq_cmd', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

        # Thông số robot
        self.L = 0.2      # khoảng cách tâm -> trục trước/sau (m)
        self.W = 0.15     # khoảng cách tâm -> trục trái/phải (m)
        self.R = 0.05     # bán kính bánh xe (m)
        self.ppr = 5000   # số xung mỗi vòng
        self.MAX_FREQ = 30000  # Tần số tối đa cho phép (Hz)

        # Khởi tạo joystick
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            print("[Error] Không tìm thấy tay cầm.")
            exit()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"[Info] Tay cầm: {self.joystick.get_name()}")

    def omega_to_freq_dir(self, omega):
        freq = abs(omega) * self.ppr / (2 * math.pi)
        freq = min(freq, self.MAX_FREQ)  # Giới hạn tần số
        direction = 0 if omega >= 0 else 1 
        return int(freq), direction

    def timer_callback(self):
        pygame.event.pump()

        axis_x = self.joystick.get_axis(0)
        axis_y = self.joystick.get_axis(1)
        axis_z = self.joystick.get_axis(3)

        vx = -axis_y
        vy = axis_x
        wz = axis_z

        # Thêm xoay tại tâm bằng nút L1 (4) và R1 (5)
        rotate_left = self.joystick.get_button(4)
        rotate_right = self.joystick.get_button(5)
        rotate_speed = 1.0  # bạn có thể điều chỉnh tốc độ xoay tại chỗ ở đây

        if rotate_left:
            vx = vy = 0.0
            wz = rotate_speed
        elif rotate_right:
            vx = vy = 0.0
            wz = -rotate_speed

        Lw = self.L + self.W
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

        # Nếu nằm trong vùng gần trung tâm (center deadzone) thì đặt về 1
        idle_freqs = [146, 21, 21, 102] #Calib idle frequencies
        offset = 15
        if all(abs(f - idle) <= offset for f, idle in zip([f1, f2, f3, f4], idle_freqs)):
            f1 = f2 = f3 = f4 = 1
            d1 = d2 = d3 = d4 = 0

        msg = String()
        msg.data = f"SET {f1} {d1} {f2} {d2} {f3} {d3} {f4} {d4}"
        self.publisher_.publish(msg)
        self.get_logger().info(f'[INFO] Xuất: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = MecanumJoystickPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[Info] Dừng node.")
    finally:
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
