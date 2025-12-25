import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import math

class MecanumJoyNode(Node):
    def __init__(self):
        super().__init__('mecanum_joy_node')

        # ===== ROS =====
        self.sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.pub = self.create_publisher(String, 'freq_cmd', 10)
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        # ===== ROBOT PARAM =====
        self.L = 0.2
        self.W = 0.15
        self.R = 0.05
        self.ppr = 5000
        self.MAX_FREQ = 10000

        # ===== CONTROL PARAM =====
        self.deadzone = 0.2

        self.max_v = 1.0        # m/s
        self.max_wz = 2.0       # rad/s
        self.ramp_rate = 2.5    # m/s^2 or rad/s^2

        # ===== STATES =====
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.get_logger().info("Mecanum Joy Node ready. Lần này là bản tử tế.")

    # ======================
    # JOYSTICK CALLBACK
    # ======================
    def joy_cb(self, msg: Joy):
        ax_x = msg.axes[0]  # trái / phải
        ax_y = msg.axes[1]  # tiến / lùi
        ax_r = msg.axes[3]  # xoay

        # ----- VX -----
        if ax_y > self.deadzone:
            self.target_vx = self.max_v
        elif ax_y < -self.deadzone:
            self.target_vx = -self.max_v
        else:
            self.target_vx = 0.0

        # ----- VY -----
        if ax_x > self.deadzone:
            self.target_vy = self.max_v       # sang trái
        elif ax_x < -self.deadzone:
            self.target_vy = -self.max_v      # sang phải
        else:
            self.target_vy = 0.0

        # ----- WZ -----
        if ax_r > self.deadzone:
            self.target_wz = self.max_wz       # xoay trái
        elif ax_r < -self.deadzone:
            self.target_wz = -self.max_wz      # xoay phải
        else:
            self.target_wz = 0.0

    # ======================
    # RAMP FUNCTION
    # ======================
    def ramp(self, current, target, dt):
        delta = self.ramp_rate * dt
        if current < target:
            return min(current + delta, target)
        elif current > target:
            return max(current - delta, target)
        return current

    # ======================
    # OMEGA → FREQ
    # ======================
    def omega_to_freq_dir(self, omega):
        freq = abs(omega) * self.ppr / (2 * math.pi)
        freq = min(freq, self.MAX_FREQ)
        direction = 0 if omega >= 0 else 1
        return int(freq), direction

    # ======================
    # MAIN CONTROL LOOP
    # ======================
    def control_loop(self):
        dt = 0.05

        # Ramp mượt
        self.vx = self.ramp(self.vx, self.target_vx, dt)
        self.vy = self.ramp(self.vy, self.target_vy, dt)
        self.wz = self.ramp(self.wz, self.target_wz, dt)

        Lw = self.L + self.W

        # Mecanum kinematics
        v1 = (self.vx - self.vy - Lw * self.wz)
        v2 = (self.vx + self.vy + Lw * self.wz)
        v3 = (self.vx + self.vy - Lw * self.wz)
        v4 = (self.vx - self.vy + Lw * self.wz)

        omegas = [v1/self.R, v2/self.R, v3/self.R, v4/self.R]
        freqs_dirs = [self.omega_to_freq_dir(o) for o in omegas]

        msg = String()
        msg.data = "SET " + " ".join(f"{f} {d}" for f, d in freqs_dirs)
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = MecanumJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

