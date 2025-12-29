#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import math
import time


class JoyDynamicsMecanum(Node):
    def __init__(self):
        super().__init__('joy_dynamics_mecanum')

        # ================= ROS =================
        self.sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.pub = self.create_publisher(String, 'freq_cmd', 10)

        # dynamics chạy nhanh
        self.dyn_timer = self.create_timer(0.02, self.dynamics_cb)   # 50 Hz
        # publish chậm
        self.pub_timer = self.create_timer(0.1, self.publish_cb)    # 10 Hz

        # ================= ROBOT PARAM =================
        self.L = 0.20
        self.W = 0.15
        self.R = 0.05
        self.PPR = 5000
        self.MAX_FREQ = 5000

        # ================= JOYSTICK =================
        self.deadzone = 0.2
        self.max_v = 1.0
        self.max_wz = 2.0

        # ================= STEP CONTROL =================
        self.min_step = 0.005
        self.max_step = 0.05
        self.step_gain = 0.005

        # ================= MECANUM FIX =================
        # Thứ tự: FL, FR, RL, RR
        self.wheel_sign = [+1, +1, -1, -1]

        # ================= STATE =================
        self.joy = None

        self.prev_ax = 0.0
        self.prev_ay = 0.0
        self.prev_ar = 0.0
        self.prev_time = time.time()

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.last_set = None
        self.stop_sent = False

        self.get_logger().info("Joy dynamics + mecanum + 10Hz publish (FINAL)")

    # ------------------------------------------------
    def joy_cb(self, msg: Joy):
        self.joy = msg

    # ------------------------------------------------
    def apply_deadzone(self, val):
        if abs(val) < self.deadzone:
            return 0.0
        return (abs(val) - self.deadzone) / (1.0 - self.deadzone) * math.copysign(1.0, val)

    # ------------------------------------------------
    def smooth_update(self, cur, target, d_axis):
        step = abs(d_axis) * self.step_gain
        step = max(self.min_step, min(self.max_step, step))

        diff = target - cur
        if abs(diff) <= step:
            return target
        return cur + step * math.copysign(1.0, diff)

    # ------------------------------------------------
    # 50 Hz – chỉ làm dynamics
    def dynamics_cb(self):
        if self.joy is None:
            return

        now = time.time()
        dt = now - self.prev_time
        if dt <= 0.0:
            return

        ax = self.joy.axes[0]
        ay = self.joy.axes[1]
        ar = self.joy.axes[3]

        dx = (ax - self.prev_ax) / dt
        dy = (ay - self.prev_ay) / dt
        dr = (ar - self.prev_ar) / dt

        self.prev_ax = ax
        self.prev_ay = ay
        self.prev_ar = ar
        self.prev_time = now

        dz_x = self.apply_deadzone(ax)
        dz_y = self.apply_deadzone(ay)
        dz_r = self.apply_deadzone(ar)

        target_vx = dz_y * self.max_v
        target_vy = dz_x * self.max_v
        target_wz = dz_r * self.max_wz

        self.vx = self.smooth_update(self.vx, target_vx, dy)
        self.vy = self.smooth_update(self.vy, target_vy, dx)
        self.wz = self.smooth_update(self.wz, target_wz, dr)

    # ------------------------------------------------
    # 10 Hz – chỉ publish nếu CẦN
    def publish_cb(self):
        # ===== STOP =====
        if self.vx == 0.0 and self.vy == 0.0 and self.wz == 0.0:
            if not self.stop_sent:
                self.send_set([(0, 0)] * 4)
                self.stop_sent = True
            return
        else:
            self.stop_sent = False

        # ===== MECANUM =====
        Lw = self.L + self.W

        omega = [
            ( self.vx - self.vy - Lw * self.wz) / self.R,
            ( self.vx + self.vy + Lw * self.wz) / self.R,
            ( self.vx + self.vy - Lw * self.wz) / self.R,
            ( self.vx - self.vy + Lw * self.wz) / self.R
        ]

        for i in range(4):
            omega[i] *= self.wheel_sign[i]

        freqs = []
        for w in omega:
            freq = int(min(abs(w) * self.PPR / (2 * math.pi), self.MAX_FREQ))
            dir_ = 0 if w >= 0 else 1
            freqs.append((freq, dir_))

        set_data = "SET " + " ".join(f"{f} {d}" for f, d in freqs)

        # ===== ANTI-SPAM =====
        if set_data == self.last_set:
            return

        msg = String()
        msg.data = set_data
        self.pub.publish(msg)
        self.last_set = set_data

    # ------------------------------------------------
    def send_set(self, freqs):
        msg = String()
        msg.data = "SET " + " ".join(f"{f} {d}" for f, d in freqs)
        self.pub.publish(msg)
        self.last_set = msg.data


def main():
    rclpy.init()
    node = JoyDynamicsMecanum()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

