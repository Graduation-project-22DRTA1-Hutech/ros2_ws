import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped


class JoyToXarmServo(Node):
    def __init__(self):
        super().__init__('joy_to_xarm_servo')

        self.sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )

        self.linear_scale = 0.1  # m/s

        self.get_logger().info('Joy → xArm MoveIt Servo ready')

    def joy_callback(self, msg: Joy):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'

        # ===== MAPPING =====
        # axes[8] : X
        # axes[7] : Y
        # axes[3] + axes[6] : Z
        twist.twist.linear.x = self.linear_scale * msg.axes[7]
        twist.twist.linear.y = self.linear_scale * msg.axes[6]
        twist.twist.linear.z = self.linear_scale * (msg.axes[2] + msg.axes[6])

        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0

        self.pub.publish(twist)


def main():
    rclpy.init()
    node = JoyToXarmServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
