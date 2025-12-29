import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class WheelOdomNode(Node):
    def __init__(self):
        super().__init__('wheel_odom_node')

        self.sub = self.create_subscription(
            Odometry,
            '/odom',          # odom thô từ MCU
            self.odom_callback,
            50
        )

        self.pub = self.create_publisher(
            Odometry,
            '/wheel_odom',    # EKF đọc topic này
            50
        )

        self.get_logger().info('wheel_odom_node started (NO TF, SAFE QUATERNION, SAFE COVARIANCE)')

    def odom_callback(self, msg: Odometry):
        odom = Odometry()

        # ===== Header =====
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # ===== Pose =====
        odom.pose.pose.position = msg.pose.pose.position

        # 🔒 ÉP quaternion hợp lệ (chống NaN, chống (0,0,0,0))
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0

        # Pose covariance (KHÔNG ĐƯỢC = 0)
        odom.pose.covariance = [
            0.05, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.05, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.1,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.2,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.2,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.3
        ]

        # ===== Twist =====
        odom.twist.twist.linear.x = msg.twist.twist.linear.x
        odom.twist.twist.linear.y = msg.twist.twist.linear.y
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = msg.twist.twist.angular.z

        # Twist covariance (CỰC KỲ QUAN TRỌNG – không được = 0)
        odom.twist.covariance = [
            0.05, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.05, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.1,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.2,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.2,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.3
        ]

        self.pub.publish(odom)


def main():
    rclpy.init()
    node = WheelOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

